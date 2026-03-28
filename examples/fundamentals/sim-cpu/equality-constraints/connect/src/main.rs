//! Connect Constraint — Ball-and-Socket (3-DOF Position Lock)
//!
//! A double pendulum built entirely from equality constraints instead of joints.
//! Link1 is a free body connected to the world at the origin via `<connect>`.
//! Link2 is a free body connected to link1's tip. Both links rotate freely at
//! their connection points (ball-and-socket behavior).
//!
//! Validates:
//! - Pivot 1 stays at world origin (< 5mm)
//! - Pivot 2 stays at link1 tip (< 5mm)
//! - Rotation is free (angular velocity > 0.1 rad/s)
//! - Energy is bounded (< 5% growth)
//!
//! Run with: `cargo run -p example-equality-connect --release`

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
// Link1 origin at world origin (pivot 1). Link2 origin at link1 tip (pivot 2).
// Both have freejoints with light damping. Stiff solref for tight constraints.

const MJCF: &str = r#"
<mujoco model="connect-double-pendulum">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Link 1: free body, pivot at origin -->
    <body name="link1" pos="0 0 0">
      <freejoint damping="0.01"/>
      <geom name="rod1" type="capsule" fromto="0 0 0 0 0 -0.5" size="0.03" mass="1"/>
      <geom name="ball1" type="sphere" pos="0 0 -0.5" size="0.06" mass="0.5"/>
    </body>

    <!-- Link 2: free body, starts at link1 tip -->
    <body name="link2" pos="0 0 -0.5">
      <freejoint damping="0.01"/>
      <geom name="rod2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.025" mass="0.8"/>
      <geom name="ball2" type="sphere" pos="0 0 -0.4" size="0.05" mass="0.3"/>
    </body>
  </worldbody>

  <equality>
    <!-- Link1 pivot locked to world at body origin -->
    <connect body1="link1" anchor="0 0 0" solref="0.005 1.0"/>
    <!-- Link2 top locked to link1 bottom -->
    <connect body1="link1" body2="link2" anchor="0 0 -0.5" solref="0.005 1.0"/>
  </equality>
</mujoco>
"#;

// Body indices in the model (0 = world)
const BODY_LINK1: usize = 1;
const BODY_LINK2: usize = 2;

// Anchor offset in link1's local frame (tip of rod1)
const ANCHOR_LOCAL: Vector3<f64> = Vector3::new(0.0, 0.0, -0.5);

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Connect Constraint (Double Pendulum) ===");
    println!("  Two free bodies linked by connect constraints");
    println!("  Pivot 1: link1 → world at origin");
    println!("  Pivot 2: link1 tip → link2 origin");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Connect Constraint (Double Pendulum)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ConnectValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|m, d| {
                    let e = d.energy_kinetic + d.energy_potential;
                    let angvel = nalgebra::Vector3::new(d.qvel[3], d.qvel[4], d.qvel[5]).norm();
                    let _ = m; // used for API consistency
                    format!("E={e:.3}J  w1={angvel:.2}rad/s")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                connect_diagnostics,
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

    // Angular kick to start swinging (rotation around Y axis)
    data.qvel[4] = 2.0;
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    // ── Materials ───────────────────────────────────────────────────────
    let mat_rod1 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_ball1 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.3, 0.2)));
    let mat_rod2 = materials.add(MetalPreset::BrushedMetal.material());
    let mat_ball2 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.4, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("rod1", mat_rod1),
            ("ball1", mat_ball1),
            ("rod2", mat_rod2),
            ("ball2", mat_ball2),
        ],
    );

    // Camera: look at the midpoint of the pendulum, side view
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, -0.25, 0.0), // Bevy Y-up: center slightly below origin
        2.5,                        // distance
        std::f32::consts::FRAC_PI_4, // azimuth: 45°
        0.3,                        // slight elevation
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Connect — Double Pendulum");

    // Pivot 1 error: link1 origin should be at world origin
    let pivot1_err = data.xpos[BODY_LINK1].norm() * 1000.0;
    hud.scalar("pivot1 err (mm)", pivot1_err, 2);

    // Pivot 2 error: anchor in link1 frame should match link2 origin
    let anchor_world =
        data.xpos[BODY_LINK1] + data.xquat[BODY_LINK1].transform_vector(&ANCHOR_LOCAL);
    let pivot2_err = (anchor_world - data.xpos[BODY_LINK2]).norm() * 1000.0;
    hud.scalar("pivot2 err (mm)", pivot2_err, 2);

    // Angular velocities
    let angvel1 = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    let angvel2 = Vector3::new(data.qvel[9], data.qvel[10], data.qvel[11]).norm();
    hud.scalar("w1 (rad/s)", angvel1, 2);
    hud.scalar("w2 (rad/s)", angvel2, 2);

    // Energy
    let energy = data.energy_kinetic + data.energy_potential;
    hud.scalar("energy (J)", energy, 3);

    // Constraint forces (3 rows per connect = 6 total)
    if data.ne >= 6 {
        let f1 = Vector3::new(data.efc_force[0], data.efc_force[1], data.efc_force[2]).norm();
        let f2 = Vector3::new(data.efc_force[3], data.efc_force[4], data.efc_force[5]).norm();
        hud.scalar("efc |F1| (N)", f1, 1);
        hud.scalar("efc |F2| (N)", f2, 1);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ConnectValidation {
    max_pivot1_err: f64,
    max_pivot2_err: f64,
    max_angvel1: f64,
    initial_energy: Option<f64>,
    max_energy_growth: f64,
    reported: bool,
}

fn connect_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ConnectValidation>,
) {
    let _ = &*model; // suppress unused warning

    // Record initial energy
    if val.initial_energy.is_none() {
        val.initial_energy = Some(data.energy_kinetic + data.energy_potential);
    }

    // Pivot 1: link1 origin stays at world origin
    let err1 = data.xpos[BODY_LINK1].norm();
    val.max_pivot1_err = val.max_pivot1_err.max(err1);

    // Pivot 2: anchor at link1 tip stays at link2 origin
    let anchor_world =
        data.xpos[BODY_LINK1] + data.xquat[BODY_LINK1].transform_vector(&ANCHOR_LOCAL);
    let err2 = (anchor_world - data.xpos[BODY_LINK2]).norm();
    val.max_pivot2_err = val.max_pivot2_err.max(err2);

    // Angular velocity of link1
    let angvel1 = Vector3::new(data.qvel[3], data.qvel[4], data.qvel[5]).norm();
    val.max_angvel1 = val.max_angvel1.max(angvel1);

    // Energy growth
    if let Some(e0) = val.initial_energy {
        let e = data.energy_kinetic + data.energy_potential;
        let growth = (e - e0) / e0.abs().max(1e-10);
        val.max_energy_growth = val.max_energy_growth.max(growth);
    }

    // Final report
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
                name: "Rotation free",
                pass: val.max_angvel1 > 0.1,
                detail: format!("max w = {:.3} rad/s", val.max_angvel1),
            },
            Check {
                name: "Energy bounded",
                pass: val.max_energy_growth < 0.05,
                detail: format!("max growth = {:.2}%", val.max_energy_growth * 100.0),
            },
        ];
        let _ = print_report("Connect Constraint (t=5s)", &checks);
    }
}
