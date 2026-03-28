//! Weld to World — Immovable Body
//!
//! A single free body welded to the world. Despite gravity pulling it down,
//! the 6-DOF pose lock keeps it fixed in space — both position AND orientation
//! are locked. This is the simplest weld: one body, frozen in place.
//!
//! Demonstrates: `<weld>` to world, 6-DOF pose lock, penalty stiffness
//! visible as tiny sag under gravity.
//!
//! Validates:
//! - Body stays at initial position (< 2mm displacement)
//! - Body orientation unchanged (< 0.01 rad)
//! - Constraint forces resist gravity
//!
//! Run with: `cargo run -p example-equality-weld-to-world --release`

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
<mujoco model="weld-to-world">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="fixed" pos="0 0 1.0">
      <freejoint/>
      <geom name="sphere" type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>

  <equality>
    <weld body1="fixed" solref="0.002 1.0"/>
  </equality>
</mujoco>
"#;

const BODY_FIXED: usize = 1;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Weld to World ===");
    println!("  Single body welded to world — immovable despite gravity");
    println!("  6-DOF pose lock (position + orientation)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Weld to World".into(),
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
                    let z = d.xpos[1][2];
                    format!("z={z:.4}")
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

    let init_pos = data.xpos[BODY_FIXED];
    let init_quat = data.xquat[BODY_FIXED];

    println!(
        "  Model: {} bodies, {} joints, {} equality constraints",
        model.nbody, model.njnt, model.neq
    );
    println!(
        "  Initial pos: [{:.3}, {:.3}, {:.3}]\n",
        init_pos[0], init_pos[1], init_pos[2]
    );

    let mat = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.6, 0.3)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("sphere", mat)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.6, 0.0), // Bevy Y-up
        2.0,
        std::f32::consts::FRAC_PI_4,
        0.2,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(InitialState {
        pos: init_pos,
        quat: init_quat,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

#[derive(Resource)]
struct InitialState {
    pos: Vector3<f64>,
    quat: nalgebra::UnitQuaternion<f64>,
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, init: Res<InitialState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Weld to World");

    let disp = (data.xpos[BODY_FIXED] - init.pos).norm() * 1000.0;
    hud.scalar("displacement (mm)", disp, 2);

    let q = (init.quat.inverse() * data.xquat[BODY_FIXED]).into_inner();
    let angle = quat_rotation_angle(q.w, q.i, q.j, q.k);
    hud.scalar("rotation err (rad)", angle, 4);

    hud.scalar("z position", data.xpos[BODY_FIXED][2], 4);

    if data.ne >= 6 {
        let f: f64 = (0..6).map(|i| data.efc_force[i] * data.efc_force[i]).sum();
        hud.scalar("efc |F| (N)", f.sqrt(), 1);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    max_displacement: f64,
    max_angle_err: f64,
    reported: bool,
}

fn diagnostics(
    data: Res<PhysicsData>,
    init: Res<InitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    let disp = (data.xpos[BODY_FIXED] - init.pos).norm();
    val.max_displacement = val.max_displacement.max(disp);

    let q = (init.quat.inverse() * data.xquat[BODY_FIXED]).into_inner();
    let angle = quat_rotation_angle(q.w, q.i, q.j, q.k);
    val.max_angle_err = val.max_angle_err.max(angle);

    if harness.reported() && !val.reported {
        val.reported = true;
        let checks = vec![
            Check {
                name: "Fixed in space",
                pass: val.max_displacement < 0.002,
                detail: format!("max disp = {:.2} mm", val.max_displacement * 1000.0),
            },
            Check {
                name: "Orientation locked",
                pass: val.max_angle_err < 0.01,
                detail: format!("max angle = {:.4} rad", val.max_angle_err),
            },
        ];
        let _ = print_report("Weld to World (t=5s)", &checks);
    }
}
