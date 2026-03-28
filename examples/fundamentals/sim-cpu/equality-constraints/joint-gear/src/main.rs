//! Joint Gear — 2:1 Gear Ratio
//!
//! Two hinge arms with a joint equality constraint enforcing `j2 = 2*j1`
//! (polycoef="0 2"). A motor actuator drives j1 with a sinusoidal signal;
//! j2 follows at double the angle — a 2:1 gear ratio.
//!
//! Demonstrates: `<joint>` equality constraint, polycoef gear ratio,
//! motor driving through a constraint.
//!
//! Validates:
//! - Gear ratio holds (|j2 - 2*j1| < 0.1 rad)
//! - Motor drives (j1 oscillates visibly)
//!
//! Run with: `cargo run -p example-equality-joint-gear --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use bevy::prelude::*;
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
// Two hinge arms with 2:1 gear coupling. Motor on j1, j2 follows at 2x.

const MJCF: &str = r#"
<mujoco model="joint-gear">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <body name="arm1" pos="-0.3 0 1.5">
      <joint name="j1" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>
    <body name="arm2" pos="0.3 0 1.5">
      <joint name="j2" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="drive" joint="j1" gear="1"/>
  </actuator>

  <equality>
    <!-- j2 = 2*j1 -->
    <joint joint1="j2" joint2="j1" polycoef="0 2" solref="0.05 1.0"/>
  </equality>
</mujoco>
"#;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Joint Gear (2:1) ===");
    println!("  Motor drives j1, j2 follows at 2x angle (gear ratio)");
    println!("  Sinusoidal motor: ctrl = 2*sin(2*pi*t)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Joint Gear (2:1)".into(),
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
                    let gear_err = (d.qpos[1] - 2.0 * d.qpos[0]).abs();
                    format!(
                        "j1={:.3}  j2={:.3}  gear_err={gear_err:.4}rad",
                        d.qpos[0], d.qpos[1]
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (apply_motor, step_physics_realtime).chain())
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

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} equality constraints\n",
        model.nbody, model.njnt, model.nu, model.neq
    );

    let mat1 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat2 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("arm1", mat1), ("arm2", mat2)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.9, 0.0),
        2.5,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Motor Control ───────────────────────────────────────────────────────────

fn apply_motor(data: ResMut<PhysicsData>) {
    let t = data.time;
    let ctrl = 2.0 * (2.0 * std::f64::consts::PI * t).sin();
    data.into_inner().set_ctrl(0, ctrl);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(_model: Res<PhysicsModel>, data: Res<PhysicsData>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Joint Gear (2:1)");

    hud.scalar("j1 (rad)", data.qpos[0], 3);
    hud.scalar("j2 (rad)", data.qpos[1], 3);
    hud.scalar(
        "gear err (rad)",
        (data.qpos[1] - 2.0 * data.qpos[0]).abs(),
        4,
    );

    if !data.actuator_force.is_empty() {
        hud.scalar("motor torque (N·m)", data.actuator_force[0], 2);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct Validation {
    max_gear_err: f64,
    j1_min: f64,
    j1_max: f64,
    started: bool,
    reported: bool,
}

fn diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<Validation>,
) {
    if !val.started {
        val.j1_min = f64::MAX;
        val.j1_max = f64::MIN;
        val.started = true;
    }

    let gear_err = (data.qpos[1] - 2.0 * data.qpos[0]).abs();
    if data.time >= 1.0 {
        val.max_gear_err = val.max_gear_err.max(gear_err);
    }

    if data.time > 0.5 {
        val.j1_min = val.j1_min.min(data.qpos[0]);
        val.j1_max = val.j1_max.max(data.qpos[0]);
    }

    if harness.reported() && !val.reported {
        val.reported = true;
        let motor_oscillated = (val.j1_max - val.j1_min) > 0.05;
        let checks = vec![
            Check {
                name: "Gear ratio",
                pass: val.max_gear_err < 0.1,
                detail: format!("max |j2 - 2*j1| = {:.4} rad", val.max_gear_err),
            },
            Check {
                name: "Motor drives",
                pass: motor_oscillated,
                detail: format!("j1 range = {:.3}..{:.3}", val.j1_min, val.j1_max),
            },
        ];
        let _ = print_report("Joint Gear (t=5s)", &checks);
    }
}
