//! Joint Coupling Constraint — Gear Trains and Mimic Joints
//!
//! Two articulated mechanisms demonstrating joint equality constraints:
//!
//! (A) Mimic (1:1) — Left pair. Two hinge arms start at different angles
//!     (0.5, -0.3 rad). The constraint pulls them together. After convergence
//!     they swing in unison under gravity.
//!
//! (B) Gear (2:1) — Right pair. A motor drives j_b1 with a sinusoidal signal.
//!     The constraint enforces j_b2 = 2*j_b1, so the second arm swings at
//!     double the angle.
//!
//! Validates:
//! - Mimic converges (|j_a1 - j_a2| < 0.05 rad after 1s)
//! - Mimic tracks (stays < 0.05 rad after convergence)
//! - Gear ratio holds (|j_b2 - 2*j_b1| < 0.1 rad)
//! - Motor drives (j_b1 oscillates visibly)
//!
//! Run with: `cargo run -p example-equality-joint-coupling --release`

#![allow(
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use,
    clippy::similar_names,
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
// Four hinge arms in two pairs:
//   Left (mimic):  j_a1 and j_a2 coupled 1:1 via polycoef="0 1"
//   Right (gear):  j_b2 = 2*j_b1 via polycoef="0 2", motor on j_b1
//
// Softer solref (0.05) for stability with joint-space constraints.

const MJCF: &str = r#"
<mujoco model="joint-coupling">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001">
    <flag energy="enable"/>
  </option>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <!-- Mimic pair (left) -->
    <body name="arm_a1" pos="-0.8 0 1.5">
      <joint name="j_a1" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm_a1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>
    <body name="arm_a2" pos="-0.4 0 1.5">
      <joint name="j_a2" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm_a2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>

    <!-- Gear pair (right) — motor drives j_b1, j_b2 follows at 2x -->
    <body name="arm_b1" pos="0.4 0 1.5">
      <joint name="j_b1" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm_b1" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>
    <body name="arm_b2" pos="0.8 0 1.5">
      <joint name="j_b2" type="hinge" axis="0 1 0" damping="0.5"/>
      <geom name="arm_b2" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.03" mass="0.5"/>
    </body>
  </worldbody>

  <actuator>
    <motor name="drive" joint="j_b1" gear="1"/>
  </actuator>

  <equality>
    <!-- Mimic: j_a1 = j_a2 (1:1) -->
    <joint joint1="j_a1" joint2="j_a2" polycoef="0 1" solref="0.05 1.0"/>
    <!-- Gear: j_b2 = 2*j_b1 -->
    <joint joint1="j_b2" joint2="j_b1" polycoef="0 2" solref="0.05 1.0"/>
  </equality>
</mujoco>
"#;

// Joint indices: j_a1=0, j_a2=1, j_b1=2, j_b2=3

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Joint Coupling (Mimic + Gear) ===");
    println!("  Left:  mimic pair (1:1) — start at different angles, converge");
    println!("  Right: gear pair (2:1) — motor drives j_b1, j_b2 follows at 2x");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Joint Coupling (Mimic + Gear)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<CouplingValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let mimic_err = (d.qpos[0] - d.qpos[1]).abs();
                    let gear_err = (d.qpos[3] - 2.0 * d.qpos[2]).abs();
                    format!(
                        "mimic_err={:.3}rad  gear_err={:.3}rad  q=[{:.2},{:.2},{:.2},{:.2}]",
                        mimic_err, gear_err, d.qpos[0], d.qpos[1], d.qpos[2], d.qpos[3]
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
                coupling_diagnostics,
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

    // Mimic pair: start at different angles to show convergence
    data.qpos[0] = 0.5; // j_a1
    data.qpos[1] = -0.3; // j_a2
    let _ = data.forward(&model);

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} equality constraints\n",
        model.nbody, model.njnt, model.nu, model.neq
    );

    // ── Materials ───────────────────────────────────────────────────────
    // Mimic pair: matching warm colors
    let mat_a1 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.45, 0.15)));
    let mat_a2 =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.55, 0.25)));
    // Gear pair: matching cool colors
    let mat_b1 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    let mat_b2 = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.6, 0.85)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("arm_a1", mat_a1),
            ("arm_a2", mat_a2),
            ("arm_b1", mat_b1),
            ("arm_b2", mat_b2),
        ],
    );

    // Camera: centered between the two pairs, looking from the front
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.9, 0.0), // Bevy Y-up: MuJoCo z=1.5 → Bevy ~0.9 (center of arm swing)
        3.5,
        std::f32::consts::FRAC_PI_2, // azimuth: 90° — face the XZ plane
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
    hud.section("Joint Coupling");

    // Joint angles
    let q_a1 = data.qpos[0];
    let q_a2 = data.qpos[1];
    let q_b1 = data.qpos[2];
    let q_b2 = data.qpos[3];

    hud.section("Mimic (1:1)");
    hud.scalar("j_a1 (rad)", q_a1, 3);
    hud.scalar("j_a2 (rad)", q_a2, 3);
    hud.scalar("mimic err (rad)", (q_a1 - q_a2).abs(), 4);

    hud.section("Gear (2:1)");
    hud.scalar("j_b1 (rad)", q_b1, 3);
    hud.scalar("j_b2 (rad)", q_b2, 3);
    hud.scalar("gear err (rad)", (q_b2 - 2.0 * q_b1).abs(), 4);

    // Motor torque
    if !data.actuator_force.is_empty() {
        hud.scalar("motor torque (N·m)", data.actuator_force[0], 2);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct CouplingValidation {
    mimic_converged: bool,
    max_mimic_err_after_1s: f64,
    max_gear_err_after_1s: f64,
    j_b1_min: f64,
    j_b1_max: f64,
    started: bool,
    reported: bool,
}

fn coupling_diagnostics(
    _model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<CouplingValidation>,
) {
    if !val.started {
        val.j_b1_min = f64::MAX;
        val.j_b1_max = f64::MIN;
        val.started = true;
    }

    let q_a1 = data.qpos[0];
    let q_a2 = data.qpos[1];
    let q_b1 = data.qpos[2];
    let q_b2 = data.qpos[3];

    // Mimic convergence
    let mimic_err = (q_a1 - q_a2).abs();
    if data.time >= 1.0 {
        if !val.mimic_converged && mimic_err < 0.1 {
            val.mimic_converged = true;
        }
        val.max_mimic_err_after_1s = val.max_mimic_err_after_1s.max(mimic_err);
    }

    // Gear ratio
    let gear_err = (q_b2 - 2.0 * q_b1).abs();
    if data.time >= 1.0 {
        val.max_gear_err_after_1s = val.max_gear_err_after_1s.max(gear_err);
    }

    // Motor oscillation range
    if data.time > 0.5 {
        val.j_b1_min = val.j_b1_min.min(q_b1);
        val.j_b1_max = val.j_b1_max.max(q_b1);
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let motor_oscillated = (val.j_b1_max - val.j_b1_min) > 0.05;

        let checks = vec![
            Check {
                name: "Mimic converges",
                pass: val.mimic_converged,
                detail: "converged by 1s".into(),
            },
            Check {
                name: "Mimic tracks",
                pass: val.max_mimic_err_after_1s < 0.1,
                detail: format!("max err = {:.4} rad", val.max_mimic_err_after_1s),
            },
            Check {
                name: "Gear ratio",
                pass: val.max_gear_err_after_1s < 0.15,
                detail: format!("max |q_b2 - 2*q_b1| = {:.4} rad", val.max_gear_err_after_1s),
            },
            Check {
                name: "Motor drives",
                pass: motor_oscillated,
                detail: format!("q range = {:.3}..{:.3}", val.j_b1_min, val.j_b1_max),
            },
        ];
        let _ = print_report("Joint Coupling (t=5s)", &checks);
    }
}
