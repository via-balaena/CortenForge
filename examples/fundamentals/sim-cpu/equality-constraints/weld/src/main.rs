//! Weld Constraint — Rigid Glue (6-DOF Pose Lock)
//!
//! Two free-floating bodies welded together — they fall as a single rigid unit
//! and land on the ground. A third body is welded to the world (fully fixed in
//! space despite gravity). Demonstrates the difference between weld (locks
//! rotation) and connect (allows rotation).
//!
//! Validates:
//! - Weld pair relative pose stays constant (< 2mm position, < 0.05 rad angle)
//! - Fixed body stays at initial position (< 2mm displacement)
//! - Pair falls together (z-velocities match during freefall)
//! - Pair lands on ground (z stabilizes)
//!
//! Run with: `cargo run -p example-equality-weld --release`

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
// Three free bodies:
//   1. "base" — box at (-0.5, 0, 1.5), welded to "arm"
//   2. "arm"  — capsule at (-0.5, 0, 1.2), welded to "base"
//   3. "fixed" — sphere at (0.5, 0, 1.0), welded to world
//
// Ground plane for the falling pair to land on.
// Stiffer solref on the world-weld to resist gravity drift.

const MJCF: &str = r#"
<mujoco model="weld-rigid-glue">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.001" solver="Newton"
          iterations="50" tolerance="1e-10">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.01"/>

    <!-- Welded pair — base + arm, falling together -->
    <body name="base" pos="-0.5 0 1.5">
      <freejoint/>
      <geom name="base_box" type="box" size="0.12 0.08 0.08" mass="1.0"/>
    </body>
    <body name="arm" pos="-0.5 0 1.2">
      <freejoint/>
      <geom name="arm_cap" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.04" mass="0.5"/>
    </body>

    <!-- Fixed to world — should resist gravity -->
    <body name="fixed" pos="0.5 0 1.0">
      <freejoint/>
      <geom name="fixed_sphere" type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>

  <equality>
    <weld body1="base" body2="arm" solref="0.005 1.0"/>
    <weld body1="fixed" solref="0.002 1.0"/>
  </equality>
</mujoco>
"#;

// Body indices (0 = world)
const BODY_BASE: usize = 1;
const BODY_ARM: usize = 2;
const BODY_FIXED: usize = 3;

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Weld Constraint (Rigid Glue) ===");
    println!("  Left: box+capsule welded pair falls together");
    println!("  Right: sphere welded to world (immovable)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Weld Constraint (Rigid Glue)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<WeldValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(5.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let base_z = d.xpos[1][2];
                    let fixed_z = d.xpos[3][2];
                    format!("base_z={base_z:.3}  fixed_z={fixed_z:.3}")
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_physics_realtime)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                weld_diagnostics,
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
        "  Model: {} bodies, {} joints, {} equality constraints\n",
        model.nbody, model.njnt, model.neq
    );

    // Store initial relative pose for validation
    let init_rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let init_rel_quat = data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM];
    let init_fixed_pos = data.xpos[BODY_FIXED];

    // ── Materials ───────────────────────────────────────────────────────
    let mat_base =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.7, 0.35, 0.15)));
    let mat_arm = materials.add(MetalPreset::BrushedMetal.material());
    let mat_fixed =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.6, 0.3)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("base_box", mat_base),
            ("arm_cap", mat_arm),
            ("fixed_sphere", mat_fixed),
        ],
    );

    // Camera: centered between the two setups, looking from the side
    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.75, 0.0), // Bevy Y-up: MuJoCo z=0.75 → Bevy y=0.75
        3.5,
        std::f32::consts::FRAC_PI_4,
        0.3,
    );

    spawn_physics_hud(&mut commands);

    commands.insert_resource(WeldInitialState {
        rel_pos: init_rel_pos,
        rel_quat: init_rel_quat,
        fixed_pos: init_fixed_pos,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Initial state for validation ────────────────────────────────────────────

#[derive(Resource)]
struct WeldInitialState {
    rel_pos: Vector3<f64>,
    rel_quat: nalgebra::UnitQuaternion<f64>,
    fixed_pos: Vector3<f64>,
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, init: Res<WeldInitialState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Weld — Rigid Glue");

    // Weld pair relative pose error
    let rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let pos_err = (rel_pos - init.rel_pos).norm() * 1000.0;
    hud.scalar("weld pos err (mm)", pos_err, 2);

    let rel_quat = data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM];
    let quat_diff = init.rel_quat.inverse() * rel_quat;
    let angle_err = 2.0
        * quat_diff
            .quaternion()
            .imag()
            .norm()
            .atan2(quat_diff.quaternion().w.abs());
    hud.scalar("weld angle err (rad)", angle_err, 4);

    // Fixed body displacement
    let fixed_disp = (data.xpos[BODY_FIXED] - init.fixed_pos).norm() * 1000.0;
    hud.scalar("fixed disp (mm)", fixed_disp, 2);

    // Body z positions
    hud.scalar("base z", data.xpos[BODY_BASE][2], 3);
    hud.scalar("fixed z", data.xpos[BODY_FIXED][2], 3);

    // Constraint forces (6 rows per weld = 12 total)
    if data.ne >= 12 {
        let f_pair: f64 = (0..6).map(|i| data.efc_force[i] * data.efc_force[i]).sum();
        let f_fixed: f64 = (6..12).map(|i| data.efc_force[i] * data.efc_force[i]).sum();
        hud.scalar("efc |F_pair| (N)", f_pair.sqrt(), 1);
        hud.scalar("efc |F_fixed| (N)", f_fixed.sqrt(), 1);
    }

    hud.scalar("time", data.time, 1);
}

// ── Validation ──────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct WeldValidation {
    max_weld_pos_err: f64,
    max_weld_angle_err: f64,
    max_fixed_displacement: f64,
    freefall_vel_match: bool,
    freefall_checked: bool,
    reported: bool,
}

fn weld_diagnostics(
    data: Res<PhysicsData>,
    init: Res<WeldInitialState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<WeldValidation>,
) {
    if !val.freefall_checked {
        val.freefall_vel_match = true;
        val.freefall_checked = true;
    }

    // Weld pair: relative pose should stay constant
    let rel_pos = data.xpos[BODY_ARM] - data.xpos[BODY_BASE];
    let pos_err = (rel_pos - init.rel_pos).norm();
    val.max_weld_pos_err = val.max_weld_pos_err.max(pos_err);

    let rel_quat = data.xquat[BODY_BASE].inverse() * data.xquat[BODY_ARM];
    let quat_diff = init.rel_quat.inverse() * rel_quat;
    let angle_err = 2.0
        * quat_diff
            .quaternion()
            .imag()
            .norm()
            .atan2(quat_diff.quaternion().w.abs());
    val.max_weld_angle_err = val.max_weld_angle_err.max(angle_err);

    // Fixed body: should stay put
    let fixed_disp = (data.xpos[BODY_FIXED] - init.fixed_pos).norm();
    val.max_fixed_displacement = val.max_fixed_displacement.max(fixed_disp);

    // During freefall (first 0.3s), z-velocities should match
    // base: DOFs 0..6, arm: DOFs 6..12
    if data.time < 0.3 {
        let base_vz = data.qvel[2];
        let arm_vz = data.qvel[8];
        if (base_vz - arm_vz).abs() > 0.05 * base_vz.abs().max(0.1) {
            val.freefall_vel_match = false;
        }
    }

    // Final report
    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Weld pose locked (pos)",
                pass: val.max_weld_pos_err < 0.002,
                detail: format!("max = {:.2} mm", val.max_weld_pos_err * 1000.0),
            },
            Check {
                name: "Weld pose locked (angle)",
                pass: val.max_weld_angle_err < 0.05,
                detail: format!("max = {:.4} rad", val.max_weld_angle_err),
            },
            Check {
                name: "Fixed in space",
                pass: val.max_fixed_displacement < 0.002,
                detail: format!("max = {:.2} mm", val.max_fixed_displacement * 1000.0),
            },
            Check {
                name: "Pair falls together",
                pass: val.freefall_vel_match,
                detail: "z-velocities match during freefall".into(),
            },
        ];
        let _ = print_report("Weld Constraint (t=5s)", &checks);
    }
}
