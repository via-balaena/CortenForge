//! Multi-Body — Two-Link Arm with Control State
//!
//! A two-link planar arm with 2 hinge joints and 2 position actuators cycles
//! through 3 named keyframes (home, reach-left, reach-right) every 3 seconds.
//! Demonstrates keyframes with both `qpos` and `ctrl` fields — the control
//! state is part of the snapshot.
//!
//! Validates:
//! - qpos and ctrl match keyframe values after each reset
//! - Simulation runs cleanly after reset (no NaN)
//!
//! Run with: `cargo run -p example-keyframes-multi-body --release`

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

// Two-link planar arm. Both hinges rotate around the Y axis (motion in the
// XZ plane). Positive qpos rotates the arm's local +X toward -Z.
//
// Shoulder at (0, 0, 2), upper arm 0.6m along +X.
// Elbow at end of upper arm, forearm 0.5m along +X.
//
// Position actuators: gain=50, bias=[0, -50, -5] gives a stiff P+D
// controller: force = 50*(ctrl - qpos) - 5*qvel.
const MJCF: &str = r#"
<mujoco model="two-link-arm">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="upper" pos="0 0 2">
      <joint name="shoulder" type="hinge" axis="0 1 0"/>
      <geom name="upper_arm" type="capsule" size="0.04"
            fromto="0 0 0 0.6 0 0" mass="1.0" rgba="0.7 0.7 0.75 1"/>
      <body name="lower" pos="0.6 0 0">
        <joint name="elbow" type="hinge" axis="0 1 0"/>
        <geom name="forearm" type="capsule" size="0.035"
              fromto="0 0 0 0.5 0 0" mass="0.7" rgba="0.6 0.65 0.7 1"/>
        <geom name="hand" type="sphere" size="0.05" pos="0.5 0 0"
              mass="0.3" rgba="0.85 0.30 0.15 1"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="shoulder_ctrl" joint="shoulder" kp="50" kv="5"/>
    <position name="elbow_ctrl"    joint="elbow"    kp="50" kv="5"/>
  </actuator>

  <keyframe>
    <key name="home"
         qpos="1.5708 0.0"
         ctrl="1.5708 0.0"/>
    <key name="reach-left"
         qpos="0.5236 -0.7854"
         ctrl="0.5236 -0.7854"/>
    <key name="reach-right"
         qpos="2.6180 0.7854"
         ctrl="2.6180 0.7854"/>
  </keyframe>
</mujoco>
"#;

// ── Constants ─────────────────────────────────────────────────────────────

const RESET_PERIOD: f64 = 3.0;
const KEYFRAME_NAMES: [&str; 3] = ["home", "reach-left", "reach-right"];

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Multi-Body — Two-Link Arm with Control State ===");
    println!("  Cycles through 3 keyframes every {RESET_PERIOD}s: home, reach-left, reach-right");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Multi-Body (Keyframes)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ArmValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|_m, d| {
                    let kf_name = KEYFRAME_NAMES[ArmState::cycle_index(d.time)];
                    format!(
                        "kf=\"{kf_name}\"  shoulder={:.3}  elbow={:.3}  ctrl=({:.3}, {:.3})",
                        d.qpos[0], d.qpos[1], d.ctrl[0], d.ctrl[1],
                    )
                }),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, (reset_cycle, step_physics_realtime).chain())
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                validation_system,
                arm_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── State ─────────────────────────────────────────────────────────────────

#[derive(Resource)]
struct ArmState {
    last_reset: f64,
    current_idx: usize,
    keyframe_ids: [usize; 3],
}

impl ArmState {
    fn cycle_index(sim_time: f64) -> usize {
        let cycle = (sim_time / RESET_PERIOD) as usize;
        cycle % KEYFRAME_NAMES.len()
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = sim_mjcf::load_model(MJCF).expect("MJCF should parse");
    let mut data = model.make_data();

    let keyframe_ids: [usize; 3] = std::array::from_fn(|i| {
        model
            .keyframe_id(KEYFRAME_NAMES[i])
            .unwrap_or_else(|| panic!("keyframe '{}' not found", KEYFRAME_NAMES[i]))
    });

    // Start from home.
    data.reset_to_keyframe(&model, keyframe_ids[0])
        .expect("reset");
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, {} actuators, {} DOFs",
        model.nbody, model.njnt, model.nu, model.nv
    );
    println!(
        "  Keyframes: {} ({})\n",
        model.nkeyframe,
        KEYFRAME_NAMES.join(", ")
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_upper =
        materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.7, 0.7, 0.75)));
    let mat_forearm =
        materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.6, 0.65, 0.7)));
    let mat_hand =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.30, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[
            ("upper_arm", mat_upper),
            ("forearm", mat_forearm),
            ("hand", mat_hand),
        ],
    );

    // Camera: face the arm from the side (looking along -Y)
    spawn_example_camera(
        &mut commands,
        physics_pos(0.3, 0.0, 1.8),
        4.0,
        std::f32::consts::FRAC_PI_2,
        0.05,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(ArmState {
        last_reset: 0.0,
        current_idx: 0,
        keyframe_ids,
    });
    commands.insert_resource(PhysicsModel(model));
    commands.insert_resource(PhysicsData(data));
}

// ── Reset Cycle ───────────────────────────────────────────────────────────

fn reset_cycle(
    model: Res<PhysicsModel>,
    mut data: ResMut<PhysicsData>,
    mut state: ResMut<ArmState>,
) {
    if data.time - state.last_reset >= RESET_PERIOD {
        state.current_idx = (state.current_idx + 1) % KEYFRAME_NAMES.len();
        let kf_idx = state.keyframe_ids[state.current_idx];
        let name = KEYFRAME_NAMES[state.current_idx];

        data.reset_to_keyframe(&model, kf_idx).expect("reset");
        data.forward(&model).expect("forward");
        state.last_reset = data.time;

        println!(
            "  [{:.1}s] Reset to \"{name}\" — shoulder={:.4}, elbow={:.4}, \
             ctrl=({:.4}, {:.4})",
            data.time, data.qpos[0], data.qpos[1], data.ctrl[0], data.ctrl[1],
        );
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, state: Res<ArmState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Multi-Body — Arm Keyframes");

    let name = KEYFRAME_NAMES[state.current_idx];
    hud.raw(format!("keyframe: \"{name}\""));
    hud.scalar("t since reset", data.time - state.last_reset, 2);
    hud.scalar("shoulder (rad)", data.qpos[0], 4);
    hud.scalar("elbow (rad)", data.qpos[1], 4);
    hud.scalar("ctrl[0]", data.ctrl[0], 4);
    hud.scalar("ctrl[1]", data.ctrl[1], 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ArmValidation {
    exact_resets: u32,
    total_resets: u32,
    nan_detected: bool,
    prev_cycle: usize,
    reported: bool,
}

fn arm_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    state: Res<ArmState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ArmValidation>,
) {
    if data.qpos.iter().any(|v| !v.is_finite()) || data.ctrl.iter().any(|v| !v.is_finite()) {
        val.nan_detected = true;
    }

    let cycle = ArmState::cycle_index(data.time);
    if cycle != val.prev_cycle && data.time > 0.1 {
        val.total_resets += 1;

        let t_since = data.time - state.last_reset;
        if t_since < 0.01 {
            let kf_idx = state.keyframe_ids[state.current_idx];
            let kf = &model.keyframes[kf_idx];
            let qpos_ok = data.qpos[0] == kf.qpos[0] && data.qpos[1] == kf.qpos[1];
            let ctrl_ok = data.ctrl[0] == kf.ctrl[0] && data.ctrl[1] == kf.ctrl[1];
            if qpos_ok && ctrl_ok {
                val.exact_resets += 1;
            }
        }
    }
    val.prev_cycle = cycle;

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Keyframe resets are bitwise-exact (qpos + ctrl)",
                pass: val.exact_resets > 0 && val.exact_resets == val.total_resets,
                detail: format!("{}/{} resets exact", val.exact_resets, val.total_resets),
            },
            Check {
                name: "No NaN in simulation state",
                pass: !val.nan_detected,
                detail: format!("nan_detected = {}", val.nan_detected),
            },
        ];
        let _ = print_report("Multi-Body Arm (t=15s)", &checks);
    }
}
