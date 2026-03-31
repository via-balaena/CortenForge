//! Save-Restore — Pendulum Keyframe Cycling
//!
//! A single-link pendulum cycles through 3 named keyframes (rest, horizontal,
//! inverted) every 3 seconds. Demonstrates `<key>` MJCF elements,
//! `model.keyframe_id()` for name lookup, and `Data::reset_to_keyframe()` for
//! full state restoration.
//!
//! Validates:
//! - qpos/qvel match keyframe values exactly after each reset
//! - Simulation runs cleanly after reset (no NaN, finite energy)
//!
//! Run with: `cargo run -p example-keyframes-save-restore --release`

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
<mujoco model="pendulum-keyframes">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>

  <worldbody>
    <body name="arm" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom name="rod" type="capsule" size="0.03" fromto="0 0 0 0.8 0 0"
            mass="0.5" rgba="0.7 0.7 0.75 1"/>
      <geom name="tip" type="sphere" size="0.06" pos="0.8 0 0"
            mass="0.5" rgba="0.85 0.30 0.15 1"/>
    </body>
  </worldbody>

  <keyframe>
    <key name="rest"       qpos="1.5707963268"  qvel="0.0"/>
    <key name="horizontal" qpos="0.0"           qvel="0.0"/>
    <key name="inverted"   qpos="-1.5707963268" qvel="0.0"/>
  </keyframe>
</mujoco>
"#;

// ── Constants ─────────────────────────────────────────────────────────────

const RESET_PERIOD: f64 = 3.0;
const KEYFRAME_NAMES: [&str; 3] = ["rest", "horizontal", "inverted"];

// ── Bevy App ──────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Save-Restore — Pendulum Keyframe Cycling ===");
    println!("  Cycles through 3 keyframes every {RESET_PERIOD}s: rest, horizontal, inverted");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Save-Restore (Keyframes)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<ResetValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(15.0)
                .print_every(1.0)
                .display(|m, d| {
                    let kf_name = KEYFRAME_NAMES[ResetState::cycle_index(d.time)];
                    let _ = m; // unused but required by signature
                    format!(
                        "keyframe=\"{kf_name}\"  qpos={:.4}  qvel={:.4}",
                        d.qpos[0], d.qvel[0],
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
                reset_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── State ─────────────────────────────────────────────────────────────────

#[derive(Resource)]
struct ResetState {
    /// Simulation time of the last reset.
    last_reset: f64,
    /// Index into KEYFRAME_NAMES for the *current* keyframe.
    current_idx: usize,
    /// Model-level keyframe indices (looked up by name at startup).
    keyframe_ids: [usize; 3],
}

impl ResetState {
    /// Which cycle index (0..3) we're on at a given sim time.
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

    // Look up keyframe indices by name.
    let keyframe_ids: [usize; 3] = std::array::from_fn(|i| {
        model
            .keyframe_id(KEYFRAME_NAMES[i])
            .unwrap_or_else(|| panic!("keyframe '{}' not found", KEYFRAME_NAMES[i]))
    });

    // Start from the first keyframe.
    data.reset_to_keyframe(&model, keyframe_ids[0])
        .expect("reset");
    data.forward(&model).expect("forward");

    println!(
        "  Model: {} bodies, {} joints, {} DOFs",
        model.nbody, model.njnt, model.nv
    );
    println!(
        "  Keyframes: {} ({})\n",
        model.nkeyframe,
        KEYFRAME_NAMES.join(", ")
    );

    // ── Materials ──────────────────────────────────────────────────────
    let mat_rod = materials.add(MetalPreset::BrushedMetal.with_color(Color::srgb(0.7, 0.7, 0.75)));
    let mat_tip =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.30, 0.15)));

    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    // Camera: face the pendulum from the side (looking along -Y)
    spawn_example_camera(
        &mut commands,
        physics_pos(0.4, 0.0, 2.0),
        3.5,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);

    commands.insert_resource(ResetState {
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
    mut state: ResMut<ResetState>,
) {
    if data.time - state.last_reset >= RESET_PERIOD {
        // Advance to next keyframe in cycle.
        state.current_idx = (state.current_idx + 1) % KEYFRAME_NAMES.len();
        let kf_idx = state.keyframe_ids[state.current_idx];
        let name = KEYFRAME_NAMES[state.current_idx];

        let qpos_before = data.qpos[0];
        let qvel_before = data.qvel[0];

        data.reset_to_keyframe(&model, kf_idx).expect("reset");
        data.forward(&model).expect("forward");
        state.last_reset = data.time;

        println!(
            "  [{:.1}s] Reset to \"{name}\" — qpos: {qpos_before:.4} -> {:.4}, \
             qvel: {qvel_before:.4} -> {:.4}",
            data.time, data.qpos[0], data.qvel[0],
        );
    }
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(data: Res<PhysicsData>, state: Res<ResetState>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Save-Restore — Keyframe Cycling");

    let name = KEYFRAME_NAMES[state.current_idx];
    hud.raw(format!("keyframe: \"{name}\""));
    hud.scalar("t since reset", data.time - state.last_reset, 2);
    hud.scalar("qpos (rad)", data.qpos[0], 4);
    hud.scalar("qvel (rad/s)", data.qvel[0], 4);

    let e = data.total_energy();
    hud.scalar("E_total", e, 4);
    hud.scalar("time", data.time, 1);
}

// ── Validation ────────────────────────────────────────────────────────────

#[derive(Resource, Default)]
struct ResetValidation {
    /// Count of resets where qpos matched exactly.
    exact_resets: u32,
    /// Total resets observed.
    total_resets: u32,
    /// Any NaN detected.
    nan_detected: bool,
    /// Track previous cycle index to detect transitions.
    prev_cycle: usize,
    reported: bool,
}

fn reset_diagnostics(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    state: Res<ResetState>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ResetValidation>,
) {
    // Check for NaN every frame.
    if data.qpos.iter().any(|v| !v.is_finite()) {
        val.nan_detected = true;
    }

    // Detect reset transitions.
    let cycle = ResetState::cycle_index(data.time);
    if cycle != val.prev_cycle && data.time > 0.1 {
        val.total_resets += 1;

        // Right after reset, qpos/qvel should match the keyframe exactly.
        let t_since = data.time - state.last_reset;
        if t_since < 0.01 {
            let kf_idx = state.keyframe_ids[state.current_idx];
            let kf = &model.keyframes[kf_idx];
            if data.qpos[0] == kf.qpos[0] && data.qvel[0] == kf.qvel[0] {
                val.exact_resets += 1;
            }
        }
    }
    val.prev_cycle = cycle;

    if harness.reported() && !val.reported {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Keyframe resets are bitwise-exact",
                pass: val.exact_resets > 0 && val.exact_resets == val.total_resets,
                detail: format!("{}/{} resets exact", val.exact_resets, val.total_resets),
            },
            Check {
                name: "No NaN in simulation state",
                pass: !val.nan_detected,
                detail: format!("nan_detected = {}", val.nan_detected),
            },
        ];
        let _ = print_report("Save-Restore (t=15s)", &checks);
    }
}
