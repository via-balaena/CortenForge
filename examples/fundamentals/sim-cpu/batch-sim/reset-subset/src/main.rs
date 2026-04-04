//! Reset Subset — Inverted Pendulum Survival
//!
//! Eight inverted pendulums, each starting with a different tilt from
//! vertical. A PD controller tries to balance them upright. Pendulums
//! that fall past 45 deg from vertical get reset via
//! `BatchSim::reset_where(mask)` and try again from their assigned tilt.
//!
//! - Small initial tilt: controller catches it, balances → survives
//! - Large initial tilt: falls too fast, controller can't save it → reset
//!
//! Run: `cargo run -p example-batch-sim-reset-subset --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::similar_names,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp
)]

use std::sync::Arc;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{
    PhysicsHud, ValidationHarness, render_physics_hud, spawn_example_camera, spawn_physics_hud,
    validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{PhysicsScenes, spawn_scene_geoms, sync_scene_geom_transforms};
use sim_core::batch::BatchSim;
use sim_core::validation::{Check, print_report};

// ── Config ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 8;
const SPACING: f32 = 1.5;
const REPORT_TIME: f64 = 10.0;

/// Check for reset every physics step (continuous monitoring).
const RESET_INTERVAL: f64 = 0.01;

/// Target: fully upright (pi rad from hanging).
const TARGET: f64 = std::f64::consts::PI;

/// Fall threshold: more than 45 deg from vertical → fallen → reset.
const FALL_ANGLE: f64 = 45.0 * std::f64::consts::PI / 180.0;

/// PD controller gains (same for all envs).
const KP: f64 = 40.0;
const KD: f64 = 8.0;

/// Per-env initial tilt from vertical (degrees). Small → easy, large → hard.
/// Past ~50 deg, gravity overwhelms the controller and the pendulum falls.
const INIT_TILTS_DEG: [f64; NUM_ENVS] = [1.0, 5.0, 15.0, 30.0, 50.0, 75.0, 110.0, 150.0];

fn init_tilt_rad(i: usize) -> f64 {
    INIT_TILTS_DEG[i] * std::f64::consts::PI / 180.0
}

// ── MJCF ─────────────────────────────────────────────────────────────────

/// Single-link pendulum with motor actuator for balance control.
/// No joint damping — all damping comes from the PD controller.
const MJCF: &str = r#"
<mujoco model="balance">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable" contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <geom name="bracket" type="box" size="0.04 0.03 0.02"
          pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0"/>
      <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="socket" type="sphere" size="0.03"
            pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
      <geom name="rod" type="capsule" size="0.02"
            fromto="0 0 0  0 0 -1.0" rgba="0.50 0.50 0.53 1"/>
      <geom name="tip" type="sphere" size="0.05"
            pos="0 0 -1.0"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="torque" joint="hinge" gear="1"/>
  </actuator>
</mujoco>
"#;

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct BatchResource {
    batch: BatchSim,
    model: Arc<sim_core::Model>,
    accumulator: f64,
    sim_time: f64,
    last_reset_time: f64,
    reset_counts: [u32; NUM_ENVS],
    /// Whether each env is currently balanced (survived).
    balanced: [bool; NUM_ENVS],
}

#[derive(Resource, Default)]
struct ResetValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Inverted Pendulum Survival ===");
    println!("  8 inverted pendulums, different initial tilts");
    println!("  PD controller tries to balance. Fall over → reset.");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Inverted Pendulum Survival (reset_where)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<ResetValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(2.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_batch)
        .add_systems(
            PostUpdate,
            (
                sync_batch_to_scenes,
                sync_scene_geom_transforms,
                sync_dummy_time,
                validation_system,
                track_validation,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup ────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF should parse"));

    let mut batch = BatchSim::new(Arc::clone(&model), NUM_ENVS);

    // Each env starts nearly upright with a different tilt
    for (i, env) in batch.envs_mut().enumerate() {
        env.qpos[0] = TARGET - init_tilt_rad(i); // pi - tilt
        env.forward(&model).expect("forward");
    }

    // PhysicsScenes for visual rendering
    let mut scenes = PhysicsScenes::default();

    // Color: green (easy, small tilt) → red (hard, large tilt)
    for i in 0..NUM_ENVS {
        let t = i as f32 / (NUM_ENVS - 1) as f32;
        let tip_color = Color::srgb(
            0.15 + 0.70 * t, // R: 0.15 → 0.85
            0.75 - 0.55 * t, // G: 0.75 → 0.20
            0.15,            // B: constant
        );

        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.qpos[0] = TARGET - init_tilt_rad(i);
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(
            format!("{:.0}°", INIT_TILTS_DEG[i]),
            scene_model,
            scene_data,
        );

        let mat_bracket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_socket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(tip_color));

        let x = (i as f32 - (NUM_ENVS as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            Vec3::new(x, 0.0, 0.0),
            &[
                ("bracket", mat_bracket),
                ("socket", mat_socket),
                ("rod", mat_rod),
                ("tip", mat_tip),
            ],
        );
    }

    commands.insert_resource(scenes);

    commands.insert_resource(BatchResource {
        batch,
        model: Arc::clone(&model),
        accumulator: 0.0,
        sim_time: 0.0,
        last_reset_time: 0.0,
        reset_counts: [0; NUM_ENVS],
        balanced: [false; NUM_ENVS],
    });

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.5),
        11.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud(&mut commands);

    // Dummy PhysicsModel/PhysicsData for ValidationHarness
    let dummy_model = (*model).clone();
    let dummy_data = dummy_model.make_data();
    commands.insert_resource(sim_bevy::model_data::PhysicsModel(dummy_model));
    commands.insert_resource(sim_bevy::model_data::PhysicsData(dummy_data));
}

// ── Stepping + Reset ─────────────────────────────────────────────────────

fn step_batch(mut res: ResMut<BatchResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;

    let dt_sim = res.batch.model().timestep;
    let mut steps = 0;

    while res.accumulator >= dt_sim && steps < 200 {
        // PD control: drive toward upright (TARGET = pi)
        for env in res.batch.envs_mut() {
            let error = TARGET - env.qpos[0];
            env.ctrl[0] = KP * error - KD * env.qvel[0];
        }

        let _errors = res.batch.step_all();
        res.accumulator -= dt_sim;
        res.sim_time += dt_sim;
        steps += 1;

        // Reset check
        if res.sim_time - res.last_reset_time >= RESET_INTERVAL {
            res.last_reset_time = res.sim_time;

            let mut mask = [false; NUM_ENVS];
            for i in 0..NUM_ENVS {
                if let Some(env) = res.batch.env(i) {
                    let tilt_from_vertical = (TARGET - env.qpos[0]).abs();

                    // Mark as balanced if within 2 deg of vertical
                    if tilt_from_vertical < 2.0 * std::f64::consts::PI / 180.0
                        && env.qvel[0].abs() < 0.5
                    {
                        res.balanced[i] = true;
                    }

                    // Reset if fallen: more than FALL_ANGLE from vertical
                    // (but don't reset already-balanced envs — they might
                    // oscillate briefly through the threshold)
                    if tilt_from_vertical > FALL_ANGLE && !res.balanced[i] {
                        mask[i] = true;
                    }
                }
            }

            if mask.iter().any(|&m| m) {
                res.batch.reset_where(&mask);

                // Restore each reset env to its assigned initial tilt
                for i in 0..NUM_ENVS {
                    if mask[i] {
                        res.reset_counts[i] += 1;
                        if let Some(env) = res.batch.env_mut(i) {
                            env.qpos[0] = TARGET - init_tilt_rad(i);
                            env.qvel[0] = 0.0;
                        }
                    }
                }
            }
        }
    }

    // Post-step forward to refresh geom poses
    if steps > 0 {
        let model = Arc::clone(&res.model);
        for env in res.batch.envs_mut() {
            let _ = env.forward(&model);
        }
    }
}

// ── Keep dummy PhysicsData time in sync for ValidationHarness ────────────

fn sync_dummy_time(res: Res<BatchResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Sync batch geom poses → PhysicsScenes for rendering ─────────────────

fn sync_batch_to_scenes(res: Res<BatchResource>, mut scenes: ResMut<PhysicsScenes>) {
    for i in 0..NUM_ENVS {
        if let (Some(env), Some(scene)) = (res.batch.env(i), scenes.get_mut(i)) {
            for g in 0..env.geom_xpos.len() {
                if g < scene.data.geom_xpos.len() {
                    scene.data.geom_xpos[g] = env.geom_xpos[g];
                    scene.data.geom_xmat[g] = env.geom_xmat[g];
                }
            }
        }
    }
}

// ── Validation ───────────────────────────────────────────────────────────

fn track_validation(
    res: Res<BatchResource>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ResetValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    // Check 1: Easy envs balanced (small tilt)
    let easy_balanced = (0..3).all(|i| res.balanced[i]);
    let check1 = Check {
        name: "Easy envs balanced",
        pass: easy_balanced,
        detail: format!(
            "balanced[0..3] = [{}, {}, {}] (tilts: {}°, {}°, {}°)",
            res.balanced[0],
            res.balanced[1],
            res.balanced[2],
            INIT_TILTS_DEG[0],
            INIT_TILTS_DEG[1],
            INIT_TILTS_DEG[2],
        ),
    };

    // Check 2: Hard envs fell and were reset
    let hard_reset = (6..NUM_ENVS).all(|i| res.reset_counts[i] > 0);
    let check2 = Check {
        name: "Hard envs fell and reset",
        pass: hard_reset,
        detail: format!(
            "reset_counts[6..8] = [{}, {}] (tilts: {}°, {}°)",
            res.reset_counts[6], res.reset_counts[7], INIT_TILTS_DEG[6], INIT_TILTS_DEG[7],
        ),
    };

    // Check 3: Balanced envs were never reset
    let balanced_never_reset = (0..NUM_ENVS)
        .filter(|&i| res.balanced[i])
        .all(|i| res.reset_counts[i] == 0);
    let check3 = Check {
        name: "Balanced envs never reset",
        pass: balanced_never_reset,
        detail: format!(
            "balanced={:?}, reset_counts={:?}",
            res.balanced, res.reset_counts,
        ),
    };

    // Check 4: reset_where left balanced envs untouched
    let balanced_continuous = (0..NUM_ENVS)
        .filter(|&i| res.balanced[i])
        .all(|i| res.batch.env(i).is_some_and(|e| e.time > REPORT_TIME - 0.5));
    let check4 = Check {
        name: "Balanced envs untouched",
        pass: balanced_continuous,
        detail: format!(
            "balanced env times: {:?}",
            (0..NUM_ENVS)
                .filter(|&i| res.balanced[i])
                .map(|i| format!("{}:{:.1}", i, res.batch.env(i).map_or(0.0, |e| e.time)))
                .collect::<Vec<_>>()
        ),
    };

    let _ = print_report(
        &format!("Inverted Pendulum Survival (t={REPORT_TIME}s)"),
        &[check1, check2, check3, check4],
    );
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<BatchResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Inverted Pendulum");
    hud.scalar("time", res.sim_time, 1);

    let total_resets: u32 = res.reset_counts.iter().sum();
    let num_balanced = res.balanced.iter().filter(|&&b| b).count();
    hud.raw(format!("balanced: {num_balanced}/{NUM_ENVS}"));
    hud.raw(format!("total resets: {total_resets}"));

    hud.section("Per-Env");
    for i in 0..NUM_ENVS {
        if let Some(env) = res.batch.env(i) {
            let tilt_deg = (TARGET - env.qpos[0]).to_degrees();
            let status = if res.balanced[i] {
                "BAL"
            } else if res.reset_counts[i] > 0 {
                "RST"
            } else {
                "..."
            };
            hud.raw(format!(
                "{:>2.0}° tilt  {:>+5.1}° now  {status}  #{}",
                INIT_TILTS_DEG[i], tilt_deg, res.reset_counts[i],
            ));
        }
    }
}
