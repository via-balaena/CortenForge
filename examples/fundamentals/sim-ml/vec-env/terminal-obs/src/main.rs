//! Terminal Observations — Done vs Truncated
//!
//! Four pendulums driven by different constant torques. Done fires at
//! |qpos| > 2.0; truncated fires at episode time > 2.0. When an env
//! auto-resets, the HUD highlights terminal obs vs post-reset obs for
//! 1 second, showing the "Bootstrap?" distinction:
//!
//! - **Done** → task completed → do NOT bootstrap value → Bootstrap? No
//! - **Truncated** → time limit → bootstrap value from terminal state → Bootstrap? Yes
//!
//! Run: `cargo run -p example-ml-vec-env-terminal-obs --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp,
    clippy::struct_excessive_bools
)]

use std::sync::Arc;

use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::convert::physics_pos;
use sim_bevy::examples::{
    HudPosition, PhysicsHud, ValidationHarness, insert_batch_validation_dummies,
    render_physics_hud, spawn_example_camera, spawn_physics_hud_at, validation_system,
};
use sim_bevy::materials::MetalPreset;
use sim_bevy::multi_scene::{
    PhysicsScenes, spawn_scene_geoms, sync_batch_geoms, sync_scene_geom_transforms,
};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{ActionSpace, ObservationSpace, Tensor, VecEnv};

// ── Config ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 4;
const SPACING: f32 = 1.8;
const REPORT_TIME: f64 = 15.0;
const DONE_THRESHOLD: f64 = 2.0;
const TRUNCATE_TIME: f64 = 2.0;
/// Per-env constant actions: chosen so some envs hit done quickly,
/// others timeout. Strong torque → done; weak torque → truncated.
const ACTIONS: [f32; NUM_ENVS] = [0.8, 0.3, -0.2, -0.7];

// ── MJCF ─────────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="terminal-obs">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <geom name="bracket" type="box" size="0.04 0.03 0.02"
          pos="0 0 0" rgba="0.35 0.33 0.32 1"/>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.2"/>
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
    <motor name="torque" joint="hinge" gear="10"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

// ── Per-env reset event tracking ─────────────────────────────────────────

/// Last reset info for each env — always visible in HUD.
#[derive(Clone)]
struct LastReset {
    was_done: bool,
    terminal_qpos: f32,
    initial_qpos: f32,
}

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct TermObsResource {
    vec_env: VecEnv,
    actions: Tensor,
    accumulator: f64,
    sim_time: f64,
    episode_time: Vec<f64>,

    // Per-env last reset (always shown in HUD)
    last_resets: Vec<Option<LastReset>>,
    total_dones: usize,
    total_truncateds: usize,

    // Validation tracking
    done_terminal_beyond_threshold: bool,
    truncated_terminal_populated: bool,
    non_reset_terminal_is_none: bool,
    done_precedence_ok: bool,
    post_reset_continues: bool,
    steps_after_first_reset: usize,
}

#[derive(Resource, Default)]
struct TermObsValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Terminal Observations — Done vs Truncated ===");
    println!("  4 pendulums, actions: {ACTIONS:?}");
    println!("  done: |qpos| > {DONE_THRESHOLD}  truncated: t > {TRUNCATE_TIME}s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Terminal Observations".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<TermObsValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(2.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_env)
        .add_systems(
            PostUpdate,
            (
                sync_env_to_scenes,
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
    let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF parse"));

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    let done_threshold = DONE_THRESHOLD;
    let truncate_time = TRUNCATE_TIME;

    let mut vec_env = VecEnv::builder(model.clone(), NUM_ENVS)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(move |_m, d| d.qpos[0].abs() > done_threshold)
        .truncated(move |_m, d| d.time > truncate_time)
        .build()
        .expect("vec_env build");

    vec_env.reset_all().expect("reset");

    let actions = Tensor::from_slice(&ACTIONS, &[NUM_ENVS, 1]);

    // PhysicsScenes for rendering
    let mut scenes = PhysicsScenes::default();

    let env_colors = [
        Color::srgb(0.2, 0.5, 0.9), // blue
        Color::srgb(0.3, 0.8, 0.4), // green
        Color::srgb(0.9, 0.6, 0.1), // orange
        Color::srgb(0.8, 0.2, 0.3), // red
    ];

    for i in 0..NUM_ENVS {
        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("env{i}"), scene_model, scene_data);

        let mat_bracket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_socket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(env_colors[i]));

        let lane = (i as f32 - (NUM_ENVS as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            physics_pos(lane, 0.0, 0.0),
            &[
                ("bracket", mat_bracket),
                ("socket", mat_socket),
                ("rod", mat_rod),
                ("tip", mat_tip),
            ],
        );
    }

    commands.insert_resource(scenes);

    commands.insert_resource(TermObsResource {
        vec_env,
        actions,
        accumulator: 0.0,
        sim_time: 0.0,
        episode_time: vec![0.0; NUM_ENVS],
        last_resets: vec![None; NUM_ENVS],
        total_dones: 0,
        total_truncateds: 0,
        done_terminal_beyond_threshold: false,
        truncated_terminal_populated: false,
        non_reset_terminal_is_none: true,
        done_precedence_ok: true,
        post_reset_continues: false,
        steps_after_first_reset: 0,
    });

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, -0.5, 0.0),
        8.0,
        std::f32::consts::FRAC_PI_2,
        0.25,
    );

    spawn_physics_hud_at(&mut commands, HudPosition::BottomLeft);
    insert_batch_validation_dummies(&mut commands, &model);
}

// ── Stepping ─────────────────────────────────────────────────────────────

fn step_env(mut res: ResMut<TermObsResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;
    let dt_sim = res.vec_env.model().timestep;

    #[allow(clippy::cast_sign_loss)]
    let budget = ((res.accumulator / dt_sim).max(0.0) as u32).min(200);
    let actions = res.actions.clone();

    for _ in 0..budget {
        let result = res.vec_env.step(&actions).expect("vec step");

        for i in 0..NUM_ENVS {
            res.episode_time[i] += dt_sim;
            let was_done = result.dones[i];
            let was_truncated = result.truncateds[i];

            if was_done || was_truncated {
                // Extract terminal obs
                let terminal_qpos = result.terminal_observations[i]
                    .as_ref()
                    .map_or(f32::NAN, |t| t.as_slice()[0]);
                let initial_qpos = result.observations.row(i)[0];

                res.last_resets[i] = Some(LastReset {
                    was_done,
                    terminal_qpos,
                    initial_qpos,
                });

                if was_done {
                    res.total_dones += 1;
                    // Validate: terminal qpos should be beyond threshold
                    if terminal_qpos.abs() > DONE_THRESHOLD as f32 {
                        res.done_terminal_beyond_threshold = true;
                    }
                    // Validate: done takes precedence (shouldn't also be truncated)
                    if was_truncated {
                        res.done_precedence_ok = false;
                    }
                }

                if was_truncated {
                    res.total_truncateds += 1;
                    // Validate: terminal obs populated on truncated too
                    if result.terminal_observations[i].is_some() {
                        res.truncated_terminal_populated = true;
                    }
                }

                // Reset episode timer
                res.episode_time[i] = 0.0;
            } else {
                // Validate: non-reset env should have None terminal obs
                if result.terminal_observations[i].is_some() {
                    res.non_reset_terminal_is_none = false;
                }
            }
        }

        // Track post-reset stepping
        if res.total_dones + res.total_truncateds > 0 {
            res.steps_after_first_reset += 1;
            if res.steps_after_first_reset > 10 {
                res.post_reset_continues = true;
            }
        }

        res.accumulator -= dt_sim;
        res.sim_time += dt_sim;
    }
}

// ── Sync ─────────────────────────────────────────────────────────────────

fn sync_env_to_scenes(res: Res<TermObsResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<TermObsResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ───────────────────────────────────────────────────────────

fn track_validation(
    res: Res<TermObsResource>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<TermObsValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let checks = [
        Check {
            name: "Done: terminal_obs qpos beyond threshold",
            pass: res.done_terminal_beyond_threshold,
            detail: format!(
                "seen={} (threshold={DONE_THRESHOLD})",
                res.done_terminal_beyond_threshold
            ),
        },
        Check {
            name: "Truncated: terminal_obs populated",
            pass: res.truncated_terminal_populated,
            detail: format!("seen={}", res.truncated_terminal_populated),
        },
        Check {
            name: "Non-reset envs: terminal_obs == None",
            pass: res.non_reset_terminal_is_none,
            detail: format!("all_none={}", res.non_reset_terminal_is_none),
        },
        Check {
            name: "Done precedence over truncated",
            pass: res.done_precedence_ok,
            detail: format!(
                "ok={} (dones={}, truncateds={})",
                res.done_precedence_ok, res.total_dones, res.total_truncateds
            ),
        },
        Check {
            name: "Post-reset env continues stepping",
            pass: res.post_reset_continues,
            detail: format!("steps_after_first_reset={}", res.steps_after_first_reset),
        },
    ];

    let _ = print_report("Terminal Observations", &checks);
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<TermObsResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Terminal Observations");
    hud.scalar("time", res.sim_time, 1);
    hud.raw(format!(
        "resets: {} done + {} truncated",
        res.total_dones, res.total_truncateds
    ));

    // Per-env status
    hud.section("Per-Env");
    for i in 0..NUM_ENVS {
        hud.raw(format!(
            "{i}: act={:+.1}  ep_t={:.1}s",
            ACTIONS[i], res.episode_time[i]
        ));
    }

    // Last reset per env (always visible once first reset happens)
    if res.last_resets.iter().any(Option::is_some) {
        hud.section("Last Reset Per Env");
        hud.raw("env  terminal  initial  type       Bootstrap?".into());
        hud.raw("───  ────────  ───────  ─────────  ──────────".into());
        for i in 0..NUM_ENVS {
            if let Some(ref lr) = res.last_resets[i] {
                let reset_type = if lr.was_done {
                    "done     "
                } else {
                    "truncated"
                };
                let bootstrap = if lr.was_done { "No " } else { "Yes" };
                hud.raw(format!(
                    " {i}   {:+6.3}   {:+6.3}  {reset_type}  {bootstrap}",
                    lr.terminal_qpos, lr.initial_qpos,
                ));
            }
        }
    }
}
