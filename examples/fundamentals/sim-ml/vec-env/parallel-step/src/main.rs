//! Parallel Stepping — 8 Pendulums via VecEnv
//!
//! Eight pendulums side by side, each driven by a different constant action
//! linearly spaced from -1.0 to +1.0. Blue-to-red color gradient matches
//! the action value. All share one Model and step in parallel via
//! `VecEnv::step()`.
//!
//! Run: `cargo run -p example-ml-vec-env-parallel-step --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp
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
use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv};

// ── Config ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 8;
const SPACING: f32 = 1.5;
const REPORT_TIME: f64 = 10.0;

// ── MJCF ─────────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="parallel-step">
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
    <motor name="torque" joint="hinge" gear="1"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

/// Per-env constant actions: linearly spaced from -1.0 to +1.0.
fn action_for(i: usize) -> f32 {
    let t = i as f32 / (NUM_ENVS - 1) as f32;
    t.mul_add(2.0, -1.0)
}

/// Blue-to-red gradient: action=-1 → blue, action=+1 → red.
fn tip_color(i: usize) -> Color {
    let t = i as f32 / (NUM_ENVS - 1) as f32;
    Color::srgb(t, 0.15, 1.0 - t)
}

// ── Resources ────────────────────────────────────────────────────────────

#[derive(Resource)]
struct VecEnvResource {
    vec_env: VecEnv,
    actions: Tensor,
    last_result: Option<sim_ml_bridge::VecStepResult>,
    accumulator: f64,
    sim_time: f64,
    /// Sequential SimEnvs for bit-exact parity check.
    sim_envs: Vec<SimEnv>,
    parity_ok: bool,
    parity_checked: bool,
}

#[derive(Resource, Default)]
struct ParallelValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Parallel Stepping (VecEnv) ===");
    println!("  8 pendulums, actions from -1.0 to +1.0");
    println!("  Blue (act=-1) to Red (act=+1)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Parallel Stepping (VecEnv)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<ParallelValidation>()
        .insert_resource(
            ValidationHarness::new()
                .report_at(REPORT_TIME)
                .print_every(2.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_vec_env)
        .add_systems(
            PostUpdate,
            (
                sync_vec_env_to_scenes,
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

    // ── Build VecEnv ──
    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act build");

    let mut vec_env = VecEnv::builder(model.clone(), NUM_ENVS)
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(|_m, _d| false) // never terminates
        .truncated(|_m, _d| false)
        .build()
        .expect("vec_env build");

    vec_env.reset_all().expect("reset");

    // ── Build sequential SimEnvs for parity check ──
    let sim_envs: Vec<SimEnv> = (0..NUM_ENVS)
        .map(|_| {
            let obs_s = ObservationSpace::builder()
                .all_qpos()
                .all_qvel()
                .build(&model)
                .expect("obs");
            let act_s = ActionSpace::builder()
                .all_ctrl()
                .build(&model)
                .expect("act");
            let mut env = SimEnv::builder(model.clone())
                .observation_space(obs_s)
                .action_space(act_s)
                .reward(|_m, d| -d.qpos[0].powi(2))
                .done(|_m, _d| false)
                .truncated(|_m, _d| false)
                .build()
                .expect("sim_env");
            env.reset().expect("reset");
            env
        })
        .collect();

    // ── Build action tensor ──
    let action_vals: Vec<f32> = (0..NUM_ENVS).map(action_for).collect();
    let actions = Tensor::from_slice(&action_vals, &[NUM_ENVS, 1]);

    // ── PhysicsScenes for visual rendering ──
    let mut scenes = PhysicsScenes::default();

    for i in 0..NUM_ENVS {
        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("act={:.2}", action_for(i)), scene_model, scene_data);

        let mat_bracket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_socket =
            materials.add(MetalPreset::CastIron.with_color(Color::srgb(0.35, 0.33, 0.32)));
        let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
        let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(tip_color(i)));

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

    commands.insert_resource(VecEnvResource {
        vec_env,
        actions,
        last_result: None,
        accumulator: 0.0,
        sim_time: 0.0,
        sim_envs,
        parity_ok: true,
        parity_checked: false,
    });

    // Camera: side view of the 8-pendulum row.
    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, -0.5, 0.0),
        11.0,
        std::f32::consts::FRAC_PI_2,
        0.25,
    );

    spawn_physics_hud_at(&mut commands, HudPosition::BottomLeft);

    insert_batch_validation_dummies(&mut commands, &model);
}

// ── Stepping ─────────────────────────────────────────────────────────────

fn step_vec_env(mut res: ResMut<VecEnvResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;

    let dt_sim = res.vec_env.model().timestep;

    #[allow(clippy::cast_sign_loss)]
    let budget = ((res.accumulator / dt_sim).max(0.0) as u32).min(200);

    // Clone actions out to avoid borrowing res immutably and mutably.
    let actions = res.actions.clone();

    for _ in 0..budget {
        let result = res.vec_env.step(&actions).expect("vec step");

        // ── Parity check against sequential SimEnvs (first 10 steps only) ──
        if !res.parity_checked {
            let action_vals: Vec<f32> = (0..NUM_ENVS).map(action_for).collect();
            let mut parity_ok = true;
            for (i, env) in res.sim_envs.iter_mut().enumerate() {
                let a = Tensor::from_slice(&[action_vals[i]], &[1]);
                let sr = env.step(&a).expect("sim step");

                if result.observations.row(i) != sr.observation.as_slice()
                    || result.rewards[i] != sr.reward
                {
                    parity_ok = false;
                }
            }
            if !parity_ok {
                res.parity_ok = false;
            }
        }

        res.last_result = Some(result);
        res.accumulator -= dt_sim;
        res.sim_time += dt_sim;
    }

    // Mark parity check done after ~10 steps (0.02s)
    if !res.parity_checked && res.sim_time > 0.02 {
        res.parity_checked = true;
    }
}

// ── Keep dummy PhysicsData time in sync for ValidationHarness ────────────

fn sync_dummy_time(res: Res<VecEnvResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Sync VecEnv batch → PhysicsScenes for rendering ─────────────────────

fn sync_vec_env_to_scenes(res: Res<VecEnvResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

// ── Validation ───────────────────────────────────────────────────────────

fn track_validation(
    res: Res<VecEnvResource>,
    harness: Res<ValidationHarness>,
    mut val: ResMut<ParallelValidation>,
) {
    if !harness.reported() || val.reported {
        return;
    }
    val.reported = true;

    let last = res.last_result.as_ref().expect("should have stepped");

    // Check 1: observations shape == [8, 2]
    let obs_shape_ok = last.observations.shape() == [NUM_ENVS, 2];
    let check1 = Check {
        name: "Observations shape",
        pass: obs_shape_ok,
        detail: format!(
            "shape={:?}, expected=[{NUM_ENVS}, 2]",
            last.observations.shape()
        ),
    };

    // Check 2: Per-env observations differ (different actions → different states)
    let mut all_differ = true;
    for i in 0..NUM_ENVS {
        for j in (i + 1)..NUM_ENVS {
            if last.observations.row(i) == last.observations.row(j) {
                all_differ = false;
            }
        }
    }
    let check2 = Check {
        name: "Per-env observations differ",
        pass: all_differ,
        detail: format!("all_distinct={all_differ}"),
    };

    // Check 3: rewards.len() == 8
    let rew_ok = last.rewards.len() == NUM_ENVS;
    let check3 = Check {
        name: "Rewards length",
        pass: rew_ok,
        detail: format!("len={}, expected={NUM_ENVS}", last.rewards.len()),
    };

    // Check 4: Bit-exact match vs sequential SimEnv
    let check4 = Check {
        name: "Bit-exact vs SimEnv",
        pass: res.parity_ok,
        detail: format!(
            "parity_ok={} (checked first ~10 physics steps)",
            res.parity_ok
        ),
    };

    let _ = print_report(
        &format!("Parallel Stepping (t={REPORT_TIME}s)"),
        &[check1, check2, check3, check4],
    );
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<VecEnvResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("VecEnv Parallel Step");
    hud.scalar("time", res.sim_time, 1);

    if let Some(ref result) = res.last_result {
        hud.raw(format!(
            "obs: {:?}  rew: [{}]",
            result.observations.shape(),
            NUM_ENVS,
        ));

        hud.section("Per-Env");
        for i in 0..NUM_ENVS {
            let act = action_for(i);
            let qpos = result.observations.row(i)[0];
            let reward = result.rewards[i];
            hud.raw(format!(
                "{i:>2} act={act:+.2}  qpos={qpos:+.3}  rew={reward:.4}",
            ));
        }
    }
}
