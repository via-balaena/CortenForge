#![allow(
    missing_docs,
    clippy::doc_markdown,
    clippy::needless_pass_by_value,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops
)]

use std::sync::Arc;

use bevy::ecs::system::NonSendMut;
use bevy::prelude::*;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, render_physics_hud, spawn_example_camera, spawn_physics_hud};
use sim_bevy::materials::MetalPreset;
use sim_bevy::model_data::{PhysicsData, PhysicsModel, spawn_model_geoms, sync_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{ActionSpace, Environment, ObservationSpace, SimEnv, Tensor};

// ── Constants ──────────────────────────────────────────────────────────────

const DONE_THRESHOLD: f64 = 2.0;
const TIME_LIMIT: f64 = 5.0;
const REPORT_TIME: f64 = 15.0;

// ── Resources ──────────────────────────────────────────────────────────────

/// Wrapper for `SimEnv`.  Inserted as a **non-send** resource because
/// `SimEnv` contains `Box<dyn Fn(...)>` closures that are not `Send + Sync`.
struct SimEnvRes(SimEnv);

/// Wall-clock accumulator for fixed-timestep stepping.
#[derive(Resource, Default)]
struct StepAccumulator(f64);

#[derive(Resource)]
struct EpisodeState {
    episode: u32,
    step_count: u32,
    cumulative_reward: f64,
    last_reward: f64,
    last_done: bool,
    last_truncated: bool,
}

impl Default for EpisodeState {
    fn default() -> Self {
        Self {
            episode: 1,
            step_count: 0,
            cumulative_reward: 0.0,
            last_reward: 0.0,
            last_done: false,
            last_truncated: false,
        }
    }
}

#[derive(Resource)]
#[allow(clippy::struct_excessive_bools)]
struct EpisodeValidation {
    episodes_completed: u32,
    reset_obs_ok: bool,
    done_qpos_ok: bool,
    truncated_time_ok: bool,
    reward_ok: bool,
    saw_done: bool,
    saw_truncated: bool,
    last_print: f64,
    reported: bool,
}

impl Default for EpisodeValidation {
    fn default() -> Self {
        Self {
            episodes_completed: 0,
            reset_obs_ok: true,
            done_qpos_ok: true,
            truncated_time_ok: true,
            reward_ok: true,
            saw_done: false,
            saw_truncated: false,
            last_print: 0.0,
            reported: false,
        }
    }
}

// ── MJCF Model ─────────────────────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0" damping="0.02"/>
      <geom name="rod" type="capsule" size="0.05"
            fromto="0 0 0 0 0 -0.5" mass="1"
            rgba="0.48 0.48 0.50 1"/>
      <geom name="tip" type="sphere" size="0.07"
            pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
"#;

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Episode Lifecycle (Pattern B spike) ===");
    println!("  Done: |angle| > {DONE_THRESHOLD:.1} rad");
    println!("  Truncated: time > {TIME_LIMIT:.0} s");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    // ── Build SimEnv outside Bevy ──
    //
    // SimEnv contains Box<dyn Fn(...)> closures that are not Send+Sync,
    // so it must be a non-send resource.  We build it here and hand it
    // to the App via insert_non_send_resource().

    let model_arc = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF parse"));

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model_arc)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model_arc)
        .expect("act build");

    println!("  obs_dim={}, act_dim={}", obs.dim(), act.dim());

    let done_threshold = DONE_THRESHOLD;
    let mut env = SimEnv::builder(model_arc.clone())
        .observation_space(obs)
        .action_space(act)
        .reward(|_m, d| -d.qpos[0].powi(2))
        .done(move |_m, d| d.qpos[0].abs() > done_threshold)
        .truncated(move |_m, d| d.qpos[0].abs() <= done_threshold && d.time > TIME_LIMIT)
        .build()
        .expect("env build");

    env.reset().expect("reset");

    let model_owned = (*model_arc).clone();
    let data_owned = env.data().clone();

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Episode Lifecycle".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .insert_resource(PhysicsModel(model_owned))
        .insert_resource(PhysicsData(data_owned))
        .insert_non_send_resource(SimEnvRes(env))
        .init_resource::<StepAccumulator>()
        .init_resource::<PhysicsHud>()
        .init_resource::<EpisodeState>()
        .init_resource::<EpisodeValidation>()
        .add_systems(Startup, setup)
        .add_systems(Update, step_env)
        .add_systems(
            PostUpdate,
            (
                sync_geom_transforms,
                episode_diagnostics,
                update_hud,
                render_physics_hud,
            )
                .chain(),
        )
        .run();
}

// ── Setup (rendering only — SimEnv is already built) ──────────────────────

fn setup(
    mut commands: Commands,
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mat_rod = materials.add(MetalPreset::BrushedMetal.material());
    let mat_tip = materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.2, 0.5, 0.85)));
    spawn_model_geoms(
        &mut commands,
        &mut meshes,
        &mut materials,
        &model,
        &data,
        &[("rod", mat_rod), ("tip", mat_tip)],
    );

    spawn_example_camera(
        &mut commands,
        Vec3::new(0.0, 0.0, 0.75),
        2.0,
        std::f32::consts::FRAC_PI_2,
        0.0,
    );
    spawn_physics_hud(&mut commands);
}

// ── Stepping (Update) ─────────────────────────────────────────────────────

fn step_env(
    mut env: NonSendMut<SimEnvRes>,
    model: Res<PhysicsModel>,
    mut physics_data: ResMut<PhysicsData>,
    time: Res<Time>,
    mut acc: ResMut<StepAccumulator>,
    mut state: ResMut<EpisodeState>,
    mut val: ResMut<EpisodeValidation>,
) {
    acc.0 += time.delta_secs_f64();
    let dt = model.0.timestep;
    let mut steps = 0_u32;

    while acc.0 >= dt && steps < 200 {
        // Sinusoidal policy near pendulum resonance (ω₀ ≈ 5.4 rad/s).
        // Amplitude 4.0 exceeds the gravitational restoring torque (~2.4 Nm)
        // at all angles, ensuring the pendulum swings past the done threshold.
        let t = env.0.data().time;
        let action_val = (t * 5.0).sin() as f32 * 4.0;
        let action = Tensor::from_slice(&[action_val], &[1]);

        let result = env.0.step(&action).expect("step");
        steps += 1;
        state.step_count += 1;
        state.cumulative_reward += result.reward;
        state.last_reward = result.reward;
        state.last_done = result.done;
        state.last_truncated = result.truncated;

        // ── Validate reward against manual calculation ──
        let manual_reward = -(env.0.data().qpos[0].powi(2));
        if (result.reward - manual_reward).abs() > 1e-10 {
            val.reward_ok = false;
        }

        // ── Handle episode end ──
        if result.done || result.truncated {
            if result.done {
                val.saw_done = true;
                if env.0.data().qpos[0].abs() <= DONE_THRESHOLD {
                    val.done_qpos_ok = false;
                }
            }
            if result.truncated {
                val.saw_truncated = true;
                if env.0.data().time <= TIME_LIMIT {
                    val.truncated_time_ok = false;
                }
            }

            val.episodes_completed += 1;
            state.episode += 1;
            state.step_count = 0;
            state.cumulative_reward = 0.0;

            let obs = env.0.reset().expect("reset");
            // After reset, obs should be near [0, 0].
            if obs.as_slice().iter().any(|v| v.abs() > 1e-4) {
                val.reset_obs_ok = false;
            }
        }

        acc.0 -= dt;
    }

    // ── Pattern B: refresh kinematics, then copy to PhysicsData ──
    //
    // sim-core's step() integrates but does NOT recompute kinematics
    // (geom_xpos, geom_xmat, sensordata).  We call forward() once
    // after all sub-steps, then clone the full Data for rendering.
    if steps > 0 {
        env.0
            .data_mut()
            .forward(&model.0)
            .expect("post-step forward");
    }
    physics_data.0 = env.0.data().clone();
}

// ── HUD (PostUpdate) ──────────────────────────────────────────────────────

fn update_hud(
    model: Res<PhysicsModel>,
    data: Res<PhysicsData>,
    state: Res<EpisodeState>,
    mut hud: ResMut<PhysicsHud>,
) {
    hud.clear();

    hud.section("Episode");
    hud.raw(format!("episode:  {}", state.episode));
    hud.raw(format!("steps:    {}", state.step_count));
    hud.scalar("reward_sum", state.cumulative_reward, 2);

    hud.section("Last Step");
    hud.scalar("reward", state.last_reward, 4);
    hud.raw(format!("done:      {}", state.last_done));
    hud.raw(format!("truncated: {}", state.last_truncated));

    hud.section("State");
    let angle = data.sensor_scalar(&model, "angle").unwrap_or(0.0);
    let angvel = data.sensor_scalar(&model, "angvel").unwrap_or(0.0);
    hud.scalar("angle", angle, 4);
    hud.scalar("angvel", angvel, 4);
    hud.scalar("sim_time", data.time, 2);
}

// ── Validation (PostUpdate) ───────────────────────────────────────────────

fn episode_diagnostics(
    time: Res<Time>,
    state: Res<EpisodeState>,
    data: Res<PhysicsData>,
    model: Res<PhysicsModel>,
    mut val: ResMut<EpisodeValidation>,
) {
    let elapsed = time.elapsed_secs_f64();

    // Periodic console output (every 1 s wall clock).
    if elapsed - val.last_print >= 1.0 {
        val.last_print = elapsed;
        let angle = data.sensor_scalar(&model, "angle").unwrap_or(0.0);
        println!(
            "  t={elapsed:.1}  ep={}  steps={}  angle={angle:+.4}  reward={:.4}  eps_done={}",
            state.episode, state.step_count, state.last_reward, val.episodes_completed,
        );
    }

    // Report at REPORT_TIME wall-clock seconds.
    if !val.reported && elapsed > REPORT_TIME {
        val.reported = true;

        let checks = vec![
            Check {
                name: "Episodes completed",
                pass: val.episodes_completed >= 2,
                detail: format!("{} (>= 2)", val.episodes_completed),
            },
            Check {
                name: "Reset obs near initial",
                pass: val.reset_obs_ok,
                detail: format!("all_ok={}", val.reset_obs_ok),
            },
            Check {
                name: "Done fires at threshold",
                pass: val.saw_done && val.done_qpos_ok,
                detail: format!("saw_done={}, qpos_ok={}", val.saw_done, val.done_qpos_ok),
            },
            Check {
                name: "Truncated fires at time limit",
                pass: !val.saw_truncated || val.truncated_time_ok,
                detail: format!(
                    "saw_truncated={}, time_ok={}",
                    val.saw_truncated, val.truncated_time_ok
                ),
            },
            Check {
                name: "Reward matches manual",
                pass: val.reward_ok,
                detail: format!("all_ok={}", val.reward_ok),
            },
        ];
        let _ = print_report("Episode Lifecycle (Pattern B)", &checks);
    }
}
