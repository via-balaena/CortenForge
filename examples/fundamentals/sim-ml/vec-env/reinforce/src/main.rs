//! REINFORCE — Reaching Arm + Policy Gradient
//!
//! 50 two-link arms learn to reach a target using REINFORCE (vanilla policy
//! gradient). All envs share ONE policy — exploration comes from Gaussian
//! action noise. The visual story: all 50 arms converge to a smooth reach
//! in unison, unlike CEM where each env runs a different perturbation.
//!
//! Run: `cargo run -p example-ml-vec-env-reinforce --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::needless_range_loop,
    clippy::let_underscore_must_use,
    clippy::suboptimal_flops,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::unwrap_used,
    clippy::imprecise_flops,
    dead_code
)]

use std::sync::Arc;

use bevy::prelude::*;
use rand::Rng;
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

// ── Constants (inherited from CEM) ─────────────────────────────────────────

const NUM_ENVS: usize = 50;
const SPACING: f32 = 0.8;
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];
const REACH_THRESHOLD: f64 = 0.05;
const VEL_THRESHOLD: f64 = 0.5;
const EPISODE_TIMEOUT: f64 = 3.0;
const PAUSE_TIME: f64 = 1.5;
const NUM_PARAMS: usize = 10; // W[2x4] + b[2]

/// Target joint angles (IK solution for TARGET end-effector position).
const TARGET_QPOS: [f64; 2] = [-0.242, 1.982];

// ── REINFORCE-specific constants ───────────────────────────────────────────

const MAX_EPOCHS: usize = 30;
const VALIDATION_EPOCH: usize = 25;
const GAMMA: f64 = 0.99;
const LR: f64 = 0.05;
const SIGMA_INIT: f64 = 0.5;
const SIGMA_MIN: f64 = 0.05;
const SIGMA_DECAY: f64 = 0.90;
const MAX_GRAD_NORM: f64 = 1.0;
const ADAM_BETA1: f64 = 0.9;
const ADAM_BETA2: f64 = 0.999;
const ADAM_EPS: f64 = 1e-8;

// ── MJCF (identical to CEM) ───────────────────────────────────────────────

const MJCF: &str = r#"
<mujoco model="reaching-arm">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 -1 0"
             limited="true" range="-3.14159 3.14159" damping="2.0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0"
            size="0.03" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0"
               limited="true" range="-2.6 2.6" damping="1.0"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0"
              size="0.025" mass="0.3" rgba="0.85 0.4 0.2 1"/>
        <site name="fingertip" pos="0.4 0 0" size="0.015"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="10"
           ctrllimited="true" ctrlrange="-1 1"/>
    <motor name="elbow_motor" joint="elbow" gear="5"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

// ── Policy (same structure as CEM, now with gradient) ──────────────────────

/// Observation normalization scales (keeps pre-tanh values moderate).
const OBS_SCALE: [f64; 4] = [
    1.0 / std::f64::consts::PI,
    1.0 / std::f64::consts::PI,
    0.1,
    0.1,
];

/// Compute mean action: mu(s) = tanh(W * s_scaled + b)
fn policy_mean(params: &[f64], obs: &[f32]) -> [f64; 2] {
    let mut mu = [0.0f64; 2];
    for a in 0..2 {
        let mut z = params[8 + a]; // bias
        for o in 0..4 {
            z += params[a * 4 + o] * f64::from(obs[o]) * OBS_SCALE[o];
        }
        mu[a] = z.tanh();
    }
    mu
}

/// Sample stochastic action: a = mu(s) + sigma * eps
fn sample_action(params: &[f64], obs: &[f32], sigma: f64, rng: &mut impl Rng) -> [f64; 2] {
    let mu = policy_mean(params, obs);
    [mu[0] + sigma * randn(rng), mu[1] + sigma * randn(rng)]
}

/// Compute policy gradient for one (obs, action) pair.
/// Returns gradient w.r.t. all 10 params.
///
/// For Gaussian policy pi(a|s) = N(a; tanh(W*s+b), sigma^2*I):
///   d/dtheta log pi = (a - mu) / sigma^2 * d/dtheta mu
///   d/dW mu = (1 - mu^2) * s_scaled^T
///   d/db mu = (1 - mu^2)
fn policy_gradient(
    params: &[f64],
    obs: &[f32],
    action: &[f64; 2],
    sigma: f64,
) -> [f64; NUM_PARAMS] {
    let mu = policy_mean(params, obs);
    let sigma2 = sigma * sigma;
    let mut grad = [0.0f64; NUM_PARAMS];

    for a in 0..2 {
        let tanh_deriv = 1.0 - mu[a] * mu[a];
        let score = (action[a] - mu[a]) / sigma2 * tanh_deriv;

        // d/dW[a, o]
        for o in 0..4 {
            let s_scaled = f64::from(obs[o]) * OBS_SCALE[o];
            grad[a * 4 + o] = score * s_scaled;
        }
        // d/db[a]
        grad[8 + a] = score;
    }
    grad
}

// ── Helpers ────────────────────────────────────────────────────────────────

/// Box-Muller normal sample.
fn randn(rng: &mut impl Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

fn build_vec_env(model: &Arc<sim_core::Model>, n_envs: usize) -> VecEnv {
    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(model)
        .expect("obs build");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(model)
        .expect("act build");

    let target = TARGET;
    VecEnv::builder(model.clone(), n_envs)
        .observation_space(obs)
        .action_space(act)
        .reward(move |_m, d| {
            // Joint-space squared error — same as CEM.
            // Cartesian distance plateaus for both CEM and REINFORCE.
            let e0 = d.qpos[0] - TARGET_QPOS[0];
            let e1 = d.qpos[1] - TARGET_QPOS[1];
            -(e0 * e0 + e1 * e1)
        })
        .done(move |_m, d| {
            let tip = d.site_xpos[0];
            let dx = tip.x - target[0];
            let dz = tip.z - target[2];
            let dist = (dx * dx + dz * dz).sqrt();
            let vel = (d.qvel[0] * d.qvel[0] + d.qvel[1] * d.qvel[1]).sqrt();
            dist < REACH_THRESHOLD && vel < VEL_THRESHOLD
        })
        .truncated(|_m, d| d.time > EPISODE_TIMEOUT)
        .sub_steps(5)
        .build()
        .expect("vec_env build")
}

// ── Trajectory recording ───────────────────────────────────────────────────

struct Trajectory {
    obs: Vec<[f32; 4]>,
    actions: Vec<[f64; 2]>,
    rewards: Vec<f64>,
}

impl Trajectory {
    fn new() -> Self {
        Self {
            obs: Vec::with_capacity(350),
            actions: Vec::with_capacity(350),
            rewards: Vec::with_capacity(350),
        }
    }

    const fn len(&self) -> usize {
        self.rewards.len()
    }

    /// Discounted returns: R_t = sum_k gamma^k * r_{t+k}
    fn discounted_returns(&self, gamma: f64) -> Vec<f64> {
        let n = self.rewards.len();
        let mut returns = vec![0.0; n];
        if n == 0 {
            return returns;
        }
        returns[n - 1] = self.rewards[n - 1];
        for t in (0..n - 1).rev() {
            returns[t] = self.rewards[t] + gamma * returns[t + 1];
        }
        returns
    }
}

// ── REINFORCE update ───────────────────────────────────────────────────────

/// Adam optimizer state.
struct AdamState {
    m: [f64; NUM_PARAMS], // first moment
    v: [f64; NUM_PARAMS], // second moment
    t: usize,             // step count
}

impl AdamState {
    const fn new() -> Self {
        Self {
            m: [0.0; NUM_PARAMS],
            v: [0.0; NUM_PARAMS],
            t: 0,
        }
    }
}

struct ReinforceResult {
    grad_norm: f64,
}

fn reinforce_update(
    params: &mut [f64; NUM_PARAMS],
    adam: &mut AdamState,
    trajectories: &[Trajectory],
    sigma: f64,
    gamma: f64,
    lr: f64,
    max_grad_norm: f64,
) -> ReinforceResult {
    // Compute all discounted returns
    let all_returns: Vec<Vec<f64>> = trajectories
        .iter()
        .map(|t| t.discounted_returns(gamma))
        .collect();

    // Collect all advantages (return - baseline), then normalize
    let mut all_advantages: Vec<f64> = Vec::new();
    for returns in &all_returns {
        for &r in returns {
            all_advantages.push(r);
        }
    }

    // Baseline subtraction
    let mean_adv = all_advantages.iter().sum::<f64>() / all_advantages.len().max(1) as f64;
    for a in &mut all_advantages {
        *a -= mean_adv;
    }

    // Advantage normalization (critical for REINFORCE stability)
    let std_adv = (all_advantages.iter().map(|&a| a * a).sum::<f64>()
        / all_advantages.len().max(1) as f64)
        .sqrt();
    if std_adv > 1e-8 {
        for a in &mut all_advantages {
            *a /= std_adv;
        }
    }

    // Accumulate gradient using normalized advantages.
    let mut grad = [0.0f64; NUM_PARAMS];
    let mut n_samples = 0;
    let mut adv_idx = 0;

    for traj in trajectories {
        for t in 0..traj.len() {
            let advantage = all_advantages[adv_idx];
            adv_idx += 1;
            let g = policy_gradient(params, &traj.obs[t], &traj.actions[t], sigma);
            for k in 0..NUM_PARAMS {
                grad[k] += g[k] * advantage;
            }
            n_samples += 1;
        }
    }

    // Average over all samples
    if n_samples > 0 {
        for k in 0..NUM_PARAMS {
            grad[k] /= f64::from(n_samples);
        }
    }

    // Gradient clipping (before Adam)
    let grad_norm = grad.iter().map(|&g| g * g).sum::<f64>().sqrt();
    if grad_norm > max_grad_norm {
        let scale = max_grad_norm / grad_norm;
        for k in 0..NUM_PARAMS {
            grad[k] *= scale;
        }
    }

    // Adam update (gradient ascent: we ADD the update)
    adam.t += 1;
    #[allow(clippy::cast_possible_wrap)]
    let t_i32 = adam.t as i32;
    let bc1 = 1.0 - ADAM_BETA1.powi(t_i32);
    let bc2 = 1.0 - ADAM_BETA2.powi(t_i32);

    for k in 0..NUM_PARAMS {
        adam.m[k] = ADAM_BETA1 * adam.m[k] + (1.0 - ADAM_BETA1) * grad[k];
        adam.v[k] = ADAM_BETA2 * adam.v[k] + (1.0 - ADAM_BETA2) * grad[k] * grad[k];
        let m_hat = adam.m[k] / bc1;
        let v_hat = adam.v[k] / bc2;
        params[k] += lr * m_hat / (v_hat.sqrt() + ADAM_EPS);
    }

    ReinforceResult { grad_norm }
}

// ── Bevy Resources ─────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Running,
    Updating,
}

#[derive(Resource)]
struct ReinforceResource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,

    params: [f64; NUM_PARAMS],
    adam: AdamState,
    sigma: f64,

    epoch: usize,
    phase: Phase,
    env_complete: Vec<bool>,
    env_reached: Vec<bool>,
    trajectories: Vec<Trajectory>,

    accumulator: f64,
    sim_time: f64,
    pause_timer: f64,

    rng: rand::rngs::StdRng,

    // History (per epoch)
    mean_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
    grad_norms: Vec<f64>,
}

#[derive(Resource, Default)]
struct ReinforceValidation {
    reported: bool,
}

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: REINFORCE — Reaching Arm + Policy Gradient ===");
    println!("  {NUM_ENVS} arms, {MAX_EPOCHS} epochs, lr={LR}, gamma={GAMMA}");
    println!(
        "  Target: ({:.1}, {:.1}, {:.1})",
        TARGET[0], TARGET[1], TARGET[2]
    );
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — REINFORCE (Reaching Arm + Policy Gradient)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<ReinforceValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_reinforce)
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

// ── Setup ──────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF parse"));
    let mut vec_env = build_vec_env(&model, NUM_ENVS);
    let init_obs = vec_env.reset_all().expect("reset");

    let rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };

    // ── PhysicsScenes ──
    let mut scenes = PhysicsScenes::default();

    let mat_upper =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.3, 0.5, 0.85)));
    let mat_forearm =
        materials.add(MetalPreset::PolishedSteel.with_color(Color::srgb(0.85, 0.4, 0.2)));

    let mat_target = materials.add(StandardMaterial {
        base_color: Color::srgba(0.2, 0.9, 0.2, 0.5),
        alpha_mode: AlphaMode::Blend,
        unlit: true,
        ..default()
    });
    let target_mesh = meshes.add(Sphere::new(0.03));

    for i in 0..NUM_ENVS {
        let scene_model = (*model).clone();
        let mut scene_data = scene_model.make_data();
        scene_data.forward(&scene_model).expect("forward");

        let id = scenes.add(format!("env-{i}"), scene_model, scene_data);

        let lane = (i as f32 - (NUM_ENVS as f32 - 1.0) / 2.0) * SPACING;
        spawn_scene_geoms(
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut scenes,
            id,
            physics_pos(lane, 0.0, 0.0),
            &[
                ("upper_geom", mat_upper.clone()),
                ("forearm_geom", mat_forearm.clone()),
            ],
        );

        let target_bevy = physics_pos(lane + TARGET[0] as f32, 0.0, TARGET[2] as f32);
        commands.spawn((
            Mesh3d(target_mesh.clone()),
            MeshMaterial3d(mat_target.clone()),
            Transform::from_translation(target_bevy),
        ));
    }

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, 2]);
    let trajectories: Vec<Trajectory> = (0..NUM_ENVS).map(|_| Trajectory::new()).collect();

    commands.insert_resource(ReinforceResource {
        vec_env,
        actions,
        current_obs: init_obs,
        params: [0.0f64; NUM_PARAMS],
        adam: AdamState::new(),
        sigma: SIGMA_INIT,
        epoch: 0,
        phase: Phase::Running,
        env_complete: vec![false; NUM_ENVS],
        env_reached: vec![false; NUM_ENVS],
        trajectories,
        accumulator: 0.0,
        sim_time: 0.0,
        pause_timer: 0.0,
        rng,
        mean_rewards: Vec::new(),
        reach_counts: Vec::new(),
        grad_norms: Vec::new(),
    });

    spawn_example_camera(
        &mut commands,
        physics_pos(0.0, -0.5, 0.0),
        35.0,
        std::f32::consts::FRAC_PI_2,
        0.15,
    );

    spawn_physics_hud_at(&mut commands, HudPosition::BottomLeft);
    insert_batch_validation_dummies(&mut commands, &model);
}

// ── Stepping (REINFORCE state machine) ─────────────────────────────────────

fn step_reinforce(mut res: ResMut<ReinforceResource>, time: Res<Time>) {
    match res.phase {
        Phase::Running => {
            if res.epoch >= MAX_EPOCHS {
                return;
            }

            let wall_dt = time.delta_secs_f64();
            res.accumulator += wall_dt;
            let dt_action = 5.0 * res.vec_env.model().timestep;

            #[allow(clippy::cast_sign_loss)]
            let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

            for _ in 0..budget {
                // Sample actions from stochastic policy
                // Copy obs out to avoid borrow conflict with rng
                let obs_snapshot: Vec<[f32; 4]> = (0..NUM_ENVS)
                    .map(|i| {
                        let r = res.current_obs.row(i);
                        [r[0], r[1], r[2], r[3]]
                    })
                    .collect();

                let params = res.params;
                let sigma = res.sigma;
                for i in 0..NUM_ENVS {
                    if !res.env_complete[i] {
                        let obs_arr = obs_snapshot[i];
                        let action = sample_action(&params, &obs_arr, sigma, &mut res.rng);

                        res.trajectories[i].obs.push(obs_arr);
                        res.trajectories[i].actions.push(action);

                        let row = res.actions.row_mut(i);
                        row[0] = action[0] as f32;
                        row[1] = action[1] as f32;
                    }
                }

                let actions = res.actions.clone();
                let result = res.vec_env.step(&actions).expect("vec step");

                for i in 0..NUM_ENVS {
                    if !res.env_complete[i] {
                        res.trajectories[i].rewards.push(result.rewards[i]);
                        if result.dones[i] || result.truncateds[i] {
                            res.env_complete[i] = true;
                            if result.dones[i] {
                                res.env_reached[i] = true;
                            }
                        }
                    }
                }

                res.current_obs = result.observations;
                res.accumulator -= dt_action;
                res.sim_time += dt_action;

                if res.env_complete.iter().all(|&c| c) {
                    // All envs done — compute stats and transition
                    let total_reward: f64 = res
                        .trajectories
                        .iter()
                        .map(|t| t.rewards.iter().sum::<f64>())
                        .sum::<f64>();
                    let mean_reward = total_reward / NUM_ENVS as f64;
                    let reached = res.env_reached.iter().filter(|&&r| r).count();

                    res.mean_rewards.push(mean_reward);
                    res.reach_counts.push(reached);

                    println!(
                        "epoch {:>2}: reached={reached:>2}/{NUM_ENVS}  mean_R={mean_reward:>8.1}  sigma={:.3}",
                        res.epoch + 1,
                        res.sigma
                    );

                    res.phase = Phase::Updating;
                    res.pause_timer = PAUSE_TIME;
                    break;
                }
            }
        }
        Phase::Updating => {
            let wall_dt = time.delta_secs_f64();
            res.pause_timer -= wall_dt;

            if res.pause_timer <= 0.0 {
                // REINFORCE update
                let trajectories = std::mem::take(&mut res.trajectories);
                let sigma = res.sigma;
                let inner = &mut *res;
                let result = reinforce_update(
                    &mut inner.params,
                    &mut inner.adam,
                    &trajectories,
                    sigma,
                    GAMMA,
                    LR,
                    MAX_GRAD_NORM,
                );
                inner.grad_norms.push(result.grad_norm);

                // Anneal sigma
                res.sigma = (res.sigma * SIGMA_DECAY).max(SIGMA_MIN);

                // Next epoch
                res.epoch += 1;
                res.current_obs = res.vec_env.reset_all().expect("reset");
                res.env_complete.fill(false);
                res.env_reached.fill(false);
                res.trajectories = (0..NUM_ENVS).map(|_| Trajectory::new()).collect();
                res.phase = Phase::Running;
            }
        }
    }
}

// ── Sync ───────────────────────────────────────────────────────────────────

fn sync_vec_env_to_scenes(res: Res<ReinforceResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(
    res: Res<ReinforceResource>,
    mut data: ResMut<sim_bevy::model_data::PhysicsData>,
) {
    data.0.time = res.sim_time;
}

// ── Validation ─────────────────────────────────────────────────────────────

fn track_validation(res: Res<ReinforceResource>, mut val: ResMut<ReinforceValidation>) {
    if res.epoch < VALIDATION_EPOCH || val.reported {
        return;
    }
    val.reported = true;

    let ep_idx = VALIDATION_EPOCH - 1;
    let ep1_reward = res
        .mean_rewards
        .first()
        .copied()
        .unwrap_or(f64::NEG_INFINITY);
    let ep_reward = if ep_idx < res.mean_rewards.len() {
        res.mean_rewards[ep_idx]
    } else {
        f64::NEG_INFINITY
    };

    let improvement = if ep1_reward.abs() > 1e-10 {
        (ep_reward - ep1_reward) / ep1_reward.abs()
    } else {
        0.0
    };

    let checks = [
        Check {
            name: "Reward improvement >=80%",
            pass: improvement > 0.8,
            detail: format!(
                "ep1={ep1_reward:.1} -> ep{VALIDATION_EPOCH}={ep_reward:.1} ({:.0}%)",
                improvement * 100.0
            ),
        },
        Check {
            name: "Policy learned (||theta|| > 0.5)",
            pass: res.params.iter().map(|&v| v * v).sum::<f64>().sqrt() > 0.5,
            detail: format!(
                "||theta||={:.3}",
                res.params.iter().map(|&v| v * v).sum::<f64>().sqrt()
            ),
        },
        Check {
            name: "Sigma decreased",
            pass: res.sigma < SIGMA_INIT * 0.5,
            detail: format!("sigma={:.3}", res.sigma),
        },
    ];

    let _ = print_report(&format!("REINFORCE (epoch {VALIDATION_EPOCH})"), &checks);
}

// ── HUD ────────────────────────────────────────────────────────────────────

fn update_hud(res: Res<ReinforceResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Reaching Arm (VecEnv + REINFORCE)");
    hud.raw(format!("epoch: {} / {MAX_EPOCHS}", res.epoch + 1));
    hud.raw(format!(
        "phase: {:?}{}",
        res.phase,
        if res.phase == Phase::Updating {
            format!(" ({:.1}s)", res.pause_timer.max(0.0))
        } else {
            String::new()
        }
    ));

    let cur_reached = res.env_reached.iter().filter(|&&r| r).count();
    let cur_complete = res.env_complete.iter().filter(|&&c| c).count();
    hud.raw(format!(
        "reached: {cur_reached}/{NUM_ENVS}  done: {cur_complete}/{NUM_ENVS}"
    ));

    if let Some(&mean_r) = res.mean_rewards.last() {
        hud.raw(format!("mean reward: {mean_r:.1}"));
    }

    hud.raw(format!("sigma: {:.3}", res.sigma));

    if let Some(&gn) = res.grad_norms.last() {
        hud.raw(format!("|grad|: {gn:.4}"));
    }

    // Reward curve (compact)
    if res.mean_rewards.len() >= 2 {
        hud.section("Reward Curve");
        let rewards = &res.mean_rewards;
        let n = rewards.len();
        // Show first, middle, and last
        let mut entries = Vec::new();
        entries.push(format!("ep1:{:.0}", rewards[0]));
        if n > 5 {
            entries.push(format!("ep5:{:.0}", rewards[4]));
        }
        if n > 10 {
            entries.push(format!("ep10:{:.0}", rewards[9]));
        }
        if n > 15 {
            entries.push(format!("ep15:{:.0}", rewards[14]));
        }
        if n > 20 {
            entries.push(format!("ep20:{:.0}", rewards[19]));
        }
        entries.push(format!("ep{}:{:.0}", n, rewards[n - 1]));
        hud.raw(entries.join("  "));
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Headless REINFORCE runner ──

    struct ReinforceHeadlessResult {
        ep1_mean_reward: f64,
        last_mean_reward: f64,
        last_reached: usize,
        param_norm: f64,
        final_sigma: f64,
        per_epoch_reached: Vec<usize>,
    }

    fn run_reinforce_headless(seed: u64) -> ReinforceHeadlessResult {
        use rand::SeedableRng;

        let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF"));
        let mut vec_env = build_vec_env(&model, NUM_ENVS);
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut params = [0.0f64; NUM_PARAMS];
        let mut adam = AdamState::new();
        let mut sigma = SIGMA_INIT;

        let mut ep1_mean_reward = 0.0;
        let mut last_mean_reward = 0.0;
        let mut last_reached = 0;
        let mut per_epoch_reached = Vec::new();

        let max_steps = (EPISODE_TIMEOUT / (5.0 * 0.002)) as usize + 50;

        for epoch in 0..MAX_EPOCHS {
            let mut current_obs = vec_env.reset_all().expect("reset");
            let mut trajectories: Vec<Trajectory> =
                (0..NUM_ENVS).map(|_| Trajectory::new()).collect();
            let mut env_complete = [false; NUM_ENVS];
            let mut env_reached = [false; NUM_ENVS];
            let mut actions = Tensor::zeros(&[NUM_ENVS, 2]);

            for _ in 0..max_steps {
                for i in 0..NUM_ENVS {
                    if !env_complete[i] {
                        let obs = current_obs.row(i);
                        let obs_arr: [f32; 4] = [obs[0], obs[1], obs[2], obs[3]];
                        let action = sample_action(&params, obs, sigma, &mut rng);

                        trajectories[i].obs.push(obs_arr);
                        trajectories[i].actions.push(action);

                        let row = actions.row_mut(i);
                        row[0] = action[0] as f32;
                        row[1] = action[1] as f32;
                    }
                }

                let result = vec_env.step(&actions).expect("step");

                for i in 0..NUM_ENVS {
                    if !env_complete[i] {
                        trajectories[i].rewards.push(result.rewards[i]);
                        if result.dones[i] || result.truncateds[i] {
                            env_complete[i] = true;
                            if result.dones[i] {
                                env_reached[i] = true;
                            }
                        }
                    }
                }

                current_obs = result.observations;
                if env_complete.iter().all(|&c| c) {
                    break;
                }
            }

            let total_reward: f64 = trajectories
                .iter()
                .map(|t| t.rewards.iter().sum::<f64>())
                .sum::<f64>();
            let mean_reward = total_reward / NUM_ENVS as f64;
            let reached = env_reached.iter().filter(|&&r| r).count();

            if epoch == 0 {
                ep1_mean_reward = mean_reward;
            }
            last_mean_reward = mean_reward;
            last_reached = reached;
            per_epoch_reached.push(reached);

            reinforce_update(
                &mut params,
                &mut adam,
                &trajectories,
                sigma,
                GAMMA,
                LR,
                MAX_GRAD_NORM,
            );
            sigma = (sigma * SIGMA_DECAY).max(SIGMA_MIN);
        }

        let param_norm = params.iter().map(|&v| v * v).sum::<f64>().sqrt();

        ReinforceHeadlessResult {
            ep1_mean_reward,
            last_mean_reward,
            last_reached,
            param_norm,
            final_sigma: sigma,
            per_epoch_reached,
        }
    }

    // ── Assumption tests ──

    #[test]
    fn gradient_matches_finite_difference() {
        // Verify hand-coded gradient against numerical approximation
        let params = [0.1, -0.2, 0.3, 0.15, -0.1, 0.25, -0.05, 0.2, 0.1, -0.1];
        let obs: [f32; 4] = [0.5, -0.3, 0.8, -0.4];
        let action = [0.3, -0.2];
        let sigma = 0.3;

        let analytic = policy_gradient(&params, &obs, &action, sigma);

        let eps = 1e-5;
        for k in 0..NUM_PARAMS {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[k] += eps;
            p_minus[k] -= eps;

            // log pi(a|s) = -0.5 * sum_a (a - mu)^2 / sigma^2  (plus const)
            let mu_plus = policy_mean(&p_plus, &obs);
            let mu_minus = policy_mean(&p_minus, &obs);

            let log_pi_plus: f64 = (0..2)
                .map(|a| -0.5 * (action[a] - mu_plus[a]).powi(2) / (sigma * sigma))
                .sum();
            let log_pi_minus: f64 = (0..2)
                .map(|a| -0.5 * (action[a] - mu_minus[a]).powi(2) / (sigma * sigma))
                .sum();

            let numerical = (log_pi_plus - log_pi_minus) / (2.0 * eps);

            assert!(
                (analytic[k] - numerical).abs() < 1e-3,
                "param {k}: analytic={:.6}, numerical={:.6}, diff={:.6}",
                analytic[k],
                numerical,
                (analytic[k] - numerical).abs()
            );
        }
    }

    #[test]
    fn reinforce_converges_in_30_epochs() {
        let r = run_reinforce_headless(42);
        // REINFORCE achieves 90%+ reward improvement but fewer done triggers
        // than CEM — the gradient is too noisy for 5cm precision.
        // This is the honest result: vanilla PG gets close but can't finish.
        let improvement = (r.last_mean_reward - r.ep1_mean_reward) / r.ep1_mean_reward.abs();
        assert!(
            improvement > 0.8,
            "improvement={:.0}%, want >80%",
            improvement * 100.0
        );
        assert!(
            r.param_norm > 0.5,
            "param_norm={:.3}, want >0.5",
            r.param_norm
        );
        // Verify reward significantly improved (not still at starting level)
        assert!(
            r.last_mean_reward > r.ep1_mean_reward * 0.15,
            "last_mean_reward={:.1}, want better than {:.1}",
            r.last_mean_reward,
            r.ep1_mean_reward * 0.15
        );
    }

    #[test]
    fn all_envs_identical_on_mean_policy() {
        // With same deterministic policy (no noise), all envs should produce
        // identical trajectories.
        let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF"));
        let mut vec_env = build_vec_env(&model, NUM_ENVS);
        let mut current_obs = vec_env.reset_all().expect("reset");

        let params = [0.1, -0.2, 0.3, 0.15, -0.1, 0.25, -0.05, 0.2, 0.1, -0.1];
        let mut actions = Tensor::zeros(&[NUM_ENVS, 2]);

        for _ in 0..50 {
            // Deterministic policy (no noise)
            for i in 0..NUM_ENVS {
                let obs = current_obs.row(i);
                let mu = policy_mean(&params, obs);
                let row = actions.row_mut(i);
                row[0] = mu[0] as f32;
                row[1] = mu[1] as f32;
            }
            let result = vec_env.step(&actions).expect("step");
            current_obs = result.observations;
        }

        // All envs should have identical observations
        let ref_obs = current_obs.row(0);
        for i in 1..NUM_ENVS {
            let obs = current_obs.row(i);
            for j in 0..4 {
                assert!(
                    (obs[j] - ref_obs[j]).abs() < 1e-6,
                    "env {i} obs[{j}]={}, env 0 obs[{j}]={}",
                    obs[j],
                    ref_obs[j]
                );
            }
        }
    }

    #[test]
    fn sigma_schedule_reaches_minimum() {
        let mut sigma = SIGMA_INIT;
        for _ in 0..MAX_EPOCHS {
            sigma = (sigma * SIGMA_DECAY).max(SIGMA_MIN);
        }
        assert!(
            (sigma - SIGMA_MIN).abs() < 0.01,
            "final sigma={sigma:.4}, expected near {SIGMA_MIN}"
        );
    }

    #[test]
    fn baseline_reduces_gradient_variance() {
        use rand::SeedableRng;

        let model = Arc::new(sim_mjcf::load_model(MJCF).expect("MJCF"));
        let mut vec_env = build_vec_env(&model, NUM_ENVS);
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let params = [0.0f64; NUM_PARAMS];
        let sigma = SIGMA_INIT;

        // Collect one epoch of trajectories
        let mut current_obs = vec_env.reset_all().expect("reset");
        let mut trajectories: Vec<Trajectory> = (0..NUM_ENVS).map(|_| Trajectory::new()).collect();
        let mut env_complete = [false; NUM_ENVS];
        let mut actions = Tensor::zeros(&[NUM_ENVS, 2]);
        let max_steps = (EPISODE_TIMEOUT / (5.0 * 0.002)) as usize + 50;

        for _ in 0..max_steps {
            for i in 0..NUM_ENVS {
                if !env_complete[i] {
                    let obs = current_obs.row(i);
                    let obs_arr: [f32; 4] = [obs[0], obs[1], obs[2], obs[3]];
                    let action = sample_action(&params, obs, sigma, &mut rng);
                    trajectories[i].obs.push(obs_arr);
                    trajectories[i].actions.push(action);
                    let row = actions.row_mut(i);
                    row[0] = action[0] as f32;
                    row[1] = action[1] as f32;
                }
            }
            let result = vec_env.step(&actions).expect("step");
            for i in 0..NUM_ENVS {
                if !env_complete[i] {
                    trajectories[i].rewards.push(result.rewards[i]);
                    if result.dones[i] || result.truncateds[i] {
                        env_complete[i] = true;
                    }
                }
            }
            current_obs = result.observations;
            if env_complete.iter().all(|&c| c) {
                break;
            }
        }

        // Compute gradients with and without baseline
        let all_returns: Vec<Vec<f64>> = trajectories
            .iter()
            .map(|t| t.discounted_returns(GAMMA))
            .collect();

        let mut total = 0.0;
        let mut count = 0;
        for returns in &all_returns {
            for &r in returns {
                total += r;
                count += 1;
            }
        }
        let baseline = total / f64::from(count);

        // Compute per-sample gradient norms with and without baseline
        let mut norms_no_baseline = Vec::new();
        let mut norms_with_baseline = Vec::new();

        for (env_idx, traj) in trajectories.iter().enumerate() {
            let returns = &all_returns[env_idx];
            for t in 0..traj.len() {
                let g = policy_gradient(&params, &traj.obs[t], &traj.actions[t], sigma);

                let no_bl: f64 = g
                    .iter()
                    .map(|&gi| (gi * returns[t]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                let with_bl: f64 = g
                    .iter()
                    .map(|&gi| (gi * (returns[t] - baseline)).powi(2))
                    .sum::<f64>()
                    .sqrt();

                norms_no_baseline.push(no_bl);
                norms_with_baseline.push(with_bl);
            }
        }

        let var_no_bl = variance(&norms_no_baseline);
        let var_with_bl = variance(&norms_with_baseline);

        assert!(
            var_with_bl < var_no_bl,
            "baseline should reduce variance: without={var_no_bl:.2}, with={var_with_bl:.2}"
        );
    }

    #[test]
    fn discounted_returns_correctness() {
        let mut traj = Trajectory::new();
        traj.rewards = vec![1.0, 2.0, 3.0];
        let returns = traj.discounted_returns(0.9);
        // R_2 = 3.0
        // R_1 = 2.0 + 0.9 * 3.0 = 4.7
        // R_0 = 1.0 + 0.9 * 4.7 = 5.23
        assert!((returns[2] - 3.0).abs() < 1e-10);
        assert!((returns[1] - 4.7).abs() < 1e-10);
        assert!((returns[0] - 5.23).abs() < 1e-10);
    }

    #[test]
    fn multi_seed_robustness() {
        let mut converged = 0;
        for seed in [42, 123, 999] {
            let r = run_reinforce_headless(seed);
            let improvement = (r.last_mean_reward - r.ep1_mean_reward) / r.ep1_mean_reward.abs();
            if improvement > 0.7 && r.param_norm > 0.3 {
                converged += 1;
            }
        }
        assert!(converged >= 2, "converged {converged}/3 seeds");
    }

    fn variance(vals: &[f64]) -> f64 {
        let n = vals.len() as f64;
        let mean = vals.iter().sum::<f64>() / n;
        vals.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n
    }
}
