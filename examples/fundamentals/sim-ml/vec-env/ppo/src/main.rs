//! PPO — Reaching Arm + Actor-Critic
//!
//! 50 two-link arms learn to reach a target using PPO (Proximal Policy
//! Optimization). Builds on REINFORCE: same arm, same VecEnv, same policy —
//! adds a learned value function (critic) and clipped surrogate objective.
//! The visual story: all 50 arms reach and park at the green target,
//! fixing REINFORCE's inability to achieve precision.
//!
//! Run: `cargo run -p example-ml-vec-env-ppo --release`

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
    clippy::too_many_lines,
    dead_code
)]

use bevy::prelude::*;
use example_ml_shared::{setup_reaching_arms, sync_batch_geoms};
use rand_distr::{Distribution, Normal};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_chassis::{
    DifferentiablePolicy, LinearPolicy, LinearValue, Optimizer, OptimizerConfig, Policy, Tensor,
    Trajectory, ValueFn, VecEnv, compute_gae,
};
use sim_rl::reaching_2dof;

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 50;
const PAUSE_TIME: f64 = 1.5;
const MAX_EPOCHS: usize = 30;
const VALIDATION_EPOCH: usize = 25;
const GAMMA: f64 = 0.99;
const GAE_LAMBDA: f64 = 0.95;
const CLIP_EPS: f64 = 0.2;
const K_OPT_PASSES: usize = 2;
const LR_ACTOR: f64 = 0.025;
const LR_CRITIC: f64 = 0.03;
const SIGMA_INIT: f64 = 0.5;
const SIGMA_MIN: f64 = 0.05;
const SIGMA_DECAY: f64 = 0.90;
const MAX_GRAD_NORM: f64 = 0.5;

// ── PPO Update ──────────────────────────────────────────────────────────────

struct PpoResult {
    actor_grad_norm: f64,
    mean_value_loss: f64,
    clip_fraction: f64,
}

#[allow(clippy::too_many_arguments)]
fn ppo_update(
    policy: &mut LinearPolicy,
    value_fn: &mut LinearValue,
    policy_optimizer: &mut dyn Optimizer,
    value_optimizer: &mut dyn Optimizer,
    trajectories: &[Trajectory],
    mu_old_all: &[Vec<Vec<f64>>],
    v_old_all: &[Vec<f64>],
    sigma: f64,
    k_passes: usize,
) -> PpoResult {
    let n_actor_params = policy.n_params();
    let n_critic_params = value_fn.n_params();

    // 1. Compute GAE advantages and value targets using shared compute_gae
    let gae_results: Vec<(Vec<f64>, Vec<f64>)> = trajectories
        .iter()
        .enumerate()
        .map(|(i, traj)| {
            let next_value = if traj.done {
                0.0
            } else {
                traj.terminal_obs
                    .as_ref()
                    .map_or(0.0, |obs| value_fn.forward(obs))
            };
            compute_gae(&traj.rewards, &v_old_all[i], next_value, GAMMA, GAE_LAMBDA)
        })
        .collect();

    // 2. Flatten and normalize advantages
    let mut all_advantages: Vec<f64> = Vec::new();
    for (advantages, _) in &gae_results {
        all_advantages.extend(advantages);
    }

    let n_total = all_advantages.len().max(1) as f64;
    let mean_adv = all_advantages.iter().sum::<f64>() / n_total;
    for a in &mut all_advantages {
        *a -= mean_adv;
    }
    let std_adv = (all_advantages.iter().map(|&a| a * a).sum::<f64>() / n_total).sqrt();
    if std_adv > 1e-8 {
        for a in &mut all_advantages {
            *a /= std_adv;
        }
    }

    // Rebuild per-trajectory advantage offsets
    let mut traj_adv_offsets: Vec<usize> = Vec::new();
    let mut offset = 0;
    for traj in trajectories {
        traj_adv_offsets.push(offset);
        offset += traj.rewards.len();
    }

    // 3. K optimization passes over frozen rollout data
    let mut last_actor_grad_norm = 0.0;
    let mut total_value_loss = 0.0;
    let mut total_clip_count = 0usize;
    let mut total_samples = 0usize;

    for _pass in 0..k_passes {
        let mut actor_grad = vec![0.0f64; n_actor_params];
        let mut critic_grad = vec![0.0f64; n_critic_params];
        let mut n_samples = 0usize;
        let mut clip_count = 0usize;
        let mut value_loss = 0.0;

        for (traj_idx, traj) in trajectories.iter().enumerate() {
            let (_, ref value_targets) = gae_results[traj_idx];
            let adv_offset = traj_adv_offsets[traj_idx];

            for t in 0..traj.rewards.len() {
                let obs = &traj.obs[t];
                let action = &traj.actions[t];
                let advantage = all_advantages[adv_offset + t];
                let mu_old = &mu_old_all[traj_idx][t];

                // Recompute mu_new from current actor params
                let mu_new = policy.forward(obs);

                // Log importance ratio: log(pi_new/pi_old)
                let sigma2 = sigma * sigma;
                let mut log_ratio = 0.0;
                for a in 0..action.len() {
                    log_ratio += -0.5
                        * ((action[a] - mu_new[a]).powi(2) - (action[a] - mu_old[a]).powi(2))
                        / sigma2;
                }
                let ratio = log_ratio.exp();

                // Clipped surrogate
                let unclipped = ratio * advantage;
                let clamped_ratio = ratio.clamp(1.0 - CLIP_EPS, 1.0 + CLIP_EPS);
                let clipped = clamped_ratio * advantage;

                if unclipped <= clipped {
                    let g = policy.log_prob_gradient(obs, action, sigma);
                    for k in 0..n_actor_params {
                        actor_grad[k] += advantage * ratio * g[k];
                    }
                } else {
                    clip_count += 1;
                }

                // Critic: MSE loss gradient against frozen value targets
                let target = value_targets[t];
                let v_new = value_fn.forward(obs);
                value_loss += (v_new - target).powi(2);

                let vg = value_fn.mse_gradient(obs, target);
                for k in 0..n_critic_params {
                    critic_grad[k] += vg[k];
                }

                n_samples += 1;
            }
        }

        // Average gradients
        if n_samples > 0 {
            let n_f = n_samples as f64;
            for g in &mut actor_grad {
                *g /= n_f;
            }
            for g in &mut critic_grad {
                *g /= n_f;
            }
            value_loss /= n_f;
        }

        last_actor_grad_norm = actor_grad.iter().map(|&g| g * g).sum::<f64>().sqrt();
        total_value_loss += value_loss;
        total_clip_count += clip_count;
        total_samples += n_samples;

        // Optimizer steps
        policy_optimizer.step(&actor_grad, true); // ascent — maximize surrogate
        value_optimizer.step(&critic_grad, false); // descent — minimize value loss

        // Pitfall #1: sync both optimizer params back to networks
        policy.set_params(policy_optimizer.params());
        value_fn.set_params(value_optimizer.params());
    }

    PpoResult {
        actor_grad_norm: last_actor_grad_norm,
        mean_value_loss: total_value_loss / k_passes as f64,
        clip_fraction: if total_samples > 0 {
            total_clip_count as f64 / total_samples as f64
        } else {
            0.0
        },
    }
}

// ── Bevy Resources ───────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Running,
    Updating,
}

#[derive(Resource)]
struct PpoResource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,

    policy: LinearPolicy,
    value_fn: LinearValue,
    policy_optimizer: Box<dyn Optimizer>,
    value_optimizer: Box<dyn Optimizer>,
    sigma: f64,

    epoch: usize,
    phase: Phase,
    env_complete: Vec<bool>,
    env_reached: Vec<bool>,
    trajectories: Vec<Trajectory>,
    // Pitfall #2: mu_old and v_old stored in parallel alongside Trajectory
    mu_old: Vec<Vec<Vec<f64>>>, // [env][step][act_dim]
    v_old: Vec<Vec<f64>>,       // [env][step]

    accumulator: f64,
    sim_time: f64,
    pause_timer: f64,

    rng: rand::rngs::StdRng,

    // History (per epoch)
    mean_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
    actor_grad_norms: Vec<f64>,
    value_losses: Vec<f64>,
    clip_fractions: Vec<f64>,
}

#[derive(Resource, Default)]
struct PpoValidation {
    reported: bool,
}

// ── Bevy App ─────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: PPO — Reaching Arm + Actor-Critic ===");
    println!("  {NUM_ENVS} arms, {MAX_EPOCHS} epochs, lr_actor={LR_ACTOR}, lr_critic={LR_CRITIC}");
    println!("  K={K_OPT_PASSES} opt passes, clip_eps={CLIP_EPS}, gae_lambda={GAE_LAMBDA}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — PPO (Reaching Arm + Actor-Critic)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<PpoValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_ppo)
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
    let task = reaching_2dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let value_fn = LinearValue::new(task.obs_dim(), task.obs_scale());

    let make_optimizer = |n_params, lr| {
        OptimizerConfig::Adam {
            lr,
            beta1: 0.9,
            beta2: 0.999,
            eps: 1e-8,
            max_grad_norm: MAX_GRAD_NORM,
        }
        .build(n_params)
    };
    let policy_optimizer = make_optimizer(policy.n_params(), LR_ACTOR);
    let value_optimizer = make_optimizer(value_fn.n_params(), LR_CRITIC);

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");

    let rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);

    commands.insert_resource(PpoResource {
        vec_env,
        actions,
        current_obs: init_obs,
        policy,
        value_fn,
        policy_optimizer,
        value_optimizer,
        sigma: SIGMA_INIT,
        epoch: 0,
        phase: Phase::Running,
        env_complete: vec![false; NUM_ENVS],
        env_reached: vec![false; NUM_ENVS],
        trajectories: (0..NUM_ENVS).map(|_| new_trajectory()).collect(),
        mu_old: (0..NUM_ENVS).map(|_| Vec::with_capacity(350)).collect(),
        v_old: (0..NUM_ENVS).map(|_| Vec::with_capacity(350)).collect(),
        accumulator: 0.0,
        sim_time: 0.0,
        pause_timer: 0.0,
        rng,
        mean_rewards: Vec::new(),
        reach_counts: Vec::new(),
        actor_grad_norms: Vec::new(),
        value_losses: Vec::new(),
        clip_fractions: Vec::new(),
    });
}

fn new_trajectory() -> Trajectory {
    Trajectory {
        obs: Vec::with_capacity(350),
        actions: Vec::with_capacity(350),
        rewards: Vec::with_capacity(350),
        done: false,
        terminal_obs: None,
    }
}

// ── Stepping (PPO state machine) ─────────────────────────────────────────

fn step_ppo(mut res: ResMut<PpoResource>, time: Res<Time>) {
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
                let inner = &mut *res;
                let noise = Normal::new(0.0, inner.sigma).unwrap();
                for i in 0..NUM_ENVS {
                    if !inner.env_complete[i] {
                        let obs = inner.current_obs.row(i);
                        let mu = inner.policy.forward(obs);
                        let v = inner.value_fn.forward(obs);
                        let action: Vec<f64> = mu
                            .iter()
                            .map(|&m| m + noise.sample(&mut inner.rng))
                            .collect();

                        inner.trajectories[i].obs.push(obs.to_vec());
                        inner.trajectories[i].actions.push(action.clone());
                        // Pitfall #2: store mu_old and v_old in parallel
                        inner.mu_old[i].push(mu);
                        inner.v_old[i].push(v);

                        let row = inner.actions.row_mut(i);
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
                                res.trajectories[i].done = true;
                            }
                            // Store terminal obs for GAE bootstrap
                            let obs = result.observations.row(i);
                            res.trajectories[i].terminal_obs = Some(obs.to_vec());
                        }
                    }
                }

                res.current_obs = result.observations;
                res.accumulator -= dt_action;
                res.sim_time += dt_action;

                if res.env_complete.iter().all(|&c| c) {
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
                let trajectories = std::mem::take(&mut res.trajectories);
                let mu_old = std::mem::take(&mut res.mu_old);
                let v_old = std::mem::take(&mut res.v_old);
                let sigma = res.sigma;
                let inner = &mut *res;
                let result = ppo_update(
                    &mut inner.policy,
                    &mut inner.value_fn,
                    inner.policy_optimizer.as_mut(),
                    inner.value_optimizer.as_mut(),
                    &trajectories,
                    &mu_old,
                    &v_old,
                    sigma,
                    K_OPT_PASSES,
                );

                inner.actor_grad_norms.push(result.actor_grad_norm);
                inner.value_losses.push(result.mean_value_loss);
                inner.clip_fractions.push(result.clip_fraction);

                // Anneal sigma
                res.sigma = (res.sigma * SIGMA_DECAY).max(SIGMA_MIN);

                // Next epoch
                res.epoch += 1;
                res.current_obs = res.vec_env.reset_all().expect("reset");
                res.env_complete.fill(false);
                res.env_reached.fill(false);
                res.trajectories = (0..NUM_ENVS).map(|_| new_trajectory()).collect();
                res.mu_old = (0..NUM_ENVS).map(|_| Vec::with_capacity(350)).collect();
                res.v_old = (0..NUM_ENVS).map(|_| Vec::with_capacity(350)).collect();
                res.phase = Phase::Running;
            }
        }
    }
}

// ── Sync ─────────────────────────────────────────────────────────────────

fn sync_vec_env_to_scenes(res: Res<PpoResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<PpoResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ───────────────────────────────────────────────────────────

fn track_validation(res: Res<PpoResource>, mut val: ResMut<PpoValidation>) {
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

    let last_vloss = res.value_losses.last().copied().unwrap_or(f64::INFINITY);
    let first_vloss = res.value_losses.first().copied().unwrap_or(f64::INFINITY);

    let any_clipping = res.clip_fractions.iter().any(|&f| f > 0.0);

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
            name: "Value loss decreased",
            pass: last_vloss < first_vloss,
            detail: format!("first={first_vloss:.2} -> last={last_vloss:.2}"),
        },
        Check {
            name: "Clipping activated",
            pass: any_clipping,
            detail: format!(
                "clip_fractions={:?}",
                res.clip_fractions
                    .iter()
                    .take(5)
                    .map(|f| format!("{f:.3}"))
                    .collect::<Vec<_>>()
            ),
        },
    ];

    let _ = print_report(&format!("PPO (epoch {VALIDATION_EPOCH})"), &checks);
}

// ── HUD ──────────────────────────────────────────────────────────────────

fn update_hud(res: Res<PpoResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Reaching Arm (VecEnv + PPO)");
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

    if let Some(&vl) = res.value_losses.last() {
        hud.raw(format!("value loss: {vl:.2}"));
    }
    if let Some(&cf) = res.clip_fractions.last() {
        hud.raw(format!("clip fraction: {cf:.3}"));
    }
    if let Some(&gn) = res.actor_grad_norms.last() {
        hud.raw(format!("|actor grad|: {gn:.4}"));
    }

    // Reward curve (compact)
    if res.mean_rewards.len() >= 2 {
        hud.section("Reward Curve");
        let rewards = &res.mean_rewards;
        let n = rewards.len();
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

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Headless PPO runner ──

    struct PpoHeadlessResult {
        ep1_mean_reward: f64,
        last_mean_reward: f64,
        best_reached: usize,
        per_epoch_reached: Vec<usize>,
        value_losses: Vec<f64>,
        clip_fractions: Vec<f64>,
        final_sigma: f64,
        actor_param_norm: f64,
    }

    fn run_ppo_headless(seed: u64) -> PpoHeadlessResult {
        run_ppo_headless_with_k_lr(seed, K_OPT_PASSES, LR_ACTOR)
    }

    fn run_ppo_headless_with_k_lr(seed: u64, k_passes: usize, lr_actor: f64) -> PpoHeadlessResult {
        use rand::SeedableRng;

        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut value_fn = LinearValue::new(task.obs_dim(), task.obs_scale());

        let make_optimizer = |n_params, lr| {
            OptimizerConfig::Adam {
                lr,
                beta1: 0.9,
                beta2: 0.999,
                eps: 1e-8,
                max_grad_norm: MAX_GRAD_NORM,
            }
            .build(n_params)
        };
        let mut policy_optimizer = make_optimizer(policy.n_params(), lr_actor);
        let mut value_optimizer = make_optimizer(value_fn.n_params(), LR_CRITIC);

        let mut vec_env = task.build_vec_env(NUM_ENVS, 0).expect("build_vec_env");
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut sigma = SIGMA_INIT;

        let mut ep1_mean_reward = 0.0;
        let mut last_mean_reward = 0.0;
        let mut per_epoch_reached = Vec::new();
        let mut value_losses_out = Vec::new();
        let mut clip_fractions_out = Vec::new();

        let episode_timeout = 3.0;
        let timestep = vec_env.model().timestep;
        let max_steps = (episode_timeout / (5.0 * timestep)) as usize + 50;

        for epoch in 0..MAX_EPOCHS {
            let mut current_obs = vec_env.reset_all().expect("reset");
            let mut trajectories: Vec<Trajectory> =
                (0..NUM_ENVS).map(|_| new_trajectory()).collect();
            let mut mu_old_all: Vec<Vec<Vec<f64>>> = (0..NUM_ENVS).map(|_| Vec::new()).collect();
            let mut v_old_all: Vec<Vec<f64>> = (0..NUM_ENVS).map(|_| Vec::new()).collect();
            let mut env_complete = [false; NUM_ENVS];
            let mut env_reached = [false; NUM_ENVS];
            let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
            let noise = Normal::new(0.0, sigma).unwrap();

            for _ in 0..max_steps {
                for i in 0..NUM_ENVS {
                    if !env_complete[i] {
                        let obs = current_obs.row(i);
                        let mu = policy.forward(obs);
                        let v = value_fn.forward(obs);
                        let action: Vec<f64> =
                            mu.iter().map(|&m| m + noise.sample(&mut rng)).collect();

                        trajectories[i].obs.push(obs.to_vec());
                        trajectories[i].actions.push(action.clone());
                        mu_old_all[i].push(mu);
                        v_old_all[i].push(v);

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
                                trajectories[i].done = true;
                            }
                            let obs = result.observations.row(i);
                            trajectories[i].terminal_obs = Some(obs.to_vec());
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
            per_epoch_reached.push(reached);

            let result = ppo_update(
                &mut policy,
                &mut value_fn,
                policy_optimizer.as_mut(),
                value_optimizer.as_mut(),
                &trajectories,
                &mu_old_all,
                &v_old_all,
                sigma,
                k_passes,
            );

            value_losses_out.push(result.mean_value_loss);
            clip_fractions_out.push(result.clip_fraction);

            sigma = (sigma * SIGMA_DECAY).max(SIGMA_MIN);
        }

        let best_reached = per_epoch_reached.iter().max().copied().unwrap_or(0);
        let actor_param_norm = policy.params().iter().map(|&v| v * v).sum::<f64>().sqrt();

        PpoHeadlessResult {
            ep1_mean_reward,
            last_mean_reward,
            best_reached,
            per_epoch_reached,
            value_losses: value_losses_out,
            clip_fractions: clip_fractions_out,
            final_sigma: sigma,
            actor_param_norm,
        }
    }

    // ── Stress tests ──

    #[test]
    fn critic_gradient_matches_finite_difference() {
        let task = reaching_2dof();
        let mut value_fn = LinearValue::new(task.obs_dim(), task.obs_scale());
        let params = [0.3, -0.1, 0.5, 0.2, -0.15];
        value_fn.set_params(&params);

        let obs: [f32; 4] = [0.5, -0.3, 0.8, -0.4];
        let target = -50.0;

        let analytic = value_fn.mse_gradient(&obs, target);

        let eps = 1e-5;
        for k in 0..value_fn.n_params() {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[k] += eps;
            p_minus[k] -= eps;

            value_fn.set_params(&p_plus);
            let loss_plus = (value_fn.forward(&obs) - target).powi(2);
            value_fn.set_params(&p_minus);
            let loss_minus = (value_fn.forward(&obs) - target).powi(2);
            let numerical = (loss_plus - loss_minus) / (2.0 * eps);

            assert!(
                (analytic[k] - numerical).abs() < 1e-3,
                "critic param {k}: analytic={:.6}, numerical={:.6}, diff={:.6}",
                analytic[k],
                numerical,
                (analytic[k] - numerical).abs()
            );
        }
    }

    #[test]
    fn gae_lambda_one_matches_discounted_returns() {
        let rewards = vec![1.0, 2.0, 3.0];
        let values = vec![-5.0; 3];
        let (advantages, _) = compute_gae(&rewards, &values, 0.0, GAMMA, 1.0);

        // Hand-compute discounted returns (done → no bootstrap):
        let r2 = 3.0;
        let r1 = 2.0 + GAMMA * r2;
        let r0 = 1.0 + GAMMA * r1;

        assert!(
            (advantages[2] - (r2 - (-5.0))).abs() < 1e-8,
            "A[2]={:.6}, expected={:.6}",
            advantages[2],
            r2 + 5.0
        );
        assert!(
            (advantages[1] - (r1 - (-5.0))).abs() < 1e-8,
            "A[1]={:.6}, expected={:.6}",
            advantages[1],
            r1 + 5.0
        );
        assert!(
            (advantages[0] - (r0 - (-5.0))).abs() < 1e-8,
            "A[0]={:.6}, expected={:.6}",
            advantages[0],
            r0 + 5.0
        );
    }

    #[test]
    fn clipping_activates_in_early_epochs() {
        let r = run_ppo_headless(42);
        let early_clipping = r.clip_fractions.iter().take(10).any(|&f| f > 0.0);
        assert!(
            early_clipping,
            "clip fraction should be > 0 in early epochs: {:?}",
            &r.clip_fractions[..10.min(r.clip_fractions.len())]
        );
    }

    #[test]
    fn ppo_converges_in_30_epochs() {
        let r = run_ppo_headless(42);
        let improvement = (r.last_mean_reward - r.ep1_mean_reward) / r.ep1_mean_reward.abs();
        assert!(
            improvement > 0.80,
            "improvement={:.0}%, want >80%",
            improvement * 100.0
        );
        assert!(
            r.actor_param_norm > 0.5,
            "actor_param_norm={:.3}, want >0.5",
            r.actor_param_norm
        );
    }

    #[test]
    fn ppo_learns_value_function() {
        let r = run_ppo_headless(42);
        let n = r.value_losses.len();
        let early = r.value_losses[..3].iter().sum::<f64>() / 3.0;
        let late = r.value_losses[n - 3..].iter().sum::<f64>() / 3.0;
        assert!(
            late < early * 0.5,
            "value loss should decrease by >50%: early={early:.1}, late={late:.1}"
        );

        let improvement = (r.last_mean_reward - r.ep1_mean_reward) / r.ep1_mean_reward.abs();
        assert!(
            improvement > 0.80,
            "improvement={:.0}%, want >80%",
            improvement * 100.0
        );
    }

    #[test]
    fn value_loss_decreases() {
        let r = run_ppo_headless(42);
        let n = r.value_losses.len();
        assert!(n >= 6, "need at least 6 epochs of value losses");

        let early_avg: f64 = r.value_losses[..3].iter().sum::<f64>() / 3.0;
        let late_avg: f64 = r.value_losses[n - 3..].iter().sum::<f64>() / 3.0;

        assert!(
            late_avg < early_avg,
            "value loss should decrease: early={early_avg:.2} -> late={late_avg:.2}"
        );
    }

    #[test]
    fn clip_fraction_reasonable() {
        let r = run_ppo_headless(42);
        for (i, &cf) in r.clip_fractions.iter().enumerate() {
            assert!(
                cf < 0.5,
                "epoch {}: clip_fraction={cf:.3}, should be <0.5",
                i + 1
            );
        }
    }

    #[test]
    fn multiple_opt_passes_help() {
        let lr = 0.01;
        let r_k1 = run_ppo_headless_with_k_lr(42, 1, lr);
        let r_k4 = run_ppo_headless_with_k_lr(42, 4, lr);

        let improvement_k1 =
            (r_k1.last_mean_reward - r_k1.ep1_mean_reward) / r_k1.ep1_mean_reward.abs();
        let improvement_k4 =
            (r_k4.last_mean_reward - r_k4.ep1_mean_reward) / r_k4.ep1_mean_reward.abs();

        assert!(
            improvement_k4 >= improvement_k1,
            "K=4 ({:.0}%) should be >= K=1 ({:.0}%) at lr={lr}",
            improvement_k4 * 100.0,
            improvement_k1 * 100.0
        );
    }

    #[test]
    fn actor_gradient_matches_finite_difference() {
        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let params = [0.1, -0.2, 0.3, 0.15, -0.1, 0.25, -0.05, 0.2, 0.1, -0.1];
        policy.set_params(&params);

        let obs: [f32; 4] = [0.5, -0.3, 0.8, -0.4];
        let action = [0.3, -0.2];
        let sigma = 0.3;

        let analytic = policy.log_prob_gradient(&obs, &action, sigma);

        let eps = 1e-5;
        for k in 0..policy.n_params() {
            let mut p_plus = params;
            let mut p_minus = params;
            p_plus[k] += eps;
            p_minus[k] -= eps;

            policy.set_params(&p_plus);
            let mu_plus = policy.forward(&obs);
            policy.set_params(&p_minus);
            let mu_minus = policy.forward(&obs);

            let log_pi_plus: f64 = (0..2)
                .map(|a| -0.5 * (action[a] - mu_plus[a]).powi(2) / (sigma * sigma))
                .sum();
            let log_pi_minus: f64 = (0..2)
                .map(|a| -0.5 * (action[a] - mu_minus[a]).powi(2) / (sigma * sigma))
                .sum();

            let numerical = (log_pi_plus - log_pi_minus) / (2.0 * eps);

            assert!(
                (analytic[k] - numerical).abs() < 1e-3,
                "actor param {k}: analytic={:.6}, numerical={:.6}",
                analytic[k],
                numerical,
            );
        }
    }
}
