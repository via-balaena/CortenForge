//! TD3 — Reaching Arm + Twin Delayed DDPG
//!
//! 50 two-link arms learn to reach a target using TD3 (Twin Delayed Deep
//! Deterministic Policy Gradient). Off-policy: transitions are stored in a
//! replay buffer and reused for learning. The visual story: random warmup,
//! then arms gradually coordinate as the deterministic policy improves.
//!
//! TD3 innovations over DDPG:
//! - Twin Q-networks: min(Q1, Q2) reduces overestimation bias
//! - Delayed policy update: actor updates less often than critics
//! - Target policy smoothing: noise on target actions regularizes Q
//!
//! Run: `cargo run -p example-ml-vec-env-td3 --release`

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
use rand::Rng;
use rand_distr::{Distribution, Normal};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{
    DifferentiablePolicy, LinearPolicy, LinearQ, Optimizer, OptimizerConfig, Policy, QFunction,
    ReplayBuffer, Tensor, VecEnv, reaching_2dof, soft_update, soft_update_policy,
};

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 50;
const WARMUP_STEPS: usize = 50;
const BATCH_SIZE: usize = 64;
const BUFFER_CAPACITY: usize = 100_000;
const GAMMA: f64 = 0.95;
const TAU: f64 = 0.005;
const LR_ACTOR: f64 = 0.00005;
const LR_CRITIC: f64 = 0.005;
const EXPLORATION_NOISE: f64 = 0.3;
const TARGET_NOISE: f64 = 0.2;
const NOISE_CLIP: f64 = 0.5;
const POLICY_DELAY: usize = 2;
const MAX_STEPS: usize = 6000;
const PAUSE_TIME: f64 = 1.0;
const VALIDATION_EPOCH: usize = 15;
const MAX_EPOCHS: usize = 20;
const MAX_GRAD_NORM: f64 = 1.0;
const REWARD_SCALE: f64 = 0.01;

// ── TD3 Update ──────────────────────────────────────────────────────────────

struct Td3UpdateResult {
    mean_q1: f64,
    critic_loss: f64,
    actor_grad_norm: Option<f64>,
}

#[allow(clippy::too_many_arguments)]
fn td3_update(
    policy: &mut LinearPolicy,
    target_policy: &mut LinearPolicy,
    q1: &mut LinearQ,
    q2: &mut LinearQ,
    target_q1: &mut LinearQ,
    target_q2: &mut LinearQ,
    policy_optimizer: &mut dyn Optimizer,
    q1_optimizer: &mut dyn Optimizer,
    q2_optimizer: &mut dyn Optimizer,
    replay_buffer: &ReplayBuffer,
    critic_update_count: usize,
    rng: &mut rand::rngs::StdRng,
) -> Td3UpdateResult {
    let batch = replay_buffer.sample(BATCH_SIZE, rng);
    let obs_dim = batch.obs_dim;
    let act_dim = batch.act_dim;

    // 1. Compute TD targets with target policy smoothing
    let target_noise_dist = Normal::new(0.0, TARGET_NOISE).unwrap();
    let mut targets = Vec::with_capacity(BATCH_SIZE);

    for i in 0..BATCH_SIZE {
        let next_obs = &batch.next_obs[i * obs_dim..(i + 1) * obs_dim];

        // Target policy smoothing: add clipped noise to target actions
        let a_next = target_policy.forward(next_obs);
        let a_smoothed: Vec<f64> = a_next
            .iter()
            .map(|&a| {
                let n = target_noise_dist.sample(rng).clamp(-NOISE_CLIP, NOISE_CLIP);
                (a + n).clamp(-1.0, 1.0)
            })
            .collect();

        // Twin Q: take min of two target Q-values (reduces overestimation)
        let q1_next = target_q1.forward(next_obs, &a_smoothed);
        let q2_next = target_q2.forward(next_obs, &a_smoothed);
        let q_min = q1_next.min(q2_next);

        let done_mask = if batch.dones[i] { 0.0 } else { 1.0 };
        targets.push(batch.rewards[i] + GAMMA * done_mask * q_min);
    }

    // 2. Update Q1 and Q2 via MSE against TD targets
    let n_q_params = q1.n_params();
    let mut q1_grad = vec![0.0; n_q_params];
    let mut q2_grad = vec![0.0; n_q_params];
    let mut critic_loss = 0.0;
    let mut mean_q1 = 0.0;

    for i in 0..BATCH_SIZE {
        let obs = &batch.obs[i * obs_dim..(i + 1) * obs_dim];
        let action = &batch.actions[i * act_dim..(i + 1) * act_dim];

        let q1_val = q1.forward(obs, action);
        mean_q1 += q1_val;
        critic_loss += (q1_val - targets[i]).powi(2);

        let g1 = q1.mse_gradient(obs, action, targets[i]);
        let g2 = q2.mse_gradient(obs, action, targets[i]);

        for k in 0..n_q_params {
            q1_grad[k] += g1[k];
            q2_grad[k] += g2[k];
        }
    }

    let bs = BATCH_SIZE as f64;
    for g in &mut q1_grad {
        *g /= bs;
    }
    for g in &mut q2_grad {
        *g /= bs;
    }
    critic_loss /= bs;
    mean_q1 /= bs;

    // Critic optimizer steps (descent — minimize MSE)
    q1_optimizer.step(&q1_grad, false);
    q2_optimizer.step(&q2_grad, false);

    // Pitfall #1: sync optimizer params back to Q-networks
    q1.set_params(q1_optimizer.params());
    q2.set_params(q2_optimizer.params());

    // 3. Delayed policy update (every POLICY_DELAY critic updates)
    let actor_grad_norm = if critic_update_count.is_multiple_of(POLICY_DELAY) {
        // Deterministic policy gradient via chain rule:
        // ∇θ J = E[ ∇a Q1(s, μ(s)) · ∇θ μ(s) ]
        let n_policy_params = policy.n_params();
        let mut actor_grad = vec![0.0; n_policy_params];

        for i in 0..BATCH_SIZE {
            let obs = &batch.obs[i * obs_dim..(i + 1) * obs_dim];
            let a = policy.forward(obs);
            let dq_da = q1.action_gradient(obs, &a);
            let vjp = policy.forward_vjp(obs, &dq_da);
            for k in 0..n_policy_params {
                actor_grad[k] += vjp[k];
            }
        }

        for g in &mut actor_grad {
            *g /= bs;
        }
        let norm = actor_grad.iter().map(|&g| g * g).sum::<f64>().sqrt();

        // Policy optimizer step (ascent — maximize Q)
        policy_optimizer.step(&actor_grad, true);
        // Pitfall #1: sync optimizer params back to policy
        policy.set_params(policy_optimizer.params());

        // Soft-update all target networks (Polyak averaging)
        soft_update(target_q1, q1, TAU);
        soft_update(target_q2, q2, TAU);

        soft_update_policy(target_policy, policy, TAU);

        Some(norm)
    } else {
        None
    };

    Td3UpdateResult {
        mean_q1,
        critic_loss,
        actor_grad_norm,
    }
}

// ── Bevy Resources ─────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Warmup,
    Running,
    Paused,
    Done,
}

#[derive(Resource)]
struct Td3Resource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,

    policy: LinearPolicy,
    target_policy: LinearPolicy,
    q1: LinearQ,
    q2: LinearQ,
    target_q1: LinearQ,
    target_q2: LinearQ,

    policy_optimizer: Box<dyn Optimizer>,
    q1_optimizer: Box<dyn Optimizer>,
    q2_optimizer: Box<dyn Optimizer>,

    replay_buffer: ReplayBuffer,

    phase: Phase,
    step_count: usize,
    critic_updates: usize,
    display_epoch: usize,

    // Per-env episode tracking
    episode_rewards: Vec<f64>,
    // Completed episodes since last display epoch
    completed_rewards: Vec<f64>,
    epoch_reached: usize,
    // All completed episode rewards (for validation)
    all_completed_rewards: Vec<f64>,

    accumulator: f64,
    sim_time: f64,
    pause_timer: f64,

    rng: rand::rngs::StdRng,

    // Latest update stats (for HUD)
    last_mean_q1: f64,
    last_critic_loss: f64,
    last_actor_grad_norm: f64,

    // History (per display epoch)
    mean_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
}

#[derive(Resource, Default)]
struct Td3Validation {
    reported: bool,
}

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: TD3 — Reaching Arm + Twin Delayed DDPG ===");
    println!("  {NUM_ENVS} arms, {MAX_STEPS} steps, lr_actor={LR_ACTOR}, lr_critic={LR_CRITIC}");
    println!("  warmup={WARMUP_STEPS}, buffer={BUFFER_CAPACITY}, batch={BATCH_SIZE}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — TD3 (Reaching Arm + Twin Delayed DDPG)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<Td3Validation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_td3)
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
    let task = reaching_2dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let q2 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q2 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());

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
    let q1_optimizer = make_optimizer(q1.n_params(), LR_CRITIC);
    let q2_optimizer = make_optimizer(q2.n_params(), LR_CRITIC);

    let replay_buffer = ReplayBuffer::new(BUFFER_CAPACITY, task.obs_dim(), task.act_dim());

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");

    let rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);

    commands.insert_resource(Td3Resource {
        vec_env,
        actions,
        current_obs: init_obs,
        policy,
        target_policy,
        q1,
        q2,
        target_q1,
        target_q2,
        policy_optimizer,
        q1_optimizer,
        q2_optimizer,
        replay_buffer,
        phase: Phase::Warmup,
        step_count: 0,
        critic_updates: 0,
        display_epoch: 0,
        episode_rewards: vec![0.0; NUM_ENVS],
        completed_rewards: Vec::new(),
        epoch_reached: 0,
        all_completed_rewards: Vec::new(),
        accumulator: 0.0,
        sim_time: 0.0,
        pause_timer: 0.0,
        rng,
        last_mean_q1: 0.0,
        last_critic_loss: 0.0,
        last_actor_grad_norm: 0.0,
        mean_rewards: Vec::new(),
        reach_counts: Vec::new(),
    });
}

// ── Stepping (TD3 state machine) ──────────────────────────────────────────

fn step_td3(mut res: ResMut<Td3Resource>, time: Res<Time>) {
    if res.phase == Phase::Paused {
        let wall_dt = time.delta_secs_f64();
        res.pause_timer -= wall_dt;
        if res.pause_timer <= 0.0 {
            if res.step_count < MAX_STEPS {
                res.phase = Phase::Running;
            } else {
                res.phase = Phase::Done;
            }
        }
        return;
    }

    if res.phase == Phase::Done {
        return;
    }

    if res.step_count >= MAX_STEPS {
        if res.phase != Phase::Done {
            res.phase = Phase::Done;
        }
        return;
    }

    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;
    let dt_action = 5.0 * res.vec_env.model().timestep;

    #[allow(clippy::cast_sign_loss)]
    let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

    for _ in 0..budget {
        let is_warmup = res.phase == Phase::Warmup;

        // Select actions
        let inner = &mut *res;
        let noise_dist = Normal::new(0.0, EXPLORATION_NOISE).unwrap();
        let mut step_actions: Vec<Vec<f64>> = Vec::with_capacity(NUM_ENVS);

        for i in 0..NUM_ENVS {
            let obs = inner.current_obs.row(i);
            let action: Vec<f64> = if is_warmup {
                vec![
                    inner.rng.random_range(-1.0..1.0),
                    inner.rng.random_range(-1.0..1.0),
                ]
            } else {
                inner
                    .policy
                    .forward(obs)
                    .iter()
                    .map(|&m| m + noise_dist.sample(&mut inner.rng))
                    .collect()
            };
            let row = inner.actions.row_mut(i);
            row[0] = action[0] as f32;
            row[1] = action[1] as f32;
            step_actions.push(action);
        }

        let actions_tensor = res.actions.clone();
        let result = res.vec_env.step(&actions_tensor).expect("step");

        // Store transitions and track episode rewards
        let inner = &mut *res;
        for i in 0..NUM_ENVS {
            let obs = inner.current_obs.row(i);
            let next_obs = result.observations.row(i);
            let done = result.dones[i] || result.truncateds[i];
            inner.replay_buffer.push(
                obs,
                &step_actions[i],
                result.rewards[i] * REWARD_SCALE,
                next_obs,
                done,
            );

            inner.episode_rewards[i] += result.rewards[i];
            if done {
                inner.completed_rewards.push(inner.episode_rewards[i]);
                if result.dones[i] {
                    inner.epoch_reached += 1;
                }
                inner.episode_rewards[i] = 0.0;
            }
        }

        res.current_obs = result.observations;
        res.step_count += 1;
        res.accumulator -= dt_action;
        res.sim_time += dt_action;

        // Warmup → Running transition
        if is_warmup {
            if res.step_count >= WARMUP_STEPS {
                println!(
                    "Warmup complete ({} transitions). Starting TD3 training...",
                    res.replay_buffer.len()
                );
                res.phase = Phase::Running;
                res.completed_rewards.clear();
                res.epoch_reached = 0;
            }
            continue; // No learning or display epochs during warmup
        }

        // TD3 update (every step after warmup)
        if res.replay_buffer.len() >= BATCH_SIZE {
            res.critic_updates += 1;
            let inner = &mut *res;
            let update_result = td3_update(
                &mut inner.policy,
                &mut inner.target_policy,
                &mut inner.q1,
                &mut inner.q2,
                &mut inner.target_q1,
                &mut inner.target_q2,
                inner.policy_optimizer.as_mut(),
                inner.q1_optimizer.as_mut(),
                inner.q2_optimizer.as_mut(),
                &inner.replay_buffer,
                inner.critic_updates,
                &mut inner.rng,
            );
            if let Some(norm) = update_result.actor_grad_norm {
                inner.last_actor_grad_norm = norm;
            }
            inner.last_mean_q1 = update_result.mean_q1;
            inner.last_critic_loss = update_result.critic_loss;
        }

        // Display epoch: trigger when NUM_ENVS episodes have completed
        if res.display_epoch < MAX_EPOCHS && res.completed_rewards.len() >= NUM_ENVS {
            let inner = &mut *res;
            let mean_reward =
                inner.completed_rewards.iter().sum::<f64>() / inner.completed_rewards.len() as f64;
            let reached = inner.epoch_reached;

            inner.mean_rewards.push(mean_reward);
            inner.reach_counts.push(reached);
            inner
                .all_completed_rewards
                .extend_from_slice(&inner.completed_rewards);

            res.display_epoch += 1;
            println!(
                "epoch {:>2}: reached={reached:>2}/{NUM_ENVS}  mean_R={mean_reward:>8.1}  Q1={:.2}  buf={}",
                res.display_epoch,
                res.last_mean_q1,
                res.replay_buffer.len()
            );

            res.completed_rewards.clear();
            res.epoch_reached = 0;
            res.phase = Phase::Paused;
            res.pause_timer = PAUSE_TIME;
            break;
        }
    }
}

// ── Sync ──────────────────────────────────────────────────────────────────

fn sync_vec_env_to_scenes(res: Res<Td3Resource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<Td3Resource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ────────────────────────────────────────────────────────────

fn track_validation(res: Res<Td3Resource>, mut val: ResMut<Td3Validation>) {
    if res.display_epoch < VALIDATION_EPOCH || val.reported {
        return;
    }
    val.reported = true;

    let n = res.mean_rewards.len();
    if n < 2 {
        return;
    }

    // Compare first-half vs last-half of all completed episodes
    let all = &res.all_completed_rewards;
    let total = all.len();
    if total < 4 {
        return;
    }
    let half = total / 2;
    let early_mean = all[..half].iter().sum::<f64>() / half as f64;
    let late_mean = all[half..].iter().sum::<f64>() / (total - half) as f64;
    let improvement = if early_mean.abs() > 1e-10 {
        (late_mean - early_mean) / early_mean.abs()
    } else {
        0.0
    };

    let param_norm = res
        .policy
        .params()
        .iter()
        .map(|&v| v * v)
        .sum::<f64>()
        .sqrt();

    let checks = [
        Check {
            name: "Reward improved (late > early episodes)",
            pass: improvement > 0.1,
            detail: format!(
                "early={early_mean:.1} -> late={late_mean:.1} ({:.0}%)",
                improvement * 100.0
            ),
        },
        Check {
            name: "Replay buffer filled",
            pass: res.replay_buffer.len() >= WARMUP_STEPS * NUM_ENVS,
            detail: format!("buffer_len={}", res.replay_buffer.len()),
        },
        Check {
            name: "Policy learned (||θ|| > 0.1)",
            pass: param_norm > 0.1,
            detail: format!("||θ||={param_norm:.3}"),
        },
    ];

    let _ = print_report(&format!("TD3 (epoch {VALIDATION_EPOCH})"), &checks);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(res: Res<Td3Resource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Reaching Arm (VecEnv + TD3)");
    hud.raw(format!("step: {} / {MAX_STEPS}", res.step_count));
    if res.phase == Phase::Done {
        hud.raw(format!("training complete ({} epochs)", res.display_epoch));
    } else {
        hud.raw(format!(
            "epoch: {} / {MAX_EPOCHS}  ({}/{NUM_ENVS} episodes)",
            res.display_epoch,
            res.completed_rewards.len()
        ));
        hud.raw(format!(
            "phase: {:?}{}",
            res.phase,
            if res.phase == Phase::Paused {
                format!(" ({:.1}s)", res.pause_timer.max(0.0))
            } else {
                String::new()
            }
        ));
    }

    hud.raw(format!(
        "buffer: {} / {BUFFER_CAPACITY}",
        res.replay_buffer.len()
    ));
    hud.raw(format!(
        "Q1: {:.2}  |grad|: {:.4}",
        res.last_mean_q1, res.last_actor_grad_norm
    ));

    if let Some(&mean_r) = res.mean_rewards.last() {
        hud.raw(format!("mean ep reward: {mean_r:.1}"));
    }

    let total_reached: usize = res.reach_counts.iter().sum();
    hud.raw(format!("total reached: {total_reached}"));

    // Reward curve
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

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Headless TD3 runner ──

    struct Td3HeadlessResult {
        early_mean_reward: f64,
        late_mean_reward: f64,
        total_reached: usize,
        buffer_len: usize,
        param_norm: f64,
    }

    const TEST_STEPS: usize = 3000;

    #[allow(clippy::too_many_lines)]
    fn run_td3_headless(seed: u64) -> Td3HeadlessResult {
        use rand::SeedableRng;

        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut target_policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut q2 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut target_q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut target_q2 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());

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
        let mut policy_optimizer = make_optimizer(policy.n_params(), LR_ACTOR);
        let mut q1_optimizer = make_optimizer(q1.n_params(), LR_CRITIC);
        let mut q2_optimizer = make_optimizer(q2.n_params(), LR_CRITIC);

        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build");
        let mut replay_buffer = ReplayBuffer::new(BUFFER_CAPACITY, task.obs_dim(), task.act_dim());
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let noise_dist = Normal::new(0.0, EXPLORATION_NOISE).unwrap();

        let mut episode_rewards = vec![0.0f64; NUM_ENVS];
        let mut completed_rewards: Vec<f64> = Vec::new();
        let mut total_reached = 0usize;
        let mut critic_updates = 0usize;

        let mut current_obs = vec_env.reset_all().expect("reset");
        let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);

        for step in 0..TEST_STEPS {
            let is_warmup = step < WARMUP_STEPS;

            // Select actions
            let mut step_actions: Vec<Vec<f64>> = Vec::with_capacity(NUM_ENVS);
            for i in 0..NUM_ENVS {
                let obs = current_obs.row(i);
                let action: Vec<f64> = if is_warmup {
                    vec![rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0)]
                } else {
                    policy
                        .forward(obs)
                        .iter()
                        .map(|&m| m + noise_dist.sample(&mut rng))
                        .collect()
                };
                let row = actions.row_mut(i);
                row[0] = action[0] as f32;
                row[1] = action[1] as f32;
                step_actions.push(action);
            }

            let result = vec_env.step(&actions).expect("step");

            for i in 0..NUM_ENVS {
                let obs = current_obs.row(i);
                let next_obs = result.observations.row(i);
                let done = result.dones[i] || result.truncateds[i];
                replay_buffer.push(
                    obs,
                    &step_actions[i],
                    result.rewards[i] * REWARD_SCALE,
                    next_obs,
                    done,
                );

                episode_rewards[i] += result.rewards[i];
                if done {
                    completed_rewards.push(episode_rewards[i]);
                    if result.dones[i] {
                        total_reached += 1;
                    }
                    episode_rewards[i] = 0.0;
                }
            }

            current_obs = result.observations;

            // Learn after warmup
            if !is_warmup && replay_buffer.len() >= BATCH_SIZE {
                critic_updates += 1;
                td3_update(
                    &mut policy,
                    &mut target_policy,
                    &mut q1,
                    &mut q2,
                    &mut target_q1,
                    &mut target_q2,
                    policy_optimizer.as_mut(),
                    q1_optimizer.as_mut(),
                    q2_optimizer.as_mut(),
                    &replay_buffer,
                    critic_updates,
                    &mut rng,
                );
            }
        }

        let n = completed_rewards.len();
        let half = n / 2;
        let early_mean = if half > 0 {
            completed_rewards[..half].iter().sum::<f64>() / half as f64
        } else {
            0.0
        };
        let late_mean = if n > half && n - half > 0 {
            completed_rewards[half..].iter().sum::<f64>() / (n - half) as f64
        } else {
            0.0
        };

        let param_norm = policy.params().iter().map(|&v| v * v).sum::<f64>().sqrt();

        Td3HeadlessResult {
            early_mean_reward: early_mean,
            late_mean_reward: late_mean,
            total_reached,
            buffer_len: replay_buffer.len(),
            param_norm,
        }
    }

    // ── Convergence ──

    #[test]
    fn td3_converges_in_3000_steps() {
        let r = run_td3_headless(42);
        let improvement = if r.early_mean_reward.abs() > 1e-10 {
            (r.late_mean_reward - r.early_mean_reward) / r.early_mean_reward.abs()
        } else {
            0.0
        };
        assert!(
            improvement > 0.1,
            "improvement={:.0}%, want >10% (early={:.1}, late={:.1})",
            improvement * 100.0,
            r.early_mean_reward,
            r.late_mean_reward
        );
        assert!(
            r.param_norm > 0.1,
            "param_norm={:.3}, want >0.1",
            r.param_norm
        );
    }

    // ── Buffer fill ──

    #[test]
    fn warmup_fills_buffer() {
        let r = run_td3_headless(42);
        let expected_min = WARMUP_STEPS * NUM_ENVS;
        assert!(
            r.buffer_len >= expected_min,
            "buffer_len={}, want >={expected_min}",
            r.buffer_len
        );
    }

    // ── Policy gradient direction ──

    #[test]
    fn deterministic_policy_gradient_direction() {
        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());

        // Set non-zero Q weights so action_gradient is non-zero
        let mut q_params = vec![0.0; q1.n_params()];
        q_params[task.obs_dim()] = 1.0; // weight for action[0]
        q_params[task.obs_dim() + 1] = 1.0; // weight for action[1]
        q1.set_params(&q_params);

        let obs: Vec<f32> = vec![0.5, -0.3, 0.8, -0.4];
        let a = policy.forward(&obs);
        let dq_da = q1.action_gradient(&obs, &a);
        let grad = policy.forward_vjp(&obs, &dq_da);

        // Stepping params in gradient direction should increase Q(s, μ(s))
        let q_before = q1.forward(&obs, &a);

        let eps = 0.01;
        let new_params: Vec<f64> = policy
            .params()
            .iter()
            .zip(&grad)
            .map(|(&p, &g)| p + eps * g)
            .collect();
        policy.set_params(&new_params);
        let a_new = policy.forward(&obs);
        let q_after = q1.forward(&obs, &a_new);

        assert!(
            q_after > q_before,
            "Q should increase: before={q_before:.6}, after={q_after:.6}"
        );
    }

    // ── Soft update ──

    #[test]
    fn soft_update_interpolates() {
        let task = reaching_2dof();
        let mut q = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut target = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());

        let params: Vec<f64> = (0..q.n_params()).map(|i| (i + 1) as f64 * 0.1).collect();
        q.set_params(&params);

        soft_update(&mut target, &q, TAU);

        for (i, (&t, &s)) in target.params().iter().zip(q.params().iter()).enumerate() {
            let expected = TAU * s;
            assert!(
                (t - expected).abs() < 1e-10,
                "param {i}: target={t:.6}, expected={expected:.6}"
            );
        }
    }

    // ── Multi-seed robustness ──

    #[test]
    fn multi_seed_robustness() {
        let mut converged = 0;
        for seed in [42, 123, 999] {
            let r = run_td3_headless(seed);
            let improvement = if r.early_mean_reward.abs() > 1e-10 {
                (r.late_mean_reward - r.early_mean_reward) / r.early_mean_reward.abs()
            } else {
                0.0
            };
            if improvement > 0.1 && r.param_norm > 0.1 {
                converged += 1;
            }
        }
        assert!(converged >= 2, "converged {converged}/3 seeds");
    }
}
