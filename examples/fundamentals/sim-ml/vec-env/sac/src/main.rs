//! SAC — Reaching Arm + Soft Actor-Critic
//!
//! 50 two-link arms learn to reach a target using SAC (Soft Actor-Critic).
//! Off-policy with learned exploration: the stochastic policy has per-action
//! log-std parameters that are auto-tuned alongside the entropy temperature
//! alpha. The visual story: random warmup, then arms coordinate with more
//! exploration early (high alpha) and less later (alpha decreases).
//!
//! SAC innovations over TD3:
//! - Stochastic policy with learned exploration (no fixed noise)
//! - Entropy regularization: Q targets include -α log π(a|s) bonus
//! - Alpha auto-tuning: entropy temperature adapts to target entropy
//! - Reparameterization trick: a = μ(s) + σ(s) * ε for differentiable sampling
//!
//! Run: `cargo run -p example-ml-vec-env-sac --release`

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
    LinearQ, LinearStochasticPolicy, Optimizer, OptimizerConfig, Policy, QFunction, ReplayBuffer,
    StochasticPolicy, Tensor, VecEnv, gaussian_log_prob, reaching_2dof, soft_update,
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
const LR_ALPHA: f64 = 0.005;
const INIT_LOG_STD: f64 = -0.5;
const INIT_LOG_ALPHA: f64 = -4.6; // alpha ≈ 0.01, matching reward scale
const MAX_STEPS: usize = 6000;
const PAUSE_TIME: f64 = 1.0;
const VALIDATION_EPOCH: usize = 15;
const MAX_EPOCHS: usize = 20;
const MAX_GRAD_NORM: f64 = 1.0;
const REWARD_SCALE: f64 = 0.01;

// ── SAC Update ──────────────────────────────────────────────────────────────

struct SacUpdateResult {
    mean_q1: f64,
    actor_grad_norm: f64,
    alpha: f64,
    entropy: f64,
    new_log_alpha: f64,
}

#[allow(clippy::too_many_arguments)]
fn sac_update(
    policy: &mut LinearStochasticPolicy,
    q1: &mut LinearQ,
    q2: &mut LinearQ,
    target_q1: &mut LinearQ,
    target_q2: &mut LinearQ,
    policy_optimizer: &mut dyn Optimizer,
    q1_optimizer: &mut dyn Optimizer,
    q2_optimizer: &mut dyn Optimizer,
    replay_buffer: &ReplayBuffer,
    act_dim: usize,
    log_alpha: f64,
    rng: &mut rand::rngs::StdRng,
) -> SacUpdateResult {
    let alpha = log_alpha.exp();
    let target_entropy = -(act_dim as f64);

    let batch = replay_buffer.sample(BATCH_SIZE, rng);
    let obs_dim = batch.obs_dim;
    let std_normal = Normal::new(0.0, 1.0).unwrap();

    // 1. Compute TD targets with entropy bonus
    let mut targets = Vec::with_capacity(BATCH_SIZE);

    for i in 0..BATCH_SIZE {
        let next_obs = &batch.next_obs[i * obs_dim..(i + 1) * obs_dim];

        // Sample next action from current stochastic policy
        let (mu_next, log_std_next) = policy.forward_stochastic(next_obs);
        let eps_next: Vec<f64> = (0..act_dim).map(|_| std_normal.sample(rng)).collect();
        let a_next: Vec<f64> = mu_next
            .iter()
            .zip(&log_std_next)
            .zip(&eps_next)
            .map(|((&m, &ls), &e)| m + ls.exp() * e)
            .collect();

        let log_prob_next = gaussian_log_prob(&a_next, &mu_next, &log_std_next);

        // Twin Q with entropy bonus: y = r + γ(1-d)(min(Q1',Q2') - α log π)
        let q1_next = target_q1.forward(next_obs, &a_next);
        let q2_next = target_q2.forward(next_obs, &a_next);
        let q_min = q1_next.min(q2_next);

        let done_mask = if batch.dones[i] { 0.0 } else { 1.0 };
        targets.push(batch.rewards[i] + GAMMA * done_mask * (q_min - alpha * log_prob_next));
    }

    // 2. Update Q1, Q2 via MSE against entropy-augmented TD targets
    let n_q_params = q1.n_params();
    let mut q1_grad = vec![0.0; n_q_params];
    let mut q2_grad = vec![0.0; n_q_params];
    let mut mean_q1 = 0.0;

    for i in 0..BATCH_SIZE {
        let obs = &batch.obs[i * obs_dim..(i + 1) * obs_dim];
        let action = &batch.actions[i * act_dim..(i + 1) * act_dim];

        mean_q1 += q1.forward(obs, action);

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
    mean_q1 /= bs;

    q1_optimizer.step(&q1_grad, false);
    q2_optimizer.step(&q2_grad, false);
    q1.set_params(q1_optimizer.params());
    q2.set_params(q2_optimizer.params());

    // 3. Update policy via reparameterized gradient
    //    Maximize: E[Q(s,a) - α log π(a|s)]  where a = μ + σ*ε
    let n_policy_params = policy.n_params();
    let log_std_offset = n_policy_params - act_dim;
    let mut actor_grad = vec![0.0; n_policy_params];
    let mut mean_log_prob = 0.0;
    let mut mean_entropy = 0.0;

    for i in 0..BATCH_SIZE {
        let obs = &batch.obs[i * obs_dim..(i + 1) * obs_dim];
        let (mu, log_std) = policy.forward_stochastic(obs);
        let eps: Vec<f64> = (0..act_dim).map(|_| std_normal.sample(rng)).collect();
        let a: Vec<f64> = mu
            .iter()
            .zip(&log_std)
            .zip(&eps)
            .map(|((&m, &ls), &e)| m + ls.exp() * e)
            .collect();

        let log_prob = gaussian_log_prob(&a, &mu, &log_std);
        mean_log_prob += log_prob;
        mean_entropy += policy.entropy(obs);

        // Q gradient through reparameterized action
        let dq_da = q1.action_gradient(obs, &a);
        let q_vjp = policy.reparameterized_vjp(obs, &eps, &dq_da);
        for k in 0..n_policy_params {
            actor_grad[k] += q_vjp[k];
        }

        // Entropy bonus: push log_std up (more entropy)
        for a_idx in 0..act_dim {
            actor_grad[log_std_offset + a_idx] += alpha;
        }
    }

    for g in &mut actor_grad {
        *g /= bs;
    }
    mean_log_prob /= bs;
    mean_entropy /= bs;

    let actor_grad_norm = actor_grad.iter().map(|&g| g * g).sum::<f64>().sqrt();

    // Ascent: maximize Q(s,a) + α * entropy
    policy_optimizer.step(&actor_grad, true);
    policy.set_params(policy_optimizer.params());

    // 4. Auto-tune alpha toward target entropy (vanilla gradient descent)
    //    Minimize: -α * E[log π + H_target]
    let alpha_grad = -alpha * (mean_log_prob + target_entropy);
    let new_log_alpha = log_alpha - LR_ALPHA * alpha_grad;
    let new_alpha = new_log_alpha.exp();

    // 5. Soft-update target Q-networks
    soft_update(target_q1, q1, TAU);
    soft_update(target_q2, q2, TAU);

    SacUpdateResult {
        mean_q1,
        actor_grad_norm,
        alpha: new_alpha,
        entropy: mean_entropy,
        new_log_alpha,
    }
}

// ── Bevy Resources ─────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Warmup,
    Running,
    Paused,
}

#[derive(Resource)]
struct SacResource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,

    policy: LinearStochasticPolicy,
    q1: LinearQ,
    q2: LinearQ,
    target_q1: LinearQ,
    target_q2: LinearQ,

    policy_optimizer: Box<dyn Optimizer>,
    q1_optimizer: Box<dyn Optimizer>,
    q2_optimizer: Box<dyn Optimizer>,

    log_alpha: f64,

    replay_buffer: ReplayBuffer,
    act_dim: usize,

    phase: Phase,
    step_count: usize,
    display_epoch: usize,

    episode_rewards: Vec<f64>,
    completed_rewards: Vec<f64>,
    epoch_reached: usize,
    all_completed_rewards: Vec<f64>,

    accumulator: f64,
    sim_time: f64,
    pause_timer: f64,

    rng: rand::rngs::StdRng,

    last_mean_q1: f64,
    last_actor_grad_norm: f64,
    last_alpha: f64,
    last_entropy: f64,

    mean_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
}

#[derive(Resource, Default)]
struct SacValidation {
    reported: bool,
}

// ── Bevy App ───────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: SAC — Reaching Arm + Soft Actor-Critic ===");
    println!("  {NUM_ENVS} arms, {MAX_STEPS} steps, lr_actor={LR_ACTOR}, lr_critic={LR_CRITIC}");
    println!("  warmup={WARMUP_STEPS}, buffer={BUFFER_CAPACITY}, batch={BATCH_SIZE}");
    println!("  alpha auto-tuned (lr={LR_ALPHA}), init_log_std={INIT_LOG_STD}");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — SAC (Reaching Arm + Soft Actor-Critic)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<SacValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_sac)
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
    let act_dim = task.act_dim();
    let policy =
        LinearStochasticPolicy::new(task.obs_dim(), act_dim, task.obs_scale(), INIT_LOG_STD);
    let q1 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
    let q2 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
    let target_q1 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
    let target_q2 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());

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
    let replay_buffer = ReplayBuffer::new(BUFFER_CAPACITY, task.obs_dim(), act_dim);

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");

    let rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, act_dim]);

    commands.insert_resource(SacResource {
        vec_env,
        actions,
        current_obs: init_obs,
        policy,
        q1,
        q2,
        target_q1,
        target_q2,
        policy_optimizer,
        q1_optimizer,
        q2_optimizer,
        log_alpha: INIT_LOG_ALPHA,
        replay_buffer,
        act_dim,
        phase: Phase::Warmup,
        step_count: 0,
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
        last_actor_grad_norm: 0.0,
        last_alpha: 1.0,
        last_entropy: 0.0,
        mean_rewards: Vec::new(),
        reach_counts: Vec::new(),
    });
}

// ── Stepping (SAC state machine) ──────────────────────────────────────────

fn step_sac(mut res: ResMut<SacResource>, time: Res<Time>) {
    if res.phase == Phase::Paused {
        let wall_dt = time.delta_secs_f64();
        res.pause_timer -= wall_dt;
        if res.pause_timer <= 0.0 && res.step_count < MAX_STEPS {
            res.phase = Phase::Running;
        }
        return;
    }

    if res.step_count >= MAX_STEPS {
        return;
    }

    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;
    let dt_action = 5.0 * res.vec_env.model().timestep;

    #[allow(clippy::cast_sign_loss)]
    let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

    for _ in 0..budget {
        let is_warmup = res.phase == Phase::Warmup;

        // Select actions: random during warmup, stochastic policy during training
        let inner = &mut *res;
        let std_normal = Normal::new(0.0, 1.0).unwrap();
        let mut step_actions: Vec<Vec<f64>> = Vec::with_capacity(NUM_ENVS);

        for i in 0..NUM_ENVS {
            let obs = inner.current_obs.row(i);
            let action: Vec<f64> = if is_warmup {
                vec![
                    inner.rng.random_range(-1.0..1.0),
                    inner.rng.random_range(-1.0..1.0),
                ]
            } else {
                let (mu, log_std) = inner.policy.forward_stochastic(obs);
                mu.iter()
                    .zip(&log_std)
                    .map(|(&m, &ls)| m + ls.exp() * std_normal.sample(&mut inner.rng))
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
                    "Warmup complete ({} transitions). Starting SAC training...",
                    res.replay_buffer.len()
                );
                res.phase = Phase::Running;
                res.completed_rewards.clear();
                res.epoch_reached = 0;
            }
            continue;
        }

        // SAC update (every step after warmup)
        if res.replay_buffer.len() >= BATCH_SIZE {
            let act_dim = res.act_dim;
            let log_alpha = res.log_alpha;
            let inner = &mut *res;
            let update_result = sac_update(
                &mut inner.policy,
                &mut inner.q1,
                &mut inner.q2,
                &mut inner.target_q1,
                &mut inner.target_q2,
                inner.policy_optimizer.as_mut(),
                inner.q1_optimizer.as_mut(),
                inner.q2_optimizer.as_mut(),
                &inner.replay_buffer,
                act_dim,
                log_alpha,
                &mut inner.rng,
            );
            inner.last_mean_q1 = update_result.mean_q1;
            inner.last_actor_grad_norm = update_result.actor_grad_norm;
            inner.last_alpha = update_result.alpha;
            inner.last_entropy = update_result.entropy;
            inner.log_alpha = update_result.new_log_alpha;
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
                "epoch {:>2}: reached={reached:>2}/{NUM_ENVS}  mean_R={:>8.1}  Q1={:.2}  α={:.3}  H={:.2}",
                res.display_epoch, mean_reward, res.last_mean_q1, res.last_alpha, res.last_entropy
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

fn sync_vec_env_to_scenes(res: Res<SacResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<SacResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ────────────────────────────────────────────────────────────

fn track_validation(res: Res<SacResource>, mut val: ResMut<SacValidation>) {
    if res.display_epoch < VALIDATION_EPOCH || val.reported {
        return;
    }
    val.reported = true;

    let n = res.mean_rewards.len();
    if n < 2 {
        return;
    }

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
            name: "Alpha adapted",
            pass: (res.last_alpha - INIT_LOG_ALPHA.exp()).abs() > 0.001,
            detail: format!("α={:.4}", res.last_alpha),
        },
    ];

    let _ = print_report(&format!("SAC (epoch {VALIDATION_EPOCH})"), &checks);
}

// ── HUD ───────────────────────────────────────────────────────────────────

fn update_hud(res: Res<SacResource>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Reaching Arm (VecEnv + SAC)");
    hud.raw(format!("step: {} / {MAX_STEPS}", res.step_count));
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

    hud.raw(format!(
        "buffer: {} / {BUFFER_CAPACITY}",
        res.replay_buffer.len()
    ));
    hud.raw(format!(
        "Q1: {:.2}  |grad|: {:.4}",
        res.last_mean_q1, res.last_actor_grad_norm
    ));
    hud.raw(format!(
        "α: {:.4}  entropy: {:.2}",
        res.last_alpha, res.last_entropy
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
        entries.push(format!("ep{}:{:.0}", n, rewards[n - 1]));
        hud.raw(entries.join("  "));
    }
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Headless SAC runner ──

    const TEST_STEPS: usize = 3000;

    struct SacHeadlessResult {
        early_mean_reward: f64,
        late_mean_reward: f64,
        total_reached: usize,
        buffer_len: usize,
        param_norm: f64,
        final_alpha: f64,
    }

    #[allow(clippy::too_many_lines)]
    fn run_sac_headless(seed: u64) -> SacHeadlessResult {
        use rand::SeedableRng;

        let task = reaching_2dof();
        let act_dim = task.act_dim();
        let mut policy =
            LinearStochasticPolicy::new(task.obs_dim(), act_dim, task.obs_scale(), INIT_LOG_STD);
        let mut q1 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
        let mut q2 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
        let mut target_q1 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());
        let mut target_q2 = LinearQ::new(task.obs_dim(), act_dim, task.obs_scale());

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
        let mut log_alpha = INIT_LOG_ALPHA;

        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build");
        let mut replay_buffer = ReplayBuffer::new(BUFFER_CAPACITY, task.obs_dim(), act_dim);
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let std_normal = Normal::new(0.0, 1.0).unwrap();

        let mut episode_rewards = vec![0.0f64; NUM_ENVS];
        let mut completed_rewards: Vec<f64> = Vec::new();
        let mut total_reached = 0usize;

        let mut current_obs = vec_env.reset_all().expect("reset");
        let mut actions = Tensor::zeros(&[NUM_ENVS, act_dim]);

        for step in 0..TEST_STEPS {
            let is_warmup = step < WARMUP_STEPS;

            let mut step_actions: Vec<Vec<f64>> = Vec::with_capacity(NUM_ENVS);
            for i in 0..NUM_ENVS {
                let obs = current_obs.row(i);
                let action: Vec<f64> = if is_warmup {
                    vec![rng.random_range(-1.0..1.0), rng.random_range(-1.0..1.0)]
                } else {
                    let (mu, log_std) = policy.forward_stochastic(obs);
                    mu.iter()
                        .zip(&log_std)
                        .map(|(&m, &ls)| m + ls.exp() * std_normal.sample(&mut rng))
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

            if !is_warmup && replay_buffer.len() >= BATCH_SIZE {
                let result = sac_update(
                    &mut policy,
                    &mut q1,
                    &mut q2,
                    &mut target_q1,
                    &mut target_q2,
                    policy_optimizer.as_mut(),
                    q1_optimizer.as_mut(),
                    q2_optimizer.as_mut(),
                    &replay_buffer,
                    act_dim,
                    log_alpha,
                    &mut rng,
                );
                log_alpha = result.new_log_alpha;
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
        let final_alpha = log_alpha.exp();

        SacHeadlessResult {
            early_mean_reward: early_mean,
            late_mean_reward: late_mean,
            total_reached,
            buffer_len: replay_buffer.len(),
            param_norm,
            final_alpha,
        }
    }

    // ── Convergence ──

    #[test]
    fn sac_converges_in_3000_steps() {
        let r = run_sac_headless(42);
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
        let r = run_sac_headless(42);
        let expected_min = WARMUP_STEPS * NUM_ENVS;
        assert!(
            r.buffer_len >= expected_min,
            "buffer_len={}, want >={expected_min}",
            r.buffer_len
        );
    }

    // ── Alpha adaptation ──

    #[test]
    fn alpha_adapts_from_initial() {
        let r = run_sac_headless(42);
        // Alpha should move away from 1.0 (initial value) during training
        assert!(
            (r.final_alpha - INIT_LOG_ALPHA.exp()).abs() > 0.001,
            "alpha={:.4}, should have adapted from {:.4}",
            r.final_alpha,
            r.final_alpha
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

    // ── Multi-seed ──

    #[test]
    fn multi_seed_robustness() {
        let mut converged = 0;
        for seed in [42, 123, 999] {
            let r = run_sac_headless(seed);
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
