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

use bevy::prelude::*;
use example_ml_shared::{setup_reaching_arms, sync_batch_geoms};
use rand_distr::{Distribution, Normal};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{
    DifferentiablePolicy, LinearPolicy, Optimizer, OptimizerConfig, Policy, Tensor, Trajectory,
    VecEnv, reaching_2dof,
};

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 50;
const PAUSE_TIME: f64 = 1.5;
const MAX_EPOCHS: usize = 30;
const VALIDATION_EPOCH: usize = 25;
const GAMMA: f64 = 0.99;
const LR: f64 = 0.05;
const SIGMA_INIT: f64 = 0.5;
const SIGMA_MIN: f64 = 0.05;
const SIGMA_DECAY: f64 = 0.90;
const MAX_GRAD_NORM: f64 = 1.0;

// ── REINFORCE Helpers ───────────────────────────────────────────────────────

/// Discounted returns: R_t = sum_k gamma^k * r_{t+k}
fn discounted_returns(rewards: &[f64], gamma: f64) -> Vec<f64> {
    let n = rewards.len();
    let mut returns = vec![0.0; n];
    if n == 0 {
        return returns;
    }
    returns[n - 1] = rewards[n - 1];
    for t in (0..n - 1).rev() {
        returns[t] = rewards[t] + gamma * returns[t + 1];
    }
    returns
}

struct ReinforceResult {
    grad_norm: f64,
}

fn reinforce_update(
    policy: &mut LinearPolicy,
    optimizer: &mut dyn Optimizer,
    trajectories: &[Trajectory],
    sigma: f64,
    gamma: f64,
) -> ReinforceResult {
    let n_params = policy.n_params();

    // Compute all discounted returns
    let all_returns: Vec<Vec<f64>> = trajectories
        .iter()
        .map(|t| discounted_returns(&t.rewards, gamma))
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
    let mut grad = vec![0.0f64; n_params];
    let mut n_samples = 0;
    let mut adv_idx = 0;

    for traj in trajectories {
        for t in 0..traj.rewards.len() {
            let advantage = all_advantages[adv_idx];
            adv_idx += 1;
            let g = policy.log_prob_gradient(&traj.obs[t], &traj.actions[t], sigma);
            for k in 0..n_params {
                grad[k] += g[k] * advantage;
            }
            n_samples += 1;
        }
    }

    // Average over all samples
    if n_samples > 0 {
        for g in &mut grad {
            *g /= f64::from(n_samples);
        }
    }

    let grad_norm = grad.iter().map(|&g| g * g).sum::<f64>().sqrt();

    // Optimizer step (gradient ascent — maximize expected return)
    optimizer.step(&grad, true);

    // Pitfall #1: sync optimizer params back to policy
    policy.set_params(optimizer.params());

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

    policy: LinearPolicy,
    optimizer: Box<dyn Optimizer>,
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
    let task = reaching_2dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let n_params = policy.n_params();
    let optimizer = OptimizerConfig::Adam {
        lr: LR,
        beta1: 0.9,
        beta2: 0.999,
        eps: 1e-8,
        max_grad_norm: MAX_GRAD_NORM,
    }
    .build(n_params);

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");

    let rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
    let trajectories: Vec<Trajectory> = (0..NUM_ENVS).map(|_| new_trajectory()).collect();

    commands.insert_resource(ReinforceResource {
        vec_env,
        actions,
        current_obs: init_obs,
        policy,
        optimizer,
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
                // Sample actions from stochastic policy (shared policy + Gaussian noise)
                let inner = &mut *res;
                let noise = Normal::new(0.0, inner.sigma).unwrap();
                for i in 0..NUM_ENVS {
                    if !inner.env_complete[i] {
                        let obs = inner.current_obs.row(i);
                        let mu = inner.policy.forward(obs);
                        let action: Vec<f64> = mu
                            .iter()
                            .map(|&m| m + noise.sample(&mut inner.rng))
                            .collect();

                        inner.trajectories[i].obs.push(obs.to_vec());
                        inner.trajectories[i].actions.push(action.clone());

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
                    &mut inner.policy,
                    inner.optimizer.as_mut(),
                    &trajectories,
                    sigma,
                    GAMMA,
                );
                inner.grad_norms.push(result.grad_norm);

                // Anneal sigma
                res.sigma = (res.sigma * SIGMA_DECAY).max(SIGMA_MIN);

                // Next epoch
                res.epoch += 1;
                res.current_obs = res.vec_env.reset_all().expect("reset");
                res.env_complete.fill(false);
                res.env_reached.fill(false);
                res.trajectories = (0..NUM_ENVS).map(|_| new_trajectory()).collect();
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

    let param_norm = res
        .policy
        .params()
        .iter()
        .map(|&v| v * v)
        .sum::<f64>()
        .sqrt();

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
            pass: param_norm > 0.5,
            detail: format!("||theta||={param_norm:.3}"),
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

    #[allow(clippy::too_many_lines)]
    fn run_reinforce_headless(seed: u64) -> ReinforceHeadlessResult {
        use rand::SeedableRng;

        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let n_params = policy.n_params();
        let mut optimizer = OptimizerConfig::Adam {
            lr: LR,
            beta1: 0.9,
            beta2: 0.999,
            eps: 1e-8,
            max_grad_norm: MAX_GRAD_NORM,
        }
        .build(n_params);
        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build_vec_env");
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
        let mut sigma = SIGMA_INIT;

        let mut ep1_mean_reward = 0.0;
        let mut last_mean_reward = 0.0;
        let mut last_reached = 0;
        let mut per_epoch_reached = Vec::new();

        let episode_timeout = 3.0;
        let timestep = vec_env.model().timestep;
        let max_steps = (episode_timeout / (5.0 * timestep)) as usize + 50;

        for epoch in 0..MAX_EPOCHS {
            let mut current_obs = vec_env.reset_all().expect("reset");
            let mut trajectories: Vec<Trajectory> =
                (0..NUM_ENVS).map(|_| new_trajectory()).collect();
            let mut env_complete = [false; NUM_ENVS];
            let mut env_reached = [false; NUM_ENVS];
            let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
            let noise = Normal::new(0.0, sigma).unwrap();

            for _ in 0..max_steps {
                for i in 0..NUM_ENVS {
                    if !env_complete[i] {
                        let obs = current_obs.row(i);
                        let mu = policy.forward(obs);
                        let action: Vec<f64> =
                            mu.iter().map(|&m| m + noise.sample(&mut rng)).collect();

                        trajectories[i].obs.push(obs.to_vec());
                        trajectories[i].actions.push(action.clone());

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

            reinforce_update(&mut policy, optimizer.as_mut(), &trajectories, sigma, GAMMA);
            sigma = (sigma * SIGMA_DECAY).max(SIGMA_MIN);
        }

        let param_norm = policy.params().iter().map(|&v| v * v).sum::<f64>().sqrt();

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
        assert!(
            r.last_mean_reward > r.ep1_mean_reward * 0.15,
            "last_mean_reward={:.1}, want better than {:.1}",
            r.last_mean_reward,
            r.ep1_mean_reward * 0.15
        );
    }

    #[test]
    fn all_envs_identical_on_mean_policy() {
        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let params = [0.1, -0.2, 0.3, 0.15, -0.1, 0.25, -0.05, 0.2, 0.1, -0.1];
        policy.set_params(&params);

        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build");
        let mut current_obs = vec_env.reset_all().expect("reset");
        let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);

        for _ in 0..50 {
            for i in 0..NUM_ENVS {
                let obs = current_obs.row(i);
                let mu = policy.forward(obs);
                let row = actions.row_mut(i);
                row[0] = mu[0] as f32;
                row[1] = mu[1] as f32;
            }
            let result = vec_env.step(&actions).expect("step");
            current_obs = result.observations;
        }

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

        let task = reaching_2dof();
        let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build");
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let sigma = SIGMA_INIT;
        let noise = Normal::new(0.0, sigma).unwrap();

        // Collect one epoch of trajectories
        let mut current_obs = vec_env.reset_all().expect("reset");
        let mut trajectories: Vec<Trajectory> = (0..NUM_ENVS).map(|_| new_trajectory()).collect();
        let mut env_complete = [false; NUM_ENVS];
        let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
        let episode_timeout = 3.0;
        let timestep = vec_env.model().timestep;
        let max_steps = (episode_timeout / (5.0 * timestep)) as usize + 50;

        for _ in 0..max_steps {
            for i in 0..NUM_ENVS {
                if !env_complete[i] {
                    let obs = current_obs.row(i);
                    let mu = policy.forward(obs);
                    let action: Vec<f64> = mu.iter().map(|&m| m + noise.sample(&mut rng)).collect();
                    trajectories[i].obs.push(obs.to_vec());
                    trajectories[i].actions.push(action.clone());
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
            .map(|t| discounted_returns(&t.rewards, GAMMA))
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

        let mut norms_no_baseline = Vec::new();
        let mut norms_with_baseline = Vec::new();

        for (env_idx, traj) in trajectories.iter().enumerate() {
            let returns = &all_returns[env_idx];
            for t in 0..traj.rewards.len() {
                let g = policy.log_prob_gradient(&traj.obs[t], &traj.actions[t], sigma);

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
        let rewards = vec![1.0, 2.0, 3.0];
        let returns = discounted_returns(&rewards, 0.9);
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
