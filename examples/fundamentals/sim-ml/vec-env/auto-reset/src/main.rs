//! Auto-Reset — Reaching Arm + CEM
//!
//! 50 two-link arms learn to reach a target using Cross-Entropy Method.
//! Each generation, 50 envs run different policy perturbations. Arms that
//! reach the target trigger `done` → auto-reset. After all complete, CEM
//! updates the policy toward the best performers.
//!
//! The visual story: generation 1 is chaos (arms flailing). By generation
//! 10–15, arms reach roughly the right direction. By generation 25–30,
//! most arms smoothly reach and park at the green target.
//!
//! Run: `cargo run -p example-ml-vec-env-auto-reset --release`

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
use rand::Rng;
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{LinearPolicy, Policy, Tensor, VecEnv, reaching_2dof};

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 50;
const PAUSE_TIME: f64 = 1.5;
const NUM_ELITES: usize = 15;
const SIGMA_INIT: f64 = 1.0;
const SIGMA_MIN: f64 = 0.1;
const MAX_GENERATIONS: usize = 30;
const VALIDATION_GEN: usize = 25;

// ── CEM Helpers ─────────────────────────────────────────────────────────────

/// Box-Muller normal sample.
fn randn(rng: &mut impl Rng) -> f64 {
    let u1: f64 = 1.0 - rng.random::<f64>();
    let u2: f64 = rng.random::<f64>();
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

fn sample_perturbations(mu: &[f64], sigma: &[f64], rng: &mut impl Rng, n: usize) -> Vec<Vec<f64>> {
    let n_params = mu.len();
    (0..n)
        .map(|_| {
            (0..n_params)
                .map(|k| mu[k] + sigma[k] * randn(rng))
                .collect()
        })
        .collect()
}

fn cem_update(
    mu: &mut [f64],
    sigma: &mut [f64],
    perturbations: &[Vec<f64>],
    rewards: &[f64],
    n_elites: usize,
) {
    let n = rewards.len();
    let n_params = mu.len();
    let mut indices: Vec<usize> = (0..n).collect();
    indices.sort_by(|&a, &b| rewards[b].partial_cmp(&rewards[a]).unwrap());
    let elite_idx = &indices[..n_elites];

    for k in 0..n_params {
        let elite_vals: Vec<f64> = elite_idx.iter().map(|&i| perturbations[i][k]).collect();
        let mean = elite_vals.iter().sum::<f64>() / n_elites as f64;
        let var = elite_vals.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n_elites as f64;
        mu[k] = mean;
        sigma[k] = var.sqrt().max(SIGMA_MIN);
    }
}

// ── Bevy Resources ──────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Running,
    Updating,
}

#[derive(Resource)]
struct CemResource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,

    policy: LinearPolicy,
    mu: Vec<f64>,
    sigma: Vec<f64>,
    perturbations: Vec<Vec<f64>>,

    generation: usize,
    phase: Phase,
    gen_complete: Vec<bool>,
    gen_reached: Vec<bool>,
    cum_rewards: Vec<f64>,

    accumulator: f64,
    sim_time: f64,
    pause_timer: f64,

    rng: rand::rngs::StdRng,

    // History (per generation)
    best_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
}

#[derive(Resource, Default)]
struct AutoResetValidation {
    reported: bool,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: Auto-Reset — Reaching Arm + CEM ===");
    println!("  {NUM_ENVS} arms, {MAX_GENERATIONS} generations, {NUM_ELITES} elites");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Auto-Reset (Reaching Arm + CEM)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<AutoResetValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_cem)
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

// ── Setup ───────────────────────────────────────────────────────────────────

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let task = reaching_2dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let n_params = policy.n_params();

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");

    let mut rng = {
        use rand::SeedableRng;
        rand::rngs::StdRng::seed_from_u64(42)
    };
    let mu = vec![0.0f64; n_params];
    let sigma = vec![SIGMA_INIT; n_params];
    let perturbations = sample_perturbations(&mu, &sigma, &mut rng, NUM_ENVS);

    commands.insert_resource(scenes);

    let actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
    commands.insert_resource(CemResource {
        vec_env,
        actions,
        current_obs: init_obs,
        policy,
        mu,
        sigma,
        perturbations,
        generation: 0,
        phase: Phase::Running,
        gen_complete: vec![false; NUM_ENVS],
        gen_reached: vec![false; NUM_ENVS],
        cum_rewards: vec![0.0; NUM_ENVS],
        accumulator: 0.0,
        sim_time: 0.0,
        pause_timer: 0.0,
        rng,
        best_rewards: Vec::new(),
        reach_counts: Vec::new(),
    });
}

// ── Stepping (CEM state machine) ────────────────────────────────────────────

fn step_cem(mut res: ResMut<CemResource>, time: Res<Time>) {
    match res.phase {
        Phase::Running => {
            if res.generation >= MAX_GENERATIONS {
                return;
            }

            let wall_dt = time.delta_secs_f64();
            res.accumulator += wall_dt;
            let dt_action = 5.0 * res.vec_env.model().timestep;

            #[allow(clippy::cast_sign_loss)]
            let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

            for _ in 0..budget {
                // Compute actions from perturbed policies
                let inner = &mut *res;
                for i in 0..NUM_ENVS {
                    if !inner.gen_complete[i] {
                        let obs = inner.current_obs.row(i);
                        inner.policy.set_params(&inner.perturbations[i]);
                        let acts = inner.policy.forward(obs);
                        let row = inner.actions.row_mut(i);
                        row[0] = acts[0] as f32;
                        row[1] = acts[1] as f32;
                    }
                }

                let actions = res.actions.clone();
                let result = res.vec_env.step(&actions).expect("vec step");

                for i in 0..NUM_ENVS {
                    if !res.gen_complete[i] {
                        res.cum_rewards[i] += result.rewards[i];
                        if result.dones[i] || result.truncateds[i] {
                            res.gen_complete[i] = true;
                            if result.dones[i] {
                                res.gen_reached[i] = true;
                            }
                        }
                    }
                }

                res.current_obs = result.observations;
                res.accumulator -= dt_action;
                res.sim_time += dt_action;

                if res.gen_complete.iter().all(|&c| c) {
                    // Record stats and transition to UPDATING
                    let n_params = res.mu.len();
                    let best = res
                        .cum_rewards
                        .iter()
                        .copied()
                        .fold(f64::NEG_INFINITY, f64::max);
                    let reached = res.gen_reached.iter().filter(|&&r| r).count();
                    res.best_rewards.push(best);
                    res.reach_counts.push(reached);

                    let sigma_mean = res.sigma.iter().sum::<f64>() / n_params as f64;
                    println!(
                        "gen {:>2}: reached={reached:>2}/{NUM_ENVS}  best_R={best:>8.1}  σ_mean={sigma_mean:.3}",
                        res.generation + 1
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
                // CEM update — destructure for disjoint field borrows
                let perturbations = res.perturbations.clone();
                let cum_rewards = res.cum_rewards.clone();
                let inner = &mut *res;
                cem_update(
                    &mut inner.mu,
                    &mut inner.sigma,
                    &perturbations,
                    &cum_rewards,
                    NUM_ELITES,
                );

                // Next generation
                let mu = res.mu.clone();
                let sigma = res.sigma.clone();
                res.generation += 1;
                res.perturbations = sample_perturbations(&mu, &sigma, &mut res.rng, NUM_ENVS);
                res.current_obs = res.vec_env.reset_all().expect("reset");
                res.cum_rewards.fill(0.0);
                res.gen_complete.fill(false);
                res.gen_reached.fill(false);
                res.phase = Phase::Running;
            }
        }
    }
}

// ── Sync ────────────────────────────────────────────────────────────────────

fn sync_vec_env_to_scenes(res: Res<CemResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<CemResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ──────────────────────────────────────────────────────────────

fn track_validation(res: Res<CemResource>, mut val: ResMut<AutoResetValidation>) {
    // Report after VALIDATION_GEN generations complete
    if res.generation < VALIDATION_GEN || val.reported {
        return;
    }
    val.reported = true;

    let n_params = res.mu.len();
    let gen_idx = VALIDATION_GEN - 1; // 0-indexed
    let reached = if gen_idx < res.reach_counts.len() {
        res.reach_counts[gen_idx]
    } else {
        0
    };
    let gen1_best = res
        .best_rewards
        .first()
        .copied()
        .unwrap_or(f64::NEG_INFINITY);
    let gen_best = if gen_idx < res.best_rewards.len() {
        res.best_rewards[gen_idx]
    } else {
        f64::NEG_INFINITY
    };
    let mu_norm = res.mu.iter().map(|&v| v * v).sum::<f64>().sqrt();
    let sigma_mean = res.sigma.iter().sum::<f64>() / n_params as f64;

    let improvement = if gen1_best.abs() > 1e-10 {
        (gen_best - gen1_best) / gen1_best.abs()
    } else {
        0.0
    };

    let checks = [
        Check {
            name: "Convergence (≥10 reached)",
            pass: reached >= 10,
            detail: format!("reached={reached}/{NUM_ENVS}"),
        },
        Check {
            name: "Reward improvement ≥50%",
            pass: improvement > 0.5,
            detail: format!(
                "gen1={gen1_best:.1} → gen{VALIDATION_GEN}={gen_best:.1} ({:.0}%)",
                improvement * 100.0
            ),
        },
        Check {
            name: "Policy shifted (μ norm > 0.5)",
            pass: mu_norm > 0.5,
            detail: format!("‖μ‖={mu_norm:.3}"),
        },
        Check {
            name: "σ decreased (< 0.5)",
            pass: sigma_mean < 0.5,
            detail: format!("mean(σ)={sigma_mean:.3}"),
        },
    ];

    let _ = print_report(&format!("Auto-Reset CEM (gen {VALIDATION_GEN})"), &checks);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(res: Res<CemResource>, mut hud: ResMut<PhysicsHud>) {
    let n_params = res.mu.len();
    hud.clear();
    hud.section("Reaching Arm (VecEnv + CEM)");
    hud.raw(format!(
        "generation: {} / {MAX_GENERATIONS}",
        res.generation + 1
    ));
    hud.raw(format!(
        "phase: {:?}{}",
        res.phase,
        if res.phase == Phase::Updating {
            format!(" ({:.1}s)", res.pause_timer.max(0.0))
        } else {
            String::new()
        }
    ));

    let cur_reached = res.gen_reached.iter().filter(|&&r| r).count();
    let cur_complete = res.gen_complete.iter().filter(|&&c| c).count();
    hud.raw(format!(
        "reached: {cur_reached}/{NUM_ENVS}  done: {cur_complete}/{NUM_ENVS}"
    ));

    if let Some(&best) = res.best_rewards.last() {
        hud.raw(format!("best reward: {best:.1}"));
    }

    let sigma_mean = res.sigma.iter().sum::<f64>() / n_params as f64;
    hud.raw(format!("σ mean: {sigma_mean:.3}"));

    // Per-env scoreboard (show top 8)
    if res.gen_complete.iter().any(|&c| c) {
        hud.section("Per-Env (sample)");
        let mut sorted: Vec<usize> = (0..NUM_ENVS).collect();
        sorted.sort_by(|&a, &b| res.cum_rewards[b].partial_cmp(&res.cum_rewards[a]).unwrap());

        for (rank, &i) in sorted.iter().take(8).enumerate() {
            let status = if res.gen_reached[i] {
                "DONE"
            } else if res.gen_complete[i] {
                "TRUNC"
            } else {
                "..."
            };
            let star = if rank < NUM_ELITES { "★" } else { " " };
            hud.raw(format!(
                "{star} {i:>2} R={:>8.1}  {status}",
                res.cum_rewards[i]
            ));
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use sim_ml_bridge::reaching_2dof;

    // ── Headless CEM runner for tests ──

    struct CemResult {
        gen1_best: f64,
        last_best: f64,
        mu_norm: f64,
        sigma_mean: f64,
        last_gen_reached: usize,
        terminal_obs_all_some: bool,
        reset_obs_near_zero: bool,
        reward_range: f64,
        gen1_reward_variance: f64,
    }

    #[allow(clippy::too_many_lines)]
    fn run_cem_headless(seed: u64) -> CemResult {
        use rand::SeedableRng;

        let task = reaching_2dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let n_params = policy.n_params();
        let mut vec_env = task.build_vec_env(NUM_ENVS).expect("build_vec_env");
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

        let mut mu = vec![0.0f64; n_params];
        let mut sigma = vec![SIGMA_INIT; n_params];

        let mut gen1_best = f64::NEG_INFINITY;
        let mut last_best = f64::NEG_INFINITY;
        let mut last_gen_reached = 0;
        let mut terminal_obs_all_some = true;
        let mut reset_obs_near_zero = true;
        let mut min_reward = f64::INFINITY;
        let mut max_reward = f64::NEG_INFINITY;
        let mut gen1_reward_variance = 0.0;

        let episode_timeout = 3.0;
        let timestep = vec_env.model().timestep;
        let max_steps = (episode_timeout / (5.0 * timestep)) as usize + 50;

        for generation in 0..MAX_GENERATIONS {
            let perturbations = sample_perturbations(&mu, &sigma, &mut rng, NUM_ENVS);
            let init_obs = vec_env.reset_all().expect("reset");

            for i in 0..NUM_ENVS {
                let row = init_obs.row(i);
                if row[0].abs() > 0.01 || row[1].abs() > 0.01 {
                    reset_obs_near_zero = false;
                }
            }

            let mut cum_rewards = vec![0.0f64; NUM_ENVS];
            let mut gen_complete = [false; NUM_ENVS];
            let mut reached = [false; NUM_ENVS];
            let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
            let mut current_obs = init_obs;

            for _ in 0..max_steps {
                for i in 0..NUM_ENVS {
                    if !gen_complete[i] {
                        let obs = current_obs.row(i);
                        policy.set_params(&perturbations[i]);
                        let acts = policy.forward(obs);
                        let row = actions.row_mut(i);
                        row[0] = acts[0] as f32;
                        row[1] = acts[1] as f32;
                    }
                }

                let result = vec_env.step(&actions).expect("step");

                for i in 0..NUM_ENVS {
                    if !gen_complete[i] {
                        cum_rewards[i] += result.rewards[i];
                        if result.dones[i] || result.truncateds[i] {
                            gen_complete[i] = true;
                            if result.dones[i] {
                                reached[i] = true;
                            }
                            if result.terminal_observations[i].is_none() {
                                terminal_obs_all_some = false;
                            }
                        }
                    }
                }

                current_obs = result.observations;
                if gen_complete.iter().all(|&c| c) {
                    break;
                }
            }

            let gen_best = cum_rewards
                .iter()
                .copied()
                .fold(f64::NEG_INFINITY, f64::max);
            let gen_worst = cum_rewards.iter().copied().fold(f64::INFINITY, f64::min);

            if generation == 0 {
                gen1_best = gen_best;
                let mean_r = cum_rewards.iter().sum::<f64>() / NUM_ENVS as f64;
                gen1_reward_variance = cum_rewards
                    .iter()
                    .map(|&r| (r - mean_r).powi(2))
                    .sum::<f64>()
                    / NUM_ENVS as f64;
            }

            last_best = gen_best;
            last_gen_reached = reached.iter().filter(|&&r| r).count();
            min_reward = min_reward.min(gen_worst);
            max_reward = max_reward.max(gen_best);

            cem_update(
                &mut mu,
                &mut sigma,
                &perturbations,
                &cum_rewards,
                NUM_ELITES,
            );
        }

        let mu_norm = mu.iter().map(|&v| v * v).sum::<f64>().sqrt();
        let sigma_mean = sigma.iter().sum::<f64>() / n_params as f64;

        CemResult {
            gen1_best,
            last_best,
            mu_norm,
            sigma_mean,
            last_gen_reached,
            terminal_obs_all_some,
            reset_obs_near_zero,
            reward_range: max_reward - min_reward,
            gen1_reward_variance,
        }
    }

    // ── 8 assumption tests ──

    #[test]
    fn site_xpos_populated_after_forward() {
        let task = reaching_2dof();
        let vec_env = task.build_vec_env(1).expect("build");
        let model = vec_env.model().clone();
        let mut data = model.make_data();
        data.reset(&model);
        data.forward(&model).expect("forward");
        let tip = data.site_xpos[0];
        assert!((tip.x - 0.9).abs() < 0.02, "tip.x={:.4}", tip.x);
        assert!(tip.y.abs() < 0.01, "tip.y={:.4}", tip.y);
        assert!(tip.z.abs() < 0.01, "tip.z={:.4}", tip.z);
    }

    #[test]
    fn reset_restores_qpos_to_zero() {
        let task = reaching_2dof();
        let vec_env = task.build_vec_env(1).expect("build");
        let model = vec_env.model().clone();
        let mut data = model.make_data();
        data.ctrl[0] = 1.0;
        data.ctrl[1] = 1.0;
        for _ in 0..200 {
            data.step(&model).expect("step");
        }
        data.reset(&model);
        assert_eq!(data.qpos[0], 0.0);
        assert_eq!(data.qpos[1], 0.0);
    }

    #[test]
    fn gravity_makes_arm_fall() {
        let task = reaching_2dof();
        let vec_env = task.build_vec_env(1).expect("build");
        let model = vec_env.model().clone();
        let mut data = model.make_data();
        data.reset(&model);
        data.forward(&model).expect("forward");
        let initial_z = data.site_xpos[0].z;
        for _ in 0..500 {
            data.step(&model).expect("step");
        }
        assert!(data.site_xpos[0].z < initial_z - 0.1);
    }

    #[test]
    fn cem_converges_in_30_generations() {
        let r = run_cem_headless(42);
        assert!(r.last_gen_reached >= 10, "reached={}", r.last_gen_reached);
        let improvement = (r.last_best - r.gen1_best) / r.gen1_best.abs();
        assert!(improvement > 0.5, "improvement={:.0}%", improvement * 100.0);
        assert!(r.mu_norm > 0.5, "mu_norm={:.3}", r.mu_norm);
        assert!(r.sigma_mean < 0.5, "sigma={:.3}", r.sigma_mean);
    }

    #[test]
    fn fifty_envs_sufficient_for_ten_params() {
        let mut converged = 0;
        for seed in [42, 123, 999] {
            let r = run_cem_headless(seed);
            if r.last_gen_reached >= 10 && r.mu_norm > 0.5 {
                converged += 1;
            }
        }
        assert!(converged >= 2, "converged {converged}/3");
    }

    #[test]
    fn tanh_saturation_preserves_gradient() {
        let r = run_cem_headless(42);
        assert!(
            r.gen1_reward_variance > 1.0,
            "var={:.3}",
            r.gen1_reward_variance
        );
    }

    #[test]
    fn sub_steps_give_100hz_control() {
        let task = reaching_2dof();
        let mut vec_env = task.build_vec_env(1).expect("build");
        vec_env.reset_all().expect("reset");
        let actions = Tensor::zeros(&[1, task.act_dim()]);
        vec_env.step(&actions).expect("step");
        let time = vec_env.batch().env(0).unwrap().time;
        assert!((time - 0.01).abs() < 1e-10, "time={time}");
    }

    #[test]
    fn episode_timeout_dynamic_range() {
        let r = run_cem_headless(42);
        assert!(r.reward_range > 50.0, "range={:.1}", r.reward_range);
    }
}
