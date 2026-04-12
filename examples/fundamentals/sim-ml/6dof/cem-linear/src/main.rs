//! 6-DOF Reaching Arm + CEM + Linear Policy
//!
//! 50 three-segment arms learn to reach a target via Cross-Entropy Method.
//! Same CEM algorithm as the 2-DOF example, but the arm has 3 segments and
//! 6 hinge joints. The linear policy grows from 10 params (2-DOF) to 78
//! params (6-DOF: 6×12 + 6), and CEM must search a much larger space.
//!
//! Visual story: Gen 1 — 3-segment arms flail in all directions. Gen 10–20
//! — some arms find partial solutions. Gen 30–40 — several arms reach the
//! target, though fewer than 2-DOF CEM because the search space is bigger.
//!
//! Run: `cargo run -p example-ml-6dof-cem-linear --release`

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
use example_ml_shared::{setup_reaching_6dof_arms, sync_batch_geoms};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{
    ActionSpace, LinearPolicy, ObservationSpace, Policy, Tensor, VecEnv, reaching_6dof,
};

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS: usize = 50;
const ACT_DIM: usize = 6;
const PAUSE_TIME: f64 = 1.5;
const NUM_ELITES: usize = 10;
const SIGMA_INIT: f64 = 1.0;
const SIGMA_MIN: f64 = 0.1;
const MAX_GENERATIONS: usize = 40;
const VALIDATION_GEN: usize = 30;

/// Target joint configuration (must match `reaching_6dof()`).
const TARGET_JOINTS: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];
/// Episode duration in seconds (truncation).
const EPISODE_TIME: f64 = 5.0;

// ── Forward kinematics helper ───────────────────────────────────────────────

/// Compute fingertip position at a given joint configuration.
fn fk_tip(model: &sim_core::Model, qpos: &[f64; 6]) -> [f64; 3] {
    let mut data = model.make_data();
    for (j, &q) in qpos.iter().enumerate() {
        data.qpos[j] = q;
    }
    data.forward(model).expect("FK");
    let tip = data.site_xpos[0];
    [tip.x, tip.y, tip.z]
}

// ── CEM Helpers ─────────────────────────────────────────────────────────────

fn sample_perturbations(mu: &[f64], sigma: &[f64], rng: &mut impl Rng, n: usize) -> Vec<Vec<f64>> {
    let n_params = mu.len();
    (0..n)
        .map(|_| {
            (0..n_params)
                .map(|k| {
                    let dist = Normal::new(mu[k], sigma[k]).unwrap();
                    dist.sample(rng)
                })
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
    Done,
}

#[derive(Resource)]
struct CemResource {
    vec_env: VecEnv,
    actions: Tensor,
    current_obs: Tensor,
    target_tip: [f64; 3],

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

    best_rewards: Vec<f64>,
    reach_counts: Vec<usize>,
}

#[derive(Resource, Default)]
struct CemValidation {
    reported: bool,
}

// ── Bevy App ────────────────────────────────────────────────────────────────

fn main() {
    println!("=== CortenForge: 6-DOF CEM + Linear Policy ===");
    println!("  {NUM_ENVS} arms, {MAX_GENERATIONS} generations, {NUM_ELITES} elites");
    println!("  policy: LinearPolicy (78 params = 6×12 + 6)");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — 6-DOF CEM + Linear".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<CemValidation>()
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(300.0)
                .print_every(20.0)
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
    let task = reaching_6dof();
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let n_params = policy.n_params();
    assert_eq!(n_params, 78, "6-DOF LinearPolicy should have 78 params");

    // Use shared setup for scene geometry, then replace the VecEnv with
    // a custom one that:
    //   (a) uses on_reset to hang the arm straight down (qpos[0] = pi/2).
    //       Default qpos=0 extends the arm horizontally, which sags under
    //       gravity into a visually ugly curl. A hanging pose is stable.
    //   (b) keeps the task's done/truncated/reward — just a nicer rest.
    let (throwaway_vec_env, scenes) =
        setup_reaching_6dof_arms(&task, NUM_ENVS, &mut commands, &mut meshes, &mut materials);
    let model = throwaway_vec_env.model().clone();
    drop(throwaway_vec_env);

    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs");
    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act");

    let tq = TARGET_JOINTS;
    let target_tip = fk_tip(&model, &tq);
    let mut vec_env = VecEnv::builder(model.into(), NUM_ENVS)
        .observation_space(obs_space)
        .action_space(act_space)
        .on_reset(|_m, d, _i| {
            // j1 axis is -Y; -pi/2 rotates seg1 from +X to -Z (hanging down).
            // Gravity is then aligned with the chain — no drooping.
            d.qpos[0] = -std::f64::consts::FRAC_PI_2;
        })
        .reward(move |_m, d| {
            let mut err = 0.0;
            for j in 0..6 {
                err += (d.qpos[j] - tq[j]).powi(2);
            }
            -err
        })
        // No done condition: arms that reach the target stay there (the
        // reward function pulls them toward target_joints). This avoids
        // mid-episode auto-reset, which would jerk the arm away from the
        // target it just reached. All 50 arms reset together at truncation.
        .done(|_, _| false)
        .truncated(|_m, d| d.time > EPISODE_TIME)
        .sub_steps(5)
        .build()
        .expect("vec_env build");

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
        target_tip,
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
                res.phase = Phase::Done;
                return;
            }

            let wall_dt = time.delta_secs_f64();
            res.accumulator += wall_dt;
            let dt_action = 5.0 * res.vec_env.model().timestep;

            let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

            for _ in 0..budget {
                let inner = &mut *res;
                for i in 0..NUM_ENVS {
                    if inner.gen_complete[i] {
                        continue;
                    }
                    // Once an arm has reached, freeze its actions (zero
                    // torque). Damping brings it to rest near the reach
                    // point instead of continuing the random perturbation
                    // that would drive it away from the target.
                    if inner.gen_reached[i] {
                        let row = inner.actions.row_mut(i);
                        for j in 0..ACT_DIM {
                            row[j] = 0.0;
                        }
                        continue;
                    }
                    let obs = inner.current_obs.row(i);
                    inner.policy.set_params(&inner.perturbations[i]);
                    let acts = inner.policy.forward(obs);
                    let row = inner.actions.row_mut(i);
                    for j in 0..ACT_DIM {
                        row[j] = acts[j] as f32;
                    }
                }

                let actions = res.actions.clone();
                let result = res.vec_env.step(&actions).expect("vec step");

                let target_tip_local = res.target_tip;
                for i in 0..NUM_ENVS {
                    if !res.gen_complete[i] {
                        res.cum_rewards[i] += result.rewards[i];

                        // Reach detection: fingertip within 10cm of target.
                        // (Done is disabled at task level to avoid mid-
                        // episode reset; we track reaches ourselves.)
                        if !res.gen_reached[i]
                            && let Some(env_data) = res.vec_env.batch().env(i)
                        {
                            let tip = env_data.site_xpos[0];
                            let dx = tip.x - target_tip_local[0];
                            let dy = tip.y - target_tip_local[1];
                            let dz = tip.z - target_tip_local[2];
                            let dist_sq = dx * dx + dy * dy + dz * dz;
                            if dist_sq < 0.10 * 0.10 {
                                res.gen_reached[i] = true;
                            }
                        }

                        if result.truncateds[i] {
                            res.gen_complete[i] = true;
                        }
                    }
                }

                res.current_obs = result.observations;
                res.accumulator -= dt_action;
                res.sim_time += dt_action;

                if res.gen_complete.iter().all(|&c| c) {
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
                        "gen {:>2}: reached={reached:>2}/{NUM_ENVS}  best_R={best:>8.2}  σ_mean={sigma_mean:.3}",
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
        Phase::Done => {}
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

fn track_validation(res: Res<CemResource>, mut val: ResMut<CemValidation>) {
    if res.generation < VALIDATION_GEN || val.reported {
        return;
    }
    val.reported = true;

    let n_params = res.mu.len();
    let gen_idx = VALIDATION_GEN - 1;
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
            name: "Some arms reach target (≥5)",
            pass: reached >= 5,
            detail: format!("reached={reached}/{NUM_ENVS}"),
        },
        Check {
            name: "Reward improvement ≥30%",
            pass: improvement > 0.3,
            detail: format!(
                "gen1={gen1_best:.2} → gen{VALIDATION_GEN}={gen_best:.2} ({:.0}%)",
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

    let _ = print_report(&format!("6-DOF CEM (gen {VALIDATION_GEN})"), &checks);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(res: Res<CemResource>, mut hud: ResMut<PhysicsHud>) {
    let n_params = res.mu.len();
    hud.clear();
    hud.section("6-DOF Reaching Arm (VecEnv + CEM)");
    if res.phase == Phase::Done {
        hud.raw(format!(
            "training complete ({} generations)",
            res.generation
        ));
    } else {
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
    }

    let cur_reached = res.gen_reached.iter().filter(|&&r| r).count();
    let cur_complete = res.gen_complete.iter().filter(|&&c| c).count();
    hud.raw(format!(
        "reached: {cur_reached}/{NUM_ENVS}  done: {cur_complete}/{NUM_ENVS}"
    ));

    if let Some(&best) = res.best_rewards.last() {
        hud.raw(format!("best reward: {best:.2}"));
    }

    let sigma_mean = res.sigma.iter().sum::<f64>() / n_params as f64;
    hud.raw(format!("σ mean: {sigma_mean:.3}"));
    hud.raw(format!("params: {n_params}"));

    // Per-env scoreboard (show top 8)
    if res.gen_complete.iter().any(|&c| c) {
        hud.section("Per-Env (top 8)");
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
                "{star} {i:>2} R={:>8.2}  {status}",
                res.cum_rewards[i]
            ));
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Headless env + CEM infrastructure ──

    /// Build the same custom VecEnv that `setup()` constructs, without any
    /// Bevy scene. Every test that needs to step the environment goes
    /// through this so the test path matches production.
    fn build_headless_env() -> VecEnv {
        let task = reaching_6dof();
        let throwaway = task.build_vec_env(1).expect("throwaway vec_env");
        let model = throwaway.model().clone();
        drop(throwaway);

        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .expect("obs");
        let act_space = ActionSpace::builder()
            .all_ctrl()
            .build(&model)
            .expect("act");

        let tq = TARGET_JOINTS;
        VecEnv::builder(std::sync::Arc::new(model), NUM_ENVS)
            .observation_space(obs_space)
            .action_space(act_space)
            .on_reset(|_m, d, _i| {
                d.qpos[0] = -std::f64::consts::FRAC_PI_2;
            })
            .reward(move |_m, d| {
                let mut err = 0.0;
                for j in 0..6 {
                    err += (d.qpos[j] - tq[j]).powi(2);
                }
                -err
            })
            .done(|_, _| false)
            .truncated(|_m, d| d.time > EPISODE_TIME)
            .sub_steps(5)
            .build()
            .expect("vec_env build")
    }

    struct CemResult {
        gen1_best: f64,
        last_best: f64,
        mu_norm: f64,
        sigma_mean: f64,
        gen1_reward_variance: f64,
    }

    /// Run CEM headless for `n_generations`. Mirrors `step_cem()`'s inner
    /// logic exactly (including the reach-freeze optimization), just without
    /// any Bevy systems or frame-pacing.
    #[allow(clippy::too_many_lines)]
    fn run_cem_headless(seed: u64, n_generations: usize) -> CemResult {
        use rand::SeedableRng;

        let mut vec_env = build_headless_env();
        let target_tip = {
            let model = vec_env.model();
            fk_tip(model, &TARGET_JOINTS)
        };

        let task = reaching_6dof();
        let mut policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        let n_params = policy.n_params();
        let mut rng = rand::rngs::StdRng::seed_from_u64(seed);

        let mut mu = vec![0.0f64; n_params];
        let mut sigma = vec![SIGMA_INIT; n_params];

        let mut gen1_best = f64::NEG_INFINITY;
        let mut last_best = f64::NEG_INFINITY;
        let mut gen1_reward_variance = 0.0;

        let timestep = vec_env.model().timestep;
        let dt_action = 5.0 * timestep;
        let max_steps = (EPISODE_TIME / dt_action) as usize + 10;

        for generation in 0..n_generations {
            let perturbations = sample_perturbations(&mu, &sigma, &mut rng, NUM_ENVS);
            let init_obs = vec_env.reset_all().expect("reset");

            let mut cum_rewards = [0.0f64; NUM_ENVS];
            let mut gen_complete = [false; NUM_ENVS];
            let mut gen_reached = [false; NUM_ENVS];
            let mut actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
            let mut current_obs = init_obs;

            for _ in 0..max_steps {
                for i in 0..NUM_ENVS {
                    if gen_complete[i] {
                        continue;
                    }
                    if gen_reached[i] {
                        let row = actions.row_mut(i);
                        for j in 0..ACT_DIM {
                            row[j] = 0.0;
                        }
                        continue;
                    }
                    let obs = current_obs.row(i);
                    policy.set_params(&perturbations[i]);
                    let acts = policy.forward(obs);
                    let row = actions.row_mut(i);
                    for j in 0..ACT_DIM {
                        row[j] = acts[j] as f32;
                    }
                }

                let result = vec_env.step(&actions).expect("step");

                for i in 0..NUM_ENVS {
                    if !gen_complete[i] {
                        cum_rewards[i] += result.rewards[i];

                        if !gen_reached[i]
                            && let Some(env_data) = vec_env.batch().env(i)
                        {
                            let tip = env_data.site_xpos[0];
                            let dx = tip.x - target_tip[0];
                            let dy = tip.y - target_tip[1];
                            let dz = tip.z - target_tip[2];
                            let dist_sq = dx * dx + dy * dy + dz * dz;
                            if dist_sq < 0.10 * 0.10 {
                                gen_reached[i] = true;
                            }
                        }

                        if result.truncateds[i] {
                            gen_complete[i] = true;
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
            gen1_reward_variance,
        }
    }

    // ── Assumption tests (cheap — no training loop) ──

    #[test]
    fn linear_policy_has_78_params() {
        let task = reaching_6dof();
        let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
        assert_eq!(
            policy.n_params(),
            78,
            "6-DOF LinearPolicy should have 6×12 + 6 = 78 params"
        );
    }

    #[test]
    fn obs_scale_matches_readme() {
        let task = reaching_6dof();
        let scale = task.obs_scale();
        assert_eq!(scale.len(), 12, "obs_scale length");
        let inv_pi = 1.0 / std::f64::consts::PI;
        for (i, &s) in scale.iter().take(6).enumerate() {
            assert!(
                (s - inv_pi).abs() < 1e-10,
                "qpos scale[{i}]={s}, expected 1/π = {inv_pi}"
            );
        }
        for (i, &s) in scale.iter().skip(6).take(6).enumerate() {
            assert!(
                (s - 0.1).abs() < 1e-10,
                "qvel scale[{}]={s}, expected 0.1",
                i + 6
            );
        }
    }

    #[test]
    fn fk_tip_is_deterministic() {
        let task = reaching_6dof();
        let vec_env = task.build_vec_env(1).expect("build");
        let model = vec_env.model();
        let zeros = [0.0f64; 6];
        let a = fk_tip(model, &zeros);
        let b = fk_tip(model, &zeros);
        assert_eq!(a, b, "fk_tip should be deterministic");
    }

    #[test]
    fn fk_tip_at_target_within_arm_reach() {
        let task = reaching_6dof();
        let vec_env = task.build_vec_env(1).expect("build");
        let model = vec_env.model();
        let tip = fk_tip(model, &TARGET_JOINTS);
        assert!(
            tip[0].is_finite() && tip[1].is_finite() && tip[2].is_finite(),
            "tip must be finite: {tip:?}"
        );
        // Max reach = seg1 + seg2 + seg3 = 0.30 + 0.25 + 0.20 = 0.75 m
        let norm = (tip[0] * tip[0] + tip[1] * tip[1] + tip[2] * tip[2]).sqrt();
        assert!(
            norm < 0.75 + 1e-6,
            "tip norm {norm:.4} exceeds max arm reach 0.75"
        );
    }

    #[test]
    fn sub_steps_give_100hz_control() {
        let mut vec_env = build_headless_env();
        vec_env.reset_all().expect("reset");
        let task = reaching_6dof();
        let actions = Tensor::zeros(&[NUM_ENVS, task.act_dim()]);
        vec_env.step(&actions).expect("step");
        let time = vec_env.batch().env(0).expect("env").time;
        assert!(
            (time - 0.01).abs() < 1e-10,
            "time={time}, expected 0.01 (5 sub-steps × 0.002 dt = 100 Hz)"
        );
    }

    #[test]
    fn hanging_reset_sets_j1_to_negative_half_pi() {
        let mut vec_env = build_headless_env();
        vec_env.reset_all().expect("reset");
        let data = vec_env.batch().env(0).expect("env");
        assert!(
            (data.qpos[0] - (-std::f64::consts::FRAC_PI_2)).abs() < 1e-10,
            "qpos[0]={}, expected -π/2",
            data.qpos[0]
        );
        for j in 1..6 {
            assert!(
                data.qpos[j].abs() < 1e-10,
                "qpos[{j}]={} (should be 0)",
                data.qpos[j]
            );
        }
    }

    // ── CEM math unit tests (cheap — no env) ──

    #[test]
    fn sample_perturbations_has_correct_shape() {
        use rand::SeedableRng;
        let mu = vec![0.0f64; 78];
        let sigma = vec![1.0f64; 78];
        let mut rng = rand::rngs::StdRng::seed_from_u64(42);
        let perturbations = sample_perturbations(&mu, &sigma, &mut rng, NUM_ENVS);
        assert_eq!(perturbations.len(), NUM_ENVS);
        for p in &perturbations {
            assert_eq!(p.len(), 78);
            for v in p {
                assert!(v.is_finite(), "perturbation value not finite: {v}");
            }
        }
    }

    #[test]
    fn cem_update_sigma_clamps_to_min() {
        // All perturbations identical → variance = 0 → sigma must clamp.
        let mut mu = vec![0.0f64; 78];
        let mut sigma = vec![1.0f64; 78];
        let constant = vec![0.5f64; 78];
        let perturbations = vec![constant; NUM_ENVS];
        let rewards = vec![1.0f64; NUM_ENVS];
        cem_update(&mut mu, &mut sigma, &perturbations, &rewards, NUM_ELITES);
        for (i, &s) in sigma.iter().enumerate() {
            assert!(
                (s - SIGMA_MIN).abs() < 1e-10,
                "sigma[{i}]={s}, expected SIGMA_MIN={SIGMA_MIN}"
            );
        }
    }

    #[test]
    fn cem_update_mean_equals_elite_mean() {
        // Top-NUM_ELITES have value 2.0; rest have value -5.0. Elite mean
        // should be exactly 2.0, and new μ should equal that.
        let mut mu = vec![0.0f64; 78];
        let mut sigma = vec![1.0f64; 78];
        let mut perturbations = Vec::with_capacity(NUM_ENVS);
        let mut rewards = Vec::with_capacity(NUM_ENVS);
        for i in 0..NUM_ENVS {
            if i < NUM_ELITES {
                perturbations.push(vec![2.0f64; 78]);
                rewards.push(100.0);
            } else {
                perturbations.push(vec![-5.0f64; 78]);
                rewards.push(-1000.0);
            }
        }
        cem_update(&mut mu, &mut sigma, &perturbations, &rewards, NUM_ELITES);
        for (i, &m) in mu.iter().enumerate() {
            assert!(
                (m - 2.0).abs() < 1e-10,
                "mu[{i}]={m}, expected elite mean 2.0"
            );
        }
    }

    // ── Convergence tests (expensive — full training) ──

    #[test]
    fn cem_makes_progress_in_12_generations() {
        let r = run_cem_headless(42, 12);
        let improvement = (r.last_best - r.gen1_best) / r.gen1_best.abs();
        assert!(
            improvement > 0.15,
            "CEM should show ≥15% improvement in 12 gens (seed=42), got {:.1}% \
             (gen1={:.2}, last={:.2})",
            improvement * 100.0,
            r.gen1_best,
            r.last_best
        );
        assert!(
            r.mu_norm > 0.1,
            "policy should move off origin, got ‖μ‖={:.3}",
            r.mu_norm
        );
        assert!(
            r.sigma_mean < SIGMA_INIT,
            "sigma should decrease from SIGMA_INIT={SIGMA_INIT}, got mean={:.3}",
            r.sigma_mean
        );
    }

    #[test]
    fn cem_converges_across_multiple_seeds() {
        // 50 envs is tight for 78 params (0.64 ratio), so not every seed
        // will make visible progress in 12 gens. Accept 2/3 as the bar.
        let mut successes = 0;
        for seed in [42, 123, 999] {
            let r = run_cem_headless(seed, 12);
            let improvement = (r.last_best - r.gen1_best) / r.gen1_best.abs();
            if improvement > 0.1 && r.mu_norm > 0.1 {
                successes += 1;
            }
        }
        assert!(
            successes >= 2,
            "CEM should show progress on ≥2/3 seeds, got {successes}/3"
        );
    }

    #[test]
    fn gen1_rewards_have_dynamic_range() {
        // With SIGMA_INIT=1.0, gen 1 perturbations span a wide range, so
        // per-env rewards should vary. Zero variance would mean all 50 arms
        // produce identical rewards — evidence of a degenerate task or
        // saturated policy.
        let r = run_cem_headless(42, 1);
        assert!(
            r.gen1_reward_variance > 0.1,
            "gen 1 reward variance should be > 0.1, got {:.4}",
            r.gen1_reward_variance
        );
    }
}
