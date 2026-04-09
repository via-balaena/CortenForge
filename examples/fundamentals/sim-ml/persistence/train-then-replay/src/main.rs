//! Train-then-Replay — Policy Persistence Proof
//!
//! Trains CEM on reaching-2dof headless (~20 epochs), saves a
//! PolicyArtifact to disk, reconstructs a live policy from the artifact,
//! then visualizes a single arm executing the learned policy in Bevy.
//!
//! This is the user-facing proof that the persistence system works:
//! train once, save, reconstruct, replay — all in one binary.
//!
//! Run: `cargo run -p example-ml-persistence-train-then-replay --release`

#![allow(
    clippy::doc_markdown,
    clippy::expect_used,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::needless_pass_by_value,
    clippy::let_underscore_must_use,
    clippy::similar_names,
    clippy::unwrap_used,
    dead_code
)]

use std::collections::BTreeMap;

use bevy::prelude::*;
use example_ml_shared::{setup_reaching_arms, sync_batch_geoms};
use sim_bevy::camera::OrbitCameraPlugin;
use sim_bevy::examples::{PhysicsHud, ValidationHarness, render_physics_hud, validation_system};
use sim_bevy::multi_scene::{PhysicsScenes, sync_scene_geom_transforms};
use sim_core::validation::{Check, print_report};
use sim_ml_bridge::{
    Algorithm, Cem, CemHyperparams, LinearPolicy, Policy, Tensor, TrainingBudget,
    TrainingProvenance, VecEnv, reaching_2dof,
};

// ── Constants ───────────────────────────────────────────────────────────────

const NUM_ENVS_TRAIN: usize = 50;
const NUM_EPOCHS: usize = 20;
const SEED: u64 = 42;
const ARTIFACT_PATH: &str = "trained_reaching_2dof.artifact.json";

// ── Bevy Resources ──────────────────────────────────────────────────────────

/// Staging resource: holds the trained policy until the setup system moves it out.
#[derive(Resource)]
struct TrainedPolicyInput(Option<Box<dyn Policy>>);

/// Training metadata for the HUD.
#[derive(Resource, Clone)]
struct TrainingInfo {
    epochs: usize,
    final_reward: f64,
    final_dones: usize,
    wall_time_ms: u64,
    artifact_path: String,
    artifact_saved: bool,
    artifact_reconstructed: bool,
}

/// Replay state — pure inference, no training logic.
#[derive(Resource)]
struct ReplayResource {
    vec_env: VecEnv,
    policy: Box<dyn Policy>,
    actions: Tensor,
    current_obs: Tensor,
    accumulator: f64,
    sim_time: f64,
    step_count: usize,
    episode_count: usize,
    cum_reward: f64,
    episode_reward: f64,
    episode_reached: bool,
}

#[derive(Resource, Default)]
struct ReplayValidation {
    reported: bool,
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// ISO 8601 timestamp from system clock (no external deps).
///
/// Uses Howard Hinnant's civil-date algorithm for epoch → calendar conversion.
#[allow(clippy::cast_possible_wrap, clippy::cast_sign_loss)]
fn now_iso8601() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();

    let sec = secs % 60;
    let min = (secs / 60) % 60;
    let hour = (secs / 3600) % 24;

    let z = (secs / 86400) as i64 + 719_468;
    let era = (if z >= 0 { z } else { z - 146_096 }) / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let year = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let day = doy - (153 * mp + 2) / 5 + 1;
    let month = if mp < 10 { mp + 3 } else { mp - 9 };
    let year = if month <= 2 { year + 1 } else { year };

    format!("{year:04}-{month:02}-{day:02}T{hour:02}:{min:02}:{sec:02}Z")
}

// ── main ────────────────────────────────────────────────────────────────────

#[allow(clippy::too_many_lines)]
fn main() {
    println!("=== CortenForge: Train-then-Replay — Policy Persistence ===\n");

    // ── Phase 1: Headless training ──────────────────────────────────────
    println!(
        "Phase 1: Training CEM on reaching-2dof ({NUM_EPOCHS} epochs, {NUM_ENVS_TRAIN} envs)...\n"
    );

    let task = reaching_2dof();
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let hp = CemHyperparams {
        elite_fraction: 0.2,
        noise_std: 0.3,
        noise_decay: 0.95,
        noise_min: 0.01,
        max_episode_steps: 300,
    };
    let mut cem = Cem::new(policy, hp);
    let mut env = task.build_vec_env(NUM_ENVS_TRAIN).expect("build_vec_env");

    let t0 = std::time::Instant::now();
    let metrics = cem.train(&mut env, TrainingBudget::Epochs(NUM_EPOCHS), SEED, &|m| {
        println!(
            "  epoch {:>2}: reward={:>8.2}  dones={:>2}  {}ms",
            m.epoch, m.mean_reward, m.done_count, m.wall_time_ms
        );
    });
    let wall_time_ms = t0.elapsed().as_millis() as u64;

    let final_metrics = metrics.last().expect("at least one epoch");
    println!(
        "\nTraining complete: {NUM_EPOCHS} epochs, {:.1}s\n",
        wall_time_ms as f64 / 1000.0
    );

    // ── Phase 2: Extract artifact with provenance ───────────────────────
    let artifact = cem.policy_artifact().with_provenance(TrainingProvenance {
        algorithm: "CEM".into(),
        task: "reaching-2dof".into(),
        seed: SEED,
        epochs_trained: metrics.len(),
        final_reward: final_metrics.mean_reward,
        total_steps: metrics.iter().map(|m| m.total_steps).sum(),
        wall_time_ms,
        timestamp: now_iso8601(),
        hyperparams: BTreeMap::from([
            ("elite_fraction".into(), hp.elite_fraction),
            ("noise_std".into(), hp.noise_std),
            ("noise_decay".into(), hp.noise_decay),
            ("noise_min".into(), hp.noise_min),
            ("max_episode_steps".into(), hp.max_episode_steps as f64),
        ]),
        metrics: metrics.clone(),
        parent: None,
    });

    // ── Phase 3: Save to disk ───────────────────────────────────────────
    let artifact_saved = match artifact.save(ARTIFACT_PATH) {
        Ok(()) => {
            println!("Artifact saved: {ARTIFACT_PATH}");
            true
        }
        Err(e) => {
            eprintln!("Warning: failed to save artifact: {e}");
            false
        }
    };

    // ── Phase 4: Reconstruct from artifact ──────────────────────────────
    let replay_policy = artifact
        .to_policy()
        .expect("reconstruct policy from artifact");
    println!(
        "Policy reconstructed from artifact ({} params)\n",
        replay_policy.n_params()
    );

    let training_info = TrainingInfo {
        epochs: metrics.len(),
        final_reward: final_metrics.mean_reward,
        final_dones: final_metrics.done_count,
        wall_time_ms,
        artifact_path: ARTIFACT_PATH.to_string(),
        artifact_saved,
        artifact_reconstructed: true,
    };

    println!("Phase 2: Starting Bevy replay...\n");
    println!("  Orbit: left-drag | Pan: right-drag | Zoom: scroll\n");

    // ── Phase 5: Bevy replay ────────────────────────────────────────────
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "CortenForge — Train-then-Replay (Policy Persistence)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(OrbitCameraPlugin)
        .init_resource::<PhysicsHud>()
        .init_resource::<ReplayValidation>()
        .insert_resource(training_info)
        .insert_resource(TrainedPolicyInput(Some(replay_policy)))
        .insert_resource(
            ValidationHarness::new()
                .wall_clock()
                .report_at(180.0)
                .print_every(10.0)
                .display(|_m, _d| String::new()),
        )
        .add_systems(Startup, setup)
        .add_systems(Update, step_replay)
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
    mut staging: ResMut<TrainedPolicyInput>,
) {
    let task = reaching_2dof();
    let policy = staging.0.take().expect("trained policy must be provided");

    let (mut vec_env, scenes) =
        setup_reaching_arms(&task, 1, &mut commands, &mut meshes, &mut materials);

    let init_obs = vec_env.reset_all().expect("reset");
    let actions = Tensor::zeros(&[1, task.act_dim()]);

    commands.insert_resource(scenes);
    commands.insert_resource(ReplayResource {
        vec_env,
        policy,
        actions,
        current_obs: init_obs,
        accumulator: 0.0,
        sim_time: 0.0,
        step_count: 0,
        episode_count: 0,
        cum_reward: 0.0,
        episode_reward: 0.0,
        episode_reached: false,
    });
}

// ── Stepping (pure inference) ───────────────────────────────────────────────

fn step_replay(mut res: ResMut<ReplayResource>, time: Res<Time>) {
    let wall_dt = time.delta_secs_f64();
    res.accumulator += wall_dt;
    let dt_action = 5.0 * res.vec_env.model().timestep;

    #[allow(clippy::cast_sign_loss)]
    let budget = ((res.accumulator / dt_action).max(0.0) as u32).min(200);

    for _ in 0..budget {
        // Pure inference — policy.forward() with no noise or perturbation
        let inner = &mut *res;
        let obs = inner.current_obs.row(0);
        let acts = inner.policy.forward(obs);
        let row = inner.actions.row_mut(0);
        row[0] = acts[0] as f32;
        row[1] = acts[1] as f32;

        let actions = res.actions.clone();
        let result = res.vec_env.step(&actions).expect("step");
        res.episode_reward += result.rewards[0];
        res.step_count += 1;

        if result.dones[0] || result.truncateds[0] {
            if result.dones[0] {
                res.episode_reached = true;
            }
            res.cum_reward += res.episode_reward;
            res.episode_count += 1;
            res.episode_reward = 0.0;
        }

        res.current_obs = result.observations;
        res.accumulator -= dt_action;
        res.sim_time += dt_action;
    }
}

// ── Sync ────────────────────────────────────────────────────────────────────

fn sync_vec_env_to_scenes(res: Res<ReplayResource>, mut scenes: ResMut<PhysicsScenes>) {
    sync_batch_geoms(res.vec_env.batch(), &mut scenes);
}

fn sync_dummy_time(res: Res<ReplayResource>, mut data: ResMut<sim_bevy::model_data::PhysicsData>) {
    data.0.time = res.sim_time;
}

// ── Validation ──────────────────────────────────────────────────────────────

fn track_validation(
    res: Res<ReplayResource>,
    info: Res<TrainingInfo>,
    mut val: ResMut<ReplayValidation>,
    time: Res<Time>,
) {
    if val.reported {
        return;
    }

    // Wait for at least 5 seconds of replay and at least 1 episode
    if time.elapsed_secs_f64() < 5.0 || res.episode_count < 1 {
        return;
    }
    val.reported = true;

    let avg_reward = if res.episode_count > 0 {
        res.cum_reward / res.episode_count as f64
    } else {
        0.0
    };

    let checks = [
        Check {
            name: "Training converged (reward > -1.0)",
            pass: info.final_reward > -1.0,
            detail: format!("final_reward={:.2}", info.final_reward),
        },
        Check {
            name: "Artifact saved to disk",
            pass: info.artifact_saved,
            detail: info.artifact_path.clone(),
        },
        Check {
            name: "Policy reconstructed from artifact",
            pass: info.artifact_reconstructed,
            detail: "to_policy() succeeded".into(),
        },
        Check {
            name: "Replay reaches target",
            pass: res.episode_reached,
            detail: format!(
                "episodes={}, avg_reward={:.2}",
                res.episode_count, avg_reward
            ),
        },
    ];

    let _ = print_report("Train-then-Replay (Policy Persistence)", &checks);
}

// ── HUD ─────────────────────────────────────────────────────────────────────

fn update_hud(res: Res<ReplayResource>, info: Res<TrainingInfo>, mut hud: ResMut<PhysicsHud>) {
    hud.clear();
    hud.section("Policy Persistence — Train-then-Replay");

    hud.raw(format!(
        "training: {} epochs, {:.1}s",
        info.epochs,
        info.wall_time_ms as f64 / 1000.0
    ));
    hud.raw(format!(
        "final reward: {:.2}  dones: {}",
        info.final_reward, info.final_dones
    ));
    hud.raw(format!(
        "artifact: {}  ({})",
        if info.artifact_saved {
            "saved"
        } else {
            "NOT saved"
        },
        info.artifact_path
    ));

    hud.section("Replay");
    hud.raw(format!(
        "episode: {}  steps: {}",
        res.episode_count, res.step_count
    ));
    hud.raw(format!("episode reward: {:.2}", res.episode_reward));
    if res.episode_count > 0 {
        hud.raw(format!(
            "avg reward: {:.2}",
            res.cum_reward / res.episode_count as f64
        ));
    }
    hud.raw(format!(
        "reached target: {}",
        if res.episode_reached {
            "YES"
        } else {
            "not yet"
        }
    ));
}
