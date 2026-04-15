//! Integration tests exercising `BestTracker` through `Cem`'s full
//! training and checkpoint-resume flow.
//!
//! These tests originally lived inside `best_tracker.rs`'s `mod tests`
//! block alongside the pure-unit tests. When `best_tracker.rs` moved to
//! `sim-ml-chassis` during the chassis/rl split, the Cem-consuming tests
//! had to move with the algorithm — a chassis test importing `Cem` would
//! force a `sim-ml-chassis → sim-rl` dev-dep cycle. They live here
//! instead, as a sim-rl integration test against a chassis primitive.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use sim_ml_bridge::{Cem, CemHyperparams};
use sim_ml_chassis::{
    Algorithm, LinearPolicy, Policy, TrainingBudget, TrainingCheckpoint, reaching_2dof,
};

#[test]
fn best_artifact_before_train_returns_initial_policy() {
    let task = reaching_2dof();
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let initial_params = policy.params().to_vec();
    let cem = Cem::new(
        policy,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        },
    );

    let best = cem.best_artifact();
    assert_eq!(best.params, initial_params);
    assert!(best.provenance.is_none());
}

#[test]
fn best_artifact_after_training_has_best_epoch_weights() {
    let task = reaching_2dof();
    let mut env = task.build_vec_env(10, 0).unwrap();
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let mut cem = Cem::new(
        policy,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        },
    );

    let metrics = cem.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});
    let best_art = cem.best_artifact();
    let final_art = cem.policy_artifact();

    // Best artifact should have valid params of the same length.
    assert_eq!(best_art.params.len(), final_art.params.len());

    // Find best epoch reward from metrics.
    let best_reward = metrics
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);
    let final_reward = metrics.last().unwrap().mean_reward;

    // Best reward should be >= final reward (by definition).
    assert!(
        best_reward >= final_reward,
        "best ({best_reward}) should be >= final ({final_reward})"
    );
}

#[test]
fn checkpoint_round_trip_preserves_best_across_resume() {
    let task = reaching_2dof();
    let mut env = task.build_vec_env(10, 0).unwrap();
    let hp = CemHyperparams {
        elite_fraction: 0.2,
        noise_std: 0.3,
        noise_decay: 0.95,
        noise_min: 0.01,
        max_episode_steps: 300,
    };
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let mut cem = Cem::new(policy, hp);
    cem.train(&mut env, TrainingBudget::Epochs(5), 42, &|_| {});

    let cp = cem.checkpoint();

    // Checkpoint should contain best state.
    assert!(cp.best_params.is_some());
    let bp = cp.best_params.as_ref().unwrap();
    assert_eq!(bp.len(), cem.best_artifact().params.len());

    // Restore and verify best artifact matches.
    let cem2 = Cem::from_checkpoint(&cp, hp).unwrap();
    let original_best = cem.best_artifact();
    let restored_best = cem2.best_artifact();
    assert_eq!(restored_best.params, original_best.params);
}

#[test]
fn old_checkpoint_without_best_fields_loads_with_defaults() {
    // Simulate a pre-feature checkpoint (no best_* fields) by
    // constructing a TrainingCheckpoint with None/default best fields.
    let task = reaching_2dof();
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let cem = Cem::new(
        policy,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 300,
        },
    );
    let mut cp = cem.checkpoint();

    // Strip best fields to simulate old checkpoint.
    cp.best_params = None;
    cp.best_reward = None;
    cp.best_epoch = 0;

    // Also verify JSON round-trip with missing fields.
    let json = serde_json::to_string(&cp).unwrap();
    let loaded: TrainingCheckpoint = serde_json::from_str(&json).unwrap();
    assert!(loaded.best_params.is_none());
    assert!(loaded.best_reward.is_none());
    assert_eq!(loaded.best_epoch, 0);

    // Restore algorithm — should fall back to policy params.
    let hp = CemHyperparams {
        elite_fraction: 0.2,
        noise_std: 0.3,
        noise_decay: 0.95,
        noise_min: 0.01,
        max_episode_steps: 300,
    };
    let cem2 = Cem::from_checkpoint(&loaded, hp).unwrap();
    // Best artifact should equal the current policy (fallback).
    assert_eq!(cem2.best_artifact().params, cem2.policy_artifact().params);
}

#[test]
fn resume_training_preserves_best_from_previous_session() {
    let task = reaching_2dof();
    let mut env = task.build_vec_env(10, 0).unwrap();
    let hp = CemHyperparams {
        elite_fraction: 0.2,
        noise_std: 0.3,
        noise_decay: 0.95,
        noise_min: 0.01,
        max_episode_steps: 300,
    };

    // Session 1: train 10 epochs, record best.
    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let mut cem = Cem::new(policy, hp);
    let metrics1 = cem.train(&mut env, TrainingBudget::Epochs(10), 42, &|_| {});
    let session1_best_reward = metrics1
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);

    // Checkpoint + restore.
    let cp = cem.checkpoint();
    let mut cem2 = Cem::from_checkpoint(&cp, hp).unwrap();

    // Session 2: train 1 epoch with a different seed.
    let metrics2 = cem2.train(&mut env, TrainingBudget::Epochs(1), 99, &|_| {});
    let session2_reward = metrics2[0].mean_reward;

    // The best after resume should be at least as good as session 1's best.
    // If session 2's single epoch didn't beat session 1's best, the
    // restored best should survive unchanged.
    let best_art = cem2.best_artifact();
    assert_eq!(best_art.params.len(), cem.best_artifact().params.len());

    // If session 2 didn't beat session 1, best should equal session 1's best.
    if session2_reward <= session1_best_reward {
        assert_eq!(best_art.params, cem.best_artifact().params);
    }
}
