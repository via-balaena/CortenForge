//! Phase 3 tests: `ThermCircuitEnvBuilder::build_vec()` -> `VecEnv` (batched).
//!
//! Gates per spec §7 Phase 3:
//! - Build 4-env `VecEnv`, step all, reset all
//! - Observations have correct batch shape `[n_envs, obs_dim]`
//! - Reward computed independently per env
//! - Truncation + auto-reset works per env

#![allow(clippy::unwrap_used)]

use sim_ml_chassis::Tensor;
use sim_therm_env::ThermCircuitEnv;

// ── 1. Build 4-env VecEnv, step all, reset all ────────────────────────

#[test]
fn vec_env_reset_and_step() {
    let mut env = ThermCircuitEnv::builder(1)
        .reward(|_m, _d| 1.0)
        .build_vec(4)
        .unwrap();

    // reset_all returns [n_envs, obs_dim]
    let obs = env.reset_all().unwrap();
    assert_eq!(obs.shape(), &[4, 2], "reset obs shape: [4, 2*n_particles]");

    // step with dim-0 actions (no ctrl channels)
    let actions = Tensor::zeros(&[4, 0]);
    let result = env.step(&actions).unwrap();
    assert_eq!(result.observations.shape(), &[4, 2]);
    assert_eq!(result.rewards.len(), 4);
    assert_eq!(result.dones.len(), 4);
    assert_eq!(result.truncateds.len(), 4);
}

// ── 2. Batch shape with multiple particles ─────────────────────────────

#[test]
fn batch_shape_2_particles() {
    let mut env = ThermCircuitEnv::builder(2)
        .reward(|_m, _d| 0.0)
        .build_vec(4)
        .unwrap();

    let obs = env.reset_all().unwrap();
    // 2 particles: nq=2, nv=2 -> obs_dim=4
    assert_eq!(obs.shape(), &[4, 4]);
}

#[test]
fn batch_shape_with_ctrl_temperature() {
    let mut env = ThermCircuitEnv::builder(1)
        .with_ctrl_temperature()
        .reward(|_m, _d| 0.0)
        .build_vec(4)
        .unwrap();

    let obs = env.reset_all().unwrap();
    // 1 particle: obs_dim=2, act_dim=1 (ctrl_temperature)
    assert_eq!(obs.shape(), &[4, 2]);

    let actions = Tensor::from_slice(&[1.0_f32; 4], &[4, 1]);
    let result = env.step(&actions).unwrap();
    assert_eq!(result.observations.shape(), &[4, 2]);
}

// ── 3. Per-env reward ──────────────────────────────────────────────────

#[test]
fn per_env_reward_independence() {
    let mut env = ThermCircuitEnv::builder(1)
        .reward(|_m, d| -(d.qpos[0] * d.qpos[0]))
        .build_vec(2)
        .unwrap();

    env.reset_all().unwrap();

    // Set different positions in different envs
    if let Some(data) = env.batch_mut().env_mut(0) {
        data.qpos[0] = 1.0;
    }
    if let Some(data) = env.batch_mut().env_mut(1) {
        data.qpos[0] = 2.0;
    }

    let actions = Tensor::zeros(&[2, 0]);
    let result = env.step(&actions).unwrap();

    // Rewards should differ: ~ -1.0 vs ~ -4.0
    assert!(
        (result.rewards[0] - result.rewards[1]).abs() > 1.0,
        "per-env rewards should differ: r0={}, r1={}",
        result.rewards[0],
        result.rewards[1],
    );
}

// ── 4. Truncation + auto-reset ─────────────────────────────────────────

#[test]
fn truncation_and_auto_reset() {
    let mut env = ThermCircuitEnv::builder(1)
        .episode_steps(10)
        .sub_steps(1)
        .timestep(0.001)
        .reward(|_m, _d| 1.0)
        .build_vec(2)
        .unwrap();

    env.reset_all().unwrap();

    // Push env 0 past truncation: max_time = 10 * 1 * 0.001 = 0.01
    if let Some(data) = env.batch_mut().env_mut(0) {
        data.time = 0.02;
    }

    let actions = Tensor::zeros(&[2, 0]);
    let result = env.step(&actions).unwrap();

    // Env 0 should be truncated, env 1 should not
    assert!(result.truncateds[0], "env 0 should be truncated");
    assert!(!result.truncateds[1], "env 1 should not be truncated");

    // After auto-reset, env 0's observation should be near zero (fresh state)
    let obs_0 = result.observations.row(0);
    assert!(
        obs_0[0].abs() < 1e-5,
        "post-reset qpos should be near 0, got {}",
        obs_0[0],
    );

    // Terminal observation should exist for the truncated env
    assert!(
        result.terminal_observations[0].is_some(),
        "truncated env should have terminal obs",
    );
    assert!(
        result.terminal_observations[1].is_none(),
        "non-truncated env should not have terminal obs",
    );
}
