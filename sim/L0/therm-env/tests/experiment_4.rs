//! Experiment 4 — PN guidance: CEM learns ctrl-temperature for double-well navigation.
//!
//! Validates the full thermo-RL loop via `ThermCircuitEnv`:
//!
//! - 1-particle double-well (ΔV=3, x₀=1), ctrl-temperature
//! - CEM with `LinearPolicy(2, 1)` trains for 50 epochs on 16-env `VecEnv`
//! - Gate: best-epoch `mean_reward` > constant-temperature baseline
//!
//! Requires `--release` (~1-2 min).

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::float_cmp
)]

use sim_rl::{
    Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, Policy, Tensor, TrainingBudget,
};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::DoubleWellPotential;

// ─── Parameters (spec §7 Phase 4) ───────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const TIMESTEP: f64 = 0.001;
const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 1000;
const TARGET: f64 = X_0; // +x₀ well

const N_ENVS: usize = 16;
const N_EPOCHS: usize = 50;
const SEED: u64 = 42;

const OBS_DIM: usize = 2; // [qpos, qvel]
const ACT_DIM: usize = 1; // [ctrl_temperature]

// ─── Helpers ─────────────────────────────────────────────────────────────

/// Build the standard `ThermCircuitEnvBuilder` for Experiment 4.
fn exp4_builder() -> sim_therm_env::ThermCircuitEnvBuilder {
    ThermCircuitEnv::builder(1)
        .gamma(GAMMA)
        .k_b_t(K_B_T)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .with_ctrl_temperature()
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .reward(|_m, d| {
            let x = d.qpos[0];
            -(x - TARGET) * (x - TARGET)
        })
}

/// Run one episode with a fixed ctrl value, return total episode reward.
fn constant_temperature_baseline(ctrl_value: f64) -> f64 {
    let mut env = exp4_builder().seed(SEED + 1000).build().unwrap();
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[ctrl_value], &[ACT_DIM]);

    let mut total_reward = 0.0;
    loop {
        let result = env.step(&action).unwrap();
        total_reward += result.reward;
        if result.truncated || result.done {
            break;
        }
    }
    total_reward
}

// ─── Test ────────────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~1-2 min)"]
fn experiment_4_pn_guidance() {
    // ── Baseline: constant temperature (ctrl=1.0 → kT=1.0) ──────────
    let baseline = constant_temperature_baseline(1.0);
    eprintln!("Baseline (ctrl=1.0): episode_total = {baseline:.2}");

    // ── CEM training ─────────────────────────────────────────────────
    let obs_scale = vec![1.0 / X_0, 1.0];
    let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale);
    // Initialize bias so ctrl ≈ tanh(1.0) ≈ 0.76 — a reasonable start.
    policy.set_params(&[0.0, 0.0, 1.0]);

    let mut cem = Cem::new(
        Box::new(policy),
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 1.5,
            noise_decay: 0.97,
            noise_min: 0.05,
            max_episode_steps: EPISODE_STEPS,
        },
    );

    let mut env = exp4_builder().seed(SEED).build_vec(N_ENVS).unwrap();

    let metrics = cem.train(&mut env, TrainingBudget::Epochs(N_EPOCHS), SEED, &|m| {
        if m.epoch % 10 == 0 || m.epoch == N_EPOCHS - 1 {
            eprintln!(
                "  CEM epoch {:3}: mean_reward = {:+.2}",
                m.epoch, m.mean_reward,
            );
        }
    });

    // CEM is an evolutionary method — individual epochs are noisy.
    // The best epoch across training is the robust signal.
    let best_reward = metrics
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);

    eprintln!("CEM best-epoch mean_reward = {best_reward:.2}");
    eprintln!("Baseline episode total     = {baseline:.2}");

    // ── Gate: CEM beats constant-temperature baseline ────────────────
    // Both are per-episode-total rewards (CEM averages across n_envs).
    assert!(
        best_reward > baseline,
        "CEM best-epoch should beat constant-temperature baseline: \
         best={best_reward:.2} vs baseline={baseline:.2}"
    );
}
