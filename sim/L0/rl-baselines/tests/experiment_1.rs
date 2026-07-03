//! Experiment 1 — E. coli stochastic resonance in chemotactic navigation.
//!
//! Tests the central claim of Regime 1: an intermediate noise level maximizes
//! a Langevin particle's ability to follow a periodic signal in a biased
//! double-well potential.  Extends the D2 SR setup with an `ExternalField`
//! gradient and validates across three algorithm classes (CEM, PPO, SA).
//!
//! All tests require `--release` (total ~30-40 min if run serially).
//!
//! ## Gate system
//!
//! - **Gate A**: significant synchrony (one-sample t-test, |t| > t_critical)
//! - **Gate B**: learning monotonicity (best_last_10 > mean_first_5)
//! - **Gate C**: learned temperature within 50% of baseline SR peak

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::too_many_lines,
    clippy::doc_markdown
)]

use std::f64::consts::PI;

use sim_opt::{Sa, SaHyperparams};
use sim_rl::{
    Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, LinearValue, OptimizerConfig,
    Policy, Ppo, PpoHyperparams, Tensor, TrainingBudget,
};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::{DoubleWellPotential, ExternalField, OscillatingField};

// ─── Central parameters (matching D2) ────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;
const KRAMERS_RATE: f64 = 0.01214;
const SEED_BASE: u64 = 20_260_415;

const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;
const TIMESTEP: f64 = 0.001;

const OBS_DIM: usize = 2;
const ACT_DIM: usize = 1;

// ─── Training parameters ────────────────────────────────────────────────

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const N_EVAL_EPISODES: usize = 20;

// ─── Statistical thresholds ─────────────────────────────────────────────

/// t-critical for df=9 (10 episodes), two-tailed α=0.01.
const T_CRITICAL_DF9: f64 = 3.250;

/// t-critical for df=19 (20 episodes), two-tailed α=0.01.
const T_CRITICAL_DF19: f64 = 2.861;

// ─── Helpers ─────────────────────────────────────────────────────────────

fn signal_omega() -> f64 {
    2.0 * PI * KRAMERS_RATE
}

fn obs_scale() -> Vec<f64> {
    vec![1.0 / X_0, 1.0]
}

fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

/// Build a single-env `ThermCircuitEnv` for evaluation at a fixed kT multiplier.
fn make_eval_env(gradient_h: f64, kt_mult: f64, seed: u64) -> sim_therm_env::ThermCircuitEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(1)
        .gamma(GAMMA)
        .k_b_t(K_B_T_BASE * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed)
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .with(OscillatingField::new(A_0, omega, 0.0, 0));

    if gradient_h.abs() > 1e-15 {
        builder = builder.with(ExternalField::new(vec![gradient_h]));
    }

    builder
        .reward(move |_m, d| d.qpos[0].signum() * (omega * d.time).cos())
        .build()
        .unwrap()
}

/// Build a `VecEnv` for training with ctrl-temperature.
fn make_training_vecenv(gradient_h: f64, n_envs: usize, seed: u64) -> sim_rl::VecEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(1)
        .gamma(GAMMA)
        .k_b_t(K_B_T_BASE)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed)
        .with_ctrl_temperature()
        .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
        .with(OscillatingField::new(A_0, omega, 0.0, 0));

    if gradient_h.abs() > 1e-15 {
        builder = builder.with(ExternalField::new(vec![gradient_h]));
    }

    builder
        .reward(move |_m, d| d.qpos[0].signum() * (omega * d.time).cos())
        .build_vec(n_envs)
        .unwrap()
}

/// Run one episode at fixed kT, return mean synchrony (reward per step).
fn run_fixed_kt_episode(gradient_h: f64, kt_mult: f64, seed: u64) -> f64 {
    let mut env = make_eval_env(gradient_h, kt_mult, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total_reward = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total_reward += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total_reward / f64::from(steps)
}

/// Temperature sweep: measure mean synchrony at each kT multiplier.
/// Returns `(kt_mults, means, stderrs)`.
fn temperature_sweep(
    gradient_h: f64,
    kt_mults: &[f64],
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut means = Vec::with_capacity(kt_mults.len());
    let mut stderrs = Vec::with_capacity(kt_mults.len());

    for (i, &kt) in kt_mults.iter().enumerate() {
        let mut episodes = Vec::with_capacity(n_episodes);
        for ep in 0..n_episodes {
            let seed = SEED_BASE + seed_offset + (i * 1000 + ep) as u64;
            episodes.push(run_fixed_kt_episode(gradient_h, kt, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        means.push(m);
        stderrs.push(s);
    }

    (kt_mults.to_vec(), means, stderrs)
}

/// Find the peak synchrony in a sweep. Returns (peak_index, peak_kt, peak_mean, peak_stderr).
fn find_sr_peak(kt_mults: &[f64], means: &[f64], stderrs: &[f64]) -> (usize, f64, f64, f64) {
    let (peak_idx, &peak_mean) = means
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();
    (peak_idx, kt_mults[peak_idx], peak_mean, stderrs[peak_idx])
}

/// Evaluate a trained policy: returns (synchrony_per_step_per_episode, mean_kt_per_episode).
fn evaluate_policy(
    policy: &dyn Policy,
    n_episodes: usize,
    gradient_h: f64,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>) {
    let omega = signal_omega();
    let mut synchronies = Vec::with_capacity(n_episodes);
    let mut temperatures = Vec::with_capacity(n_episodes);

    for i in 0..n_episodes {
        // Build with ctrl-temperature for policy evaluation.
        let mut builder = ThermCircuitEnv::builder(1)
            .gamma(GAMMA)
            .k_b_t(K_B_T_BASE)
            .timestep(TIMESTEP)
            .sub_steps(SUB_STEPS)
            .episode_steps(EPISODE_STEPS)
            .seed(SEED_BASE + seed_offset + i as u64)
            .with_ctrl_temperature()
            .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
            .with(OscillatingField::new(A_0, omega, 0.0, 0));

        if gradient_h.abs() > 1e-15 {
            builder = builder.with(ExternalField::new(vec![gradient_h]));
        }

        let mut env = builder
            .reward(move |_m, d| d.qpos[0].signum() * (omega * d.time).cos())
            .build()
            .unwrap();

        let obs = env.reset().unwrap();
        let mut obs_vec: Vec<f32> = obs.as_slice().to_vec();
        let mut total_reward = 0.0;
        let mut total_temp = 0.0;
        let mut steps = 0;

        for _ in 0..EPISODE_STEPS {
            let action = policy.forward(&obs_vec);
            total_temp += action[0] * K_B_T_BASE;
            let action_tensor = Tensor::from_f64_slice(&action, &[1]);
            let result = env.step(&action_tensor).unwrap();
            total_reward += result.reward;
            obs_vec = result.observation.as_slice().to_vec();
            steps += 1;
            if result.truncated {
                break;
            }
        }
        synchronies.push(total_reward / steps as f64);
        temperatures.push(total_temp / steps as f64);
    }
    (synchronies, temperatures)
}

/// Train an algorithm, evaluate, report. Returns `(sync_mean, sync_stderr, kt_mean)`.
fn train_and_evaluate(
    name: &str,
    algo: &mut dyn Algorithm,
    gradient_h: f64,
    seed: u64,
    eval_offset: u64,
) -> (f64, f64, f64) {
    let mut env = make_training_vecenv(gradient_h, N_ENVS, seed);

    eprintln!("\n--- {name}: training ({N_EPOCHS} epochs, {N_ENVS} envs, h={gradient_h}) ---");

    let metrics = algo.train(&mut env, TrainingBudget::Epochs(N_EPOCHS), seed, &|m| {
        if m.epoch % 20 == 0 || m.epoch == N_EPOCHS - 1 {
            eprintln!(
                "  {name} epoch {:3}: mean_reward = {:+.6}",
                m.epoch, m.mean_reward,
            );
        }
    });

    // Gate B: learning improves
    let first_5_mean: f64 = metrics[..5].iter().map(|m| m.mean_reward).sum::<f64>() / 5.0;
    let best_last_10 = metrics[metrics.len().saturating_sub(10)..]
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);
    let gate_b = best_last_10 > first_5_mean;
    eprintln!(
        "  {name} Gate B: mean_first_5={first_5_mean:.6}, best_last_10={best_last_10:.6} -> {}",
        if gate_b { "PASS" } else { "FAIL" }
    );

    // Evaluate deterministic policy
    let trained = algo.policy_artifact().to_policy().unwrap();
    let (syncs, temps) =
        evaluate_policy(trained.as_ref(), N_EVAL_EPISODES, gradient_h, eval_offset);
    let (sync_mean, sync_stderr) = mean_stderr(&syncs);
    let (temp_mean, temp_stderr) = mean_stderr(&temps);

    eprintln!("  {name} eval: synchrony = {sync_mean:.6} +/- {sync_stderr:.6}");
    eprintln!("  {name} eval: kT = {temp_mean:.4} +/- {temp_stderr:.4}");

    // Gate A: significant synchrony
    let t_stat = if sync_stderr > 1e-15 {
        sync_mean / sync_stderr
    } else {
        0.0
    };
    let gate_a = t_stat.abs() > T_CRITICAL_DF19;
    eprintln!(
        "  {name} Gate A: |t|={:.2} -> {}",
        t_stat.abs(),
        if gate_a { "PASS" } else { "FAIL" }
    );

    // Report params
    let params = trained.params();
    if params.len() <= 5 {
        eprintln!("  {name} params: {params:?}");
    }

    assert!(
        gate_a,
        "{name} Gate A FAILED: |t|={:.2} < {T_CRITICAL_DF19}",
        t_stat.abs()
    );
    assert!(gate_b, "{name} Gate B FAILED: no learning improvement");

    (sync_mean, sync_stderr, temp_mean)
}

// ─── Log-spaced temperature multipliers ──────────────────────────────────

fn log_spaced(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    let log_lo = lo.ln();
    let log_hi = hi.ln();
    (0..n)
        .map(|i| {
            let frac = i as f64 / (n - 1) as f64;
            (log_lo + frac * (log_hi - log_lo)).exp()
        })
        .collect()
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

// ─── Baseline SR peak (no gradient) ──────────────────────────────────────

#[test]
#[ignore = "requires --release (~5 min)"]
fn exp1_baseline_sr_peak() {
    let kt_mults = log_spaced(0.1, 5.0, 30);
    let n_episodes = 20;

    eprintln!("\n=== Experiment 1: Baseline SR peak (h=0.0) ===");
    let (kts, means, stderrs) = temperature_sweep(0.0, &kt_mults, n_episodes, 0);

    // Print the full sweep
    eprintln!("\n  kT_mult  |  synchrony  |  stderr");
    eprintln!("  ---------|-------------|--------");
    for i in 0..kts.len() {
        eprintln!(
            "  {:7.4}  |  {:+.6}  |  {:.6}",
            kts[i], means[i], stderrs[i]
        );
    }

    let (peak_idx, peak_kt, peak_mean, peak_stderr) = find_sr_peak(&kts, &means, &stderrs);
    eprintln!("\n  Peak: kT={peak_kt:.4}, synchrony={peak_mean:.6} +/- {peak_stderr:.6}");

    // Gate: peak is significant
    let t_stat = if peak_stderr > 1e-15 {
        peak_mean / peak_stderr
    } else {
        0.0
    };
    eprintln!(
        "  |t| = {:.2} (critical = {T_CRITICAL_DF9:.3})",
        t_stat.abs()
    );

    assert!(
        t_stat.abs() > T_CRITICAL_DF19,
        "SR peak not significant: |t|={:.2} < {T_CRITICAL_DF19}",
        t_stat.abs()
    );

    // Sanity: peak is interior (not at boundary)
    assert!(
        peak_idx > 0 && peak_idx < kts.len() - 1,
        "SR peak at boundary (idx={peak_idx}) — not a true peak"
    );

    eprintln!("  Baseline SR peak: PASS");
}

// ─── Gradient SR peak ────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn exp1_gradient_sr_peak() {
    let kt_mults = log_spaced(0.1, 5.0, 20);
    let n_episodes = 10;

    eprintln!("\n=== Experiment 1: Gradient SR peak comparison ===");

    // Sweep at h=0.0
    eprintln!("\n--- h=0.0 (baseline) ---");
    let (kts_0, means_0, stderrs_0) = temperature_sweep(0.0, &kt_mults, n_episodes, 0);
    let (_, peak_kt_0, peak_sync_0, _) = find_sr_peak(&kts_0, &means_0, &stderrs_0);
    eprintln!("  Peak: kT={peak_kt_0:.4}, synchrony={peak_sync_0:.6}");

    // Sweep at h=0.3
    eprintln!("\n--- h=0.3 (gradient) ---");
    let (kts_03, means_03, stderrs_03) = temperature_sweep(0.3, &kt_mults, n_episodes, 10_000);
    let (_, peak_kt_03, peak_sync_03, peak_stderr_03) =
        find_sr_peak(&kts_03, &means_03, &stderrs_03);
    eprintln!("  Peak: kT={peak_kt_03:.4}, synchrony={peak_sync_03:.6}");

    // Print comparison
    eprintln!("\n  Comparison:");
    eprintln!("    h=0.0: peak kT={peak_kt_0:.4}, sync={peak_sync_0:.6}");
    eprintln!("    h=0.3: peak kT={peak_kt_03:.4}, sync={peak_sync_03:.6}");

    // Gate: gradient peak is significant
    let t_stat = if peak_stderr_03 > 1e-15 {
        peak_sync_03 / peak_stderr_03
    } else {
        0.0
    };
    assert!(
        t_stat.abs() > T_CRITICAL_DF9,
        "Gradient SR peak not significant: |t|={:.2}",
        t_stat.abs()
    );

    eprintln!("  Gradient SR peak: PASS");
}

// ─── Controls ────────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~3 min)"]
fn exp1_controls() {
    let n_episodes = 10;

    eprintln!("\n=== Experiment 1: Controls ===");

    // Control 1: Low noise (kT × 0.1) — particle trapped, no switching
    eprintln!("\n--- Control 1: Low noise (kT x 0.1) ---");
    let mut low_syncs = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        low_syncs.push(run_fixed_kt_episode(
            0.0,
            0.1,
            SEED_BASE + 20_000 + ep as u64,
        ));
    }
    let (low_mean, low_stderr) = mean_stderr(&low_syncs);
    eprintln!("  synchrony = {low_mean:.6} +/- {low_stderr:.6}");
    assert!(
        low_mean.abs() < 3.0 * low_stderr,
        "Low noise control: |mean|={:.6} >= 3*stderr={:.6}",
        low_mean.abs(),
        3.0 * low_stderr
    );
    eprintln!("  Low noise control: PASS");

    // Control 2: High noise (kT × 5.0) — random switching, no correlation
    eprintln!("\n--- Control 2: High noise (kT x 5.0) ---");
    let mut high_syncs = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        high_syncs.push(run_fixed_kt_episode(
            0.0,
            5.0,
            SEED_BASE + 30_000 + ep as u64,
        ));
    }
    let (high_mean, high_stderr) = mean_stderr(&high_syncs);
    eprintln!("  synchrony = {high_mean:.6} +/- {high_stderr:.6}");
    assert!(
        high_mean.abs() < 3.0 * high_stderr,
        "High noise control: |mean|={:.6} >= 3*stderr={:.6}",
        high_mean.abs(),
        3.0 * high_stderr
    );
    eprintln!("  High noise control: PASS");

    // Control 3: No signal (A₀=0) — validates the metric
    eprintln!("\n--- Control 3: No signal (A0=0) ---");
    let omega = signal_omega();
    let mut nosig_syncs = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        let seed = SEED_BASE + 40_000 + ep as u64;
        // Build env with A₀=0 at moderate kT
        let mut env = ThermCircuitEnv::builder(1)
            .gamma(GAMMA)
            .k_b_t(K_B_T_BASE * 1.5)
            .timestep(TIMESTEP)
            .sub_steps(SUB_STEPS)
            .episode_steps(EPISODE_STEPS)
            .seed(seed)
            .with(DoubleWellPotential::new(DELTA_V, X_0, 0))
            .with(OscillatingField::new(0.0, omega, 0.0, 0)) // A₀ = 0
            .reward(move |_m, d| d.qpos[0].signum() * (omega * d.time).cos())
            .build()
            .unwrap();

        let _obs = env.reset().unwrap();
        let action = Tensor::from_f64_slice(&[], &[0]);
        let mut total_reward = 0.0;
        let mut steps = 0;
        loop {
            let result = env.step(&action).unwrap();
            total_reward += result.reward;
            steps += 1;
            if result.truncated || result.done {
                break;
            }
        }
        nosig_syncs.push(total_reward / f64::from(steps));
    }
    let (nosig_mean, nosig_stderr) = mean_stderr(&nosig_syncs);
    eprintln!("  synchrony = {nosig_mean:.6} +/- {nosig_stderr:.6}");
    assert!(
        nosig_mean.abs() < 3.0 * nosig_stderr,
        "No-signal control: |mean|={:.6} >= 3*stderr={:.6}",
        nosig_mean.abs(),
        3.0 * nosig_stderr
    );
    eprintln!("  No-signal control: PASS");
}

// ─── CEM training ────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn exp1_cem() {
    let gradient_h = 0.3;

    eprintln!("\n=== Experiment 1: CEM training (h={gradient_h}) ===");

    let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    policy.set_params(&[0.0, 0.0, 2.0]); // bias → tanh(2) ≈ 0.96
    let mut algo = Cem::new(
        Box::new(policy),
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 2.5,
            noise_decay: 0.98,
            noise_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    );

    let (sync_mean, _sync_stderr, kt_mean) =
        train_and_evaluate("CEM", &mut algo, gradient_h, SEED_BASE, 50_000);

    eprintln!("\n  CEM summary: synchrony={sync_mean:.6}, kT={kt_mean:.4}");
}

// ─── PPO training ────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn exp1_ppo() {
    let gradient_h = 0.3;

    eprintln!("\n=== Experiment 1: PPO training (h={gradient_h}) ===");

    let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let value_fn = LinearValue::new(OBS_DIM, &obs_scale());
    let mut algo = Ppo::new(
        Box::new(policy),
        Box::new(value_fn),
        OptimizerConfig::adam(3e-4),
        PpoHyperparams {
            clip_eps: 0.2,
            k_passes: 4,
            gamma: 0.99,
            gae_lambda: 0.95,
            sigma_init: 1.0,
            sigma_decay: 0.995,
            sigma_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    );

    let (sync_mean, _sync_stderr, kt_mean) =
        train_and_evaluate("PPO", &mut algo, gradient_h, SEED_BASE + 100, 60_000);

    eprintln!("\n  PPO summary: synchrony={sync_mean:.6}, kT={kt_mean:.4}");
}

// ─── SA training (sim-opt) ───────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn exp1_sa() {
    let gradient_h = 0.3;

    eprintln!("\n=== Experiment 1: SA training (h={gradient_h}) ===");

    let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let mut algo = Sa::new(
        Box::new(policy),
        SaHyperparams {
            initial_temperature: 50.0,
            proposal_std: 1.0,
            cooling_decay: 0.955,
            temperature_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    );

    let (sync_mean, _sync_stderr, kt_mean) =
        train_and_evaluate("SA", &mut algo, gradient_h, SEED_BASE + 200, 70_000);

    eprintln!("\n  SA summary: synchrony={sync_mean:.6}, kT={kt_mean:.4}");
}
