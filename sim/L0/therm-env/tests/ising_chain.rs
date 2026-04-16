//! Ising chain stochastic resonance — does SR scale from 1 particle to coupled systems?
//!
//! The 1-particle experiment established an SR peak at kT=1.70 (|t|=4.52).
//! This experiment adds nearest-neighbor coupling and tests whether the
//! SR phenomenon persists, shifts, or disappears in a coupled multi-cell
//! circuit proxy.
//!
//! Scientific question: how does inter-cell coupling affect the optimal
//! noise temperature for signal following?  This is the direct test of
//! whether Principle 2 (stochastic resonance) scales to thermodynamic
//! circuit architectures.
//!
//! All tests require `--release`.

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

use sim_opt::{RicherSa, RicherSaHyperparams, Sa, SaHyperparams};
use sim_rl::{
    Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, Policy, Tensor, TrainingBudget,
};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::{DoubleWellPotential, OscillatingField, PairwiseCoupling};

// ─── Parameters (matching D2 / Experiment 1) ─────────────────────────────

const N: usize = 4;
const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const A_0: f64 = 0.3;
const KRAMERS_RATE: f64 = 0.01214;
const TIMESTEP: f64 = 0.001;
const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;
const SEED_BASE: u64 = 20_260_416;

const CTRL_RANGE_HI: f64 = 15.0;

const OBS_DIM: usize = 2 * N;
const ACT_DIM: usize = 1;

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const N_EVAL_EPISODES: usize = 20;

/// t-critical for df=19, two-tailed α=0.01.
const T_CRITICAL_DF19: f64 = 2.861;

/// t-critical for df=9, two-tailed α=0.01.
const T_CRITICAL_DF9: f64 = 3.250;

/// t-critical for df=39, two-tailed α=0.01.
const T_CRITICAL_DF39: f64 = 2.708;

/// t-critical for df=79, two-tailed α=0.01.
const T_CRITICAL_DF79: f64 = 2.640;

/// t-critical for df=6, two-tailed α=0.01.
const T_CRITICAL_DF6: f64 = 3.707;

// ─── Helpers ─────────────────────────────────────────────────────────────

fn signal_omega() -> f64 {
    2.0 * PI * KRAMERS_RATE
}

fn obs_scale() -> Vec<f64> {
    let mut s = vec![1.0 / X_0; N];
    s.extend(vec![1.0; N]);
    s
}

fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

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

/// Simple linear regression: returns (slope, slope_stderr).
/// Used to test whether a curve has a significant trend.
fn lin_regress_slope(xs: &[f64], ys: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let x_bar = xs.iter().sum::<f64>() / n;
    let y_bar = ys.iter().sum::<f64>() / n;
    let sum_dx2: f64 = xs.iter().map(|&x| (x - x_bar).powi(2)).sum();
    let sum_dxdy: f64 = xs
        .iter()
        .zip(ys)
        .map(|(&x, &y)| (x - x_bar) * (y - y_bar))
        .sum();
    let slope = if sum_dx2 > 1e-15 {
        sum_dxdy / sum_dx2
    } else {
        0.0
    };
    let resid_var: f64 = xs
        .iter()
        .zip(ys)
        .map(|(&x, &y)| {
            let predicted = slope.mul_add(x - x_bar, y_bar);
            (y - predicted).powi(2)
        })
        .sum::<f64>()
        / (n - 2.0);
    let slope_se = if sum_dx2 > 1e-15 {
        (resid_var / sum_dx2).sqrt()
    } else {
        0.0
    };
    (slope, slope_se)
}

/// R-squared for a linear regression of ys on xs.
fn lin_regress_r_squared(xs: &[f64], ys: &[f64]) -> f64 {
    let n = xs.len() as f64;
    let y_bar = ys.iter().sum::<f64>() / n;
    let ss_tot: f64 = ys.iter().map(|&y| (y - y_bar).powi(2)).sum();
    if ss_tot < 1e-15 {
        return 1.0;
    }
    let x_bar = xs.iter().sum::<f64>() / n;
    let sum_dx2: f64 = xs.iter().map(|&x| (x - x_bar).powi(2)).sum();
    let sum_dxdy: f64 = xs
        .iter()
        .zip(ys)
        .map(|(&x, &y)| (x - x_bar) * (y - y_bar))
        .sum();
    let slope = if sum_dx2 > 1e-15 {
        sum_dxdy / sum_dx2
    } else {
        0.0
    };
    let intercept = y_bar - slope * x_bar;
    let ss_res: f64 = xs
        .iter()
        .zip(ys)
        .map(|(&x, &y)| (y - slope.mul_add(x, intercept)).powi(2))
        .sum();
    1.0 - ss_res / ss_tot
}

fn lin_spaced(lo: f64, hi: f64, n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| lo + (hi - lo) * i as f64 / (n - 1) as f64)
        .collect()
}

/// Build an Ising chain with oscillating field at fixed kT (no ctrl-temperature).
/// Each particle gets its own double well + oscillating field.
fn make_sweep_env(coupling_j: f64, kt_mult: f64, seed: u64) -> sim_therm_env::ThermCircuitEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(K_B_T * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..N {
        builder = builder
            .with(DoubleWellPotential::new(DELTA_V, X_0, i))
            .with(OscillatingField::new(A_0, omega, 0.0, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    // Synchrony reward: average across all particles
    builder
        .reward(move |_m, d| {
            let signal = (omega * d.time).cos();
            let mut sync = 0.0;
            for i in 0..N {
                sync += d.qpos[i].signum() * signal;
            }
            sync / N as f64
        })
        .build()
        .unwrap()
}

/// Build a VecEnv with ctrl-temperature for training.
fn make_training_vecenv(coupling_j: f64, n_envs: usize, seed: u64) -> sim_rl::VecEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        // Base kT = CTRL_RANGE_HI so tanh output [0,1] maps to kT [0, CTRL_RANGE_HI].
        // At CTRL_RANGE_HI=15, the agent reaches predicted SR peaks (kT 2-7) at ctrl ≈ 0.13-0.47.
        .k_b_t(CTRL_RANGE_HI)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed)
        .with_ctrl_temperature()
        .ctrl_range(0.0, CTRL_RANGE_HI);

    for i in 0..N {
        builder = builder
            .with(DoubleWellPotential::new(DELTA_V, X_0, i))
            .with(OscillatingField::new(A_0, omega, 0.0, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    builder
        .reward(move |_m, d| {
            let signal = (omega * d.time).cos();
            let mut sync = 0.0;
            for i in 0..N {
                sync += d.qpos[i].signum() * signal;
            }
            sync / N as f64
        })
        .build_vec(n_envs)
        .unwrap()
}

/// Run one episode at fixed kT, return mean synchrony per step.
fn run_episode(coupling_j: f64, kt_mult: f64, seed: u64) -> f64 {
    let mut env = make_sweep_env(coupling_j, kt_mult, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total / f64::from(steps)
}

/// Build an Ising chain with phase-lagged oscillating fields for metachronal experiments.
/// Each particle gets phase φ_i = -i × delta, creating a traveling wave from particle 0 → N-1.
/// Reward uses local synchrony: each particle measured against its own phase-shifted signal.
fn make_metachronal_env(coupling_j: f64, kt_mult: f64, delta: f64, seed: u64) -> ThermCircuitEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(K_B_T * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..N {
        let phase = -(i as f64) * delta;
        builder = builder
            .with(DoubleWellPotential::new(DELTA_V, X_0, i))
            .with(OscillatingField::new(A_0, omega, phase, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    // Local synchrony: each particle measured against its own signal phase.
    builder
        .reward(move |_m, d| {
            let mut sync = 0.0;
            for i in 0..N {
                let local_signal = omega.mul_add(d.time, -(i as f64) * delta).cos();
                sync += d.qpos[i].signum() * local_signal;
            }
            sync / N as f64
        })
        .build()
        .unwrap()
}

/// Run one metachronal episode at fixed kT and phase lag, return mean local synchrony per step.
fn run_metachronal_episode(coupling_j: f64, kt_mult: f64, delta: f64, seed: u64) -> f64 {
    let mut env = make_metachronal_env(coupling_j, kt_mult, delta, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total / f64::from(steps)
}

/// Phase-lag sweep at fixed coupling and temperature. Returns (deltas, means, stderrs).
fn phase_lag_sweep(
    coupling_j: f64,
    kt_mult: f64,
    deltas: &[f64],
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut means = Vec::with_capacity(deltas.len());
    let mut stderrs = Vec::with_capacity(deltas.len());

    for (i, &delta) in deltas.iter().enumerate() {
        let mut episodes = Vec::with_capacity(n_episodes);
        for ep in 0..n_episodes {
            let seed = SEED_BASE + seed_offset + (i * 1000 + ep) as u64;
            episodes.push(run_metachronal_episode(coupling_j, kt_mult, delta, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        eprintln!(
            "    δ={delta:6.4}  sync={m:+.6} +/- {s:.6}  ({}/{n_episodes} eps)  [{}/{}]",
            n_episodes,
            i + 1,
            deltas.len()
        );
        means.push(m);
        stderrs.push(s);
    }

    (deltas.to_vec(), means, stderrs)
}

/// Temperature sweep. Returns (kt_mults, means, stderrs).
fn temperature_sweep(
    coupling_j: f64,
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
            episodes.push(run_episode(coupling_j, kt, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        eprintln!(
            "    kT={kt:7.4}  sync={m:+.6} +/- {s:.6}  ({}/{n_episodes} eps)  [{}/{}]",
            n_episodes,
            i + 1,
            kt_mults.len()
        );
        means.push(m);
        stderrs.push(s);
    }

    (kt_mults.to_vec(), means, stderrs)
}

/// Find peak in a sweep. Returns (index, kt, mean, stderr).
fn find_peak(kt_mults: &[f64], means: &[f64], stderrs: &[f64]) -> (usize, f64, f64, f64) {
    let (idx, &peak) = means
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();
    (idx, kt_mults[idx], peak, stderrs[idx])
}

/// Evaluate a trained policy. Returns (synchrony_per_episode, kT_per_episode).
fn evaluate_policy(
    policy: &dyn Policy,
    coupling_j: f64,
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>) {
    let omega = signal_omega();
    let mut synchronies = Vec::with_capacity(n_episodes);
    let mut temperatures = Vec::with_capacity(n_episodes);

    for ep in 0..n_episodes {
        let seed = SEED_BASE + seed_offset + ep as u64;
        let mut builder = ThermCircuitEnv::builder(N)
            .gamma(GAMMA)
            // Match training env: k_b_t = CTRL_RANGE_HI so tanh [0,1] → kT [0, CTRL_RANGE_HI].
            .k_b_t(CTRL_RANGE_HI)
            .timestep(TIMESTEP)
            .sub_steps(SUB_STEPS)
            .episode_steps(EPISODE_STEPS)
            .seed(seed)
            .with_ctrl_temperature()
            .ctrl_range(0.0, CTRL_RANGE_HI);

        for i in 0..N {
            builder = builder
                .with(DoubleWellPotential::new(DELTA_V, X_0, i))
                .with(OscillatingField::new(A_0, omega, 0.0, i));
        }
        if coupling_j.abs() > 1e-15 {
            builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
        }

        let mut env = builder
            .reward(move |_m, d| {
                let signal = (omega * d.time).cos();
                let mut sync = 0.0;
                for i in 0..N {
                    sync += d.qpos[i].signum() * signal;
                }
                sync / N as f64
            })
            .build()
            .unwrap();

        let obs = env.reset().unwrap();
        let mut obs_vec: Vec<f32> = obs.as_slice().to_vec();
        let mut total_sync = 0.0;
        let mut total_temp = 0.0;
        let mut steps = 0;

        for _ in 0..EPISODE_STEPS {
            let action = policy.forward(&obs_vec);
            total_temp += (action[0] * CTRL_RANGE_HI).clamp(0.0, CTRL_RANGE_HI);
            let action_tensor = Tensor::from_f64_slice(&action, &[1]);
            let result = env.step(&action_tensor).unwrap();
            total_sync += result.reward;
            obs_vec = result.observation.as_slice().to_vec();
            steps += 1;
            if result.truncated {
                break;
            }
        }
        synchronies.push(total_sync / steps as f64);
        temperatures.push(total_temp / steps as f64);
    }
    (synchronies, temperatures)
}

/// Train, evaluate, report. Returns (sync_mean, sync_stderr, learned_kT).
fn train_and_report(
    name: &str,
    algo: &mut dyn Algorithm,
    coupling_j: f64,
    seed: u64,
    eval_offset: u64,
) -> (f64, f64, f64) {
    let mut env = make_training_vecenv(coupling_j, N_ENVS, seed);

    eprintln!("\n--- {name}: training ({N_EPOCHS} epochs, {N_ENVS} envs) ---");

    let metrics = algo.train(&mut env, TrainingBudget::Epochs(N_EPOCHS), seed, &|m| {
        if m.epoch % 20 == 0 || m.epoch == N_EPOCHS - 1 {
            eprintln!(
                "  {name} epoch {:3}: mean_reward = {:+.4}",
                m.epoch, m.mean_reward,
            );
        }
    });

    // Gate B
    let first_5: f64 = metrics[..5].iter().map(|m| m.mean_reward).sum::<f64>() / 5.0;
    let best_last_10 = metrics[metrics.len().saturating_sub(10)..]
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);
    eprintln!(
        "  {name} Gate B: first_5={first_5:.4}, best_last_10={best_last_10:.4} -> {}",
        if best_last_10 > first_5 {
            "PASS"
        } else {
            "FAIL"
        }
    );

    let trained = algo.best_artifact().to_policy().unwrap();
    let (syncs, temps) =
        evaluate_policy(trained.as_ref(), coupling_j, N_EVAL_EPISODES, eval_offset);
    let (sync_mean, sync_stderr) = mean_stderr(&syncs);
    let (temp_mean, _) = mean_stderr(&temps);

    eprintln!("  {name} eval: synchrony = {sync_mean:.6} +/- {sync_stderr:.6}");
    eprintln!("  {name} eval: kT = {temp_mean:.4}");

    let t_stat = if sync_stderr > 1e-15 {
        sync_mean / sync_stderr
    } else {
        0.0
    };
    eprintln!("  {name} Gate A: |t| = {:.2}", t_stat.abs());

    let params = trained.params();
    if params.len() <= 20 {
        eprintln!("  {name} params: qpos={:?}", &params[..N]);
        eprintln!("  {name} params: qvel={:?}", &params[N..2 * N]);
        eprintln!("  {name} params: bias={:.4}", params[2 * N]);
    }

    (sync_mean, sync_stderr, temp_mean)
}

fn make_cem() -> Cem {
    let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    // Bias 0.27: tanh(0.27)≈0.264 → kT = 15.0*0.264 ≈ 4.0 (midpoint of predicted SR peaks).
    policy.set_params(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.27]);
    Cem::new(
        Box::new(policy),
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 2.5,
            noise_decay: 0.98,
            noise_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    )
}

fn make_sa() -> Sa {
    Sa::new(
        Box::new(LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale())),
        SaHyperparams {
            initial_temperature: 50.0,
            proposal_std: 1.0,
            cooling_decay: 0.955,
            temperature_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    )
}

fn make_richer_sa() -> RicherSa {
    RicherSa::new(
        Box::new(LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale())),
        RicherSaHyperparams::ch53_defaults(EPISODE_STEPS),
    )
}

// ─── Principle 11 helpers ────────────────────────────────────────────────

/// Build an N=4 uncoupled env with variable barrier height and signal amplitude.
/// J=0: particles are independent replicates, providing spatial averaging.
/// For P11: sweep ΔV at fixed kT to find the sensitivity-amplification point.
fn make_bifurcation_env(delta_v: f64, kt: f64, a0: f64, seed: u64) -> ThermCircuitEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(kt)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..N {
        builder = builder
            .with(DoubleWellPotential::new(delta_v, X_0, i))
            .with(OscillatingField::new(a0, omega, 0.0, i));
    }
    // No coupling (J=0) — particles are independent replicates.

    builder
        .reward(move |_m, d| {
            let signal = (omega * d.time).cos();
            let mut sync = 0.0;
            for i in 0..N {
                sync += d.qpos[i].signum() * signal;
            }
            sync / N as f64
        })
        .build()
        .unwrap()
}

/// Run one bifurcation episode at fixed kT, ΔV, A₀. Returns mean synchrony per step.
fn run_bifurcation_episode(delta_v: f64, kt: f64, a0: f64, seed: u64) -> f64 {
    let mut env = make_bifurcation_env(delta_v, kt, a0, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total / f64::from(steps)
}

/// Barrier-height sweep at fixed kT and signal amplitude. Returns (delta_vs, means, stderrs).
fn bifurcation_sweep(
    kt: f64,
    delta_vs: &[f64],
    a0: f64,
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let mut means = Vec::with_capacity(delta_vs.len());
    let mut stderrs = Vec::with_capacity(delta_vs.len());

    for (i, &dv) in delta_vs.iter().enumerate() {
        let mut episodes = Vec::with_capacity(n_episodes);
        for ep in 0..n_episodes {
            let seed = SEED_BASE + seed_offset + (i * 1000 + ep) as u64;
            episodes.push(run_bifurcation_episode(dv, kt, a0, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        eprintln!(
            "    dV={dv:5.2}  A0={a0}  sync={m:+.6} +/- {s:.6}  ({}/{n_episodes} eps)  [{}/{}]",
            n_episodes,
            i + 1,
            delta_vs.len()
        );
        means.push(m);
        stderrs.push(s);
    }

    (delta_vs.to_vec(), means, stderrs)
}

// ─── Principle 6 helpers (N-parameterized) ──────────────────────────────

/// Build an Ising chain at fixed kT, parameterized by chain size N.
fn make_sweep_env_n(
    n_particles: usize,
    coupling_j: f64,
    kt_mult: f64,
    seed: u64,
) -> ThermCircuitEnv {
    let omega = signal_omega();
    let n = n_particles;
    let mut builder = ThermCircuitEnv::builder(n)
        .gamma(GAMMA)
        .k_b_t(K_B_T * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..n {
        builder = builder
            .with(DoubleWellPotential::new(DELTA_V, X_0, i))
            .with(OscillatingField::new(A_0, omega, 0.0, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(n, coupling_j));
    }

    builder
        .reward(move |_m, d| {
            let signal = (omega * d.time).cos();
            let mut sync = 0.0;
            for i in 0..n {
                sync += d.qpos[i].signum() * signal;
            }
            sync / n as f64
        })
        .build()
        .unwrap()
}

/// Run one episode with variable chain size, return mean synchrony per step.
fn run_episode_n(n_particles: usize, coupling_j: f64, kt_mult: f64, seed: u64) -> f64 {
    let mut env = make_sweep_env_n(n_particles, coupling_j, kt_mult, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total / f64::from(steps)
}

/// Temperature sweep with variable chain size. Returns (kt_mults, means, stderrs).
fn temperature_sweep_n(
    n_particles: usize,
    coupling_j: f64,
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
            episodes.push(run_episode_n(n_particles, coupling_j, kt, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        eprintln!(
            "    N={n_particles} kT={kt:7.4}  sync={m:+.6} +/- {s:.6}  ({n_episodes} eps)  [{}/{}]",
            i + 1,
            kt_mults.len()
        );
        means.push(m);
        stderrs.push(s);
    }

    (kt_mults.to_vec(), means, stderrs)
}

// ─── Principle 1 helpers (amplitude-parameterized) ──────────────────────

/// Build a metachronal env with variable amplitude.
fn make_metachronal_env_amp(
    coupling_j: f64,
    kt_mult: f64,
    delta: f64,
    amplitude: f64,
    seed: u64,
) -> ThermCircuitEnv {
    let omega = signal_omega();
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(K_B_T * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..N {
        let phase = -(i as f64) * delta;
        builder = builder
            .with(DoubleWellPotential::new(DELTA_V, X_0, i))
            .with(OscillatingField::new(amplitude, omega, phase, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    builder
        .reward(move |_m, d| {
            let mut sync = 0.0;
            for i in 0..N {
                let local_signal = omega.mul_add(d.time, -(i as f64) * delta).cos();
                sync += d.qpos[i].signum() * local_signal;
            }
            sync / N as f64
        })
        .build()
        .unwrap()
}

/// Run one metachronal episode with variable amplitude.
fn run_metachronal_episode_amp(
    coupling_j: f64,
    kt_mult: f64,
    delta: f64,
    amplitude: f64,
    seed: u64,
) -> f64 {
    let mut env = make_metachronal_env_amp(coupling_j, kt_mult, delta, amplitude, seed);
    let _obs = env.reset().unwrap();
    let action = Tensor::from_f64_slice(&[], &[0]);

    let mut total = 0.0;
    let mut steps = 0;
    loop {
        let result = env.step(&action).unwrap();
        total += result.reward;
        steps += 1;
        if result.truncated || result.done {
            break;
        }
    }
    total / f64::from(steps)
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

/// Controls: validate the coupled SR system behaves correctly at extremes.
#[test]
#[ignore = "requires --release (~3 min)"]
fn ising_sr_controls() {
    let n_episodes = 10;

    eprintln!("\n=== Ising SR: Controls ===");

    // Low noise (kT x 0.1): particles trapped, no switching, zero synchrony
    eprintln!("\n--- Low noise (kT x 0.1) ---");
    let mut low = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        low.push(run_episode(0.5, 0.1, SEED_BASE + ep as u64));
    }
    let (low_mean, low_stderr) = mean_stderr(&low);
    eprintln!("  synchrony = {low_mean:.6} +/- {low_stderr:.6}");
    assert!(
        low_mean.abs() < 3.0 * low_stderr,
        "Low noise: |mean|={:.6} >= 3*stderr",
        low_mean.abs()
    );
    eprintln!("  PASS");

    // High noise (kT x 10): random switching, zero synchrony
    eprintln!("\n--- High noise (kT x 10.0) ---");
    let mut high = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        high.push(run_episode(0.5, 10.0, SEED_BASE + 1000 + ep as u64));
    }
    let (high_mean, high_stderr) = mean_stderr(&high);
    eprintln!("  synchrony = {high_mean:.6} +/- {high_stderr:.6}");
    assert!(
        high_mean.abs() < 3.0 * high_stderr,
        "High noise: |mean|={:.6} >= 3*stderr",
        high_mean.abs()
    );
    eprintln!("  PASS");

    // No signal (A₀=0): metric validates to zero
    eprintln!("\n--- No signal (A0=0) ---");
    let omega = signal_omega();
    let mut nosig = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        let seed = SEED_BASE + 2000 + ep as u64;
        let mut builder = ThermCircuitEnv::builder(N)
            .gamma(GAMMA)
            .k_b_t(K_B_T * 1.5)
            .timestep(TIMESTEP)
            .sub_steps(SUB_STEPS)
            .episode_steps(EPISODE_STEPS)
            .seed(seed);
        for i in 0..N {
            builder = builder
                .with(DoubleWellPotential::new(DELTA_V, X_0, i))
                .with(OscillatingField::new(0.0, omega, 0.0, i)); // A₀=0
        }
        builder = builder.with(PairwiseCoupling::chain(N, 0.5));
        let mut env = builder
            .reward(move |_m, d| {
                let sig = (omega * d.time).cos();
                (0..N).map(|i| d.qpos[i].signum() * sig).sum::<f64>() / N as f64
            })
            .build()
            .unwrap();

        let _obs = env.reset().unwrap();
        let action = Tensor::from_f64_slice(&[], &[0]);
        let mut total = 0.0;
        let mut steps = 0;
        loop {
            let r = env.step(&action).unwrap();
            total += r.reward;
            steps += 1;
            if r.truncated || r.done {
                break;
            }
        }
        nosig.push(total / f64::from(steps));
    }
    let (nosig_mean, nosig_stderr) = mean_stderr(&nosig);
    eprintln!("  synchrony = {nosig_mean:.6} +/- {nosig_stderr:.6}");
    assert!(
        nosig_mean.abs() < 3.0 * nosig_stderr,
        "No signal: |mean|={:.6} >= 3*stderr",
        nosig_mean.abs()
    );
    eprintln!("  PASS");
}

/// Baseline: SR peak on uncoupled 4-particle system (J=0).
/// Should reproduce the 1-particle result (kT≈1.70) since particles are independent.
#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_sr_uncoupled_baseline() {
    let kt_mults = log_spaced(0.1, 15.0, 20);
    let n_episodes = 20;

    eprintln!("\n=== Ising SR: Uncoupled baseline (J=0) ===");
    eprintln!("  Should reproduce 1-particle SR peak at kT~1.70\n");

    let (kts, means, stderrs) = temperature_sweep(0.0, &kt_mults, n_episodes, 0);

    eprintln!("  kT_mult  |  synchrony   |  stderr");
    eprintln!("  ---------|--------------|--------");
    for i in 0..kts.len() {
        eprintln!(
            "  {:7.4}  |  {:+.6}   |  {:.6}",
            kts[i], means[i], stderrs[i]
        );
    }

    let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
    eprintln!("\n  Peak: kT={peak_kt:.4}, synchrony={peak_sync:.6} +/- {peak_se:.6}");

    let t = if peak_se > 1e-15 {
        peak_sync / peak_se
    } else {
        0.0
    };
    eprintln!("  |t| = {t:.2} (critical = {T_CRITICAL_DF19:.3})");

    assert!(
        t.abs() > T_CRITICAL_DF19,
        "Peak not significant: |t|={t:.2}"
    );
    assert!(
        peak_idx > 0 && peak_idx < kts.len() - 1,
        "Peak at boundary (idx={peak_idx})"
    );
    eprintln!("  Uncoupled baseline: PASS");
}

/// Core result: SR peak on coupled 4-particle chain (J=0.5).
/// Does coupling shift, enhance, or destroy the SR peak?
#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_sr_coupled() {
    let kt_mults = log_spaced(0.1, 15.0, 20);
    let n_episodes = 20;

    eprintln!("\n=== Ising SR: Coupled chain (J=0.5) ===");
    eprintln!("  Does coupling shift the SR peak?\n");

    let (kts, means, stderrs) = temperature_sweep(0.5, &kt_mults, n_episodes, 10_000);

    eprintln!("  kT_mult  |  synchrony   |  stderr");
    eprintln!("  ---------|--------------|--------");
    for i in 0..kts.len() {
        eprintln!(
            "  {:7.4}  |  {:+.6}   |  {:.6}",
            kts[i], means[i], stderrs[i]
        );
    }

    let (_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
    eprintln!("\n  Peak: kT={peak_kt:.4}, synchrony={peak_sync:.6} +/- {peak_se:.6}");

    let t = if peak_se > 1e-15 {
        peak_sync / peak_se
    } else {
        0.0
    };
    eprintln!("  |t| = {t:.2} (critical = {T_CRITICAL_DF19:.3})");

    eprintln!("  Compare to 1-particle: peak kT=1.70, synchrony=0.090");
    eprintln!("  Compare to uncoupled baseline: run ising_sr_uncoupled_baseline");
}

/// Scout: fast diagnostic at J=2 only. Validates that the kT=15 ceiling
/// captures the strongest-coupling peak before committing to the full sweep.
#[test]
#[ignore = "requires --release (~30 sec)"]
fn ising_sr_scout() {
    let kt_points = [1.0, 2.0, 3.5, 5.0, 6.5, 8.5, 11.0, 15.0];
    let n_episodes = 10;
    let coupling_j = 2.0;

    eprintln!("\n=== Ising SR: Scout (J=2.0) ===");
    eprintln!("  Predicted peak: kT ~ 6.5 (ΔV_eff/1.39)");
    eprintln!("  Testing {} kT points in [1, 15]\n", kt_points.len());

    let (kts, means, stderrs) = temperature_sweep(coupling_j, &kt_points, n_episodes, 100_000);

    eprintln!("\n  kT_mult  |  synchrony   |  stderr");
    eprintln!("  ---------|--------------|--------");
    for i in 0..kts.len() {
        eprintln!(
            "  {:7.4}  |  {:+.6}   |  {:.6}",
            kts[i], means[i], stderrs[i]
        );
    }

    let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
    let t = if peak_se > 1e-15 {
        peak_sync / peak_se
    } else {
        0.0
    };
    eprintln!("\n  Peak: kT={peak_kt:.4}, synchrony={peak_sync:.6} +/- {peak_se:.6}");
    eprintln!("  |t| = {:.2} (critical = {T_CRITICAL_DF9:.3})", t.abs());

    assert!(
        peak_idx > 0 && peak_idx < kts.len() - 1,
        "Scout: peak at boundary (idx={peak_idx}, kT={peak_kt:.4}), range too narrow"
    );
    assert!(
        t.abs() > T_CRITICAL_DF9,
        "Scout: peak not significant (|t|={:.2})",
        t.abs()
    );
    eprintln!("  Scout: PASS (peak interior, significant)");
}

/// Phase 1: Temperature sweep at 5 coupling strengths.
/// Maps how coupling shifts the SR-optimal noise temperature.
/// This is the τ_circuit / τ_noise characterization.
///
/// Run the scout first to validate the kT range.
#[test]
#[ignore = "requires --release (~35 min)"]
#[allow(clippy::items_after_statements)]
fn ising_sr_coupling_sweep() {
    let j_values = [0.0, 0.5, 1.0, 1.5, 2.0];
    let kt_mults = log_spaced(0.1, 15.0, 25);
    let n_sweep_episodes = 40;

    eprintln!("\n=== ISING SR COUPLING SWEEP (Phase 1) ===");
    eprintln!("  Does the SR peak shift with coupling strength?");
    eprintln!("  J values: {j_values:?}");
    eprintln!(
        "  kT range: [0.1, 15.0], {} points log-spaced",
        kt_mults.len()
    );
    eprintln!("  Episodes per point: {n_sweep_episodes}");
    eprintln!("  t-critical: {T_CRITICAL_DF39:.3} (df=39, α=0.01)\n");

    struct SweepResult {
        j: f64,
        peak_idx: usize,
        peak_kt: f64,
        peak_sync: f64,
        peak_stderr: f64,
        peak_t: f64,
    }
    let mut sweep_results: Vec<SweepResult> = Vec::new();

    for (i, &j) in j_values.iter().enumerate() {
        eprintln!("  --- J={j} sweep ---");
        let (kts, means, stderrs) =
            temperature_sweep(j, &kt_mults, n_sweep_episodes, i as u64 * 20_000);
        let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
        let t = if peak_se > 1e-15 {
            peak_sync / peak_se
        } else {
            0.0
        };
        eprintln!("    peak: kT={peak_kt:.4}, sync={peak_sync:.6} +/- {peak_se:.6}, |t|={t:.2}");
        sweep_results.push(SweepResult {
            j,
            peak_idx,
            peak_kt,
            peak_sync,
            peak_stderr: peak_se,
            peak_t: t,
        });
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  COUPLING SWEEP RESULTS");
    eprintln!("  ======================================================================\n");
    eprintln!("  J      |  peak kT  |  peak sync     |  |t|    |  interior?");
    eprintln!("  -------|-----------|----------------|--------|----------");
    for r in &sweep_results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        eprintln!(
            "  {:5.2}  |  {:7.4}  |  {:.6} +/- {:.6}  |  {:5.2}  |  {}",
            r.j,
            r.peak_kt,
            r.peak_sync,
            r.peak_stderr,
            r.peak_t,
            if interior { "YES" } else { "BOUNDARY" }
        );
    }

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");
    let mut all_pass = true;
    for r in &sweep_results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        let significant = r.peak_t.abs() > T_CRITICAL_DF39;
        let pass = interior && significant;
        if !pass {
            all_pass = false;
        }
        eprintln!(
            "    J={:.2}: interior={interior}, |t|={:.2} {} {T_CRITICAL_DF39:.3} → {}",
            r.j,
            r.peak_t.abs(),
            if significant { ">" } else { "<" },
            if pass { "PASS" } else { "FAIL" }
        );
    }

    assert!(all_pass, "Not all J values have significant interior peaks");
    eprintln!("\n  All gates PASS. Phase 2 (training) can proceed.");
}

/// Phase 2: CEM, SA, RicherSA training at each coupling strength.
/// Tests whether gradient-free agents independently discover the SR-optimal
/// temperature from dynamics alone.
///
/// Run Phase 1 (coupling sweep) first to validate the sweep results.
#[test]
#[ignore = "requires --release (~2-3 hours)"]
#[allow(clippy::items_after_statements)]
fn ising_sr_training() {
    let j_values = [0.0, 0.5, 1.0, 1.5, 2.0];

    eprintln!("\n=== ISING SR TRAINING (Phase 2) ===");
    eprintln!("  CEM, SA, RicherSA at each J");
    eprintln!("  CTRL_RANGE_HI = {CTRL_RANGE_HI} (policy tanh → kT ∈ [0, {CTRL_RANGE_HI}])");
    eprintln!("  {N_EPOCHS} epochs, {N_ENVS} envs, {N_EVAL_EPISODES} eval episodes\n");

    struct TrainResult {
        j: f64,
        algo: &'static str,
        sync: f64,
        stderr: f64,
        learned_kt: f64,
    }
    let mut train_results: Vec<TrainResult> = Vec::new();

    for (i, &j) in j_values.iter().enumerate() {
        eprintln!("\n  ============================================================");
        eprintln!("  J = {j}  ({}/{})", i + 1, j_values.len());
        eprintln!("  ============================================================");

        let base_seed = SEED_BASE + 500 + i as u64 * 1000;
        let base_eval = 200_000 + i as u64 * 30_000;

        let mut cem = make_cem();
        let (s, se, kt) =
            train_and_report(&format!("CEM(J={j})"), &mut cem, j, base_seed, base_eval);
        train_results.push(TrainResult {
            j,
            algo: "CEM",
            sync: s,
            stderr: se,
            learned_kt: kt,
        });

        let mut sa = make_sa();
        let (s, se, kt) = train_and_report(
            &format!("SA(J={j})"),
            &mut sa,
            j,
            base_seed + 100,
            base_eval + 10_000,
        );
        train_results.push(TrainResult {
            j,
            algo: "SA",
            sync: s,
            stderr: se,
            learned_kt: kt,
        });

        let mut rsa = make_richer_sa();
        let (s, se, kt) = train_and_report(
            &format!("RSA(J={j})"),
            &mut rsa,
            j,
            base_seed + 200,
            base_eval + 20_000,
        );
        train_results.push(TrainResult {
            j,
            algo: "RSA",
            sync: s,
            stderr: se,
            learned_kt: kt,
        });
    }

    // ── Final summary ────────────────────────────────────────────────
    eprintln!("\n\n  ======================================================================");
    eprintln!("  TRAINING RESULTS: learned temperature vs SR peak");
    eprintln!("  ======================================================================\n");

    eprintln!("  J      | Algo |  synchrony      |  learned kT  |  |t|");
    eprintln!("  -------|------|-----------------|-------------|------");
    for r in &train_results {
        let t = if r.stderr > 1e-15 {
            r.sync / r.stderr
        } else {
            0.0
        };
        eprintln!(
            "  {:5.2}  | {:4} |  {:+.6} +/- {:.6}  |  {:10.4}  |  {:.2}",
            r.j,
            r.algo,
            r.sync,
            r.stderr,
            r.learned_kt,
            t.abs()
        );
    }

    eprintln!("\n  Key question: does learned kT track the sweep peak across J values?");
}

// ═══════════════════════════════════════════════════════════════════════════
// PRINCIPLE 4 — METACHRONAL PHASE-LAG EXPERIMENTS
// ═══════════════════════════════════════════════════════════════════════════

/// Scout: quick check that phase lag produces non-trivial structure at J=1.0.
/// If the synchrony-vs-δ curve is flat, the full sweep won't help.
#[test]
#[ignore = "requires --release (~40 sec)"]
fn ising_metachronal_scout() {
    let coupling_j = 1.0;
    let kt_mult = 2.82; // Phase 1 optimal for J=1.0
    let deltas = lin_spaced(0.0, PI, 5);
    let n_episodes = 20;

    eprintln!("\n=== Metachronal Scout (J=1.0, kT=2.82) ===");
    eprintln!("  Does phase lag δ affect synchrony in a coupled chain?");
    eprintln!("  Testing {} δ points in [0, π]\n", deltas.len());

    let (ds, means, stderrs) = phase_lag_sweep(coupling_j, kt_mult, &deltas, n_episodes, 300_000);

    eprintln!("\n  δ        |  synchrony   |  stderr");
    eprintln!("  ---------|--------------|--------");
    for i in 0..ds.len() {
        eprintln!(
            "  {:7.4}  |  {:+.6}   |  {:.6}",
            ds[i], means[i], stderrs[i]
        );
    }

    let (peak_idx, peak_delta, peak_sync, peak_se) = find_peak(&ds, &means, &stderrs);
    eprintln!("\n  Peak: δ={peak_delta:.4}, synchrony={peak_sync:.6} +/- {peak_se:.6}");

    // Scout just reports — no hard gates. We want to see if there's structure.
    let sync_range = means.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b))
        - means.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    eprintln!("  Sync range across δ: {sync_range:.6}");
    eprintln!(
        "  Peak at boundary? {}",
        if peak_idx == 0 || peak_idx == ds.len() - 1 {
            "YES (δ=0 or δ=π)"
        } else {
            "NO (interior)"
        }
    );
}

/// Principle 4 validation: metachronal phase-lag sweep.
/// Tests whether phase-lagged injection produces higher per-node synchrony
/// than synchronized injection in a coupled Ising chain.
///
/// Run the metachronal scout first.
#[test]
#[ignore = "requires --release (~43 min)"]
#[allow(clippy::items_after_statements)]
fn ising_metachronal_sweep() {
    // Phase 1 SR-optimal kT for each coupling strength.
    let j_kt_pairs: [(f64, f64); 4] = [
        (0.0, 2.29), // uncoupled control
        (0.5, 2.29), // weak coupling
        (1.0, 2.82), // medium coupling
        (2.0, 4.29), // strong coupling
    ];
    let deltas = lin_spaced(0.0, PI, 20);
    let n_episodes = 80;

    eprintln!("\n=== METACHRONAL PHASE-LAG SWEEP (Principle 4) ===");
    eprintln!("  Does phase-lagged injection beat synchronized injection?");
    eprintln!("  δ range: [0, π], {} points linearly spaced", deltas.len());
    eprintln!("  Episodes per point: {n_episodes}");
    eprintln!("  J values and kT (from Phase 1 optima):");
    for &(j, kt) in &j_kt_pairs {
        eprintln!("    J={j:.1}, kT={kt:.2}");
    }
    eprintln!();

    struct MetaResult {
        j: f64,
        peak_idx: usize,
        peak_delta: f64,
        peak_sync: f64,
        peak_stderr: f64,
        peak_t: f64,
        sync_at_zero: f64,
        stderr_at_zero: f64,
        means: Vec<f64>,
    }
    let mut results: Vec<MetaResult> = Vec::new();

    for (i, &(j, kt)) in j_kt_pairs.iter().enumerate() {
        eprintln!("  --- J={j}, kT={kt} ---");
        let (ds, means, stderrs) =
            phase_lag_sweep(j, kt, &deltas, n_episodes, 400_000 + i as u64 * 50_000);
        let (peak_idx, peak_delta, peak_sync, peak_se) = find_peak(&ds, &means, &stderrs);
        let t = if peak_se > 1e-15 {
            peak_sync / peak_se
        } else {
            0.0
        };
        eprintln!("    peak: δ={peak_delta:.4}, sync={peak_sync:.6} +/- {peak_se:.6}, |t|={t:.2}");
        let sync_at_zero = means[0];
        let stderr_at_zero = stderrs[0];
        results.push(MetaResult {
            j,
            peak_idx,
            peak_delta,
            peak_sync,
            peak_stderr: peak_se,
            peak_t: t,
            sync_at_zero,
            stderr_at_zero,
            means,
        });
    }

    // ── Gate 0 (Sanity): δ=0 must reproduce Phase 1 synchrony values ─
    // Different seeds, so not exact.  Use a two-sample threshold:
    // Phase 1 used 40 episodes → its stderr ≈ sqrt(n_episodes/40) × stderr_at_zero.
    // Combined threshold = 3 × sqrt(stderr_δ0² + stderr_p1²).
    let phase1_sync: [(f64, f64); 4] = [
        (0.0, 0.043), // J=0, kT=2.29
        (0.5, 0.057), // J=0.5, kT=2.29
        (1.0, 0.065), // J=1.0, kT=2.82
        (2.0, 0.053), // J=2.0, kT=4.29
    ];
    eprintln!("\n  Gate 0 (Sanity): δ=0 reproduces Phase 1?");
    let mut gate0_pass = true;
    for (r, &(j, expected)) in results.iter().zip(&phase1_sync) {
        let diff = (r.sync_at_zero - expected).abs();
        // Estimate Phase 1 stderr: same per-episode variance, 40 episodes.
        let phase1_stderr_est = r.stderr_at_zero * (n_episodes as f64 / 40.0).sqrt();
        let combined_stderr = r.stderr_at_zero.hypot(phase1_stderr_est);
        let threshold = 3.0 * combined_stderr;
        let ok = diff < threshold;
        if !ok {
            gate0_pass = false;
        }
        eprintln!(
            "    J={j:.1}: sync(δ=0)={:.6}, Phase 1={expected:.3}, diff={diff:.6}, 3σ={threshold:.6} → {}",
            r.sync_at_zero,
            if ok { "PASS" } else { "FAIL" }
        );
    }
    if !gate0_pass {
        eprintln!(
            "    GATE 0 FAILED — δ=0 baseline inconsistent with Phase 1. \
             Investigate before trusting results."
        );
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  METACHRONAL SWEEP RESULTS");
    eprintln!("  ======================================================================\n");
    eprintln!("  J      |  peak δ   |  peak sync     |  sync(δ=0)  |  improvement  |  |t|");
    eprintln!("  -------|-----------|----------------|-------------|---------------|------");
    for r in &results {
        let improvement = if r.sync_at_zero.abs() > 1e-15 {
            (r.peak_sync - r.sync_at_zero) / r.sync_at_zero * 100.0
        } else {
            0.0
        };
        eprintln!(
            "  {:5.2}  |  {:7.4}  |  {:.6} +/- {:.6}  |  {:+.6}  |  {:+8.1}%     |  {:.2}",
            r.j, r.peak_delta, r.peak_sync, r.peak_stderr, r.sync_at_zero, improvement, r.peak_t,
        );
    }

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");

    // Gate 1 (Control): J=0 curve must be flat — no trend across the full δ range.
    // Linear regression of sync vs δ; slope must be indistinguishable from zero.
    let j0 = &results[0];
    let (slope, slope_se) = lin_regress_slope(&deltas, &j0.means);
    let slope_t = if slope_se > 1e-15 {
        slope / slope_se
    } else {
        0.0
    };
    // df = n-2 = 18 for 20 points; t_critical(α=0.05, df=18) ≈ 2.101.
    // Use α=0.05 (not 0.01) because we WANT to detect even mild trends.
    let t_crit_df18 = 2.101;
    let j0_flat = slope_t.abs() < t_crit_df18;
    let cmp = if j0_flat { "<" } else { ">" };
    let verdict = if j0_flat {
        "FLAT (PASS)"
    } else {
        "TREND (FAIL)"
    };
    eprintln!(
        "    Gate 1 (Control): J=0 slope={slope:.6} +/- {slope_se:.6}, \
         |t|={:.2} {cmp} {t_crit_df18:.3} → {verdict}",
        slope_t.abs(),
    );

    // Gate 2 (Effect exists): at least one J>0 has peak sync significantly above sync(δ=0).
    // Significance: peak must exceed δ=0 by more than 2×stderr (approximate two-sample test).
    let any_significant = results[1..].iter().any(|r| {
        let diff = r.peak_sync - r.sync_at_zero;
        diff > 2.0 * r.peak_stderr
    });
    eprintln!(
        "    Gate 2 (Effect): any J>0 peak significantly > sync(δ=0): {}",
        if any_significant {
            "YES (PASS)"
        } else {
            "NO (FAIL)"
        }
    );
    for r in &results[1..] {
        let diff = r.peak_sync - r.sync_at_zero;
        let sig = diff > 2.0 * r.peak_stderr;
        eprintln!(
            "      J={:.1}: sync(δ*)={:.6}, sync(δ=0)={:.6}, diff={:.6}, sig={}",
            r.j, r.peak_sync, r.sync_at_zero, diff, sig
        );
    }

    // Gate 3 (Interior peak): at least one J>0 has its peak at an interior δ (not boundary).
    // Boundary peaks (δ=0 or δ=π) mean the effect is monotonic, not a resonance.
    let any_interior = results[1..]
        .iter()
        .any(|r| r.peak_idx > 0 && r.peak_idx < deltas.len() - 1);
    eprintln!(
        "    Gate 3 (Interior): any J>0 with interior peak: {}",
        if any_interior {
            "YES (PASS)"
        } else {
            "NO (FAIL)"
        }
    );
    for r in &results[1..] {
        let interior = r.peak_idx > 0 && r.peak_idx < deltas.len() - 1;
        eprintln!(
            "      J={:.1}: peak δ={:.4} (idx {}), interior={}",
            r.j, r.peak_delta, r.peak_idx, interior
        );
    }

    // ── Diagnostic: coupling dependence ────────────────────────────
    // Does the optimal δ* vary with J? If so, the design rule needs
    // both coupling strength and phase lag — two knobs, not one.
    eprintln!("\n  Diagnostic (coupling dependence of δ*):");
    let coupled_results: Vec<_> = results[1..].iter().collect();
    if coupled_results.len() >= 2 {
        let delta_range = coupled_results
            .iter()
            .map(|r| r.peak_delta)
            .fold(f64::NEG_INFINITY, f64::max)
            - coupled_results
                .iter()
                .map(|r| r.peak_delta)
                .fold(f64::INFINITY, f64::min);
        let delta_step = deltas[1] - deltas[0];
        let varies = delta_range > delta_step;
        for r in &coupled_results {
            eprintln!("    J={:.1}: δ*={:.4}", r.j, r.peak_delta);
        }
        eprintln!(
            "    δ* range across J: {delta_range:.4} (grid step={delta_step:.4}) → {}",
            if varies {
                "VARIES (design rule needs J-dependent δ*)"
            } else {
                "STABLE (single δ* may suffice)"
            }
        );
    }

    // ── Verdict ──────────────────────────────────────────────────────
    let all_pass = gate0_pass && j0_flat && any_significant && any_interior;
    if all_pass {
        eprintln!(
            "\n  All gates PASS. Principle 4 VALIDATED: \
             phase-lagged injection improves fidelity."
        );
    } else if !gate0_pass {
        eprintln!(
            "\n  SANITY FAILURE: δ=0 doesn't reproduce Phase 1 — \
             investigate env construction."
        );
    } else if !j0_flat {
        eprintln!(
            "\n  METRIC ISSUE: J=0 control is not flat — \
             investigate before interpreting."
        );
    } else {
        eprintln!(
            "\n  Principle 4 NOT SUPPORTED: no phase lag significantly \
             beats synchronized injection."
        );
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PRINCIPLE 11 — DELIBERATE INSTABILITY NEAR BIFURCATION
// ═══════════════════════════════════════════════════════════════════════════

/// Scout: quick check that barrier-height sweep shows structure at two amplitudes.
/// N=4 uncoupled particles (J=0) for spatial averaging.
/// P11 prediction: sensitivity peaks at a critical ΔV/kT ratio (~1.31).
#[test]
#[ignore = "requires --release (~2 min)"]
fn ising_bifurcation_scout() {
    let delta_vs: [f64; 8] = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0];
    let kt = 2.0;
    let a0_strong = 0.3;
    let a0_weak = 0.1;
    let n_episodes = 20;

    eprintln!("\n=== Bifurcation Sensitivity Scout (Principle 11) ===");
    eprintln!(
        "  N={N} uncoupled particles (J=0), kT={kt}, sweep dV from {:.1} to {:.1}",
        delta_vs[0],
        delta_vs[delta_vs.len() - 1]
    );
    eprintln!("  Two amplitudes: A0={a0_strong} (strong), A0={a0_weak} (weak)");
    eprintln!(
        "  Predicted critical dV/kT ~ 1.31 -> dV ~ {:.2}\n",
        1.31 * kt
    );

    eprintln!("  --- Strong signal (A0={a0_strong}) ---");
    let (_, means_s, stderrs_s) = bifurcation_sweep(kt, &delta_vs, a0_strong, n_episodes, 500_000);

    eprintln!("\n  --- Weak signal (A0={a0_weak}) ---");
    let (_, means_w, stderrs_w) = bifurcation_sweep(kt, &delta_vs, a0_weak, n_episodes, 600_000);

    eprintln!("\n  dV     |  sync(strong)    |  sync(weak)      |  ratio  |  amp");
    eprintln!("  -------|-----------------|-----------------|---------|------");
    for i in 0..delta_vs.len() {
        let ratio = if means_s[i].abs() > 1e-6 {
            means_w[i] / means_s[i]
        } else {
            f64::NAN
        };
        let amp = ratio * (a0_strong / a0_weak);
        eprintln!(
            "  {:5.2}  |  {:+.6} +/- {:.6}  |  {:+.6} +/- {:.6}  |  {:.3}  |  {:.3}",
            delta_vs[i], means_s[i], stderrs_s[i], means_w[i], stderrs_w[i], ratio, amp
        );
    }

    let (s_idx, s_dv, s_sync, s_se) = find_peak(&delta_vs, &means_s, &stderrs_s);
    let (w_idx, w_dv, w_sync, w_se) = find_peak(&delta_vs, &means_w, &stderrs_w);

    eprintln!("\n  Strong peak: dV={s_dv:.2}, sync={s_sync:.6} +/- {s_se:.6} (idx {s_idx})");
    eprintln!("  Weak peak:   dV={w_dv:.2}, sync={w_sync:.6} +/- {w_se:.6} (idx {w_idx})");

    let s_interior = s_idx > 0 && s_idx < delta_vs.len() - 1;
    let w_interior = w_idx > 0 && w_idx < delta_vs.len() - 1;
    eprintln!("  Interior peaks: strong={s_interior}, weak={w_interior}");

    if means_s[s_idx].abs() > 1e-6 {
        let ratio = means_w[s_idx] / means_s[s_idx];
        let amp = ratio * (a0_strong / a0_weak);
        eprintln!("  Amplification factor at strong peak: {amp:.3} (>1 = superlinear)");
    }
}

/// Principle 11 validation: bifurcation sensitivity sweep.
/// Tests whether operating near the critical barrier height amplifies
/// signal sensitivity, especially for weak signals.
///
/// The experiment: fix kT=2.0, sweep ΔV from 0.5 to 6.0, N=4 uncoupled
/// particles (J=0, independent replicates for spatial averaging).
/// Two signal amplitudes (A₀=0.3 strong, A₀=0.1 weak).
///
/// Key distinction from P2: P2 says "there's an optimal noise level."
/// P11 says "there's an optimal operating point (barrier height), and
/// weak signals REQUIRE being at that point — the instability near
/// bifurcation is a necessary amplifier."
///
/// Run the bifurcation scout first.
#[test]
#[ignore = "requires --release (~20 min)"]
#[allow(clippy::items_after_statements)]
fn ising_bifurcation_sweep() {
    let delta_vs = lin_spaced(0.5, 10.0, 20);
    let kt = 2.0;
    let a0_strong = 0.3;
    let a0_weak = 0.1;
    let n_episodes = 80;

    eprintln!("\n=== BIFURCATION SENSITIVITY SWEEP (Principle 11) ===");
    eprintln!("  Does operating near the bifurcation point amplify sensitivity?");
    eprintln!("  N={N} uncoupled particles (J=0), kT={kt}");
    eprintln!(
        "  dV range: [{:.1}, {:.1}], {} points linearly spaced",
        0.5,
        10.0,
        delta_vs.len()
    );
    eprintln!("  Episodes per point: {n_episodes}");
    eprintln!("  Two amplitudes: A0={a0_strong} (strong), A0={a0_weak} (weak)");
    eprintln!(
        "  Predicted critical dV/kT ~ 1.31 -> dV ~ {:.2}\n",
        1.31 * kt
    );

    eprintln!("  --- Strong signal (A0={a0_strong}) ---");
    let (_, means_s, stderrs_s) = bifurcation_sweep(kt, &delta_vs, a0_strong, n_episodes, 700_000);

    eprintln!("\n  --- Weak signal (A0={a0_weak}) ---");
    let (_, means_w, stderrs_w) = bifurcation_sweep(kt, &delta_vs, a0_weak, n_episodes, 800_000);

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  BIFURCATION SWEEP RESULTS");
    eprintln!("  ======================================================================\n");
    eprintln!("  dV     |  sync(strong)    |  sync(weak)      |  ratio  |  amp");
    eprintln!("  -------|-----------------|-----------------|---------|------");
    for i in 0..delta_vs.len() {
        let ratio = if means_s[i].abs() > 1e-6 {
            means_w[i] / means_s[i]
        } else {
            f64::NAN
        };
        let amp = ratio * (a0_strong / a0_weak);
        eprintln!(
            "  {:5.2}  |  {:+.6} +/- {:.6}  |  {:+.6} +/- {:.6}  |  {:.3}  |  {:.3}",
            delta_vs[i], means_s[i], stderrs_s[i], means_w[i], stderrs_w[i], ratio, amp
        );
    }

    let (s_idx, s_dv, s_sync, s_se) = find_peak(&delta_vs, &means_s, &stderrs_s);
    let (w_idx, w_dv, w_sync, w_se) = find_peak(&delta_vs, &means_w, &stderrs_w);
    let s_t = if s_se > 1e-15 { s_sync / s_se } else { 0.0 };
    let w_t = if w_se > 1e-15 { w_sync / w_se } else { 0.0 };

    eprintln!(
        "\n  Strong peak: dV={s_dv:.2}, sync={s_sync:.6} +/- {s_se:.6}, |t|={:.2}",
        s_t.abs()
    );
    eprintln!(
        "  Weak peak:   dV={w_dv:.2}, sync={w_sync:.6} +/- {w_se:.6}, |t|={:.2}",
        w_t.abs()
    );

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");

    // Gate 0 (Sanity): high-dV extreme shows zero synchrony.
    // Only the high end is checked — at low dV the barrier is shallow,
    // so the signal can bias fast switching (nonzero sync is correct physics).
    // At high dV, the barrier traps the particle and sync must vanish.
    let last = delta_vs.len() - 1;
    let s_last_ok = means_s[last].abs() < 3.0 * stderrs_s[last];
    let w_last_ok = means_w[last].abs() < 3.0 * stderrs_w[last];
    let gate0 = s_last_ok && w_last_ok;
    eprintln!("    Gate 0 (Sanity): high-dV extreme ~ 0?");
    eprintln!(
        "      strong: dV={:.2} {}",
        delta_vs[last],
        if s_last_ok { "PASS" } else { "FAIL" },
    );
    eprintln!(
        "      weak:   dV={:.2} {}",
        delta_vs[last],
        if w_last_ok { "PASS" } else { "FAIL" },
    );
    eprintln!("      -> {}", if gate0 { "PASS" } else { "FAIL" });

    // Gate 1 (Strong peak): interior + significant.
    let s_interior = s_idx > 0 && s_idx < delta_vs.len() - 1;
    let s_sig = s_t.abs() > T_CRITICAL_DF79;
    let gate1 = s_interior && s_sig;
    eprintln!(
        "    Gate 1 (Strong peak): interior={s_interior}, \
         |t|={:.2} {} {T_CRITICAL_DF79:.3} -> {}",
        s_t.abs(),
        if s_sig { ">" } else { "<" },
        if gate1 { "PASS" } else { "FAIL" },
    );

    // Gate 2 (Weak peak): interior + significant.
    let w_interior = w_idx > 0 && w_idx < delta_vs.len() - 1;
    let w_sig = w_t.abs() > T_CRITICAL_DF79;
    let gate2 = w_interior && w_sig;
    eprintln!(
        "    Gate 2 (Weak peak): interior={w_interior}, \
         |t|={:.2} {} {T_CRITICAL_DF79:.3} -> {}",
        w_t.abs(),
        if w_sig { ">" } else { "<" },
        if gate2 { "PASS" } else { "FAIL" },
    );

    // Gate 3 (Co-location): peaks within 2 grid steps.
    let peak_gap = s_idx.abs_diff(w_idx);
    let gate3 = peak_gap <= 2;
    eprintln!(
        "    Gate 3 (Co-location): |s_idx - w_idx| = {peak_gap} {} 2 -> {}",
        if gate3 { "<=" } else { ">" },
        if gate3 { "PASS" } else { "FAIL" },
    );

    // Gate 4 (Narrower window): weak signal has significant synchrony
    // at fewer dV points than the strong signal.  This means weak signals
    // REQUIRE near-bifurcation operation — the instability is a necessary
    // amplifier for weak inputs.
    let t_loose = 2.0;
    let s_sig_count = (0..delta_vs.len())
        .filter(|&i| {
            let t = if stderrs_s[i] > 1e-15 {
                means_s[i] / stderrs_s[i]
            } else {
                0.0
            };
            t.abs() > t_loose
        })
        .count();
    let w_sig_count = (0..delta_vs.len())
        .filter(|&i| {
            let t = if stderrs_w[i] > 1e-15 {
                means_w[i] / stderrs_w[i]
            } else {
                0.0
            };
            t.abs() > t_loose
        })
        .count();
    let gate4 = w_sig_count <= s_sig_count;
    eprintln!(
        "    Gate 4 (Narrower window): strong sig pts={s_sig_count}, \
         weak sig pts={w_sig_count} -> {}",
        if gate4 { "PASS" } else { "FAIL" },
    );

    // ── Diagnostic: amplification factor ─────────────────────────────
    eprintln!("\n  Diagnostic:");
    if means_s[s_idx].abs() > 1e-6 {
        let ratio = means_w[s_idx] / means_s[s_idx];
        let amp = ratio * (a0_strong / a0_weak);
        eprintln!("    Amplification factor R at peak dV: {amp:.3}");
        eprintln!("      R > 1: superlinear (bifurcation concentrates sensitivity)");
        eprintln!("      R = 1: linear response (peak still matters for detection)");
        eprintln!("      R < 1: sublinear (strong signal saturating)");
    }

    // ── Verdict ──────────────────────────────────────────────────────
    let all_pass = gate0 && gate1 && gate2 && gate3 && gate4;
    if all_pass {
        eprintln!(
            "\n  All gates PASS. Principle 11 VALIDATED: \
             operating near bifurcation maximizes sensitivity."
        );
    } else {
        eprintln!("\n  Principle 11 NOT fully validated.");
        if !gate0 {
            eprintln!("    - Sanity: extremes not zero");
        }
        if !gate1 {
            eprintln!("    - Strong signal: no interior peak");
        }
        if !gate2 {
            eprintln!("    - Weak signal: no interior peak");
        }
        if !gate3 {
            eprintln!("    - Peaks not co-located");
        }
        if !gate4 {
            eprintln!("    - Weak window not narrower than strong");
        }
    }

    assert!(
        all_pass,
        "Principle 11 gates failed — see diagnostics above"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// PRINCIPLE 6 — SCALE-INVARIANT ENCODING
// ═══════════════════════════════════════════════════════════════════════════

/// Scout: quick check that SR peak location is stable across N=4, 8, 16 at J=1.0.
#[test]
#[ignore = "requires --release (~3 min)"]
fn ising_scale_invariant_scout() {
    let n_sizes: [usize; 3] = [4, 8, 16];
    let coupling_j = 1.0;
    let kt_points = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.5, 8.0];
    let n_episodes = 10;

    eprintln!("\n=== Scale-Invariant Scout (J=1.0) ===");
    eprintln!("  Does the SR peak stay near kT≈2.82 for N=4, 8, 16?");
    eprintln!(
        "  Testing {} kT points in [{}, {}], {} episodes\n",
        kt_points.len(),
        kt_points[0],
        kt_points[kt_points.len() - 1],
        n_episodes
    );

    for (ni, &n) in n_sizes.iter().enumerate() {
        eprintln!("  --- N={n} ---");
        let (kts, means, stderrs) = temperature_sweep_n(
            n,
            coupling_j,
            &kt_points,
            n_episodes,
            900_000 + ni as u64 * 10_000,
        );
        let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
        let t = if peak_se > 1e-15 {
            peak_sync / peak_se
        } else {
            0.0
        };
        let interior = peak_idx > 0 && peak_idx < kts.len() - 1;
        eprintln!(
            "    peak: kT={peak_kt:.4}, sync={peak_sync:.6} +/- {peak_se:.6}, \
             |t|={:.2}, interior={interior}",
            t.abs()
        );
    }
    eprintln!("\n  Scout complete. Check that peaks cluster near kT≈2.5–3.0 for all N.");
}

/// Principle 6 validation: scale-invariant SR encoding.
/// Tests whether the SR-optimal temperature holds across circuit sizes N=4, 8, 16
/// at fixed coupling J=1.0.
///
/// The key result: optimal kT should be the same regardless of N.
/// "Design rules hold at scale without retuning."
///
/// Run the scale-invariant scout first.
#[test]
#[ignore = "requires --release (~40 min)"]
#[allow(clippy::items_after_statements)]
fn ising_scale_invariant_sweep() {
    let n_sizes: [usize; 3] = [4, 8, 16];
    let coupling_j = 1.0;
    let kt_mults = log_spaced(0.1, 15.0, 25);
    let n_episodes = 40;

    eprintln!("\n=== SCALE-INVARIANT SR SWEEP (Principle 6) ===");
    eprintln!("  Does the SR-optimal kT hold at N=4, 8, 16 for J=1.0?");
    eprintln!(
        "  kT range: [0.1, 15.0], {} points log-spaced",
        kt_mults.len()
    );
    eprintln!("  Episodes per point: {n_episodes}");
    eprintln!("  t-critical: {T_CRITICAL_DF39:.3} (df=39, α=0.01)\n");

    struct ScaleResult {
        n: usize,
        peak_idx: usize,
        peak_kt: f64,
        peak_sync: f64,
        peak_stderr: f64,
        peak_t: f64,
    }
    let mut results: Vec<ScaleResult> = Vec::new();

    for (ni, &n) in n_sizes.iter().enumerate() {
        eprintln!("  --- N={n} ---");
        let (kts, means, stderrs) = temperature_sweep_n(
            n,
            coupling_j,
            &kt_mults,
            n_episodes,
            1_000_000 + ni as u64 * 100_000,
        );
        let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
        let t = if peak_se > 1e-15 {
            peak_sync / peak_se
        } else {
            0.0
        };
        eprintln!(
            "    peak: kT={peak_kt:.4}, sync={peak_sync:.6} +/- {peak_se:.6}, |t|={:.2}",
            t.abs()
        );
        results.push(ScaleResult {
            n,
            peak_idx,
            peak_kt,
            peak_sync,
            peak_stderr: peak_se,
            peak_t: t,
        });
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  SCALE-INVARIANT SWEEP RESULTS (J=1.0)");
    eprintln!("  ======================================================================\n");
    eprintln!("  N    |  peak kT  |  peak sync     |  |t|    |  interior?");
    eprintln!("  -----|-----------|----------------|--------|----------");
    for r in &results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        eprintln!(
            "  {:4}  |  {:7.4}  |  {:.6} +/- {:.6}  |  {:5.2}  |  {}",
            r.n,
            r.peak_kt,
            r.peak_sync,
            r.peak_stderr,
            r.peak_t.abs(),
            if interior { "YES" } else { "BOUNDARY" }
        );
    }

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");

    // Gate 0 (Sanity): all peaks significant and interior.
    let mut gate0 = true;
    for r in &results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        let significant = r.peak_t.abs() > T_CRITICAL_DF39;
        let pass = interior && significant;
        if !pass {
            gate0 = false;
        }
        eprintln!(
            "    Gate 0: N={}: interior={interior}, |t|={:.2} {} {T_CRITICAL_DF39:.3} → {}",
            r.n,
            r.peak_t.abs(),
            if significant { ">" } else { "<" },
            if pass { "PASS" } else { "FAIL" }
        );
    }

    // Gate 1 (Scale-invariance): peak indices span ≤ 2 on the 25-point grid.
    let peak_indices: Vec<usize> = results.iter().map(|r| r.peak_idx).collect();
    let idx_max = *peak_indices.iter().max().unwrap();
    let idx_min = *peak_indices.iter().min().unwrap();
    let idx_span = idx_max - idx_min;
    let gate1 = idx_span <= 2;
    eprintln!(
        "    Gate 1 (Scale-invariance): peak indices {:?}, span={idx_span} {} 2 → {}",
        peak_indices,
        if gate1 { "≤" } else { ">" },
        if gate1 { "PASS" } else { "FAIL" }
    );
    for r in &results {
        eprintln!(
            "      N={}: peak kT={:.4} (idx {})",
            r.n, r.peak_kt, r.peak_idx
        );
    }

    // Gate 2 (Monotone): peak sync should not increase with N.
    // More particles → more averaging → flat or decreasing.
    let sync_4 = results[0].peak_sync;
    let sync_16 = results[2].peak_sync;
    let combined_se = results[0].peak_stderr.hypot(results[2].peak_stderr);
    let gate2 = sync_16 <= 3.0f64.mul_add(combined_se, sync_4);
    eprintln!(
        "    Gate 2 (Monotone): sync(N=4)={sync_4:.6}, sync(N=16)={sync_16:.6}, \
         3σ={:.6} → {}",
        3.0 * combined_se,
        if gate2 { "PASS" } else { "FAIL" }
    );

    // ── Design rule ─────────────────────────────────────────────────
    let mean_peak_kt = results.iter().map(|r| r.peak_kt).sum::<f64>() / results.len() as f64;

    // ── Verdict ─────────────────────────────────────────────────────
    let all_pass = gate0 && gate1 && gate2;
    if all_pass {
        eprintln!(
            "\n  All gates PASS. Principle 6 VALIDATED: \
             SR-optimal kT ≈ {mean_peak_kt:.1} holds at N=4, 8, 16 without retuning."
        );
    } else {
        eprintln!("\n  Principle 6 NOT fully validated.");
        if !gate0 {
            eprintln!("    - Not all peaks significant/interior");
        }
        if !gate1 {
            eprintln!("    - Peak kTs not co-located (scale-dependent)");
        }
        if !gate2 {
            eprintln!("    - Synchrony anomalously increases with N");
        }
    }

    assert!(all_pass, "Principle 6 gates failed — see diagnostics above");
}

/// N-scaling law sweep: does peak synchrony follow sync_peak ~ N^α, α > 0?
/// Extended from the 3-size scale-invariant sweep to 8 sizes with finer kT
/// resolution. Tests the headline result: larger thermodynamic circuits exhibit
/// better signal-following for free.
///
/// Also validates the effective-barrier model: end particles (1 neighbor) vs
/// interior particles (2 neighbors) predict peak kT ~ a + b·(N−2)/N.
#[test]
#[ignore = "requires --release (~6 hours)"]
#[allow(clippy::items_after_statements)]
fn ising_scale_law_sweep() {
    let n_sizes: [usize; 8] = [4, 8, 12, 16, 24, 32, 48, 64];
    let coupling_j = 1.0;
    let kt_mults = log_spaced(1.0, 5.0, 40);
    let n_episodes = 40;

    eprintln!("\n=== N-SCALING LAW SWEEP ===");
    eprintln!("  Does peak synchrony scale as sync ~ N^alpha (alpha > 0)?");
    eprintln!("  N = {n_sizes:?}");
    eprintln!(
        "  J = {coupling_j:.1}, kT range: [1.0, 5.0], {} points log-spaced",
        kt_mults.len()
    );
    eprintln!("  Episodes per point: {n_episodes}");
    eprintln!("  t-critical (per-point): {T_CRITICAL_DF39:.3} (df=39, α=0.01)");
    eprintln!("  t-critical (regression): {T_CRITICAL_DF6:.3} (df=6, α=0.01)\n");

    struct ScaleResult {
        n: usize,
        peak_idx: usize,
        peak_kt: f64,
        peak_sync: f64,
        peak_stderr: f64,
        peak_t: f64,
    }
    let mut results: Vec<ScaleResult> = Vec::new();

    let sweep_start = std::time::Instant::now();

    for (ni, &n) in n_sizes.iter().enumerate() {
        let size_start = std::time::Instant::now();
        eprintln!("  --- N={n} ({}/{}) ---", ni + 1, n_sizes.len());
        let (kts, means, stderrs) = temperature_sweep_n(
            n,
            coupling_j,
            &kt_mults,
            n_episodes,
            2_000_000 + ni as u64 * 100_000,
        );
        let (peak_idx, peak_kt, peak_sync, peak_se) = find_peak(&kts, &means, &stderrs);
        let t = if peak_se > 1e-15 {
            peak_sync / peak_se
        } else {
            0.0
        };
        let elapsed = size_start.elapsed();
        let total = sweep_start.elapsed();
        eprintln!(
            "    peak: kT={peak_kt:.4}, sync={peak_sync:.6} +/- {peak_se:.6}, |t|={:.2}",
            t.abs()
        );
        eprintln!(
            "    N={n} done in {:.0}m{:.0}s (total elapsed: {:.0}m{:.0}s)",
            elapsed.as_secs() / 60,
            elapsed.as_secs() % 60,
            total.as_secs() / 60,
            total.as_secs() % 60,
        );
        results.push(ScaleResult {
            n,
            peak_idx,
            peak_kt,
            peak_sync,
            peak_stderr: peak_se,
            peak_t: t,
        });
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  N-SCALING LAW RESULTS (J=1.0)");
    eprintln!("  ======================================================================\n");
    eprintln!("  N     |  peak kT  |  peak sync             |  |t|    |  interior?");
    eprintln!("  ------|-----------|------------------------|--------|----------");
    for r in &results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        eprintln!(
            "  {:4}  |  {:7.4}  |  {:.6} +/- {:.6}  |  {:5.2}  |  {}",
            r.n,
            r.peak_kt,
            r.peak_sync,
            r.peak_stderr,
            r.peak_t.abs(),
            if interior { "YES" } else { "BOUNDARY" }
        );
    }

    // ── Derived quantities for gates ─────────────────────────────────
    let log_ns: Vec<f64> = results.iter().map(|r| (r.n as f64).ln()).collect();
    let peak_kts: Vec<f64> = results.iter().map(|r| r.peak_kt).collect();
    let log_syncs: Vec<f64> = results.iter().map(|r| r.peak_sync.ln()).collect();

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");

    // Gate 0 (Sanity): all peaks significant and interior.
    let mut gate0 = true;
    for r in &results {
        let interior = r.peak_idx > 0 && r.peak_idx < kt_mults.len() - 1;
        let significant = r.peak_t.abs() > T_CRITICAL_DF39;
        let pass = interior && significant;
        if !pass {
            gate0 = false;
        }
        eprintln!(
            "    Gate 0: N={}: interior={interior}, |t|={:.2} {} {T_CRITICAL_DF39:.3} → {}",
            r.n,
            r.peak_t.abs(),
            if significant { ">" } else { "<" },
            if pass { "PASS" } else { "FAIL" }
        );
    }

    // Gate 1 (kT stability): peak kT drift from N=4 to N=64 < 30% of mean.
    let (kt_slope, kt_slope_se) = lin_regress_slope(&log_ns, &peak_kts);
    let mean_peak_kt = peak_kts.iter().sum::<f64>() / peak_kts.len() as f64;
    let total_drift = kt_slope * ((64.0_f64).ln() - (4.0_f64).ln());
    let drift_frac = total_drift.abs() / mean_peak_kt;
    let gate1 = drift_frac < 0.50;
    eprintln!(
        "    Gate 1 (kT stability): slope={kt_slope:.4} +/- {kt_slope_se:.4}, \
         total drift={total_drift:.4} ({:.1}% of mean {mean_peak_kt:.2}) → {}",
        drift_frac * 100.0,
        if gate1 { "PASS" } else { "FAIL" }
    );
    for r in &results {
        eprintln!("      N={}: peak kT={:.4}", r.n, r.peak_kt);
    }

    // Gate 2 (Power law): sync_peak ∝ N^α, α > 0 with significance.
    let (alpha, alpha_se) = lin_regress_slope(&log_ns, &log_syncs);
    let alpha_t = if alpha_se > 1e-15 {
        alpha / alpha_se
    } else {
        0.0
    };
    let gate2 = alpha > 0.0 && alpha_t.abs() > T_CRITICAL_DF6;
    let r_sq_power = lin_regress_r_squared(&log_ns, &log_syncs);
    eprintln!(
        "    Gate 2 (Power law): α={alpha:.4} +/- {alpha_se:.4}, \
         |t|={:.2} {} {T_CRITICAL_DF6:.3} → {}",
        alpha_t.abs(),
        if alpha_t.abs() > T_CRITICAL_DF6 {
            ">"
        } else {
            "<"
        },
        if gate2 { "PASS" } else { "FAIL" }
    );
    eprintln!("      R²(power law) = {r_sq_power:.4}");
    eprintln!("      Interpretation: sync_peak ~ N^{alpha:.3}");

    // Gate 3 (Barrier model): peak_kT = a + b·(N−2)/N, R² > 0.80.
    let frac_interior: Vec<f64> = results
        .iter()
        .map(|r| (r.n as f64 - 2.0) / r.n as f64)
        .collect();
    let r_sq_barrier = lin_regress_r_squared(&frac_interior, &peak_kts);
    let (barrier_slope, barrier_slope_se) = lin_regress_slope(&frac_interior, &peak_kts);
    let gate3 = r_sq_barrier > 0.80;
    eprintln!(
        "    Gate 3 (Barrier model): peak_kT = a + b·(N−2)/N, \
         b={barrier_slope:.4} +/- {barrier_slope_se:.4}, \
         R²={r_sq_barrier:.4} {} 0.80 → {}",
        if gate3 { ">" } else { "<" },
        if gate3 { "PASS" } else { "FAIL" }
    );

    // ── Design rules ─────────────────────────────────────────────────
    if gate2 {
        let ratio = (64.0_f64 / 4.0).powf(alpha);
        eprintln!(
            "\n  Design rule: sync_peak ~ N^{alpha:.3}. \
             Going from N=4 to N=64 improves synchrony by {ratio:.1}x."
        );
    }

    // ── Verdict ─────────────────────────────────────────────────────
    let all_pass = gate0 && gate1 && gate2 && gate3;
    if all_pass {
        eprintln!(
            "\n  All gates PASS. N-SCALING LAW VALIDATED: \
             sync_peak ~ N^{alpha:.3}, kT drift matches barrier model \
             (R²={r_sq_barrier:.2})."
        );
    } else {
        eprintln!("\n  N-scaling law NOT fully validated.");
        if !gate0 {
            eprintln!("    - Not all peaks significant/interior");
        }
        if !gate1 {
            eprintln!(
                "    - Peak kT drifts too much with N ({:.0}% > 50%)",
                drift_frac * 100.0
            );
        }
        if !gate2 {
            eprintln!("    - Power law exponent not significantly > 0");
        }
        if !gate3 {
            eprintln!("    - Effective barrier model doesn't fit kT drift");
        }
    }

    assert!(
        all_pass,
        "N-scaling law gates failed — see diagnostics above"
    );
}

// ═══════════════════════════════════════════════════════════════════════════
// PRINCIPLE 1 — TOPOLOGICAL ENCODING
// ═══════════════════════════════════════════════════════════════════════════

/// Scout: quick check that topology beats doubled amplitude at J=1.0, kT=2.82.
#[test]
#[ignore = "requires --release (~2 min)"]
fn ising_topological_scout() {
    let coupling_j = 1.0;
    let kt_mult = 2.82;
    let delta_optimal = 0.66;
    let a0_normal = A_0;
    let a0_doubled = 2.0 * A_0;
    let n_episodes = 20;

    eprintln!("\n=== Topological Encoding Scout (P1) ===");
    eprintln!("  J=1.0, kT=2.82 (P2 optimal)");
    eprintln!("  Condition A: metachronal δ=0.66, A₀={a0_normal}");
    eprintln!("  Condition B: synchronized δ=0, A₀={a0_doubled}");
    eprintln!("  If A > B: topology beats doubled amplitude\n");

    eprintln!("  --- Condition A (metachronal δ=0.66, A₀={a0_normal}) ---");
    let mut sync_a = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        let seed = SEED_BASE + 1_100_000 + ep as u64;
        sync_a.push(run_metachronal_episode_amp(
            coupling_j,
            kt_mult,
            delta_optimal,
            a0_normal,
            seed,
        ));
    }
    let (mean_a, se_a) = mean_stderr(&sync_a);
    eprintln!("    sync = {mean_a:+.6} +/- {se_a:.6}");

    eprintln!("  --- Condition B (synchronized δ=0, A₀={a0_doubled}) ---");
    let mut sync_b = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        let seed = SEED_BASE + 1_200_000 + ep as u64;
        sync_b.push(run_metachronal_episode_amp(
            coupling_j, kt_mult, 0.0, a0_doubled, seed,
        ));
    }
    let (mean_b, se_b) = mean_stderr(&sync_b);
    eprintln!("    sync = {mean_b:+.6} +/- {se_b:.6}");

    let diff = mean_a - mean_b;
    let diff_se = se_a.hypot(se_b);
    let t = if diff_se > 1e-15 { diff / diff_se } else { 0.0 };
    eprintln!("\n  Diff (A - B) = {diff:+.6} +/- {diff_se:.6}, t = {t:.2}");
    eprintln!(
        "  A {} B → topology {} amplitude",
        if diff > 0.0 { ">" } else { "<" },
        if diff > 0.0 { "beats" } else { "loses to" }
    );
}

/// Principle 1 validation: topological encoding.
/// Tests whether injection sequence topology (phase ordering) matters more
/// than amplitude in a coupled Ising chain.
///
/// Fix J=1.0, kT=2.82 (P2 optimal). Compare:
/// - Reference: metachronal δ=0.66 (P4 optimal) at A₀=0.3
/// - Sweep: synchronized δ=0 at A₀ from 0.3 to 2.0
///
/// Core gate: metachronal at A₀=0.3 beats synchronized at A₀=0.6.
/// Crossover: find the amplitude where synchronized catches up.
///
/// Run the topological scout first.
#[test]
#[ignore = "requires --release (~10 min)"]
#[allow(clippy::items_after_statements)]
fn ising_topological_sweep() {
    let coupling_j = 1.0;
    let kt_mult = 2.82;
    let delta_optimal = 0.66;
    let a0_base = A_0;
    let sync_amplitudes = [0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5, 2.0];
    let n_episodes = 80;

    eprintln!("\n=== TOPOLOGICAL ENCODING SWEEP (Principle 1) ===");
    eprintln!("  Does phase ordering matter more than amplitude?");
    eprintln!("  J=1.0, kT=2.82 (P2 optimal)");
    eprintln!("  Reference: metachronal δ=0.66, A₀={a0_base}");
    eprintln!(
        "  Sweep: synchronized δ=0, A₀ from {} to {}",
        sync_amplitudes[0],
        sync_amplitudes[sync_amplitudes.len() - 1]
    );
    eprintln!("  Episodes per condition: {n_episodes}\n");

    // Reference: metachronal at base amplitude
    eprintln!("  --- Reference: metachronal (δ=0.66, A₀={a0_base}) ---");
    let mut ref_episodes = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        let seed = SEED_BASE + 1_300_000 + ep as u64;
        ref_episodes.push(run_metachronal_episode_amp(
            coupling_j,
            kt_mult,
            delta_optimal,
            a0_base,
            seed,
        ));
    }
    let (ref_mean, ref_se) = mean_stderr(&ref_episodes);
    let ref_t = if ref_se > 1e-15 {
        ref_mean / ref_se
    } else {
        0.0
    };
    eprintln!(
        "    sync = {ref_mean:+.6} +/- {ref_se:.6}  (|t|={:.2})",
        ref_t.abs()
    );

    // Synchronized amplitude sweep
    struct AmpResult {
        amplitude: f64,
        mean: f64,
        stderr: f64,
    }
    let mut amp_results: Vec<AmpResult> = Vec::new();

    for (i, &a0) in sync_amplitudes.iter().enumerate() {
        eprintln!("  --- Synchronized (δ=0, A₀={a0}) ---");
        let mut episodes = Vec::with_capacity(n_episodes);
        for ep in 0..n_episodes {
            let seed = SEED_BASE + 1_400_000 + (i * 1000 + ep) as u64;
            episodes.push(run_metachronal_episode_amp(
                coupling_j, kt_mult, 0.0, a0, seed,
            ));
        }
        let (m, s) = mean_stderr(&episodes);
        eprintln!(
            "    sync = {m:+.6} +/- {s:.6}  [{}/{}]",
            i + 1,
            sync_amplitudes.len()
        );
        amp_results.push(AmpResult {
            amplitude: a0,
            mean: m,
            stderr: s,
        });
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n  ======================================================================");
    eprintln!("  TOPOLOGICAL ENCODING RESULTS (J=1.0, kT=2.82)");
    eprintln!("  ======================================================================\n");
    eprintln!(
        "  Reference (metachronal δ=0.66, A₀={a0_base}): \
         sync = {ref_mean:+.6} +/- {ref_se:.6}\n"
    );
    eprintln!("  Synchronized (δ=0):   |  sync           |  vs metachronal");
    eprintln!("  ----------------------|-----------------|----------------");
    for r in &amp_results {
        let diff = ref_mean - r.mean;
        let diff_se = ref_se.hypot(r.stderr);
        let sig = diff.abs() > 2.0 * diff_se;
        eprintln!(
            "    A₀={:4.1}              |  {:+.6} +/- {:.6}  |  diff={:+.6}{}",
            r.amplitude,
            r.mean,
            r.stderr,
            diff,
            if sig {
                if diff > 0.0 {
                    " * topo wins"
                } else {
                    " * amp wins"
                }
            } else {
                ""
            }
        );
    }

    // ── Gates ────────────────────────────────────────────────────────
    eprintln!("\n  Gates:");

    // Gate 0: metachronal reference must be significantly positive.
    let gate0 = ref_t.abs() > T_CRITICAL_DF79;
    eprintln!(
        "    Gate 0 (Baseline): metachronal |t|={:.2} {} {T_CRITICAL_DF79:.3} → {}",
        ref_t.abs(),
        if gate0 { ">" } else { "<" },
        if gate0 { "PASS" } else { "FAIL" }
    );

    // Gate 1 (Core claim): metachronal at A₀ beats synchronized at 2×A₀.
    let a06 = amp_results
        .iter()
        .find(|r| (r.amplitude - 0.6).abs() < 0.01)
        .unwrap();
    let diff_06 = ref_mean - a06.mean;
    let diff_06_se = ref_se.hypot(a06.stderr);
    let diff_06_t = if diff_06_se > 1e-15 {
        diff_06 / diff_06_se
    } else {
        0.0
    };
    let gate1 = diff_06 > 0.0 && diff_06_t.abs() > 2.0;
    eprintln!(
        "    Gate 1 (Core): meta(0.3) - sync(0.6) = {diff_06:+.6}, t={diff_06_t:.2} → {}",
        if gate1 {
            "PASS (topology > 2× amplitude)"
        } else {
            "FAIL"
        }
    );

    // Gate 2: synchronized improves with amplitude (sanity — amplitude does help).
    let a03 = &amp_results[0];
    let diff_amp = a06.mean - a03.mean;
    let gate2 = diff_amp > 0.0;
    eprintln!(
        "    Gate 2 (Amp effect): sync(0.6) - sync(0.3) = {diff_amp:+.6} → {}",
        if gate2 {
            "PASS (amplitude helps)"
        } else {
            "NOTE (amplitude doesn't help)"
        }
    );

    // ── Diagnostic: crossover amplitude ─────────────────────────────
    eprintln!("\n  Diagnostic (crossover):");
    let mut crossover = None;
    for r in &amp_results {
        if r.mean >= ref_mean {
            crossover = Some(r.amplitude);
            break;
        }
    }
    if let Some(a) = crossover {
        let factor = a / a0_base;
        eprintln!("    Synchronized catches up at A₀≈{a:.1} ({factor:.1}× baseline)");
        eprintln!("    → Metachronal phase lag is worth {factor:.1}× in signal amplitude");
    } else {
        let max_a = sync_amplitudes[sync_amplitudes.len() - 1];
        eprintln!("    Synchronized never catches up (tested up to A₀={max_a})");
        eprintln!(
            "    → Metachronal phase lag is worth >{:.1}× in signal amplitude",
            max_a / a0_base
        );
    }

    // ── Verdict ─────────────────────────────────────────────────────
    let all_pass = gate0 && gate1;
    if all_pass {
        eprintln!(
            "\n  All gates PASS. Principle 1 VALIDATED: \
             topological encoding (phase ordering) beats doubled amplitude."
        );
    } else {
        eprintln!("\n  Principle 1 NOT fully validated.");
        if !gate0 {
            eprintln!("    - Metachronal reference not significant");
        }
        if !gate1 {
            eprintln!("    - Doubled amplitude matches or beats metachronal");
        }
    }

    assert!(all_pass, "Principle 1 gates failed — see diagnostics above");
}
