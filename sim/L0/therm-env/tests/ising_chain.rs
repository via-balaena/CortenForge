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
#[ignore = "requires --release (~7 min)"]
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
#[ignore = "requires --release (~1-2 hours)"]
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
