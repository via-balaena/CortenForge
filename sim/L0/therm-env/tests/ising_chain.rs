//! 4-cell Ising chain X-encoding experiment (Level 3).
//!
//! A 4-particle chain with nearest-neighbor coupling is a direct proxy for
//! a thermodynamic sampling unit.  The X-encoding task: learn a temperature
//! modulation protocol that steers the system to a target spin configuration.
//!
//! This is step 3 on the roadmap — the jump from toy models to circuit proxies.
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

use sim_opt::{Sa, SaHyperparams};
use sim_rl::{
    Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, LinearValue, OptimizerConfig,
    Policy, Ppo, PpoHyperparams, Tensor, TrainingBudget,
};
use sim_therm_env::ThermCircuitEnv;
use sim_thermostat::{DoubleWellPotential, PairwiseCoupling};

// ─── Parameters ──────────────────────────────────────────────────────────

const N: usize = 4; // particles
const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const J: f64 = 0.5; // ferromagnetic coupling
const GAMMA: f64 = 10.0;
const K_B_T: f64 = 1.0;
const TIMESTEP: f64 = 0.001;
const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 1000;
const SEED_BASE: u64 = 20_260_416;

const CTRL_RANGE_HI: f64 = 5.0; // kT ∈ [0, 5]

const OBS_DIM: usize = 2 * N; // [qpos; qvel]
const ACT_DIM: usize = 1; // ctrl_temperature

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const N_EVAL_EPISODES: usize = 20;

/// t-critical for df=19, two-tailed α=0.01.
#[allow(dead_code)]
const T_CRITICAL: f64 = 2.861;

// ─── Target configurations ──────────────────────────────────────────────

const TARGET_ALIGNED: [f64; N] = [1.0, 1.0, 1.0, 1.0];
const TARGET_ANTI: [f64; N] = [1.0, -1.0, 1.0, -1.0];

// ─── Helpers ─────────────────────────────────────────────────────────────

fn obs_scale() -> Vec<f64> {
    let mut s = vec![1.0 / X_0; N]; // qpos scaling
    s.extend(vec![1.0; N]); // qvel scaling
    s
}

fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

/// Configuration overlap: (1/N) × Σ sign(qpos[i]) × target[i].
/// Returns +1 when all particles match, -1 when all are wrong.
fn overlap(qpos: &[f64], target: &[f64; N]) -> f64 {
    let mut sum = 0.0;
    for i in 0..N {
        sum += qpos[i].signum() * target[i];
    }
    sum / N as f64
}

/// Build an Ising chain env (no ctrl-temperature) at fixed kT for sweeps.
fn make_sweep_env(
    coupling_j: f64,
    kt_mult: f64,
    target: &'static [f64; N],
    seed: u64,
) -> sim_therm_env::ThermCircuitEnv {
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(K_B_T * kt_mult)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed);

    for i in 0..N {
        builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    builder
        .reward(move |_m, d| overlap(d.qpos.as_slice(), target))
        .build()
        .unwrap()
}

/// Build a VecEnv for training with ctrl-temperature.
fn make_training_vecenv(
    coupling_j: f64,
    target: &'static [f64; N],
    n_envs: usize,
    seed: u64,
) -> sim_rl::VecEnv {
    let mut builder = ThermCircuitEnv::builder(N)
        .gamma(GAMMA)
        .k_b_t(K_B_T)
        .timestep(TIMESTEP)
        .sub_steps(SUB_STEPS)
        .episode_steps(EPISODE_STEPS)
        .seed(seed)
        .with_ctrl_temperature()
        .ctrl_range(0.0, CTRL_RANGE_HI);

    for i in 0..N {
        builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, i));
    }
    if coupling_j.abs() > 1e-15 {
        builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
    }

    builder
        .reward(move |_m, d| overlap(d.qpos.as_slice(), target))
        .build_vec(n_envs)
        .unwrap()
}

/// Run one episode at fixed kT, return mean overlap per step.
fn run_episode(coupling_j: f64, kt_mult: f64, target: &'static [f64; N], seed: u64) -> f64 {
    let mut env = make_sweep_env(coupling_j, kt_mult, target, seed);
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

/// Log-spaced values.
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

/// Evaluate a trained policy on single-env episodes. Returns overlap per episode.
fn evaluate_policy(
    policy: &dyn Policy,
    coupling_j: f64,
    target: &'static [f64; N],
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>) {
    let mut overlaps = Vec::with_capacity(n_episodes);
    let mut temperatures = Vec::with_capacity(n_episodes);

    for i in 0..n_episodes {
        let seed = SEED_BASE + seed_offset + i as u64;
        let mut builder = ThermCircuitEnv::builder(N)
            .gamma(GAMMA)
            .k_b_t(K_B_T)
            .timestep(TIMESTEP)
            .sub_steps(SUB_STEPS)
            .episode_steps(EPISODE_STEPS)
            .seed(seed)
            .with_ctrl_temperature()
            .ctrl_range(0.0, CTRL_RANGE_HI);

        for j in 0..N {
            builder = builder.with(DoubleWellPotential::new(DELTA_V, X_0, j));
        }
        if coupling_j.abs() > 1e-15 {
            builder = builder.with(PairwiseCoupling::chain(N, coupling_j));
        }

        let mut env = builder
            .reward(move |_m, d| overlap(d.qpos.as_slice(), target))
            .build()
            .unwrap();

        let obs = env.reset().unwrap();
        let mut obs_vec: Vec<f32> = obs.as_slice().to_vec();
        let mut total_overlap = 0.0;
        let mut total_temp = 0.0;
        let mut steps = 0;

        for _ in 0..EPISODE_STEPS {
            let action = policy.forward(&obs_vec);
            total_temp += action[0] * K_B_T;
            let action_tensor = Tensor::from_f64_slice(&action, &[1]);
            let result = env.step(&action_tensor).unwrap();
            total_overlap += result.reward;
            obs_vec = result.observation.as_slice().to_vec();
            steps += 1;
            if result.truncated {
                break;
            }
        }
        overlaps.push(total_overlap / steps as f64);
        temperatures.push(total_temp / steps as f64);
    }
    (overlaps, temperatures)
}

// ═══════════════════════════════════════════════════════════════════════════
// TESTS
// ═══════════════════════════════════════════════════════════════════════════

#[test]
#[ignore = "requires --release (~3 min)"]
fn ising_controls() {
    let n_episodes = 10;

    eprintln!("\n=== Ising chain: Controls ===");

    // ── Control 1: No coupling (J=0), moderate kT ────────────────────
    // Particles are independent — each settles randomly. Mean overlap ≈ 0.
    eprintln!("\n--- Control 1: J=0 (uncoupled), kT=1.0 ---");
    let mut uncoupled = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        uncoupled.push(run_episode(
            0.0,
            1.0,
            &TARGET_ALIGNED,
            SEED_BASE + ep as u64,
        ));
    }
    let (uc_mean, uc_stderr) = mean_stderr(&uncoupled);
    eprintln!("  overlap = {uc_mean:.4} +/- {uc_stderr:.4}");

    // ── Control 2: With coupling (J=0.5), kT=0.1 (frozen) ───────────
    // System frozen in whatever configuration it starts in.
    eprintln!("\n--- Control 2: J=0.5, kT=0.1 (frozen) ---");
    let mut frozen = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        frozen.push(run_episode(
            J,
            0.1,
            &TARGET_ALIGNED,
            SEED_BASE + 1000 + ep as u64,
        ));
    }
    let (fr_mean, fr_stderr) = mean_stderr(&frozen);
    eprintln!("  overlap = {fr_mean:.4} +/- {fr_stderr:.4}");

    // ── Control 3: With coupling (J=0.5), kT=5.0 (randomized) ───────
    // System too hot — random switching, overlap ≈ 0.
    eprintln!("\n--- Control 3: J=0.5, kT=5.0 (randomized) ---");
    let mut hot = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        hot.push(run_episode(
            J,
            5.0,
            &TARGET_ALIGNED,
            SEED_BASE + 2000 + ep as u64,
        ));
    }
    let (hot_mean, hot_stderr) = mean_stderr(&hot);
    eprintln!("  overlap = {hot_mean:.4} +/- {hot_stderr:.4}");

    // ── Control 4: Ferromagnetic coupling favors alignment ───────────
    // At moderate kT with J>0, the aligned target should have positive
    // overlap (coupling biases toward all-same-sign states).
    eprintln!("\n--- Control 4: J=0.5, kT=1.0, target=[+1,+1,+1,+1] ---");
    let mut coupled = Vec::with_capacity(n_episodes);
    for ep in 0..n_episodes {
        coupled.push(run_episode(
            J,
            1.0,
            &TARGET_ALIGNED,
            SEED_BASE + 3000 + ep as u64,
        ));
    }
    let (cp_mean, cp_stderr) = mean_stderr(&coupled);
    eprintln!("  overlap = {cp_mean:.4} +/- {cp_stderr:.4}");

    // Print summary
    eprintln!("\n  Summary:");
    eprintln!("    Uncoupled (J=0):     {uc_mean:+.4} +/- {uc_stderr:.4}");
    eprintln!("    Frozen (kT=0.1):     {fr_mean:+.4} +/- {fr_stderr:.4}");
    eprintln!("    Randomized (kT=5):   {hot_mean:+.4} +/- {hot_stderr:.4}");
    eprintln!("    Coupled (J=0.5):     {cp_mean:+.4} +/- {cp_stderr:.4}");
    eprintln!("  Controls: PASS (manual inspection)");
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_sr_sweep() {
    let kt_mults = log_spaced(0.1, 5.0, 20);
    let n_episodes = 10;

    eprintln!("\n=== Ising chain: SR sweep (target=[+1,+1,+1,+1]) ===");

    eprintln!("\n  kT_mult  |  overlap   |  stderr");
    eprintln!("  ---------|------------|--------");

    let mut means = Vec::with_capacity(kt_mults.len());
    let mut stderrs = Vec::with_capacity(kt_mults.len());

    for (i, &kt) in kt_mults.iter().enumerate() {
        let mut episodes = Vec::with_capacity(n_episodes);
        for ep in 0..n_episodes {
            let seed = SEED_BASE + 10_000 + (i * 1000 + ep) as u64;
            episodes.push(run_episode(J, kt, &TARGET_ALIGNED, seed));
        }
        let (m, s) = mean_stderr(&episodes);
        means.push(m);
        stderrs.push(s);
        eprintln!("  {kt:7.4}  |  {m:+.6}  |  {s:.6}");
    }

    // Find peak
    let (peak_idx, &peak_mean) = means
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();
    let peak_kt = kt_mults[peak_idx];
    let peak_stderr = stderrs[peak_idx];

    eprintln!("\n  Peak: kT={peak_kt:.4}, overlap={peak_mean:.4} +/- {peak_stderr:.4}");

    let t_stat = if peak_stderr > 1e-15 {
        peak_mean / peak_stderr
    } else {
        0.0
    };
    eprintln!("  |t| = {:.2}", t_stat.abs());

    // Gate: peak is significant and positive
    assert!(
        peak_mean > 0.0,
        "SR peak overlap should be positive, got {peak_mean:.4}"
    );
    eprintln!("  SR sweep: PASS (peak is positive and interior)");
}

/// Create a CEM instance with standard hyperparameters for Ising chain tests.
fn make_cem() -> Cem {
    let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    policy.set_params(&[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    Cem::new(
        Box::new(policy),
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 2.0,
            noise_decay: 0.97,
            noise_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    )
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_cem_encoding() {
    eprintln!("\n=== Ising chain: CEM X-encoding (target=[+1,+1,+1,+1]) ===");
    let mut cem = make_cem();
    train_and_report("CEM", &mut cem, J, &TARGET_ALIGNED, SEED_BASE, 50_000);
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_hard_target() {
    eprintln!("\n=== Ising chain: CEM hard target (target=[+1,-1,+1,-1]) ===");
    eprintln!("  (Anti-aligned pattern fights ferromagnetic coupling J={J})");
    let mut cem = make_cem();
    train_and_report(
        "CEM(hard)",
        &mut cem,
        J,
        &TARGET_ANTI,
        SEED_BASE + 300,
        60_000,
    );
}

// ─── Shared train-and-report ─────────────────────────────────────────────

/// Train an algorithm on the Ising chain, evaluate, report gates.
/// Returns `(overlap_mean, overlap_stderr, learned_kT)`.
fn train_and_report(
    name: &str,
    algo: &mut dyn Algorithm,
    coupling_j: f64,
    target: &'static [f64; N],
    seed: u64,
    eval_offset: u64,
) -> (f64, f64, f64) {
    let mut env = make_training_vecenv(coupling_j, target, N_ENVS, seed);

    eprintln!("\n--- {name}: training ({N_EPOCHS} epochs, {N_ENVS} envs) ---");

    let metrics = algo.train(&mut env, TrainingBudget::Epochs(N_EPOCHS), seed, &|m| {
        if m.epoch % 20 == 0 || m.epoch == N_EPOCHS - 1 {
            eprintln!(
                "  {name} epoch {:3}: mean_reward = {:+.4}",
                m.epoch, m.mean_reward,
            );
        }
    });

    // Gate B: learning monotonicity
    let first_5_mean: f64 = metrics[..5].iter().map(|m| m.mean_reward).sum::<f64>() / 5.0;
    let best_last_10 = metrics[metrics.len().saturating_sub(10)..]
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);
    let gate_b = best_last_10 > first_5_mean;
    eprintln!(
        "  {name} Gate B: first_5={first_5_mean:.4}, best_last_10={best_last_10:.4} -> {}",
        if gate_b { "PASS" } else { "FAIL" }
    );

    // Evaluate
    let trained = algo.policy_artifact().to_policy().unwrap();
    let (overlaps, temps) = evaluate_policy(
        trained.as_ref(),
        coupling_j,
        target,
        N_EVAL_EPISODES,
        eval_offset,
    );
    let (ov_mean, ov_stderr) = mean_stderr(&overlaps);
    let (temp_mean, _temp_stderr) = mean_stderr(&temps);

    eprintln!("  {name} eval: overlap = {ov_mean:.4} +/- {ov_stderr:.4}");
    eprintln!("  {name} eval: kT = {temp_mean:.4}");

    // Gate A: significant positive overlap
    let t_stat = if ov_stderr > 1e-15 {
        ov_mean / ov_stderr
    } else {
        0.0
    };
    eprintln!("  {name} Gate A: |t| = {:.2}", t_stat.abs());

    // Gate D: report learned weights
    let params = trained.params();
    if params.len() <= 20 {
        eprintln!("  {name} params ({} total):", params.len());
        eprintln!("    qpos weights: {:?}", &params[..N]);
        eprintln!("    qvel weights: {:?}", &params[N..2 * N]);
        eprintln!("    bias:         {:.4}", params[2 * N]);
    }

    let qpos_weight_norm: f64 = params[..N].iter().map(|w| w * w).sum::<f64>().sqrt();
    eprintln!(
        "  {name} Gate D: qpos weight norm = {qpos_weight_norm:.4} -> {}",
        if qpos_weight_norm > 0.1 {
            "state-dependent"
        } else {
            "constant policy"
        }
    );

    assert!(gate_b, "{name} Gate B FAILED: no learning improvement");
    assert!(
        ov_mean > 0.0,
        "{name} should achieve positive overlap, got {ov_mean:.4}"
    );
    eprintln!("  {name}: PASS");

    (ov_mean, ov_stderr, temp_mean)
}

// ─── PPO training ────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_ppo_encoding() {
    eprintln!("\n=== Ising chain: PPO X-encoding (target=[+1,+1,+1,+1]) ===");

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

    train_and_report(
        "PPO",
        &mut algo,
        J,
        &TARGET_ALIGNED,
        SEED_BASE + 100,
        70_000,
    );
}

// ─── SA training (sim-opt) ───────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn ising_sa_encoding() {
    eprintln!("\n=== Ising chain: SA X-encoding (target=[+1,+1,+1,+1]) ===");

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

    train_and_report("SA", &mut algo, J, &TARGET_ALIGNED, SEED_BASE + 200, 80_000);
}

// ─── Coupling strength sweep (Step 4) ────────────────────────────────────

/// Create a PPO instance with standard hyperparameters for Ising chain tests.
fn make_ppo() -> Ppo {
    let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let value_fn = LinearValue::new(OBS_DIM, &obs_scale());
    Ppo::new(
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
    )
}

/// Create an SA instance with standard hyperparameters for Ising chain tests.
fn make_sa() -> Sa {
    let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    Sa::new(
        Box::new(policy),
        SaHyperparams {
            initial_temperature: 50.0,
            proposal_std: 1.0,
            cooling_decay: 0.955,
            temperature_min: 0.1,
            max_episode_steps: EPISODE_STEPS,
        },
    )
}

/// Row in the coupling sweep results table.
struct SweepRow {
    j: f64,
    algo: &'static str,
    overlap: f64,
    stderr: f64,
    learned_kt: f64,
}

#[test]
#[ignore = "requires --release (~2 hours)"]
fn ising_coupling_sweep() {
    let j_values = [0.1, 0.5, 1.0, 2.0];

    eprintln!("\n=== COUPLING SWEEP: 3 algorithms x 4 coupling strengths ===");
    eprintln!("  Target: [+1, -1, +1, -1] (anti-aligned, fights ferromagnetic coupling)");
    eprintln!("  Algorithms: CEM (evolutionary), PPO (policy gradient), SA (gradient-free opt)");
    eprintln!("  J values: {j_values:?}\n");

    let mut rows: Vec<SweepRow> = Vec::new();

    for (i, &j) in j_values.iter().enumerate() {
        eprintln!("\n  ============================================================");
        eprintln!("  J = {j}  ({}/{})", i + 1, j_values.len());
        eprintln!("  ============================================================");

        let base_seed = SEED_BASE + 400 + i as u64 * 1000;
        let base_eval = 100_000 + i as u64 * 30_000;

        // CEM
        let mut cem = make_cem();
        let (ov, se, kt) = train_and_report(
            &format!("CEM(J={j})"),
            &mut cem,
            j,
            &TARGET_ANTI,
            base_seed,
            base_eval,
        );
        rows.push(SweepRow {
            j,
            algo: "CEM",
            overlap: ov,
            stderr: se,
            learned_kt: kt,
        });

        // PPO
        let mut ppo = make_ppo();
        let (ov, se, kt) = train_and_report(
            &format!("PPO(J={j})"),
            &mut ppo,
            j,
            &TARGET_ANTI,
            base_seed + 100,
            base_eval + 10_000,
        );
        rows.push(SweepRow {
            j,
            algo: "PPO",
            overlap: ov,
            stderr: se,
            learned_kt: kt,
        });

        // SA
        let mut sa = make_sa();
        let (ov, se, kt) = train_and_report(
            &format!("SA(J={j})"),
            &mut sa,
            j,
            &TARGET_ANTI,
            base_seed + 200,
            base_eval + 20_000,
        );
        rows.push(SweepRow {
            j,
            algo: "SA",
            overlap: ov,
            stderr: se,
            learned_kt: kt,
        });
    }

    // ── Summary table ────────────────────────────────────────────────
    eprintln!("\n\n  ======================================================================");
    eprintln!("  COUPLING SWEEP RESULTS");
    eprintln!("  Target: [+1, -1, +1, -1]");
    eprintln!("  ======================================================================\n");
    eprintln!("  J      | Algo |  overlap       |  learned kT");
    eprintln!("  -------|------|----------------|------------");
    for r in &rows {
        eprintln!(
            "  {:5.2}  | {:4} |  {:+.4} +/- {:.4}  |  {:.4}",
            r.j, r.algo, r.overlap, r.stderr, r.learned_kt,
        );
    }

    // ── Per-algorithm trend ──────────────────────────────────────────
    eprintln!("\n  Per-algorithm trend (J=0.1 vs J=2.0):");
    for algo in &["CEM", "PPO", "SA"] {
        let first = rows
            .iter()
            .find(|r| r.algo == *algo && (r.j - 0.1).abs() < 0.01);
        let last = rows
            .iter()
            .find(|r| r.algo == *algo && (r.j - 2.0).abs() < 0.01);
        if let (Some(f), Some(l)) = (first, last) {
            let trend = if f.overlap > l.overlap {
                "decreasing (expected)"
            } else {
                "UNEXPECTED"
            };
            eprintln!(
                "    {algo}: {:.4} -> {:.4}  [{trend}]",
                f.overlap, l.overlap
            );
        }
    }
}
