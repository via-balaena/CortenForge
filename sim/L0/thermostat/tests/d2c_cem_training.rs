//! D2c — Algorithm comparison: which RL method discovers the SR-optimal temperature?
//!
//! Trains four algorithms (CEM, TD3, PPO, SAC) on the SR environment and
//! compares their deterministic evaluation results. Gates A–D are checked
//! per algorithm; the comparison table is the headline result.
//!
//! All tests require `--release` (~10 min each, ~40 min total if serial).
//!
//! Spec: `docs/thermo_computing/03_phases/d2_stochastic_resonance.md` §11 D2c
//!
//! Post-Ch 41 §2.2 Decision 1: every algorithm's reported `mean_reward`
//! is in per-episode-total units (sum of rewards across `n_envs`
//! trajectories, divided by `n_envs`). The per-step mean is no longer
//! directly reported. Gate B's within-algorithm monotonicity check
//! (`best_last_10 > first_5_mean`) is robust to the rescaling by
//! construction — a positive scalar factor preserves the `>` ordering.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::doc_markdown,
    clippy::too_many_lines
)]

use std::f64::consts::PI;
use std::sync::Arc;

use sim_core::DVector;
use sim_ml_bridge::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, LinearQ,
    LinearStochasticPolicy, LinearValue, ObservationSpace, OptimizerConfig, Policy, Ppo,
    PpoHyperparams, Sac, SacHyperparams, SimEnv, Td3, Td3Hyperparams, Tensor, TrainingBudget,
    VecEnv,
};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, OscillatingField, PassiveStack};

// ─── MJCF model (spec §6) ─────────────────────────────────────────────────

const SR_XML: &str = r#"
<mujoco model="stochastic-resonance">
  <option timestep="0.001" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="particle">
      <joint name="x" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <general name="temp_ctrl" joint="x" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 10"/>
  </actuator>
</mujoco>
"#;

// ─── Central parameters (spec §9) ─────────────────────────────────────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;
const SEED_BASE: u64 = 20_260_412;

const KRAMERS_RATE: f64 = 0.01214;

fn signal_omega() -> f64 {
    2.0 * PI * KRAMERS_RATE
}

// ─── Episode parameters (spec §8.5) ──────────────────────────────────────

const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;

// ─── Training ─────────────────────────────────────────────────────────────

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const N_EVAL_EPISODES: usize = 20;

const OBS_DIM: usize = 2;
const ACT_DIM: usize = 1;

// ─── Helpers ──────────────────────────────────────────────────────────────

fn make_training_vecenv(seed: u64) -> VecEnv {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, seed, 0)
            .with_ctrl_temperature(0);
    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, omega, 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let model = Arc::new(model);
    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .unwrap();
    let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

    VecEnv::builder(model, N_ENVS)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

fn make_eval_env(seed: u64) -> SimEnv {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, seed, 0)
            .with_ctrl_temperature(0);
    let double_well = DoubleWellPotential::new(DELTA_V, X_0, 0);
    let signal = OscillatingField::new(A_0, omega, 0.0, 0);

    PassiveStack::builder()
        .with(thermostat)
        .with(double_well)
        .with(signal)
        .build()
        .install(&mut model);

    let model = Arc::new(model);
    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .unwrap();
    let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

    SimEnv::builder(model)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

/// Evaluate a policy: returns (synchrony_per_episode, mean_kT_per_episode).
fn evaluate_policy(
    policy: &dyn Policy,
    n_episodes: usize,
    seed_offset: u64,
) -> (Vec<f64>, Vec<f64>) {
    let mut synchronies = Vec::with_capacity(n_episodes);
    let mut temperatures = Vec::with_capacity(n_episodes);

    for i in 0..n_episodes {
        let mut env = make_eval_env(SEED_BASE + seed_offset + i as u64);
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

fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

fn obs_scale() -> Vec<f64> {
    vec![1.0 / X_0, 1.0]
}

/// Train an algorithm, evaluate, report results. Returns (sync_mean, kT_mean).
fn train_and_evaluate(
    name: &str,
    algo: &mut dyn Algorithm,
    seed: u64,
    eval_offset: u64,
) -> (f64, f64) {
    let mut env = make_training_vecenv(seed);

    eprintln!("\n--- {name}: training ({N_EPOCHS} epochs, {N_ENVS} envs) ---");

    let metrics = algo.train(&mut env, TrainingBudget::Epochs(N_EPOCHS), seed, &|m| {
        if m.epoch % 10 == 0 || m.epoch == N_EPOCHS - 1 {
            eprintln!(
                "  {name} epoch {:3}: mean_reward = {:+.6} (per-episode total across n_envs trajectories)",
                m.epoch, m.mean_reward,
            );
        }
    });

    // Gate B: learning improves (mean_first_5 vs best_last_10)
    let first_5_mean: f64 = metrics[..5.min(metrics.len())]
        .iter()
        .map(|m| m.mean_reward)
        .sum::<f64>()
        / 5.0_f64.min(metrics.len() as f64);
    let best_last_10 = metrics[metrics.len().saturating_sub(10)..]
        .iter()
        .map(|m| m.mean_reward)
        .fold(f64::NEG_INFINITY, f64::max);
    let gate_b = best_last_10 > first_5_mean;
    eprintln!(
        "  {name} Gate B: mean_first_5={first_5_mean:.6}, best_last_10={best_last_10:.6} → {}",
        if gate_b { "PASS" } else { "FAIL" }
    );

    // Evaluate deterministic policy
    let trained = algo.policy_artifact().to_policy().unwrap();
    let (syncs, temps) = evaluate_policy(trained.as_ref(), N_EVAL_EPISODES, eval_offset);
    let (sync_mean, sync_stderr) = mean_stderr(&syncs);
    let (temp_mean, temp_stderr) = mean_stderr(&temps);

    eprintln!("  {name} eval: synchrony = {sync_mean:.6} ± {sync_stderr:.6}");
    eprintln!("  {name} eval: kT = {temp_mean:.4} ± {temp_stderr:.4}");

    // Gate A: significant synchrony
    let t_stat = if sync_stderr > 1e-15 {
        sync_mean / sync_stderr
    } else {
        0.0
    };
    let gate_a = t_stat.abs() > 2.861;
    eprintln!(
        "  {name} Gate A: |t|={:.2} → {}",
        t_stat.abs(),
        if gate_a { "PASS" } else { "FAIL" }
    );

    // Policy params
    let params = trained.params();
    if params.len() <= 5 {
        eprintln!("  {name} params: {params:?}");
    }

    (sync_mean, temp_mean)
}

// ─── Tests ────────────────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~10 min)"]
fn d2c_cem() {
    let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    policy.set_params(&[0.0, 0.0, 2.0]);
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
    let (sync, kt) = train_and_evaluate("CEM", &mut algo, SEED_BASE, 5000);
    eprintln!("\nCEM summary: synchrony={sync:.6}, kT={kt:.4}");
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn d2c_td3() {
    let scale = obs_scale();
    let policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &scale);
    let target_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &scale);
    let q1 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let q2 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let target_q1 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let target_q2 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);

    let mut algo = Td3::new(
        Box::new(policy),
        Box::new(target_policy),
        Box::new(q1),
        Box::new(q2),
        Box::new(target_q1),
        Box::new(target_q2),
        OptimizerConfig::adam(3e-4),
        Td3Hyperparams {
            gamma: 0.99,
            tau: 0.005,
            policy_noise: 0.2,
            noise_clip: 0.5,
            exploration_noise: 0.5,
            policy_delay: 2,
            batch_size: 256,
            buffer_capacity: 100_000,
            warmup_steps: 1000,
            max_episode_steps: EPISODE_STEPS,
        },
    );
    let (sync, kt) = train_and_evaluate("TD3", &mut algo, SEED_BASE + 100, 7000);
    eprintln!("\nTD3 summary: synchrony={sync:.6}, kT={kt:.4}");
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn d2c_ppo() {
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
    let (sync, kt) = train_and_evaluate("PPO", &mut algo, SEED_BASE + 200, 8000);
    eprintln!("\nPPO summary: synchrony={sync:.6}, kT={kt:.4}");
}

#[test]
#[ignore = "requires --release (~10 min)"]
fn d2c_sac() {
    let scale = obs_scale();
    let policy = LinearStochasticPolicy::new(OBS_DIM, ACT_DIM, &scale, 0.0);
    let q1 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let q2 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let target_q1 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);
    let target_q2 = LinearQ::new(OBS_DIM, ACT_DIM, &scale);

    let mut algo = Sac::new(
        Box::new(policy),
        Box::new(q1),
        Box::new(q2),
        Box::new(target_q1),
        Box::new(target_q2),
        OptimizerConfig::adam(3e-4),
        SacHyperparams {
            gamma: 0.99,
            tau: 0.005,
            alpha_init: 0.2,
            auto_alpha: true,
            target_entropy: -(ACT_DIM as f64),
            alpha_lr: 3e-4,
            batch_size: 256,
            buffer_capacity: 100_000,
            warmup_steps: 1000,
            max_episode_steps: EPISODE_STEPS,
        },
    );
    let (sync, kt) = train_and_evaluate("SAC", &mut algo, SEED_BASE + 300, 9000);
    eprintln!("\nSAC summary: synchrony={sync:.6}, kT={kt:.4}");
}
