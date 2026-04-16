//! D1c — CEM trains a policy that produces directed ratchet current.
//!
//! The headline test for D1: an RL agent (CEM with `LinearPolicy`)
//! discovers how to flash a Brownian ratchet to harvest thermal noise
//! for directed transport. Three gates:
//!
//! - Gate A: trained policy produces statistically significant ratchet
//!   current (p < 0.01)
//! - Gate B: CEM's fitness improves over training
//! - Gate C: trained policy outperforms the random baseline by ≥ 2×
//!
//! All tests require `--release` (160M+ physics steps across training).
//!
//! Spec: `docs/thermo_computing/03_phases/d1_brownian_ratchet.md` §10 D1c

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::float_cmp
)]

use std::sync::Arc;

use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;
use rand_distr::{Distribution, Uniform};
use sim_core::DVector;
use sim_rl::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, ObservationSpace,
    Policy, SimEnv, Tensor, TrainingBudget, VecEnv,
};
use sim_thermostat::{LangevinThermostat, PassiveStack, RatchetPotential};

// ─── MJCF model (spec §5) ─────────────────────────────────────────────────

const RATCHET_XML: &str = r#"
<mujoco model="brownian-ratchet">
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
    <general name="ratchet_ctrl" joint="x" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 1"/>
  </actuator>
</mujoco>
"#;

// ─── Central parameters (spec §8) ─────────────────────────────────────────

const V1: f64 = 1.0;
const V2: f64 = 0.25;
const PHI: f64 = std::f64::consts::FRAC_PI_4;
const PERIOD: f64 = 1.0;
const GAMMA: f64 = 1.0;
const K_B_T: f64 = 1.0;
const SEED_BASE: u64 = 20_260_410;

// ─── Episode parameters (spec §7.4) ───────────────────────────────────────

const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 1_000;

// ─── CEM hyperparameters (spec §10 D1c) ───────────────────────────────────

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const ELITE_FRACTION: f64 = 0.2;
const NOISE_STD: f64 = 0.5;
const NOISE_DECAY: f64 = 0.99;
const NOISE_MIN: f64 = 0.05;

// ─── Evaluation ────────────────────────────────────────────────────────────

const N_EVAL_EPISODES: usize = 50;

// ─── Helpers ───────────────────────────────────────────────────────────────

/// Build a `VecEnv` for CEM training with negated reward (spec §7.3).
fn make_training_vecenv(seed: u64) -> VecEnv {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, seed, 0);
    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(ratchet)
        .build();
    stack.install(&mut model);

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
        // Negated reward: CEM maximizes fitness, drift is in -x,
        // so -qvel[0] is positive for good ratchet performance.
        .reward(|_m, data| -data.qvel[0])
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

/// Build a single `SimEnv` for evaluation with negated reward.
fn make_eval_env(seed: u64) -> SimEnv {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, seed, 0);
    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(ratchet)
        .build();
    stack.install(&mut model);

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
        .reward(|_m, data| -data.qvel[0])
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

/// Evaluate a policy over N episodes, returning per-episode displacements
/// (raw qpos[0], NOT negated — displacement is the physical quantity).
fn evaluate_displacement(policy: &dyn Policy, n_episodes: usize, seed_offset: u64) -> Vec<f64> {
    let mut displacements = Vec::with_capacity(n_episodes);
    for i in 0..n_episodes {
        let mut env = make_eval_env(SEED_BASE + seed_offset + i as u64);
        let obs = env.reset().unwrap();
        let mut obs_vec: Vec<f32> = obs.as_slice().to_vec();

        for _ in 0..EPISODE_STEPS {
            // forward() returns action values; obs_scale is baked into the policy
            let action = policy.forward(&obs_vec);
            let action_tensor = Tensor::from_f64_slice(&action, &[1]);
            let result = env.step(&action_tensor).unwrap();
            obs_vec = result.observation.as_slice().to_vec();
            if result.truncated {
                break;
            }
        }
        displacements.push(env.data().qpos[0]);
    }
    displacements
}

/// Run random-action episodes, return displacements.
fn random_baseline_displacements(n_episodes: usize, seed_offset: u64) -> Vec<f64> {
    let mut rng = ChaCha8Rng::seed_from_u64(SEED_BASE + seed_offset);
    let dist = Uniform::new(0.0_f32, 1.0_f32).unwrap();
    let mut displacements = Vec::with_capacity(n_episodes);

    for i in 0..n_episodes {
        let mut env = make_eval_env(SEED_BASE + seed_offset + 100 + i as u64);
        env.reset().unwrap();

        for _ in 0..EPISODE_STEPS {
            let alpha = dist.sample(&mut rng);
            let action = Tensor::from_slice(&[alpha], &[1]);
            let result = env.step(&action).unwrap();
            if result.truncated {
                break;
            }
        }
        displacements.push(env.data().qpos[0]);
    }
    displacements
}

/// Mean and standard error.
fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

// ─── Tests ─────────────────────────────────────────────────────────────────

/// D1c: CEM discovers the ratchet flashing strategy.
///
/// Trains CEM for 100 epochs with 32 envs, evaluates the trained policy,
/// and checks all three gates.
#[test]
#[ignore = "requires --release (160M+ physics steps, ~30s release)"]
fn cem_discovers_ratchet_flashing() {
    // ── 1. Train CEM ────────────────────────────────────────────────────

    let obs_dim = 2; // qpos + qvel
    let act_dim = 1; // ctrl[0]
    let obs_scale = vec![1.0 / PERIOD, 1.0]; // normalize qpos by period, qvel by v_th=1

    let policy = LinearPolicy::new(obs_dim, act_dim, &obs_scale);
    let hp = CemHyperparams {
        elite_fraction: ELITE_FRACTION,
        noise_std: NOISE_STD,
        noise_decay: NOISE_DECAY,
        noise_min: NOISE_MIN,
        max_episode_steps: EPISODE_STEPS,
    };
    let mut cem = Cem::new(Box::new(policy), hp);

    let mut env = make_training_vecenv(SEED_BASE);

    let metrics = cem.train(
        &mut env,
        TrainingBudget::Epochs(N_EPOCHS),
        SEED_BASE,
        &|m| {
            eprintln!(
                "  epoch {:3}: mean_reward = {:+.4}, done = {}",
                m.epoch, m.mean_reward, m.done_count,
            );
        },
    );

    assert!(
        !metrics.is_empty(),
        "CEM should have produced at least one epoch of metrics",
    );

    // ── 2. Gate B: learning improves ────────────────────────────────────

    let first_fitness = metrics[0].mean_reward;
    let last_10: Vec<f64> = metrics[metrics.len().saturating_sub(10)..]
        .iter()
        .map(|m| m.mean_reward)
        .collect();
    let best_last_10 = last_10.iter().copied().fold(f64::NEG_INFINITY, f64::max);

    eprintln!("\nGate B: first_fitness = {first_fitness:.4}, best_last_10 = {best_last_10:.4}");

    assert!(
        best_last_10 > first_fitness,
        "Gate B FAIL: fitness did not improve. \
         first = {first_fitness:.4}, best_last_10 = {best_last_10:.4}",
    );
    eprintln!("Gate B: PASS");

    // ── 3. Evaluate trained policy ──────────────────────────────────────

    let trained_policy = cem.policy_artifact().to_policy().unwrap();
    let trained_disps = evaluate_displacement(trained_policy.as_ref(), N_EVAL_EPISODES, 5000);
    let (trained_mean, trained_stderr) = mean_stderr(&trained_disps);

    eprintln!("\ntrained policy: mean displacement = {trained_mean:.4} ± {trained_stderr:.4}");

    // ── 4. Gate A: ratchet effect (displacement ≠ 0, p < 0.01) ─────────

    // One-sample t-test: t = mean / stderr, reject H0 at p<0.01 for
    // |t| > 2.68 (df=49, two-tailed). Use 2.68 as the critical value.
    let t_stat = trained_mean / trained_stderr;
    let t_critical = 2.68; // df=49, two-tailed, α=0.01

    eprintln!("Gate A: |t| = {:.2}, critical = {t_critical}", t_stat.abs());

    assert!(
        t_stat.abs() > t_critical,
        "Gate A FAIL: trained policy displacement not significantly different from zero. \
         mean = {trained_mean:.4}, stderr = {trained_stderr:.4}, |t| = {:.2}, need > {t_critical}",
        t_stat.abs(),
    );
    eprintln!("Gate A: PASS");

    // ── 5. Gate C: beats random baseline by ≥ 2× ───────────────────────

    let random_disps = random_baseline_displacements(N_EVAL_EPISODES, 6000);
    let (random_mean, random_stderr) = mean_stderr(&random_disps);

    eprintln!("random baseline: mean displacement = {random_mean:.4} ± {random_stderr:.4}");

    // Both displacements are negative (drift in -x). The trained policy
    // should have MORE negative displacement (stronger ratchet current).
    // Compare magnitudes: |trained_mean| > 2 × |random_mean|.
    let trained_magnitude = trained_mean.abs();
    let random_magnitude = random_mean.abs();

    eprintln!(
        "Gate C: |trained| = {trained_magnitude:.4}, |random| = {random_magnitude:.4}, \
         ratio = {:.2}",
        trained_magnitude / random_magnitude.max(1e-10),
    );

    assert!(
        trained_magnitude > 2.0 * random_magnitude,
        "Gate C FAIL: trained policy not 2× better than random. \
         |trained| = {trained_magnitude:.4}, |random| = {random_magnitude:.4}, \
         ratio = {:.2}",
        trained_magnitude / random_magnitude.max(1e-10),
    );
    eprintln!("Gate C: PASS");

    eprintln!("\n=== D1c: ALL GATES PASS ===");
}
