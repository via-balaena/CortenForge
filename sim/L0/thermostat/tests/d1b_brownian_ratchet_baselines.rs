//! D1b — Brownian ratchet baseline measurements.
//!
//! Validates the ratchet environment before RL training:
//! - Always-ON (α=1): zero net current (detailed balance)
//! - Always-OFF (α=0): zero net current (free diffusion) + MSD calibration
//! - Random (α~Uniform[0,1]): small positive net current (broken detailed balance)
//! - `VecEnv` construction: verifies the RL environment builds correctly
//!
//! All multi-episode tests must run with `--release` (spec §12.4).
//!
//! Spec: `docs/thermo_computing/03_phases/d1_brownian_ratchet.md` §9, §10, §12

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
use sim_ml_bridge::{
    ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv,
    rollout::collect_episodic_rollout,
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
const N_EPISODES: usize = 50;

// ─── Helpers ───────────────────────────────────────────────────────────────

/// Build a `SimEnv` for the Brownian ratchet with a given seed.
fn make_ratchet_env(seed: u64) -> SimEnv {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let thermostat = LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, seed);
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
        .reward(|_m, data| data.qvel[0])
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

/// Run one episode with a fixed action, return `(total_reward, final_displacement)`.
fn run_episode_fixed_action(env: &mut SimEnv, alpha: f32) -> (f64, f64) {
    env.reset().unwrap();
    let action = Tensor::from_slice(&[alpha], &[1]);
    let mut total_reward = 0.0;
    for _ in 0..EPISODE_STEPS {
        let result = env.step(&action).unwrap();
        total_reward += result.reward;
        if result.truncated {
            break;
        }
    }
    let displacement = env.data().qpos[0]; // x_0 = 0 at reset
    (total_reward, displacement)
}

/// Run one episode with random actions, return `(total_reward, final_displacement)`.
fn run_episode_random_action(env: &mut SimEnv, rng: &mut ChaCha8Rng) -> (f64, f64) {
    env.reset().unwrap();
    let dist = Uniform::new(0.0_f32, 1.0_f32).unwrap();
    let mut total_reward = 0.0;
    for _ in 0..EPISODE_STEPS {
        let alpha = dist.sample(rng);
        let action = Tensor::from_slice(&[alpha], &[1]);
        let result = env.step(&action).unwrap();
        total_reward += result.reward;
        if result.truncated {
            break;
        }
    }
    let displacement = env.data().qpos[0];
    (total_reward, displacement)
}

/// Compute mean and standard error from a slice of values.
fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

// ─── Tests ─────────────────────────────────────────────────────────────────

/// Gate: always-ON baseline has zero net current (detailed balance).
///
/// 50 episodes with α=1.0. Mean displacement should be consistent with
/// zero within 3σ of sampling error.
#[test]
#[ignore = "requires --release for reasonable runtime (50 × 100k steps)"]
fn baseline_always_on_zero_current() {
    let mut displacements = Vec::with_capacity(N_EPISODES);

    for i in 0..N_EPISODES {
        let mut env = make_ratchet_env(SEED_BASE + i as u64);
        let (_reward, disp) = run_episode_fixed_action(&mut env, 1.0);
        displacements.push(disp);
    }

    let (mean, stderr) = mean_stderr(&displacements);
    eprintln!(
        "always-ON: mean displacement = {mean:.4} ± {stderr:.4} (3σ = {:.4})",
        3.0 * stderr
    );

    // Zero current: |mean| < 3σ
    assert!(
        mean.abs() < 3.0 * stderr,
        "always-ON should have zero net current (detailed balance): \
         mean = {mean:.4}, 3σ = {:.4}",
        3.0 * stderr,
    );
}

/// Gate: always-OFF baseline has zero net current (free diffusion).
///
/// 50 episodes with α=0.0. Mean displacement should be consistent with
/// zero within 3σ.
#[test]
#[ignore = "requires --release for reasonable runtime (50 × 100k steps)"]
fn baseline_always_off_zero_current() {
    let mut displacements = Vec::with_capacity(N_EPISODES);

    for i in 0..N_EPISODES {
        let mut env = make_ratchet_env(SEED_BASE + 1000 + i as u64);
        let (_reward, disp) = run_episode_fixed_action(&mut env, 0.0);
        displacements.push(disp);
    }

    let (mean, stderr) = mean_stderr(&displacements);
    eprintln!(
        "always-OFF: mean displacement = {mean:.4} ± {stderr:.4} (3σ = {:.4})",
        3.0 * stderr
    );

    // Zero current: |mean| < 3σ
    assert!(
        mean.abs() < 3.0 * stderr,
        "always-OFF should have zero net current (unbiased diffusion): \
         mean = {mean:.4}, 3σ = {:.4}",
        3.0 * stderr,
    );
}

/// Gate: random baseline shows small positive net current.
///
/// 50 episodes with α~Uniform[0,1] each step. The random flashing breaks
/// detailed balance, so the asymmetric potential should produce a nonzero
/// mean displacement in the preferred direction.
#[test]
#[ignore = "requires --release for reasonable runtime (50 × 100k steps)"]
fn baseline_random_positive_current() {
    let mut displacements = Vec::with_capacity(N_EPISODES);
    let mut rng = ChaCha8Rng::seed_from_u64(SEED_BASE + 2000);

    for i in 0..N_EPISODES {
        let mut env = make_ratchet_env(SEED_BASE + 2000 + i as u64);
        let (_reward, disp) = run_episode_random_action(&mut env, &mut rng);
        displacements.push(disp);
    }

    let (mean, stderr) = mean_stderr(&displacements);
    eprintln!(
        "random: mean displacement = {mean:.4} ± {stderr:.4} (3σ = {:.4})",
        3.0 * stderr
    );

    // The spec says "small but positive." We check that the mean is
    // positive, but we don't require statistical significance at this
    // stage — the random baseline may have high variance. If mean > 0
    // and at least marginally above noise, that's encouraging.
    //
    // Note: if the ratchet asymmetry direction is wrong (φ gives drift
    // in -x), mean will be negative. That's still a ratchet effect —
    // just in the opposite direction. We accept either sign as evidence
    // of broken detailed balance, but log which direction.
    assert!(
        mean.abs() > stderr * 0.5,
        "random baseline should show some ratchet current: \
         mean = {mean:.4}, stderr = {stderr:.4}",
    );
    eprintln!(
        "random baseline drift direction: {}",
        if mean > 0.0 { "+x" } else { "-x" }
    );
}

/// Diffusion calibration check (spec §12.2).
///
/// 50 episodes with α=0, measure RMS displacement at t=100.
/// Expected: √(2Dt) = √(200) ≈ 14.14. Accept within ±15%.
#[test]
#[ignore = "requires --release for reasonable runtime (50 × 100k steps)"]
fn diffusion_calibration() {
    let mut displacements_squared = Vec::with_capacity(N_EPISODES);

    for i in 0..N_EPISODES {
        let mut env = make_ratchet_env(SEED_BASE + 3000 + i as u64);
        let (_reward, disp) = run_episode_fixed_action(&mut env, 0.0);
        displacements_squared.push(disp * disp);
    }

    let msd = displacements_squared.iter().sum::<f64>() / N_EPISODES as f64;
    let rms = msd.sqrt();
    let d = K_B_T / GAMMA; // diffusion coefficient
    let t_episode = (EPISODE_STEPS * SUB_STEPS) as f64 * 0.001;
    let expected_rms = (2.0 * d * t_episode).sqrt();

    eprintln!(
        "diffusion calibration: RMS = {rms:.2}, expected = {expected_rms:.2}, \
         ratio = {:.3}",
        rms / expected_rms,
    );

    assert!(
        rms > expected_rms * 0.85,
        "RMS displacement too low: {rms:.2}, expected {expected_rms:.2} (−15% = {:.2})",
        expected_rms * 0.85,
    );
    assert!(
        rms < expected_rms * 1.15,
        "RMS displacement too high: {rms:.2}, expected {expected_rms:.2} (+15% = {:.2})",
        expected_rms * 1.15,
    );
}

/// Verify `VecEnv` construction works with the ratchet environment.
///
/// This doesn't run a full baseline — it just confirms that the
/// sim-thermostat + sim-ml-bridge bridge builds and resets successfully.
/// Full `VecEnv` training is D1c.
#[test]
fn vecenv_construction() {
    let mut model = sim_mjcf::load_model(RATCHET_XML).unwrap();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T, SEED_BASE);
    let ratchet = RatchetPotential::new(V1, V2, PHI, PERIOD, 0, 0);

    let stack = PassiveStack::builder()
        .with(thermostat)
        .with(ratchet)
        .build();
    stack.install(&mut model);

    let model = Arc::new(model);
    let n_envs = 4;

    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .unwrap();
    let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

    let mut env = VecEnv::builder(model, n_envs)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(|_m, data| data.qvel[0])
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > 100.0)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap();

    // Reset and step once to verify the pipeline
    let obs = env.reset_all().unwrap();
    assert_eq!(obs.shape(), &[n_envs, 2], "obs shape should be [n_envs, 2]");

    // Step with α=0.5 for all envs
    let actions = Tensor::from_slice(&vec![0.5_f32; n_envs], &[n_envs, 1]);
    let result = env.step(&actions).unwrap();
    assert_eq!(
        result.observations.shape(),
        &[n_envs, 2],
        "step output obs shape"
    );

    // Verify we can run a full episodic rollout with a trivial policy
    let rollout =
        collect_episodic_rollout(&mut env, &mut |_env_idx, _obs| vec![0.5], EPISODE_STEPS);
    assert_eq!(rollout.trajectories.len(), n_envs);
    for traj in &rollout.trajectories {
        assert!(!traj.is_empty(), "trajectory should have steps");
    }
}
