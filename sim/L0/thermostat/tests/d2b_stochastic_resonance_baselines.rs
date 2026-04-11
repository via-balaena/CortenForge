//! D2b — Stochastic resonance baseline measurements.
//!
//! Validates the SR environment before RL training:
//! - Temperature sweep: SR curve (synchrony vs kT) with clear peak
//! - Low noise (kT×0.1): zero synchrony (no switching)
//! - High noise (kT×5.0): zero synchrony (random switching)
//! - `VecEnv` construction: verifies the RL environment builds correctly
//!
//! All multi-episode tests must run with `--release` (spec §13.4).
//!
//! Spec: `docs/thermo_computing/03_phases/d2_stochastic_resonance.md` §10, §11 D2b

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_lossless,
    clippy::float_cmp,
    clippy::suboptimal_flops,
    clippy::doc_markdown
)]

use std::f64::consts::PI;
use std::sync::Arc;

use sim_core::DVector;
use sim_ml_bridge::{
    ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv,
    rollout::collect_episodic_rollout,
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
const SEED_BASE: u64 = 20_260_411; // different from D2a

/// Kramers rate at kT=1 for Phase 3 central parameters (ΔV=3, γ=10, M=1).
const KRAMERS_RATE: f64 = 0.01214;

/// Signal angular frequency: ω = 2π·k₀(kT=1) (spec §9).
fn signal_omega() -> f64 {
    2.0 * PI * KRAMERS_RATE
}

// ─── Episode parameters (spec §8.5) ──────────────────────────────────────

const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;

// ─── Sweep parameters (spec §10.1) ───────────────────────────────────────

const N_KT_VALUES: usize = 30;
const KT_MIN: f64 = 0.1;
const KT_MAX: f64 = 5.0;
const N_EPISODES_PER_KT: usize = 20;

// ─── Baseline parameters ─────────────────────────────────────────────────

const N_BASELINE_EPISODES: usize = 20;

// ─── Helpers ──────────────────────────────────────────────────────────────

/// Build a `SimEnv` for stochastic resonance with a given thermostat seed.
///
/// The reward is the synchrony metric: `sign(qpos[0]) · cos(ω·t)`.
/// The action is the kT multiplier written to `data.ctrl[0]`.
fn make_sr_env(seed: u64) -> SimEnv {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    let thermostat =
        LangevinThermostat::new(DVector::from_element(model.nv, GAMMA), K_B_T_BASE, seed)
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

/// Run one episode with a fixed kT multiplier.
/// Returns mean synchrony (total reward / episode steps).
fn run_episode_fixed_temp(env: &mut SimEnv, kt_mult: f32) -> f64 {
    env.reset().unwrap();
    let action = Tensor::from_slice(&[kt_mult], &[1]);
    let mut total_reward = 0.0;
    let mut steps = 0;
    for _ in 0..EPISODE_STEPS {
        let result = env.step(&action).unwrap();
        total_reward += result.reward;
        steps += 1;
        if result.truncated {
            break;
        }
    }
    total_reward / steps as f64
}

/// Compute mean and standard error from a slice of values.
fn mean_stderr(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
    let stderr = (variance / n).sqrt();
    (mean, stderr)
}

/// Generate logarithmically spaced kT multipliers.
fn log_spaced_kt_values() -> Vec<f64> {
    let log_min = KT_MIN.ln();
    let log_max = KT_MAX.ln();
    (0..N_KT_VALUES)
        .map(|i| {
            let t = i as f64 / (N_KT_VALUES - 1) as f64;
            (log_min + t * (log_max - log_min)).exp()
        })
        .collect()
}

// ─── Tests ────────────────────────────────────────────────────────────────

/// Verify `VecEnv` construction works with the SR environment.
///
/// Confirms that the sim-thermostat + sim-ml-bridge bridge builds with the
/// synchrony reward, resets successfully, and can step + run an episodic
/// rollout. Does NOT run a full baseline — that's the `#[ignore]` tests.
#[test]
fn vecenv_construction() {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    let thermostat = LangevinThermostat::new(
        DVector::from_element(model.nv, GAMMA),
        K_B_T_BASE,
        SEED_BASE,
    )
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
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > 100.0)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap();

    // Reset and step once to verify the pipeline
    let obs = env.reset_all().unwrap();
    assert_eq!(obs.shape(), &[n_envs, 2], "obs shape should be [n_envs, 2]");

    // Step with kT_mult=1.0 for all envs
    let actions = Tensor::from_slice(&vec![1.0_f32; n_envs], &[n_envs, 1]);
    let result = env.step(&actions).unwrap();
    assert_eq!(
        result.observations.shape(),
        &[n_envs, 2],
        "step output obs shape"
    );

    // Verify we can run a full episodic rollout with a trivial policy
    let rollout =
        collect_episodic_rollout(&mut env, &mut |_env_idx, _obs| vec![1.0], EPISODE_STEPS);
    assert_eq!(rollout.trajectories.len(), n_envs);
    for traj in &rollout.trajectories {
        assert!(!traj.is_empty(), "trajectory should have steps");
    }
}

/// Gate: temperature sweep reveals a clear SR peak.
///
/// 30 kT multipliers in [0.1, 5.0] (log-spaced), 20 episodes each.
/// The peak synchrony must be significantly positive (p < 0.01).
#[test]
#[ignore = "requires --release (300M+ physics steps, ~30s release)"]
fn temperature_sweep_sr_curve() {
    let kt_values = log_spaced_kt_values();
    let mut sweep_means = Vec::with_capacity(N_KT_VALUES);
    let mut sweep_stderrs = Vec::with_capacity(N_KT_VALUES);

    eprintln!("\nSR temperature sweep ({N_KT_VALUES} values, {N_EPISODES_PER_KT} episodes each):");
    eprintln!("{:>10}  {:>12}  {:>10}", "kT_mult", "synchrony", "± stderr");
    eprintln!("{}", "-".repeat(36));

    for (j, &kt_mult) in kt_values.iter().enumerate() {
        let mut synchronies = Vec::with_capacity(N_EPISODES_PER_KT);
        for i in 0..N_EPISODES_PER_KT {
            let seed = SEED_BASE + (j * N_EPISODES_PER_KT + i) as u64;
            let mut env = make_sr_env(seed);
            let sync = run_episode_fixed_temp(&mut env, kt_mult as f32);
            synchronies.push(sync);
        }
        let (mean, stderr) = mean_stderr(&synchronies);
        sweep_means.push(mean);
        sweep_stderrs.push(stderr);

        eprintln!("{kt_mult:10.4}  {mean:12.6}  {stderr:10.6}");
    }

    // Find the peak
    let (peak_idx, &peak_mean) = sweep_means
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap();
    let peak_kt = kt_values[peak_idx];
    let peak_stderr = sweep_stderrs[peak_idx];

    eprintln!("\n--- Empirical SR peak ---");
    eprintln!("kT_mult = {peak_kt:.4}, synchrony = {peak_mean:.6} ± {peak_stderr:.6}");

    // Gate: peak synchrony significantly positive (p < 0.01)
    // One-sample t-test: t = mean / stderr, df = N-1 = 19.
    // Critical value for df=19, two-tailed α=0.01: t_crit ≈ 2.861
    let t_stat = peak_mean / peak_stderr;
    let t_critical = 2.861;

    eprintln!(
        "t-test: |t| = {:.2}, critical = {t_critical} (df=19, α=0.01)",
        t_stat.abs()
    );

    assert!(
        t_stat.abs() > t_critical,
        "Gate FAIL: peak synchrony not significantly > 0. \
         mean = {peak_mean:.6}, stderr = {peak_stderr:.6}, |t| = {:.2}, need > {t_critical}",
        t_stat.abs(),
    );
    eprintln!("Gate: PASS — SR peak is statistically significant\n");

    // Also verify the curve has a non-trivial shape: peak > 2× the tails
    let tail_low = sweep_means[0]; // kT = 0.1
    let tail_high = *sweep_means.last().unwrap(); // kT = 5.0
    let tail_max = tail_low.abs().max(tail_high.abs());

    eprintln!(
        "Tail check: peak = {peak_mean:.6}, |low tail| = {:.6}, |high tail| = {:.6}",
        tail_low.abs(),
        tail_high.abs()
    );

    if tail_max > 1e-10 {
        assert!(
            peak_mean > 2.0 * tail_max,
            "SR curve shape: peak ({peak_mean:.6}) should be > 2× tails ({tail_max:.6})",
        );
        eprintln!("Tail check: PASS — peak > 2× tails");
    } else {
        eprintln!("Tail check: SKIP — tails are effectively zero (as expected)");
    }
}

/// Gate: low noise (kT×0.1) produces zero synchrony.
///
/// At kT_eff = 0.1, the Kramers rate is ~10⁻¹³ — effectively no switching.
/// The particle is trapped in one well. Expected synchrony ≈ 0.
#[test]
#[ignore = "requires --release (100M physics steps)"]
fn baseline_low_noise_zero_synchrony() {
    let mut synchronies = Vec::with_capacity(N_BASELINE_EPISODES);

    for i in 0..N_BASELINE_EPISODES {
        let seed = SEED_BASE + 10_000 + i as u64;
        let mut env = make_sr_env(seed);
        let sync = run_episode_fixed_temp(&mut env, 0.1);
        synchronies.push(sync);
    }

    let (mean, stderr) = mean_stderr(&synchronies);
    eprintln!(
        "low noise (kT×0.1): mean synchrony = {mean:.6} ± {stderr:.6} (3σ = {:.6})",
        3.0 * stderr
    );

    // Zero synchrony: |mean| < 3σ
    assert!(
        mean.abs() < 3.0 * stderr,
        "low noise should have zero synchrony: mean = {mean:.6}, 3σ = {:.6}",
        3.0 * stderr,
    );
}

/// Gate: high noise (kT×5.0) produces zero synchrony.
///
/// At kT_eff = 5.0, switching is rapid and uncorrelated with the signal.
/// Expected synchrony ≈ 0.
#[test]
#[ignore = "requires --release (100M physics steps)"]
fn baseline_high_noise_zero_synchrony() {
    let mut synchronies = Vec::with_capacity(N_BASELINE_EPISODES);

    for i in 0..N_BASELINE_EPISODES {
        let seed = SEED_BASE + 20_000 + i as u64;
        let mut env = make_sr_env(seed);
        let sync = run_episode_fixed_temp(&mut env, 5.0);
        synchronies.push(sync);
    }

    let (mean, stderr) = mean_stderr(&synchronies);
    eprintln!(
        "high noise (kT×5.0): mean synchrony = {mean:.6} ± {stderr:.6} (3σ = {:.6})",
        3.0 * stderr
    );

    // Zero synchrony: |mean| < 3σ
    assert!(
        mean.abs() < 3.0 * stderr,
        "high noise should have zero synchrony: mean = {mean:.6}, 3σ = {:.6}",
        3.0 * stderr,
    );
}
