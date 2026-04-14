//! D2c SR rematch — CEM vs richer-proposal SA (Ch 53 §2.2).
//!
//! Ch 53's robustness check on Ch 51 §2.3's `final_reward`-primary
//! framework runs two SA-family variants through the same
//! `run_rematch` driver that Ch 52's basic-SA run used.  This
//! fixture is the richer-proposal SA entry — "richer" in the
//! Ch 53 §2.2 class sense: the proposal distribution evolves
//! during training (here, via the Rechenberg 1/5 success rule).
//!
//! The fixture is a near-copy of `tests/d2c_sr_rematch.rs`, with
//! only the SA builder replaced: the CEM builder, the matched-
//! complexity gate, the MJCF model, the reward shape, the
//! `REMATCH_MASTER_SEED`, and the `BOOTSTRAP_RNG_SEED` are all
//! bit-for-bit identical, so the only source of any difference
//! in `TwoMetricOutcome` between Ch 52 and this fixture is the
//! SA variant itself.  Ch 53 §2.4's "same measurement through the
//! same driver" commitment is honored by this duplication.
//!
//! **Two tests live here:**
//! 1. `d2c_sr_rematch_richer_sa_smoke` — runs in debug-mode
//!    seconds under `cargo test -p sim-opt --release` without
//!    `--ignored`, at a reduced `n_envs = 4`, `episode_steps =
//!    50`, `sub_steps = 10`, and a 2-epoch-per-replicate budget.
//!    The smoke test is a plumbing gate: it verifies that
//!    `run_rematch` accepts `RicherSa` builders, produces a
//!    `TwoMetricOutcome`, and prints the verdict.  The classified
//!    outcome is meaningless (episodes see far less than one
//!    driving-field period at 0.5 s of sim time) and is not
//!    asserted on.
//! 2. `d2c_sr_rematch_richer_sa` — the Ch 53 §2.2 production
//!    run at `n_envs = 32`, `episode_steps = 5000`, `sub_steps =
//!    100`, and `Steps(16_000_000)` per replicate.  `#[ignore]`d
//!    because it requires ~30-60 minutes under `--release`; this
//!    is the run Ch 53 §4's richer-SA digest commit will report
//!    after the second-MBP fires it.
//!
//! ```text
//! # Smoke (runs by default):
//! cargo test -p sim-opt --release d2c_sr_rematch_richer_sa_smoke
//! # Production (fires the 16M-step run):
//! cargo test -p sim-opt --release --ignored d2c_sr_rematch_richer_sa
//! ```

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

use std::sync::Arc;

use rand::SeedableRng;
use rand::rngs::StdRng;
use sim_core::DVector;
use sim_ml_bridge::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, ObservationSpace,
    Policy, TaskConfig, TrainingBudget, VecEnv,
};
use sim_opt::{REMATCH_MASTER_SEED, REMATCH_TASK_NAME, RicherSa, RicherSaHyperparams, run_rematch};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, OscillatingField, PassiveStack};

// ─── MJCF model (duplicated from d2c_sr_rematch.rs:59-75) ─────────────────

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

// ─── Central parameters (duplicated from d2c_sr_rematch.rs:79-91) ──────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;

const KRAMERS_RATE: f64 = 0.01214;

fn signal_omega() -> f64 {
    2.0 * std::f64::consts::PI * KRAMERS_RATE
}

const OBS_DIM: usize = 2;
const ACT_DIM: usize = 1;
const BOOTSTRAP_RNG_SEED: u64 = 0xB007_0057_00AA_0055;

fn obs_scale() -> Vec<f64> {
    vec![1.0 / X_0, 1.0]
}

// ─── Task builder, parameterized on n_envs / episode_steps / sub_steps ────
//
// The production run uses (32, 5000, 100), matching Ch 42 §6's
// anchors.  The smoke test uses (4, 50, 10), cheap enough to
// complete inside one CI-minute under `--release` without
// `--ignored`.  The physics is identical — only the loop counts
// change — so the helper is shared between the two tests.

fn make_training_vecenv(
    master_seed: u64,
    n_envs: usize,
    episode_steps: usize,
    sub_steps: usize,
) -> VecEnv {
    let mut model = sim_mjcf::load_model(SR_XML).unwrap();
    let omega = signal_omega();

    let thermostat = LangevinThermostat::new(
        DVector::from_element(model.nv, GAMMA),
        K_B_T_BASE,
        master_seed,
        0,
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
    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .unwrap();
    let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();
    let truncation_time = (episode_steps as f64) * (sub_steps as f64) * 0.001;

    VecEnv::builder(model, n_envs)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(move |_m, data| data.time > truncation_time)
        .sub_steps(sub_steps)
        .build()
        .unwrap()
}

/// Build the single `TaskConfig` the rematch uses.  `n_envs`
/// comes in through the `build_fn` closure from
/// `Competition::run_replicates`'s call site at per-replicate
/// dispatch time, so it is not baked into the task — only
/// `episode_steps` and `sub_steps` are captured here.
fn rematch_task(episode_steps: usize, sub_steps: usize) -> TaskConfig {
    TaskConfig::from_build_fn(
        REMATCH_TASK_NAME,
        OBS_DIM,
        ACT_DIM,
        obs_scale(),
        move |ne, seed| Ok(make_training_vecenv(seed, ne, episode_steps, sub_steps)),
    )
}

// ─── Builders shared between smoke and production ─────────────────────────

fn cem_builder(episode_steps: usize) -> impl Fn(&TaskConfig) -> Box<dyn Algorithm> + Send + Sync {
    move |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Cem::new(
            Box::new(policy),
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 2.5,
                noise_decay: 0.98,
                noise_min: 0.1,
                max_episode_steps: episode_steps,
            },
        ))
    }
}

fn richer_sa_builder(
    episode_steps: usize,
) -> impl Fn(&TaskConfig) -> Box<dyn Algorithm> + Send + Sync {
    // Ch 53 §2.2 defaults: basic SA's Ch 42 §4.5 core (T_0 = 50,
    // initial sigma = 0.5, alpha = 0.955, T_min = 0.005) plus the
    // 1/5 rule's adaptation_window = 5, sigma_growth_factor =
    // 1.22, target_uphill_rate = 0.2 (literal 1/5), and clamps
    // [0.05, 5.0].  Matched init [0.0, 0.0, 2.0] matches basic
    // SA exactly — any difference between this run and Ch 52's
    // is attributable to the proposal-width adaptation alone.
    move |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(RicherSa::new(
            Box::new(policy),
            RicherSaHyperparams::ch53_defaults(episode_steps),
        ))
    }
}

// ─── Smoke test ────────────────────────────────────────────────────────────

#[test]
fn d2c_sr_rematch_richer_sa_smoke() {
    // Tiny shape: 4 envs × 50 episode-steps × 10 sub-steps = 500
    // sim-steps per episode (~0.5 s of sim time — far less than
    // one driving-field period).  Budget Steps(n_envs *
    // episode_steps * 2) gives 2 epochs per replicate under the
    // uniform budget formula.  With N_INITIAL = 10 replicates
    // per algorithm and a possible folded expansion to 20, the
    // total work is 40 × 2 × 4 × 50 = 16_000 env steps worst-
    // case.  Completes in seconds under `--release`.
    const SMOKE_N_ENVS: usize = 4;
    const SMOKE_EPISODE_STEPS: usize = 50;
    const SMOKE_SUB_STEPS: usize = 10;
    let smoke_budget = TrainingBudget::Steps(SMOKE_N_ENVS * SMOKE_EPISODE_STEPS * 2);

    // Matched-complexity gate (same assertion the production
    // test makes; runs here too so the smoke fails early if the
    // policy shapes drift).
    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let rsa_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        rsa_policy.n_params(),
        "matched-complexity"
    );
    assert_eq!(cem_policy.n_params(), 3, "LinearPolicy(2,1) => 3 params");

    let competition = Competition::new_verbose(SMOKE_N_ENVS, smoke_budget, 0);

    let cem = cem_builder(SMOKE_EPISODE_STEPS);
    let rsa = richer_sa_builder(SMOKE_EPISODE_STEPS);
    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![&cem, &rsa];

    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);
    let task = rematch_task(SMOKE_EPISODE_STEPS, SMOKE_SUB_STEPS);

    eprintln!("\n--- D2c SR rematch (richer-SA) smoke ---");
    eprintln!("SMOKE shape: n_envs=4 episode_steps=50 sub_steps=10 budget=Steps(400)");
    eprintln!("This is a plumbing gate.  Classified outcome is NOT meaningful.");

    let outcome = run_rematch(&competition, &task, &algorithm_builders, &mut bootstrap_rng)
        .expect("smoke: run_rematch should complete without env errors");

    // Only assert the pipeline produced a value.  Ch 30's three
    // outcomes are all meaningful on a real run; in the smoke
    // they are plumbing signals, not scientific ones.
    eprintln!("\n--- smoke TwoMetricOutcome ---");
    eprintln!("best_reward:  {:?}", outcome.best_outcome);
    eprintln!("final_reward: {:?}", outcome.final_outcome);
}

// ─── Production test (Ch 53 §2.2 run) ──────────────────────────────────────

const PROD_N_ENVS: usize = 32;
const PROD_EPISODE_STEPS: usize = 5_000;
const PROD_SUB_STEPS: usize = 100;

/// Per-replicate compute budget — identical to Ch 52's basic-SA
/// run at `32 * 5000 * 100 = 16_000_000` env steps per replicate.
/// Ch 53 §2.2's compute-parity commitment.
const REMATCH_BUDGET_STEPS: usize = PROD_N_ENVS * PROD_EPISODE_STEPS * 100;

#[test]
#[ignore = "Ch 53 §2.2 production run; requires --release (~30-60 min)"]
fn d2c_sr_rematch_richer_sa() {
    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let rsa_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        rsa_policy.n_params(),
        "matched-complexity gate: CEM and richer-SA must use same n_params",
    );
    assert_eq!(
        cem_policy.n_params(),
        3,
        "D2c rematch uses LinearPolicy(2, 1) with n_params = 3 (Ch 23 §3.1)",
    );

    let competition =
        Competition::new_verbose(PROD_N_ENVS, TrainingBudget::Steps(REMATCH_BUDGET_STEPS), 0);

    let cem = cem_builder(PROD_EPISODE_STEPS);
    let rsa = richer_sa_builder(PROD_EPISODE_STEPS);
    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![&cem, &rsa];

    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);
    let task = rematch_task(PROD_EPISODE_STEPS, PROD_SUB_STEPS);

    eprintln!("\n--- D2c SR rematch: richer-proposal SA (Ch 53 §2.2) ---");
    eprintln!("Variant: Rechenberg 1/5 rule, adaptation_window=5, c=1.22, target=0.2");
    eprintln!("Master seed: {REMATCH_MASTER_SEED} (same as Ch 52's basic-SA run)");
    eprintln!("Bootstrap RNG seed: {BOOTSTRAP_RNG_SEED:#018x} (same)");
    eprintln!("Initial N: 10, expanded N (iff ambiguous): 20");
    eprintln!("Budget per replicate: Steps({REMATCH_BUDGET_STEPS}) = 16M env steps");
    eprintln!("n_envs per replicate: {PROD_N_ENVS}, episode_steps: {PROD_EPISODE_STEPS}");
    eprintln!("Matched complexity: LinearPolicy(2, 1), n_params = 3");
    eprintln!("Matched init: both policies start at [0.0, 0.0, 2.0]");
    eprintln!(
        "NOTE: Algorithm::name() is \"SA\" (not \"RicherSA\") — see richer_sa.rs \
         module doc; fixture filename identifies the variant.\n"
    );

    let outcome = run_rematch(&competition, &task, &algorithm_builders, &mut bootstrap_rng)
        .expect("rematch protocol should complete without env errors");

    eprintln!("\n--- Ch 53 §2.2 richer-SA outcome (dual metric, Ch 51 §2) ---");
    eprintln!("best_reward classification:  {:?}", outcome.best_outcome);
    eprintln!("final_reward classification: {:?}", outcome.final_outcome);
    eprintln!(
        "joint (best, final):         ({:?}, {:?})",
        outcome.best_outcome, outcome.final_outcome
    );
    eprintln!("Apply Ch 53 §3.2's three-case rule to this variant's final_reward");
    eprintln!("classification (joint with PT's) to pick Corroborate / Mixed / Contradict.");
}
