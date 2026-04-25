//! D2c SR rematch — CEM vs Parallel Tempering (Ch 53 §2.3).
//!
//! Ch 53's second robustness-check fixture.  PT runs K = 4 SA
//! chains at a geometric temperature ladder (0.5 → 50.0), swaps
//! adjacent chains every epoch under the replica-exchange
//! Metropolis criterion, and returns the lowest-temperature chain
//! (chain 0) as the policy artifact.  Total-compute parity holds
//! across all K chains: per-chain budget is
//! `16_000_000 / K = 4_000_000` env steps, so each chain does
//! `4_000_000 / (32 * 5000) = 25` Metropolis proposals at the
//! production shape.  This is 1/4 as many per-chain proposals as
//! basic SA gets under Ch 52, and that compute-cost is the
//! headline of PT's matched-compute pressure test.
//!
//! Like the richer-SA fixture, this file is a near-copy of
//! `tests/d2c_sr_rematch.rs` with only the SA builder replaced by
//! a PT builder.  CEM, the MJCF model, the reward shape,
//! `REMATCH_MASTER_SEED`, and `BOOTSTRAP_RNG_SEED` are bit-for-bit
//! identical so the only source of any `TwoMetricOutcome`
//! difference from Ch 52 is the PT replacement itself.
//!
//! **Two tests:**
//! 1. `d2c_sr_rematch_pt_smoke` — plumbing gate, non-ignored,
//!    runs in seconds at `n_envs = 4`, `episode_steps = 50`,
//!    `sub_steps = 10`.
//! 2. `d2c_sr_rematch_pt` — Ch 53 §2.3 production run at the
//!    full 16M-step-per-replicate budget, `#[ignore]`d for the
//!    second-MBP fire.
//!
//! ```text
//! cargo test -p sim-opt --release d2c_sr_rematch_pt_smoke
//! cargo test -p sim-opt --release --ignored d2c_sr_rematch_pt
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
use sim_core::test_fixtures::stochastic_resonance;
use sim_opt::{Pt, PtHyperparams, REMATCH_MASTER_SEED, REMATCH_TASK_NAME, run_rematch};
use sim_rl::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, ObservationSpace,
    Policy, TaskConfig, TrainingBudget, VecEnv,
};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, OscillatingField, PassiveStack};

// ─── Central parameters (duplicated) ───────────────────────────────────────

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

// ─── Parameterized task builder ────────────────────────────────────────────

fn make_training_vecenv(
    master_seed: u64,
    n_envs: usize,
    episode_steps: usize,
    sub_steps: usize,
) -> VecEnv {
    let mut model = stochastic_resonance();
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

fn rematch_task(episode_steps: usize, sub_steps: usize) -> TaskConfig {
    TaskConfig::from_build_fn(
        REMATCH_TASK_NAME,
        OBS_DIM,
        ACT_DIM,
        obs_scale(),
        move |ne, seed| Ok(make_training_vecenv(seed, ne, episode_steps, sub_steps)),
    )
}

// ─── Builders ──────────────────────────────────────────────────────────────

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

fn pt_builder(episode_steps: usize) -> impl Fn(&TaskConfig) -> Box<dyn Algorithm> + Send + Sync {
    // Ch 53 §2.3 defaults: K = 4, geometric ladder 0.5 → 50.0,
    // proposal_std = 0.5, swap every epoch.  Matched init
    // [0.0, 0.0, 2.0] on every chain (PT seeds every chain from
    // the same starting state).
    move |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Pt::new(
            Box::new(policy),
            PtHyperparams::ch53_defaults(episode_steps),
        ))
    }
}

// ─── Smoke test ────────────────────────────────────────────────────────────

#[test]
fn d2c_sr_rematch_pt_smoke() {
    // PT compute budget divides by K (see parallel_tempering.rs
    // `train` loop), so a 2-epoch-per-chain smoke needs
    // `K * n_envs * episode_steps * 2` env steps.  With K = 4,
    // n_envs = 4, episode_steps = 50 that is 1600 env steps per
    // replicate.  Same per-replicate cost as the richer-SA smoke
    // (400 env steps) scaled by K — still well under a second
    // under `--release` even at the 20-replicate expansion path.
    const SMOKE_N_ENVS: usize = 4;
    const SMOKE_EPISODE_STEPS: usize = 50;
    const SMOKE_SUB_STEPS: usize = 10;
    const SMOKE_K: usize = 4;
    let smoke_budget = TrainingBudget::Steps(SMOKE_K * SMOKE_N_ENVS * SMOKE_EPISODE_STEPS * 2);

    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let pt_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        pt_policy.n_params(),
        "matched-complexity"
    );
    assert_eq!(cem_policy.n_params(), 3, "LinearPolicy(2,1) => 3 params");

    let competition = Competition::new_verbose(SMOKE_N_ENVS, smoke_budget, 0);

    let cem = cem_builder(SMOKE_EPISODE_STEPS);
    let pt = pt_builder(SMOKE_EPISODE_STEPS);
    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![&cem, &pt];

    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);
    let task = rematch_task(SMOKE_EPISODE_STEPS, SMOKE_SUB_STEPS);

    eprintln!("\n--- D2c SR rematch (PT) smoke ---");
    eprintln!(
        "SMOKE shape: K=4 n_envs=4 episode_steps=50 sub_steps=10 \
         budget=Steps(1600)"
    );
    eprintln!("This is a plumbing gate.  Classified outcome is NOT meaningful.");

    let outcome = run_rematch(&competition, &task, &algorithm_builders, &mut bootstrap_rng)
        .expect("smoke: run_rematch should complete without env errors");

    eprintln!("\n--- smoke TwoMetricOutcome ---");
    eprintln!("best_reward:  {:?}", outcome.best_outcome);
    eprintln!("final_reward: {:?}", outcome.final_outcome);
}

// ─── Production test (Ch 53 §2.3 run) ──────────────────────────────────────

const PROD_N_ENVS: usize = 32;
const PROD_EPISODE_STEPS: usize = 5_000;
const PROD_SUB_STEPS: usize = 100;

/// Total per-replicate compute budget: `32 * 5000 * 100 =
/// 16_000_000` env steps — same 16M total as basic SA and CEM.
/// Under PT's total-compute-parity divisor, this becomes
/// `16_000_000 / K = 4_000_000` env steps per chain, which in
/// turn becomes `4_000_000 / (32 * 5000) = 25` Metropolis
/// proposals per chain.  Ch 53 §2.3 explicitly pre-commits this
/// shape.
const REMATCH_BUDGET_STEPS: usize = PROD_N_ENVS * PROD_EPISODE_STEPS * 100;

#[test]
#[ignore = "Ch 53 §2.3 production run; requires --release (~30-60 min)"]
fn d2c_sr_rematch_pt() {
    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let pt_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        pt_policy.n_params(),
        "matched-complexity gate: CEM and PT must use same n_params",
    );
    assert_eq!(
        cem_policy.n_params(),
        3,
        "D2c rematch uses LinearPolicy(2, 1) with n_params = 3 (Ch 23 §3.1)",
    );

    let competition =
        Competition::new_verbose(PROD_N_ENVS, TrainingBudget::Steps(REMATCH_BUDGET_STEPS), 0);

    let cem = cem_builder(PROD_EPISODE_STEPS);
    let pt = pt_builder(PROD_EPISODE_STEPS);
    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![&cem, &pt];

    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);
    let task = rematch_task(PROD_EPISODE_STEPS, PROD_SUB_STEPS);

    eprintln!("\n--- D2c SR rematch: Parallel Tempering (Ch 53 §2.3) ---");
    eprintln!("Variant: K=4 chains, ladder 0.5→50.0 geometric, swap every epoch");
    eprintln!("Per-chain budget: {REMATCH_BUDGET_STEPS}/4 = 4_000_000 env steps = 25 proposals");
    eprintln!("Master seed: {REMATCH_MASTER_SEED} (same as Ch 52's basic-SA run)");
    eprintln!("Bootstrap RNG seed: {BOOTSTRAP_RNG_SEED:#018x} (same)");
    eprintln!("Initial N: 10, expanded N (iff ambiguous): 20");
    eprintln!("Budget per replicate: Steps({REMATCH_BUDGET_STEPS}) = 16M env steps total");
    eprintln!("n_envs per replicate: {PROD_N_ENVS}, episode_steps: {PROD_EPISODE_STEPS}");
    eprintln!("Matched complexity: LinearPolicy(2, 1), n_params = 3");
    eprintln!("Matched init: both policies start at [0.0, 0.0, 2.0]");
    eprintln!(
        "NOTE: Algorithm::name() is \"SA\" (not \"PT\") — see parallel_tempering.rs \
         module doc; fixture filename identifies the variant.\n"
    );

    let outcome = run_rematch(&competition, &task, &algorithm_builders, &mut bootstrap_rng)
        .expect("rematch protocol should complete without env errors");

    eprintln!("\n--- Ch 53 §2.3 PT outcome (dual metric, Ch 51 §2) ---");
    eprintln!("best_reward classification:  {:?}", outcome.best_outcome);
    eprintln!("final_reward classification: {:?}", outcome.final_outcome);
    eprintln!(
        "joint (best, final):         ({:?}, {:?})",
        outcome.best_outcome, outcome.final_outcome
    );
    eprintln!(
        "Apply Ch 53 §3.2's three-case rule to PT's final_reward classification \
         (joint with richer-SA's)."
    );
}
