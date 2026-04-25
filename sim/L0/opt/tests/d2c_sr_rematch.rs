//! D2c SR rematch — CEM vs SA at matched complexity on the SR task.
//!
//! Implements Chapter 32's folded-pilot protocol via `sim-opt`'s
//! `run_rematch` driver.  See the ml-chassis-refactor study Chapter
//! 42 for the rendering argument and sub-decision table.
//!
//! **Requires `--release` (~30-60 min)** because each replicate runs
//! at `Steps(16M)` budget and the initial batch is 10 replicates per
//! algorithm.  An ambiguous initial outcome triggers a folded
//! expansion to 20 replicates per algorithm, doubling the wall
//! clock.  Run with:
//!
//! ```text
//! cargo test -p sim-opt --release --ignored d2c_sr_rematch
//! ```
//!
//! The test gate is "protocol-completes-cleanly" per Ch 42 §6
//! sub-decision (g) shape α: the test asserts only that
//! `run_rematch` returns `Ok(outcome)` and that the matched-
//! complexity gate holds; the verdict (Positive / Null / Ambiguous)
//! is reported via `eprintln!` regardless of which Ch 30 outcome
//! was produced.  All three outcomes are scientifically informative
//! per Ch 30, so asserting a specific one would beg the question.

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
use sim_opt::{REMATCH_MASTER_SEED, REMATCH_TASK_NAME, Sa, SaHyperparams, run_rematch};
use sim_rl::{
    ActionSpace, Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, ObservationSpace,
    Policy, TaskConfig, TrainingBudget, VecEnv,
};
use sim_thermostat::{DoubleWellPotential, LangevinThermostat, OscillatingField, PassiveStack};

// ─── Central parameters (duplicated from d2c_cem_training.rs:64-69) ────────

const DELTA_V: f64 = 3.0;
const X_0: f64 = 1.0;
const GAMMA: f64 = 10.0;
const K_B_T_BASE: f64 = 1.0;
const A_0: f64 = 0.3;
// Note: the rematch's SEED_BASE is `sim_opt::REMATCH_MASTER_SEED`,
// which equals `20_260_412` by design — Ch 32 Decision 4 matched the
// rematch's master seed to `d2c_cem_training.rs:69`'s `SEED_BASE`
// literal for three-way consistency between Ch 23 §1.2, Ch 32 §4.6,
// and the legacy D2c CEM test.  PR 3a's `analysis` module exports
// the constant, so the rematch fixture imports rather than re-declares.

const KRAMERS_RATE: f64 = 0.01214;

fn signal_omega() -> f64 {
    2.0 * std::f64::consts::PI * KRAMERS_RATE
}

// ─── Episode parameters (duplicated from d2c_cem_training.rs:79-80) ────────

const SUB_STEPS: usize = 100;
const EPISODE_STEPS: usize = 5_000;

// ─── Rematch-specific constants ────────────────────────────────────────────

const N_ENVS: usize = 32;
const OBS_DIM: usize = 2;
const ACT_DIM: usize = 1;

/// Per-replicate compute budget.  `n_envs * episode_steps * 100` =
/// `32 * 5000 * 100` = `16_000_000` env steps, which divides into
/// 100 epochs at the per-epoch step count CEM and SA both use.
/// The 16M figure matches the D2c CEM test's `N_EPOCHS = 100`
/// budget at the same `n_envs * EPISODE_STEPS` per-epoch shape.
const REMATCH_BUDGET_STEPS: usize = N_ENVS * EPISODE_STEPS * 100;

/// Bootstrap RNG seed.  Ch 32 §7 explicitly defers the bootstrap
/// RNG seed to Ch 42 with the note that "the rematch writeup
/// should name the bootstrap RNG seed for full reproducibility."
/// Ch 42 picks the literal `0xB007_0057_00AA_0055` ("BOOTSTRAP
/// AA55") as a memorable fixed value.  This RNG is independent of
/// the per-replicate training RNGs (which derive from
/// `REMATCH_MASTER_SEED` via `splitmix64`) so bootstrap resampling
/// cannot accidentally correlate with physics noise sequences.
const BOOTSTRAP_RNG_SEED: u64 = 0xB007_0057_00AA_0055;

fn obs_scale() -> Vec<f64> {
    vec![1.0 / X_0, 1.0]
}

// ─── make_training_vecenv (duplicated from d2c_cem_training.rs:93-127) ─────
//
// Two changes from the legacy D2c CEM helper:
// 1. Rename `seed` → `master_seed` to match Ch 40 PR 1b's
//    `LangevinThermostat::new(gamma, k_b_t, master_seed, traj_id)`
//    naming convention.
// 2. Parameterize `n_envs` so the helper is reusable at both the
//    `N_ENVS = 32` rematch shape and any smaller debug shape a
//    future caller might want.
//
// The Ch 40 PR 1b per-env factory pattern (BatchSim::new_per_env
// with per-env LangevinThermostat instances each carrying
// `traj_id = env_index`) is acknowledged in Ch 42 §6.4 but not yet
// wired through `VecEnv::builder` in the live source — so this
// rendering uses the single-thermostat `traj_id = 0` shape that
// d2c_cem_training.rs:97-99 uses, matching live source rather than
// the per-env spec rendering.  When the per-env path lands, both
// helpers (this one and the legacy D2c CEM one) update together.
fn make_training_vecenv(master_seed: u64, n_envs: usize) -> VecEnv {
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

    VecEnv::builder(model, n_envs)
        .observation_space(obs_space)
        .action_space(act_space)
        .reward(move |_m, data| data.qpos[0].signum() * (omega * data.time).cos())
        .done(|_m, _d| false)
        .truncated(|_m, data| data.time > (EPISODE_STEPS as f64) * (SUB_STEPS as f64) * 0.001)
        .sub_steps(SUB_STEPS)
        .build()
        .unwrap()
}

// ─── rematch_task ──────────────────────────────────────────────────────────
//
// Constructs the single `TaskConfig` the rematch uses.  Built via
// `TaskConfig::from_build_fn` (not `TaskConfig::builder()`) because
// the rematch's per-replicate physics noise sequence depends on
// the per-replicate seed threading from
// `Competition::run_replicates → task.build_vec_env(n_envs, seed)`
// into the `LangevinThermostat`'s `master_seed`.  The
// `TaskConfigBuilder::build()` path synthesizes a closure that
// drops the seed (`task.rs:253-258`'s `_seed: u64`), which would
// collapse the per-replicate variance and malformed the bootstrap
// CI.  `TaskConfig::from_build_fn` was added in PR 3b commit 1
// (`feat(sim-ml-bridge): add TaskConfig::from_build_fn ...`) for
// exactly this consumer.
fn rematch_task() -> TaskConfig {
    TaskConfig::from_build_fn(
        REMATCH_TASK_NAME,
        OBS_DIM,
        ACT_DIM,
        obs_scale(),
        |n_envs, seed| Ok(make_training_vecenv(seed, n_envs)),
    )
}

// ─── The rematch test ──────────────────────────────────────────────────────

#[test]
#[ignore = "requires --release (~30-60 min)"]
fn d2c_sr_rematch() {
    // ── Matched-complexity gate (Ch 23 §3, Ch 31 §4.3) ──
    //
    // The load-bearing review gate.  Both CEM and SA must train
    // policies of the same parametric complexity so the comparison
    // is a method comparison, not a complexity comparison.
    // `LinearPolicy(2, 1)` has `n_params = act_dim * (obs_dim + 1)
    // = 1 * 3 = 3` per `linear.rs:55-67`.
    let cem_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    let sa_policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
    assert_eq!(
        cem_policy.n_params(),
        sa_policy.n_params(),
        "matched-complexity gate: CEM and SA must use same n_params",
    );
    assert_eq!(
        cem_policy.n_params(),
        3,
        "D2c rematch uses LinearPolicy(2, 1) with n_params = 3 (Ch 23 §3.1)",
    );

    // ── Build the Competition runner ──
    //
    // The single-seed `seed` parameter (here `0`) is unused under
    // `run_rematch` because per-replicate seeds come from
    // `splitmix64(REMATCH_MASTER_SEED + i)` threaded through
    // `Competition::run_replicates`'s per-call seeds slice (see
    // `sim_opt::analysis::run_rematch_with_runner`).  The stored
    // seed only matters for `Competition::run()`, which the rematch
    // does not call.
    let competition =
        Competition::new_verbose(N_ENVS, TrainingBudget::Steps(REMATCH_BUDGET_STEPS), 0);

    // ── CEM builder ──
    //
    // Hyperparams inherited from `d2c_cem_training.rs:286-294` — the
    // D2c CEM baseline that the rematch's SA is being compared
    // against.  Non-zero policy initialization `[0.0, 0.0, 2.0]`
    // matches `d2c_cem_training.rs:285` so CEM starts at the same
    // bias the legacy D2c CEM run used.
    let cem_builder = |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Cem::new(
            Box::new(policy),
            CemHyperparams {
                elite_fraction: 0.2,
                noise_std: 2.5,
                noise_decay: 0.98,
                noise_min: 0.1,
                max_episode_steps: EPISODE_STEPS,
            },
        ))
    };

    // ── SA builder ──
    //
    // Hyperparams from Ch 42 §4.5's defaults — calibrated against
    // the D2c SR reward scale (~490 peak per-episode total) so
    // early epochs admit meaningful downhill proposals.  The
    // matched `[0.0, 0.0, 2.0]` policy initialization removes
    // initialization asymmetry from the comparison: the rematch
    // is matched-complexity AND matched-initial-conditions, so any
    // measured difference is attributable to algorithm dynamics
    // alone.
    let sa_builder = |_task: &TaskConfig| -> Box<dyn Algorithm> {
        let mut policy = LinearPolicy::new(OBS_DIM, ACT_DIM, &obs_scale());
        policy.set_params(&[0.0, 0.0, 2.0]);
        Box::new(Sa::new(
            Box::new(policy),
            SaHyperparams {
                initial_temperature: 50.0,
                proposal_std: 0.5,
                cooling_decay: 0.955,
                temperature_min: 0.005,
                max_episode_steps: EPISODE_STEPS,
            },
        ))
    };

    let algorithm_builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&cem_builder, &sa_builder];

    // ── Bootstrap RNG ──
    //
    // Independent of the per-replicate training RNGs.  Seeded from
    // the module-level `BOOTSTRAP_RNG_SEED` constant.
    let mut bootstrap_rng = StdRng::seed_from_u64(BOOTSTRAP_RNG_SEED);

    // ── Build the single TaskConfig for the rematch ──
    let task = rematch_task();

    // ── Run the rematch ──
    eprintln!("\n--- D2c SR rematch: folded-pilot protocol ---");
    eprintln!("Master seed: {REMATCH_MASTER_SEED} (matches d2c_cem_training.rs:69 SEED_BASE)");
    eprintln!("Bootstrap RNG seed: {BOOTSTRAP_RNG_SEED:#018x}");
    eprintln!("Initial N: 10, expanded N (iff ambiguous): 20");
    eprintln!("Budget per replicate: Steps({REMATCH_BUDGET_STEPS}) = 16M env steps");
    eprintln!("n_envs per replicate: {N_ENVS}, episode_steps: {EPISODE_STEPS}");
    eprintln!("Matched complexity: LinearPolicy(2, 1), n_params = 3");
    eprintln!("Matched init: both policies start at [0.0, 0.0, 2.0]\n");

    let outcome = run_rematch(&competition, &task, &algorithm_builders, &mut bootstrap_rng)
        .expect("rematch protocol should complete without env errors");

    // ── Test gate: protocol completes cleanly (Ch 42 §6 sub-decision g) ──
    //
    // The test does NOT assert a specific outcome (Positive / Null
    // / Ambiguous) on either metric.  Ch 30 names all three as
    // scientifically informative, and asserting one would bake in
    // a prior about which outcome is expected — which contradicts
    // the rematch's purpose as the experiment that reveals the
    // answer.  Under the Ch 51 dual-metric amendment the test also
    // does not assert the joint `(best, final)` pair; Ch 51 §2.3
    // commits the nine-cell interpretation framework to the
    // chapter, applied by a human reader, not to the fixture.  The
    // gate shape is still Ch 42 §6 sub-decision (g) α:
    // protocol-completes-cleanly on a non-error `TwoMetricOutcome`.
    eprintln!("\n--- Rematch outcome (dual metric, Ch 51 §2) ---");
    eprintln!("best_reward classification:  {:?}", outcome.best_outcome);
    eprintln!("final_reward classification: {:?}", outcome.final_outcome);
    eprintln!(
        "joint (best, final):         ({:?}, {:?})",
        outcome.best_outcome, outcome.final_outcome
    );
    eprintln!("Apply Ch 51 §2.3's nine-cell framework to read the verdict.  final_reward is the");
    eprintln!("primary operationalization of Ch 30's \"resolves the peak\" question.");
}
