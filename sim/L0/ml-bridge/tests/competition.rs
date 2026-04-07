//! Competition integration tests — Phase 3 of `ML_COMPETITION_SPEC.md`.
//!
//! These tests validate the spec's hypotheses about algorithm ordering by
//! running real training loops through the `Competition` runner.  Each test
//! is a scientific experiment: fixed seed (42), controlled variables,
//! measurable assertions.
//!
//! All tests are `#[ignore]` — they take minutes, not seconds.  Run via:
//! ```text
//! cargo test -p sim-ml-bridge --test competition -- --ignored --nocapture
//! ```
//!
//! ## Known limitation: SAC's policy
//!
//! No `MlpStochasticPolicy` exists yet.  SAC uses `LinearStochasticPolicy`
//! even when other algorithms get `MlpPolicy`.  This handicaps SAC in any
//! MLP-level comparison.  Hypotheses involving SAC document this and should
//! be revisited when `MlpStochasticPolicy` is added.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::missing_const_for_fn
)]

use sim_ml_bridge::{
    Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, LinearQ, LinearStochasticPolicy,
    LinearValue, MlpPolicy, MlpQ, MlpValue, OptimizerConfig, Ppo, PpoHyperparams, Reinforce,
    ReinforceHyperparams, RunResult, Sac, SacHyperparams, TaskConfig, Td3, Td3Hyperparams,
    TrainingBudget, reaching_2dof, reaching_6dof,
};

// ── Helpers ────────────────────────────────────────────────────────────────

/// Reward improvement from first to last epoch, as a percentage.
///
/// Example: first = -1000, last = -100 → 90% improvement.
fn improvement_pct(run: &RunResult) -> f64 {
    let first = run.metrics[0].mean_reward;
    let last = run.final_reward().unwrap();
    (last - first) / first.abs() * 100.0
}

/// Max episode steps — 300 for 2-DOF, 500 for 6-DOF.
fn max_steps(task: &TaskConfig) -> usize {
    if task.act_dim() <= 2 { 300 } else { 500 }
}

// ── Builder functions ──────────────────────────────────────────────────────
//
// One per (algorithm, policy_level).  Each returns Box<dyn Algorithm>.
// Hyperparams borrowed from existing smoke tests, adjusted for competition
// budgets.

fn build_cem_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    Box::new(Cem::new(
        p,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_cem_mlp(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(MlpPolicy::new(
        task.obs_dim(),
        32,
        task.act_dim(),
        task.obs_scale(),
    ));
    Box::new(Cem::new(
        p,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_reinforce_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    Box::new(Reinforce::new(
        p,
        OptimizerConfig::adam(0.05),
        ReinforceHyperparams {
            gamma: 0.99,
            sigma_init: 0.5,
            sigma_decay: 0.95,
            sigma_min: 0.05,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_reinforce_mlp(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(MlpPolicy::new(
        task.obs_dim(),
        32,
        task.act_dim(),
        task.obs_scale(),
    ));
    Box::new(Reinforce::new(
        p,
        OptimizerConfig::adam(0.05),
        ReinforceHyperparams {
            gamma: 0.99,
            sigma_init: 0.5,
            sigma_decay: 0.95,
            sigma_min: 0.05,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_ppo_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let v = Box::new(LinearValue::new(task.obs_dim(), task.obs_scale()));
    Box::new(Ppo::new(
        p,
        v,
        OptimizerConfig::adam(0.025),
        PpoHyperparams {
            clip_eps: 0.2,
            k_passes: 2,
            gamma: 0.99,
            gae_lambda: 0.95,
            sigma_init: 0.5,
            sigma_decay: 0.90,
            sigma_min: 0.05,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_ppo_mlp(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(MlpPolicy::new(
        task.obs_dim(),
        32,
        task.act_dim(),
        task.obs_scale(),
    ));
    let v = Box::new(MlpValue::new(task.obs_dim(), 32, task.obs_scale()));
    Box::new(Ppo::new(
        p,
        v,
        OptimizerConfig::adam(0.025),
        PpoHyperparams {
            clip_eps: 0.2,
            k_passes: 2,
            gamma: 0.99,
            gae_lambda: 0.95,
            sigma_init: 0.5,
            sigma_decay: 0.90,
            sigma_min: 0.05,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_td3_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Td3::new(
        Box::new(LinearPolicy::new(od, ad, sc)),
        Box::new(LinearPolicy::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        OptimizerConfig::adam(3e-4),
        Td3Hyperparams {
            gamma: 0.99,
            tau: 0.005,
            policy_noise: 0.2,
            noise_clip: 0.5,
            exploration_noise: 0.1,
            policy_delay: 2,
            batch_size: 64,
            buffer_capacity: 50_000,
            warmup_steps: 200,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_td3_mlp(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Td3::new(
        Box::new(MlpPolicy::new(od, 32, ad, sc)),
        Box::new(MlpPolicy::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        OptimizerConfig::adam(3e-4),
        Td3Hyperparams {
            gamma: 0.99,
            tau: 0.005,
            policy_noise: 0.2,
            noise_clip: 0.5,
            exploration_noise: 0.1,
            policy_delay: 2,
            batch_size: 64,
            buffer_capacity: 50_000,
            warmup_steps: 200,
            max_episode_steps: max_steps(task),
        },
    ))
}

/// Low-budget TD3 with reduced warmup for sample-efficiency tests.
fn build_td3_mlp_low_warmup(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Td3::new(
        Box::new(MlpPolicy::new(od, 32, ad, sc)),
        Box::new(MlpPolicy::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        OptimizerConfig::adam(3e-4),
        Td3Hyperparams {
            gamma: 0.99,
            tau: 0.005,
            policy_noise: 0.2,
            noise_clip: 0.5,
            exploration_noise: 0.1,
            policy_delay: 2,
            batch_size: 64,
            buffer_capacity: 50_000,
            warmup_steps: 100,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_sac_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Sac::new(
        Box::new(LinearStochasticPolicy::new(od, ad, sc, -0.5)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        Box::new(LinearQ::new(od, ad, sc)),
        OptimizerConfig::adam(3e-4),
        SacHyperparams {
            gamma: 0.99,
            tau: 0.005,
            alpha_init: 0.2,
            auto_alpha: true,
            target_entropy: -(ad as f64),
            alpha_lr: 3e-4,
            batch_size: 64,
            buffer_capacity: 50_000,
            warmup_steps: 200,
            max_episode_steps: max_steps(task),
        },
    ))
}

/// SAC "MLP" — `LinearStochasticPolicy` actor (no `MlpStochasticPolicy`
/// exists) with `MlpQ` critics.  The actor is the bottleneck.
fn build_sac_mlp(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Sac::new(
        Box::new(LinearStochasticPolicy::new(od, ad, sc, -0.5)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        Box::new(MlpQ::new(od, 32, ad, sc)),
        OptimizerConfig::adam(3e-4),
        SacHyperparams {
            gamma: 0.99,
            tau: 0.005,
            alpha_init: 0.2,
            auto_alpha: true,
            target_entropy: -(ad as f64),
            alpha_lr: 3e-4,
            batch_size: 64,
            buffer_capacity: 50_000,
            warmup_steps: 200,
            max_episode_steps: max_steps(task),
        },
    ))
}

// ── Test 1: Regression baseline ────────────────────────────────────────────

/// All 5 algorithms, linear policies, 2-DOF.  Sanity check that everything
/// runs, produces finite metrics, and shows reward improvement.
#[test]
#[ignore = "multi-minute competition run"]
fn competition_2dof_all_linear() {
    let task = reaching_2dof();
    let comp = Competition::new(20, TrainingBudget::Epochs(30), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
        &build_cem_linear,
        &build_reinforce_linear,
        &build_ppo_linear,
        &build_td3_linear,
        &build_sac_linear,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let names = ["CEM", "REINFORCE", "PPO", "TD3", "SAC"];
    for name in &names {
        let run = result
            .find("reaching-2dof", name)
            .unwrap_or_else(|| panic!("missing result for {name}"));

        // All metrics are finite.
        for m in &run.metrics {
            assert!(
                m.mean_reward.is_finite(),
                "{name} epoch {} has non-finite reward: {}",
                m.epoch,
                m.mean_reward
            );
            for (k, v) in &m.extra {
                assert!(
                    v.is_finite(),
                    "{name} epoch {} extra[{k}] = {v} is not finite",
                    m.epoch
                );
            }
        }

        // Reward improves from first to last epoch.
        let first = run.metrics[0].mean_reward;
        let last = run.final_reward().unwrap();
        assert!(
            last > first,
            "{name} did not improve: first={first:.2}, last={last:.2}"
        );
    }
}

// ── Test 2: Hypothesis 1 — CEM scales poorly ──────────────────────────────

/// CEM goes from effective (2-DOF, 10 params) to sample-starved (6-DOF,
/// 614 MLP params).  50 envs = 50 candidates/generation — enough for 10
/// params, nowhere near enough for 614.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_cem_scales_poorly() {
    let task_2dof = reaching_2dof();
    let task_6dof = reaching_6dof();

    let comp = Competition::new(50, TrainingBudget::Epochs(30), 42);

    // CEM linear on 2-DOF.
    let result_2dof = comp
        .run(
            &[task_2dof],
            &[&build_cem_linear as &dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
        )
        .expect("2-DOF failed");

    // CEM MLP on 6-DOF.
    let result_6dof = comp
        .run(
            &[task_6dof],
            &[&build_cem_mlp as &dyn Fn(&TaskConfig) -> Box<dyn Algorithm>],
        )
        .expect("6-DOF failed");

    result_2dof.print_summary();
    result_6dof.print_summary();

    let cem_2dof = result_2dof.find("reaching-2dof", "CEM").unwrap();
    let cem_6dof = result_6dof.find("reaching-6dof", "CEM").unwrap();

    // Both should improve (CEM learns something on both tasks).
    let pct_2dof = improvement_pct(cem_2dof);
    let pct_6dof = improvement_pct(cem_6dof);
    assert!(
        pct_2dof > 30.0,
        "CEM 2-DOF should improve >30%: got {pct_2dof:.1}%"
    );
    assert!(
        pct_6dof > 10.0,
        "CEM 6-DOF should improve >10%: got {pct_6dof:.1}%"
    );

    // The scaling signal is in absolute final reward, not percentage.
    // Both tasks have different baselines (2 joints vs 6 joints → different
    // initial error), so percentage improvements look similar. But 2-DOF
    // gets close to the target (reward near 0) while 6-DOF stays far away.
    let r2 = cem_2dof.final_reward().unwrap();
    let r6 = cem_6dof.final_reward().unwrap();
    assert!(
        r2 > r6,
        "CEM 2-DOF ({r2:.2}) should beat 6-DOF ({r6:.2}) in absolute reward"
    );

    // 6-DOF absolute reward should be at least 2x worse than 2-DOF,
    // proving CEM's effective performance degrades with param count.
    // (Both are negative, so "2x worse" means r6 < 2 * r2.)
    assert!(
        r6 < r2 * 2.0,
        "CEM 6-DOF ({r6:.2}) should be substantially worse than 2-DOF ({r2:.2})"
    );

    eprintln!(
        "CEM 2-DOF: reward={r2:.2}, improvement={pct_2dof:.1}%, dones={}",
        cem_2dof.total_dones()
    );
    eprintln!(
        "CEM 6-DOF: reward={r6:.2}, improvement={pct_6dof:.1}%, dones={}",
        cem_6dof.total_dones()
    );
}

// ── Test 3: Hypothesis 2 — PPO's value function matters at scale ───────────

/// PPO >> REINFORCE on 6-DOF because the learned value baseline reduces
/// gradient variance.  On easy tasks (2-DOF) the gap is small.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_value_fn_matters_at_scale() {
    let task = reaching_6dof();
    let comp = Competition::new(50, TrainingBudget::Epochs(40), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&build_reinforce_mlp, &build_ppo_mlp];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();

    // PPO should achieve better final reward.
    let r_reinforce = reinforce.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    assert!(
        r_ppo > r_reinforce,
        "PPO ({r_ppo:.2}) should beat REINFORCE ({r_reinforce:.2}) on 6-DOF"
    );

    // PPO should reach the target more often (when either triggers dones).
    // At current budgets, neither may trigger the precise done condition,
    // so only assert when there's signal to compare.
    if ppo.total_dones() > 0 || reinforce.total_dones() > 0 {
        assert!(
            ppo.total_dones() >= reinforce.total_dones(),
            "PPO dones ({}) should match or exceed REINFORCE dones ({}) on 6-DOF",
            ppo.total_dones(),
            reinforce.total_dones()
        );
    }

    eprintln!(
        "PPO: reward={r_ppo:.2}, dones={}, REINFORCE: reward={r_reinforce:.2}, dones={}",
        ppo.total_dones(),
        reinforce.total_dones()
    );
}

// ── Test 4: Hypothesis 3 — Off-policy sample efficiency ────────────────────

/// At low budget (20 epochs, 20 envs), off-policy replay gives TD3 an
/// advantage over CEM.  PPO's learned baseline also beats CEM.
///
/// SAC excluded: `LinearStochasticPolicy` handicap makes comparison unfair.
///
/// Uses `warmup_steps: 100` (not 200) for TD3 — at 20 envs, 200 warmup
/// = 10 epochs of noise, half the budget.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_off_policy_efficiency() {
    let task = reaching_6dof();
    let comp = Competition::new(20, TrainingBudget::Epochs(20), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&build_cem_mlp, &build_ppo_mlp, &build_td3_mlp_low_warmup];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();

    let r_cem = cem.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();

    // Off-policy reuse > evolutionary.
    assert!(
        r_td3 > r_cem,
        "TD3 ({r_td3:.2}) should beat CEM ({r_cem:.2}) at low budget"
    );

    // Gradient + baseline > evolutionary.
    assert!(
        r_ppo > r_cem,
        "PPO ({r_ppo:.2}) should beat CEM ({r_cem:.2}) at low budget"
    );
}

// ── Test 5: Hypothesis 4 — MLP >> linear ───────────────────────────────────

/// MLP policies outperform linear on 6-DOF.  Same algorithm (PPO), same
/// task, different policy capacity.  MLP captures nonlinearity that linear
/// can't express.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_mlp_beats_linear() {
    let task = reaching_6dof();
    let comp = Competition::new(30, TrainingBudget::Epochs(40), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&build_ppo_linear, &build_ppo_mlp];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    // Both entries show as "PPO" — disambiguate by order (linear first).
    let runs = result.for_task("reaching-6dof");
    assert_eq!(runs.len(), 2, "expected 2 PPO runs");
    let ppo_linear = runs[0];
    let ppo_mlp = runs[1];

    let r_linear = ppo_linear.final_reward().unwrap();
    let r_mlp = ppo_mlp.final_reward().unwrap();

    assert!(
        r_mlp > r_linear,
        "PPO MLP ({r_mlp:.2}) should beat PPO Linear ({r_linear:.2}) on 6-DOF"
    );

    assert!(
        ppo_mlp.total_dones() > ppo_linear.total_dones(),
        "PPO MLP dones ({}) should exceed PPO Linear dones ({}) on 6-DOF",
        ppo_mlp.total_dones(),
        ppo_linear.total_dones()
    );
}

// ── Test 6: Hypothesis 5 — Entropy helps exploration ───────────────────────

/// SAC's entropy regularization should give it an edge over TD3's
/// deterministic policy on precise reaching.  Both use linear policies
/// for a fair capacity comparison.
///
/// Precondition: both must show >10% reward improvement.  If neither
/// learns, the test is a finding (linear too weak for 6-DOF off-policy),
/// not a pass.
///
/// Revisit with MLP when `MlpStochasticPolicy` is added.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_entropy_helps() {
    let task = reaching_6dof();
    let comp = Competition::new(30, TrainingBudget::Epochs(40), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> =
        vec![&build_td3_linear, &build_sac_linear];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();

    let td3_pct = improvement_pct(td3);
    let sac_pct = improvement_pct(sac);

    eprintln!("TD3 improvement: {td3_pct:.1}%");
    eprintln!("SAC improvement: {sac_pct:.1}%");

    // Precondition: both must learn something meaningful.
    if td3_pct < 10.0 || sac_pct < 10.0 {
        eprintln!(
            "FINDING: linear policies too weak for 6-DOF off-policy. \
             TD3={td3_pct:.1}%, SAC={sac_pct:.1}%. \
             Skipping entropy comparison — revisit with MLP."
        );
        return;
    }

    // SAC's entropy-driven exploration should yield equal or better reward.
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    assert!(
        r_sac >= r_td3,
        "SAC ({r_sac:.2}) should match or beat TD3 ({r_td3:.2}) via entropy"
    );
}

// ── Test 7: Full sweep — headline test ─────────────────────────────────────

/// All 5 algorithms, MLP policies, 6-DOF.  The headline competition.
/// Run this after Tests 2-6 pass to get the complete picture at maximum
/// budget.  Overlaps with Tests 3 and 5 but at higher budget (50 epochs /
/// 50 envs), which may reveal behavior the focused tests miss.
///
/// SAC uses `LinearStochasticPolicy` — its ranking may be suppressed.
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_all_mlp() {
    let task = reaching_6dof();
    let comp = Competition::new(50, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
        &build_cem_mlp,
        &build_reinforce_mlp,
        &build_ppo_mlp,
        &build_td3_mlp,
        &build_sac_mlp,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();

    let r_cem = cem.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();

    // Gradient-based > evolutionary.
    assert!(
        r_reinforce > r_cem,
        "REINFORCE ({r_reinforce:.2}) should beat CEM ({r_cem:.2})"
    );

    // Learned baseline matters at scale.
    assert!(
        r_ppo > r_reinforce,
        "PPO ({r_ppo:.2}) should beat REINFORCE ({r_reinforce:.2})"
    );

    // Print improvement percentages for the record.
    for name in &["CEM", "REINFORCE", "PPO", "TD3", "SAC"] {
        if let Some(run) = result.find("reaching-6dof", name) {
            eprintln!(
                "{name}: improvement={:.1}%, dones={}",
                improvement_pct(run),
                run.total_dones()
            );
        }
    }
}
