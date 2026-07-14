//! Competition integration tests — Phases 3 + 6 of `ML_COMPETITION_SPEC.md`.
//!
//! These tests validate the spec's hypotheses about algorithm ordering by
//! running real training loops through the `Competition` runner.  Each test
//! is a scientific experiment: fixed seed (42), controlled variables,
//! measurable assertions.
//!
//! All tests are `#[ignore]` — they take minutes, not seconds.  Run via:
//! ```text
//! cargo test -p sim-rl --test competition -- --ignored --nocapture
//! ```
//!
//! 6-DOF baselines re-calibrated on the #591 stable arm (the fixture had
//! been numerically diverging under RK4; joint armature stabilized it). The
//! two headline competitions (`competition_6dof_all_mlp` and
//! `_autograd_1layer_parity`) are now **multi-seed** (5 seeds via
//! `run_replicates`, cheap since the runner parallelizes across runs): they
//! report each algorithm's best-reward mean ± std and assert only the
//! seed-robust invariant — off-policy (TD3, SAC) ≫ on-policy (PPO, REINFORCE).
//! The old single-seed `r_ppo > r_reinforce` asserts were removed in favor of
//! a two-sided verdict rendered from the data: on **final-epoch reward** at 40
//! epochs across 5 seeds, the value baseline **robustly helps** — PPO beats
//! REINFORCE by ~1.7× the pooled spread (see `hypothesis_value_fn_matters_at_
//! scale`). (On best-of-epochs the noisier REINFORCE draws ahead, which is why
//! that test uses final reward — variance reduction pays off at convergence,
//! not in lucky peaks.) Re-baseline finding: **"CEM scales poorly on 6-DOF"
//! was a divergence artifact** — on the stable arm CEM makes real progress at
//! 50 epochs (dones > 0), landing between off- and on-policy rather than
//! collapsing. `hypothesis_cem_scales_poorly` still passes at seed 42 (6-DOF's
//! larger joint-space error keeps it > 2× worse than 2-DOF), consistent with
//! task scale rather than CEM failure — though that specific test remains
//! single-seed and is not a swept claim.
//!
//! ## Level 0-1 (Tests 1-7): Hand-coded gradients
//!
//! SAC uses `LinearStochasticPolicy` (no `MlpStochasticPolicy` exists).
//! This handicaps SAC in any MLP-level comparison.
//!
//! ## Level 2 (Tests 8-12): Autograd backends
//!
//! `AutogradStochasticPolicy` resolves SAC's handicap — SAC gets a real
//! MLP actor for the first time.
//! - Test 8: 1-hidden-layer parity (same arch as level 0-1 → same ordering)
//! - Test 9: 2-hidden-layer headline test (gradient methods should overtake CEM)
//! - Tests 10-12: Phase 6b budget scaling (more epochs, lower LR, more envs)

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::cast_precision_loss,
    clippy::missing_const_for_fn
)]

use rand::SeedableRng;
use rand::rngs::StdRng;
use sim_ml_chassis::{
    Activation, Algorithm, AutogradPolicy, AutogradQ, AutogradStochasticPolicy, AutogradValue,
    Competition, CompetitionResult, LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue,
    MlpPolicy, MlpQ, MlpValue, OptimizerConfig, RunResult, SeedSummary, TaskConfig, TrainingBudget,
};
use sim_rl::{
    Cem, CemHyperparams, Ppo, PpoHyperparams, Reinforce, ReinforceHyperparams, Sac, SacHyperparams,
    Td3, Td3Hyperparams, obstacle_reaching_6dof, reaching_2dof, reaching_6dof,
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

/// Check if any gradient method overtook CEM and print the finding.
///
/// `gradient_results` is a slice of `(name, final_reward)` for all
/// gradient-based algorithms in the test.
fn print_reversal_check(r_cem: f64, gradient_results: &[(&str, f64)]) {
    let (best_name, best_reward) = gradient_results
        .iter()
        .copied()
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .expect("at least one gradient method");
    if best_reward > r_cem {
        eprintln!(
            "\n*** ORDERING REVERSED: {best_name} ({best_reward:.2}) overtakes CEM ({r_cem:.2}) ***"
        );
    } else {
        eprintln!(
            "\n*** CEM ({r_cem:.2}) still dominates — gradient methods' best: {best_reward:.2} ***"
        );
    }
}

/// Print the per-algorithm best-reward distribution (mean ± std across seeds)
/// for a 5-algorithm 6-DOF competition, then assert the ONE seed-robust
/// invariant: off-policy replay (TD3, SAC) beats on-policy (PPO, REINFORCE) by
/// more than the pooled per-seed spread.
///
/// Single-seed orderings on this task are noisy — the on-policy pair (PPO vs
/// REINFORCE) and CEM's exact placement swing with seed. Rather than gate on
/// those brittle orderings (the pre-#591 tests did, calibrated on a
/// numerically-diverging arm), this records the full distribution as the
/// artifact and asserts only the off-policy≫on-policy separation, which is
/// large and stable across seeds. `best_reward` is used deliberately as a
/// *conservative* metric for this invariant: `best ≥ final`, so it flatters
/// the on-policy methods (whose best can be an early, near-initial peak) —
/// off-policy dominating on `best` therefore typically dominates by even more
/// on `final` (the off-policy methods converge monotonically, so best ≈ final,
/// while the on-policy `final` degrades below `best`). The distribution table
/// (`eprintln`) is the load-bearing science output.
fn assert_off_policy_dominates(result: &CompetitionResult, task: &str, n_seeds: usize) {
    let summ = |algo: &str| {
        result
            .describe(task, algo)
            .unwrap_or_else(|| panic!("{task}: no seed summary for {algo}"))
    };
    let algos = [
        ("CEM", summ("CEM")),
        ("SAC", summ("SAC")),
        ("TD3", summ("TD3")),
        ("PPO", summ("PPO")),
        ("REINFORCE", summ("REINFORCE")),
    ];

    eprintln!("\n=== {task}: {n_seeds}-seed best-reward mean ± std ===");
    for (name, s) in &algos {
        eprintln!(
            "  {name:<10} {:>13.2} ± {:>11.2}  (n={})",
            s.mean, s.std_dev, s.n
        );
    }

    let get = |name: &str| {
        algos
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, s)| *s)
            .unwrap()
    };
    let (td3, sac, ppo, reinforce) = (get("TD3"), get("SAC"), get("PPO"), get("REINFORCE"));
    for (off_name, off) in [("TD3", td3), ("SAC", sac)] {
        for (on_name, on) in [("PPO", ppo), ("REINFORCE", reinforce)] {
            // Pooled spread = sum of the two per-seed std devs. A gap wider
            // than that is well outside seed noise for n=5.
            let margin = off.std_dev + on.std_dev;
            assert!(
                off.mean - on.mean > margin,
                "{task}: off-policy {off_name} ({:.1} ± {:.1}) must beat on-policy \
                 {on_name} ({:.1} ± {:.1}) by more than the pooled std ({margin:.1}) — \
                 this is the seed-robust invariant",
                off.mean,
                off.std_dev,
                on.mean,
                on.std_dev
            );
        }
    }
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

// ── Level 2 builder functions (autograd backends) ─────────────────────────
//
// 1-layer: same architecture as level 0-1 (1 hidden, 32 units, tanh, zero-init).
// Purpose: verify autograd produces same competition ordering.
//
// 2-layer: deeper nets (2 hidden, 64+64, relu, Xavier/He init).
// Purpose: test whether gradient methods overtake CEM.

fn build_cem_autograd_1layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(AutogradPolicy::new(
        task.obs_dim(),
        &[32],
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

fn build_reinforce_autograd_1layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(AutogradPolicy::new(
        task.obs_dim(),
        &[32],
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

fn build_ppo_autograd_1layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let p = Box::new(AutogradPolicy::new(
        task.obs_dim(),
        &[32],
        task.act_dim(),
        task.obs_scale(),
    ));
    let v = Box::new(AutogradValue::new(task.obs_dim(), &[32], task.obs_scale()));
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

fn build_td3_autograd_1layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Td3::new(
        Box::new(AutogradPolicy::new(od, &[32], ad, sc)),
        Box::new(AutogradPolicy::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
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

/// SAC autograd 1-layer — first time SAC gets an MLP actor.
/// `AutogradStochasticPolicy` resolves the `LinearStochasticPolicy` handicap
/// from level 0-1.  NOT exact parity (architecture upgrade), but comparable
/// capacity (1 hidden layer, 32 units).
fn build_sac_autograd_1layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    Box::new(Sac::new(
        Box::new(AutogradStochasticPolicy::new(od, &[32], ad, sc, -0.5)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
        Box::new(AutogradQ::new(od, &[32], ad, sc)),
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

// ── 2-layer autograd builders (ReLU + Xavier/He init) ─────────────────────

fn build_cem_autograd_2layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let mut rng = StdRng::seed_from_u64(0);
    let p = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
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

fn build_reinforce_autograd_2layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let mut rng = StdRng::seed_from_u64(0);
    let p = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
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

fn build_ppo_autograd_2layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let mut rng = StdRng::seed_from_u64(0);
    let p = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));
    let v = Box::new(AutogradValue::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));
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

fn build_td3_autograd_2layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    let mut rng = StdRng::seed_from_u64(0);
    Box::new(Td3::new(
        Box::new(AutogradPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
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

fn build_sac_autograd_2layer(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    let mut rng = StdRng::seed_from_u64(0);
    Box::new(Sac::new(
        Box::new(AutogradStochasticPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            -0.5,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
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

// ── 2-layer autograd builders with lower learning rates (Phase 6b-2) ────────

fn build_reinforce_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm> {
    let mut rng = StdRng::seed_from_u64(0);
    let p = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));
    Box::new(Reinforce::new(
        p,
        OptimizerConfig::adam(0.01),
        ReinforceHyperparams {
            gamma: 0.99,
            sigma_init: 0.5,
            sigma_decay: 0.95,
            sigma_min: 0.05,
            max_episode_steps: max_steps(task),
        },
    ))
}

fn build_ppo_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm> {
    let mut rng = StdRng::seed_from_u64(0);
    let p = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));
    let v = Box::new(AutogradValue::new_xavier(
        task.obs_dim(),
        &[64, 64],
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));
    Box::new(Ppo::new(
        p,
        v,
        OptimizerConfig::adam(0.005),
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

fn build_td3_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    let mut rng = StdRng::seed_from_u64(0);
    Box::new(Td3::new(
        Box::new(AutogradPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        OptimizerConfig::adam(1e-4),
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

fn build_sac_autograd_2layer_low_lr(task: &TaskConfig) -> Box<dyn Algorithm> {
    let od = task.obs_dim();
    let ad = task.act_dim();
    let sc = task.obs_scale();
    let mut rng = StdRng::seed_from_u64(0);
    Box::new(Sac::new(
        Box::new(AutogradStochasticPolicy::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            -0.5,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        Box::new(AutogradQ::new_xavier(
            od,
            &[64, 64],
            ad,
            sc,
            Activation::Relu,
            &mut rng,
        )),
        OptimizerConfig::adam(1e-4),
        SacHyperparams {
            gamma: 0.99,
            tau: 0.005,
            alpha_init: 0.2,
            auto_alpha: true,
            target_entropy: -(ad as f64),
            alpha_lr: 1e-4,
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

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
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

        // On-policy methods (CEM, REINFORCE, PPO) should improve from
        // epoch 0.  Off-policy methods (TD3, SAC) may appear to "worsen"
        // because warmup gives artificially neutral first-epoch rewards.
        // For those, just verify the final reward is reasonable (> -100).
        let first = run.metrics[0].mean_reward;
        let last = run.final_reward().unwrap();
        if *name == "TD3" || *name == "SAC" {
            assert!(last > -100.0, "{name} final reward too poor: {last:.2}");
        } else {
            assert!(
                last > first,
                "{name} did not improve: first={first:.2}, last={last:.2}"
            );
        }
    }

    // Print results for the record.
    for name in &names {
        let run = result.find("reaching-2dof", name).unwrap();
        let pct = if run.metrics[0].mean_reward.abs() > 1e-6 {
            format!("{:.1}%", improvement_pct(run))
        } else {
            "N/A (warmup)".to_string()
        };
        eprintln!(
            "{name}: reward={:.2}, dones={}, improvement={pct}",
            run.final_reward().unwrap(),
            run.total_dones()
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
            &[&build_cem_linear as &(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)],
        )
        .expect("2-DOF failed");

    // CEM MLP on 6-DOF.
    let result_6dof = comp
        .run(
            &[task_6dof],
            &[&build_cem_mlp as &(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)],
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

/// Does PPO's learned value baseline beat plain REINFORCE on 6-DOF? —
/// **multi-seed**, re-baselined on the #591 stable arm.
///
/// The classic claim is that the value baseline reduces gradient variance so
/// PPO ends up ahead of REINFORCE. This uses **final-epoch reward** (not
/// best-of-epochs): the baseline's payoff is end-of-training performance, and
/// best-of-epochs would reward the *noisier* method's lucky peaks — the
/// opposite of what a variance-reduction claim is about.
///
/// The pre-#591 single-seed `r_ppo > r_reinforce` assert (calibrated on the
/// diverging arm) is replaced by a two-sided verdict rendered from the 5-seed
/// distributions: the baseline "helps" / "hurts" only if the mean gap clears
/// the pooled per-seed spread, else "within seed noise". The only hard
/// assertion is finiteness; the printed comparison is the artifact.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_value_fn_matters_at_scale() {
    let task = reaching_6dof();
    let comp = Competition::new(50, TrainingBudget::Epochs(40), 42);
    let seeds = [42u64, 7, 13, 100, 2024];

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> =
        vec![&build_reinforce_mlp, &build_ppo_mlp];

    let result = comp
        .run_replicates(&[task], &builders, &seeds)
        .expect("competition failed");

    for run in &result.runs {
        run.assert_finite();
    }

    // Final-epoch reward distribution (end-of-training performance).
    let summ = |algo: &str| {
        SeedSummary::from_rewards(&result.replicate_final_rewards("reaching-6dof", algo))
            .unwrap_or_else(|| panic!("no final-reward summary for {algo}"))
    };
    let ppo = summ("PPO");
    let reinforce = summ("REINFORCE");

    eprintln!(
        "\n=== value-baseline hypothesis: PPO vs REINFORCE (40ep, {}-seed, final reward) ===",
        seeds.len()
    );
    eprintln!(
        "  PPO        {:>12.2} ± {:>10.2}  (n={})",
        ppo.mean, ppo.std_dev, ppo.n
    );
    eprintln!(
        "  REINFORCE  {:>12.2} ± {:>10.2}  (n={})",
        reinforce.mean, reinforce.std_dev, reinforce.n
    );

    // Two-sided verdict from the data: a mean gap wider than the pooled
    // per-seed spread in either direction is a robust result; otherwise the
    // two are indistinguishable at this budget.
    let margin = ppo.std_dev + reinforce.std_dev;
    if ppo.mean - reinforce.mean > margin {
        eprintln!(
            "Finding: value baseline HELPS robustly — PPO leads REINFORCE by \
             {:.1} > pooled std {margin:.1}",
            ppo.mean - reinforce.mean
        );
    } else if reinforce.mean - ppo.mean > margin {
        eprintln!(
            "Finding: value baseline HURTS — REINFORCE robustly beats PPO by \
             {:.1} > pooled std {margin:.1} (hypothesis refuted at this budget)",
            reinforce.mean - ppo.mean
        );
    } else {
        eprintln!(
            "Finding: PPO and REINFORCE are indistinguishable — |gap| {:.1} <= \
             pooled std {margin:.1} (value baseline neither helps nor hurts here)",
            (ppo.mean - reinforce.mean).abs()
        );
    }
}

// ── Test 4: Hypothesis 3 — Off-policy sample efficiency ────────────────────

/// Off-policy replay (TD3) vs on-policy (PPO) at low budget.
///
/// At low budgets, off-policy methods replay each transition ~100x from
/// the buffer, while on-policy methods use each transition once.  TD3
/// should extract more learning from the same number of env steps.
///
/// Finding from initial run: CEM actually beats gradient methods at very
/// low budgets (20ep/20env) because it has no warmup overhead and doesn't
/// need gradient estimates.  CEM is included for context but not asserted
/// against — the sample-efficiency hypothesis is about off-policy vs
/// on-policy, not evolutionary vs gradient.
///
/// SAC excluded: `LinearStochasticPolicy` handicap makes comparison unfair.
///
/// Uses `warmup_steps: 100` (not 200) for TD3.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_off_policy_efficiency() {
    let task = reaching_6dof();
    let comp = Competition::new(20, TrainingBudget::Epochs(20), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> =
        vec![&build_cem_mlp, &build_ppo_mlp, &build_td3_mlp_low_warmup];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();

    let r_cem = cem.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();

    // Off-policy replay > on-policy at low budget.
    assert!(
        r_td3 > r_ppo,
        "TD3 ({r_td3:.2}) should beat PPO ({r_ppo:.2}) at low budget via replay"
    );

    eprintln!("CEM={r_cem:.2}, TD3={r_td3:.2}, PPO={r_ppo:.2}");
    eprintln!(
        "Finding: CEM ({r_cem:.2}) competitive at very low budget — \
         no warmup, no gradient noise"
    );
}

// ── Test 5: Hypothesis 4 — MLP >> linear ───────────────────────────────────

/// Linear vs MLP policy capacity on 6-DOF.  Same algorithm (PPO), same
/// task, different policy.
///
/// Finding: on a quadratic joint-space reward -(qpos-target)^2, the
/// optimal controller is approximately linear (PD control).  Linear (78
/// params) converges faster than MLP (614 params) at level 0-1 budgets
/// because it has fewer parameters per gradient update.  MLP's extra
/// capacity is wasted overhead when the reward landscape doesn't require
/// nonlinear function approximation.
///
/// This hypothesis needs a task with a genuinely nonlinear reward
/// landscape (e.g., obstacle avoidance, contact manipulation) to test
/// properly.  For now, verify both learn and document whichever wins.
#[test]
#[ignore = "multi-minute competition run"]
fn hypothesis_mlp_beats_linear() {
    let task = reaching_6dof();
    let comp = Competition::new(30, TrainingBudget::Epochs(40), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> =
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

    // Both should improve meaningfully.
    let pct_linear = improvement_pct(ppo_linear);
    let pct_mlp = improvement_pct(ppo_mlp);
    assert!(
        pct_linear > 10.0,
        "PPO Linear should improve >10%: got {pct_linear:.1}%"
    );
    assert!(
        pct_mlp > 10.0,
        "PPO MLP should improve >10%: got {pct_mlp:.1}%"
    );

    eprintln!("PPO Linear: reward={r_linear:.2}, improvement={pct_linear:.1}%");
    eprintln!("PPO MLP:    reward={r_mlp:.2}, improvement={pct_mlp:.1}%");
    if r_linear > r_mlp {
        eprintln!("Finding: Linear beats MLP — quadratic reward is well-served by linear policy");
    } else {
        eprintln!("Finding: MLP beats Linear — nonlinear capacity helps");
    }
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

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> =
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
    // Post-Ch 41 PR 2b (Decision 1 unit correction): under the corrected
    // per-episode-total metric, SAC (-28.84) is narrowly below TD3 (-26.05)
    // at 6-DOF with LinearStochasticPolicy — a ~2.8 unit gap on a ~-30 scale
    // that is within measurement noise at one seed. The entropy-helps
    // hypothesis is inconclusive rather than refuted at this budget. A
    // definitive test requires MLP-level SAC (blocked on MlpStochasticPolicy)
    // and/or multi-seed replication via Competition::run_replicates.
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    eprintln!(
        "Finding: SAC ({r_sac:.2}) vs TD3 ({r_td3:.2}) — entropy hypothesis \
         {}",
        if r_sac >= r_td3 {
            "supported at this seed"
        } else {
            "not supported at this seed (narrow margin)"
        }
    );
}

// ── Test 7: Full sweep — headline test ─────────────────────────────────────

/// All 5 algorithms, MLP policies, 6-DOF.  The headline competition —
/// **multi-seed**, re-baselined on the #591 stable arm.
///
/// Runs `run_replicates` over 5 seeds and reports each algorithm's best-reward
/// mean ± std. The seed-robust finding on the corrected (non-diverging) arm:
///
///   off-policy (TD3, SAC) ≫ CEM ≫ on-policy (PPO, REINFORCE)
///
/// TD3/SAC converge near 0; CEM makes real progress at 50 epochs (dones > 0,
/// mean best-reward well ahead of the on-policy methods), which **refutes the
/// old "CEM scales poorly on 6-DOF" claim** — that was an artifact of the
/// numerically-diverging pre-#591 fixture, not a CEM failure. CEM does not
/// fully solve the task (best ~-200 vs TD3's ~-5), but it clearly learns.
/// PPO and REINFORCE trail well behind with high per-seed variance (their
/// final-epoch rewards degrade further than their best, so best ≫ final).
///
/// The only hard assertion is off-policy ≫ on-policy (see
/// [`assert_off_policy_dominates`]). The pre-#591 single-seed asserts
/// (`r_td3 > r_ppo`, `r_ppo > r_reinforce`) are gone: the `PPO > REINFORCE`
/// half is *metric-dependent* — REINFORCE's lucky peaks put it ahead on the
/// best-of-epochs reward this table reports, while PPO wins on final-epoch
/// reward (the variance-reduction metric; see `hypothesis_value_fn_matters_
/// at_scale`). Not a stable single-seed ordering to gate on. See Ch 41 §4.3 /
/// Ch 24 §1.8 for the earlier unit-mismatch history.
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_all_mlp() {
    let task = reaching_6dof();
    let comp = Competition::new(50, TrainingBudget::Epochs(50), 42);
    let seeds = [42u64, 7, 13, 100, 2024];

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_mlp,
        &build_reinforce_mlp,
        &build_ppo_mlp,
        &build_td3_mlp,
        &build_sac_mlp,
    ];

    let result = comp
        .run_replicates(&[task], &builders, &seeds)
        .expect("competition failed");

    // Every replicate's every epoch must be finite.
    for run in &result.runs {
        run.assert_finite();
    }

    assert_off_policy_dominates(&result, "reaching-6dof", seeds.len());
}

// ═══════════════════════════════════════════════════════════════════════════
// Level 2: Autograd competition re-run (Phase 6)
// ═══════════════════════════════════════════════════════════════════════════

// ── Test 8: Autograd 1-layer parity ──────────────────────────────────────

/// All 5 algorithms, autograd backends, 1 hidden layer (32 units), 6-DOF —
/// **multi-seed**, re-baselined on the #591 stable arm.
///
/// The point of this test is *parity*: tape-based reverse-mode autograd must
/// produce the same competition **structure** as hand-coded gradients. It
/// runs the same 5-seed `run_replicates` and asserts the same seed-robust
/// invariant — off-policy (TD3, SAC) ≫ on-policy (PPO, REINFORCE) — via
/// [`assert_off_policy_dominates`], confirming the autograd tape doesn't
/// regress the ordering.
///
/// **Reduced budget (20 epochs, vs Test 7's 50):** autograd's tape backward
/// is ~10× the cost of the hand-coded path, and the off-policy≫on-policy
/// separation is already stark by 20 epochs (cf. `hypothesis_off_policy_
/// efficiency`), so the full-budget ordering — established by the hand-coded
/// `competition_6dof_all_mlp` — need not be re-run here at 50 epochs. This
/// cuts the autograd regression guard from ~40 min to ~24 min.
/// (Because the budget differs, the absolute numbers here are *not* meant to
/// match Test 7's; the shared invariant is the parity check.)
///
/// The pre-#591 single-seed asserts (`r_td3 > r_ppo`, `r_ppo > r_reinforce`)
/// are gone for the same reason as Test 7: the `PPO > REINFORCE` half is
/// metric-dependent (final vs best-of-epochs), not a stable ordering to gate
/// on (see Test 7's doc comment).
///
/// Two differences from level 0-1 hand-coded:
/// 1. Gradients computed via tape-based reverse-mode AD, not hand-coded
/// 2. SAC gets `AutogradStochasticPolicy` (MLP actor) instead of
///    `LinearStochasticPolicy` — capacity upgrade, may improve SAC's rank
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_autograd_1layer_parity() {
    let task = reaching_6dof();
    let comp = Competition::new(50, TrainingBudget::Epochs(20), 42);
    let seeds = [42u64, 7, 13, 100, 2024];

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_1layer,
        &build_reinforce_autograd_1layer,
        &build_ppo_autograd_1layer,
        &build_td3_autograd_1layer,
        &build_sac_autograd_1layer,
    ];

    let result = comp
        .run_replicates(&[task], &builders, &seeds)
        .expect("competition failed");

    // Every replicate's every epoch must be finite.
    for run in &result.runs {
        run.assert_finite();
    }

    assert_off_policy_dominates(&result, "reaching-6dof", seeds.len());
}

// ── Test 9: Autograd 2-layer — the headline test ─────────────────────────

/// All 5 algorithms, autograd backends, 2 hidden layers (64+64), `ReLU`,
/// Xavier/He init, 6-DOF.
///
/// **The hypothesis**: deeper networks reverse the level 0-1 ordering.
/// CEM's 50 candidates are insufficient for ~5K+ parameters. Gradient
/// methods exploit local curvature via autograd.  Predicted new ordering:
///
///   CEM << REINFORCE < PPO < TD3 <= SAC
///
/// This test uses minimal assertions (all finite, all improve) and
/// documents whatever ordering emerges.  The results are the science —
/// a "failed" hypothesis is a finding, not a bug.
///
/// Parameter counts (6-DOF reaching: obs=12, act=6):
/// - `AutogradPolicy` [64,64]: 12*64 + 64 + 64*64 + 64 + 64*6 + 6 = 5,382
/// - `AutogradQ` [64,64]:      18*64 + 64 + 64*64 + 64 + 64*1 + 1 = 5,441
/// - CEM with 50 candidates exploring ~5K dims: ~108 params/candidate
///   (vs 12 params/candidate at level 0-1)
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_autograd_2layer() {
    let task = reaching_6dof();
    let comp = Competition::new_verbose(50, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_2layer,
        &build_reinforce_autograd_2layer,
        &build_ppo_autograd_2layer,
        &build_td3_autograd_2layer,
        &build_sac_autograd_2layer,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();

    // All metrics must be finite.
    for run in [cem, td3, sac, ppo, reinforce] {
        run.assert_finite();
    }

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();

    result.print_ranked(
        "reaching-6dof",
        "Level 2 headline results (2-layer autograd, ReLU, Xavier)",
    );
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Compare level 0-1 vs level 2 for context.
    // NOTE: these baseline numbers are pre-Ch-41-PR-2b reference values
    // reported under the unit-broken metric (CEM in per-step-mean, others
    // in per-episode-total). They are historical not live.
    eprintln!("\nLevel 0-1 pre-PR-2b reference (hand-coded, 1 layer, unit-broken):");
    eprintln!("  CEM: -1.05 (49 dones), TD3: -11.99, SAC: -10.82, PPO: -3449, REINFORCE: -7500");
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase 6b: Budget scaling experiments
// ═══════════════════════════════════════════════════════════════════════════
//
// Three independent experiments, each changing one variable from the
// Phase 6 baseline (Test 9: seed 42, 50ep/50env, 2-layer [64,64] ReLU
// Xavier).  Isolate the effect of each variable on the ordering.
//
// Phase 6 baseline (pre-Ch-41-PR-2b, unit-broken — CEM in per-step-mean,
// others in per-episode-total):
//   CEM (-3.07) > TD3 (-4.08) >> SAC (-30.04) >> PPO (-9026) >> REINFORCE (-11980)

// ── Test 10: 6b-1 — More epochs ──────────────────────────────────────────

/// 200 epochs instead of 50.  Same architecture, same envs, same seed.
///
/// **Hypothesis**: TD3 overtakes CEM.  At epoch 50, TD3 was at -4.08 and
/// still converging (eval trajectory: -29 → -18 → -9 → -5 → -4).  CEM
/// was at -3.07 with exploration noise at minimum — plateaued.
///
/// Only runs the top 3 (CEM, TD3, SAC).  PPO (-9,026) and REINFORCE
/// (-11,980) are 3 orders of magnitude behind — 4x more epochs won't
/// close a 1,000x gap.  Their problem is gradient variance, not
/// training duration.
#[test]
#[ignore = "multi-minute competition run"]
fn budget_scaling_more_epochs() {
    let task = reaching_6dof();
    let comp = Competition::new_verbose(50, TrainingBudget::Epochs(200), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_2layer,
        &build_td3_autograd_2layer,
        &build_sac_autograd_2layer,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();

    // All metrics must be finite.
    for run in [cem, td3, sac] {
        run.assert_finite();
    }

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();

    result.print_ranked(
        "reaching-6dof",
        "6b-1: More epochs (200ep/50env, 2-layer autograd)",
    );
    print_reversal_check(r_cem, &[("TD3", r_td3), ("SAC", r_sac)]);

    // Phase 6 baseline comparison (pre-PR-2b reference, unit-broken).
    eprintln!("\nPhase 6 baseline (50ep/50env, pre-PR-2b unit-broken):");
    eprintln!("  CEM: -3.07, TD3: -4.08, SAC: -30.04");
}

// ── Test 11: 6b-2 — Lower learning rate ──────────────────────────────────

/// Lower learning rates for all gradient methods.  Same epochs, same envs.
///
/// **Hypothesis**: SAC stabilizes (was oscillating -8 to -30 due to
/// aggressive LR).  All gradient methods converge smoother with 5K params.
///
/// LR changes (CEM unchanged — gradient-free):
/// - REINFORCE: 0.05 → 0.01 (5x reduction)
/// - PPO: 0.025 → 0.005 (5x reduction)
/// - TD3: 3e-4 → 1e-4 (3x reduction)
/// - SAC: 3e-4 → 1e-4 optimizer + `alpha_lr` (3x reduction)
#[test]
#[ignore = "multi-minute competition run"]
fn budget_scaling_lower_lr() {
    let task = reaching_6dof();
    let comp = Competition::new_verbose(50, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_2layer,
        &build_reinforce_autograd_2layer_low_lr,
        &build_ppo_autograd_2layer_low_lr,
        &build_td3_autograd_2layer_low_lr,
        &build_sac_autograd_2layer_low_lr,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();

    // All metrics must be finite.
    for run in [cem, td3, sac, ppo, reinforce] {
        run.assert_finite();
    }

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();

    result.print_ranked(
        "reaching-6dof",
        "6b-2: Lower LR (50ep/50env, 2-layer autograd)",
    );
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Phase 6 baseline comparison (pre-PR-2b reference, unit-broken).
    eprintln!("\nPhase 6 baseline (original LR, pre-PR-2b unit-broken):");
    eprintln!("  CEM: -3.07, TD3: -4.08, SAC: -30.04, PPO: -9026, REINFORCE: -11980");
}

// ── Test 12: 6b-3 — More environments ────────────────────────────────────

/// 200 environments instead of 50.  Same epochs, same LR.
///
/// **Hypothesis**: On-policy methods (PPO, REINFORCE) improve most — 4x
/// more samples per gradient estimate reduces variance.  Off-policy
/// methods benefit less because replay already multiplies effective
/// sample count.  CEM gets 200 candidates/generation instead of 50 —
/// better coverage of the 5K-dim search space.
#[test]
#[ignore = "multi-minute competition run"]
fn budget_scaling_more_envs() {
    let task = reaching_6dof();
    let comp = Competition::new_verbose(200, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_2layer,
        &build_reinforce_autograd_2layer,
        &build_ppo_autograd_2layer,
        &build_td3_autograd_2layer,
        &build_sac_autograd_2layer,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("reaching-6dof", "CEM").unwrap();
    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();

    // All metrics must be finite.
    for run in [cem, td3, sac, ppo, reinforce] {
        run.assert_finite();
    }

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();

    result.print_ranked(
        "reaching-6dof",
        "6b-3: More envs (50ep/200env, 2-layer autograd)",
    );
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Phase 6 baseline comparison (pre-PR-2b reference, unit-broken).
    eprintln!("\nPhase 6 baseline (50 envs, pre-PR-2b unit-broken):");
    eprintln!("  CEM: -3.07, TD3: -4.08, SAC: -30.04, PPO: -9026, REINFORCE: -11980");

    // CEM scaling note: 200 candidates/generation vs 50 at baseline.
    // At ~5,400 params, that's 1:27 candidates/param (vs 1:108 at baseline).
    // Still insufficient for gradient-free search, but 4x better coverage.
    eprintln!("\nCEM note: 200 candidates/gen (1:27 params/candidate) vs 50 (1:108) at baseline");
}

// ═══════════════════════════════════════════════════════════════════════════
// Phase 6c: Obstacle avoidance task
// ═══════════════════════════════════════════════════════════════════════════
//
// The obstacle avoidance task (`obstacle_reaching_6dof`) structurally
// advantages gradient methods over CEM:
//
// 1. Nonlinear obs→action mapping (go-around requires spatial reasoning)
// 2. Multi-modal reward landscape (two paths around the obstacle)
// 3. Task-space reward (fingertip position, not joint angles)
// 4. Penalty discontinuity (non-convex reward surface)
//
// Prediction: TD3 or SAC clearly outperform CEM at 50ep/50env with
// 2-layer [64,64] — the nonlinear reward landscape breaks CEM's
// gradient-free advantage that held on the smooth quadratic reaching task.

// ── Test 13: Obstacle avoidance — 2-layer autograd ────────────────────────

/// All 5 algorithms, autograd backends, 2 hidden layers (64+64), `ReLU`,
/// Xavier/He init, 6-DOF obstacle avoidance task.
///
/// **The hypothesis**: a nonlinear reward landscape (obstacle penalty +
/// task-space reaching) breaks CEM's dominance.
///
/// Post-Ch-41-PR-2b ordering at Phase 6c (seed 42, 50ep/50env):
///
///   TD3 (-0.55) > SAC (-0.97) >> CEM (-206.65) > PPO (-269.06) > REINFORCE (-269.13)
///
/// **Pre-PR-2b historical ordering** (reported against the unit-broken
/// metric): `CEM > TD3 > SAC > PPO > REINFORCE`, framed as "CEM still
/// dominates even on the nonlinear task, hypothesis wrong."
///
/// **The finding is actually the reverse under the corrected metric.**
/// The old "CEM dominates" was a unit-mismatch artifact (CEM per-step-mean
/// vs gradient methods per-episode-total). Under Ch 24 Decision 1's
/// unit-correct reporting, TD3 and SAC both score far above CEM, which
/// *validates* the original hypothesis — gradient methods do outperform
/// CEM on the nonlinear obstacle task.
///
/// Caveat: TD3 and SAC at ~-0.5 per-episode converged to a degenerate
/// fast-termination policy rather than truly solving the obstacle task
/// (both have 0 dones). CEM at -206.65 actually attempts to solve but
/// does not reach the target either. At 50ep/50env on this task, none
/// of the algorithms learn a solving policy — the test surfaces an
/// ordering-by-exploit rather than an ordering-by-solve. A higher
/// epoch budget or a per-replicate `Competition::run_replicates` call
/// would be needed to distinguish "gradient methods really do solve
/// this" from "gradient methods exploit the reward/termination structure
/// faster than CEM."
///
/// The ordering assertions from the pre-PR-2b version are removed in
/// favor of findings-mode (`print_reversal_check` + eprintln). The print
/// output is the load-bearing artifact; a future reader running this
/// test under --ignored should read the eprintln block to understand
/// the corrected ordering rather than rely on hard assertions that
/// the unit-mismatched metric could not have measured honestly.
///
/// Parameter counts (obstacle-reaching-6dof: obs=21, act=6):
/// - `AutogradPolicy` [64,64]: 21*64 + 64 + 64*64 + 64 + 64*6 + 6 = 5,958
/// - `AutogradQ` [64,64]:      27*64 + 64 + 64*64 + 64 + 64*1 + 1 = 6,017
/// - CEM with 50 candidates exploring ~6K dims: ~120 params/candidate
///
/// See Ch 41 §4.3 and Ch 24 §1.8 for the full unit-mismatch diagnosis.
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_obstacle_autograd_2layer() {
    let task = obstacle_reaching_6dof();
    let comp = Competition::new_verbose(50, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&(dyn Fn(&TaskConfig) -> Box<dyn Algorithm> + Sync)> = vec![
        &build_cem_autograd_2layer,
        &build_reinforce_autograd_2layer,
        &build_ppo_autograd_2layer,
        &build_td3_autograd_2layer,
        &build_sac_autograd_2layer,
    ];

    let result = comp.run(&[task], &builders).expect("competition failed");
    result.print_summary();

    let cem = result.find("obstacle-reaching-6dof", "CEM").unwrap();
    let td3 = result.find("obstacle-reaching-6dof", "TD3").unwrap();
    let sac = result.find("obstacle-reaching-6dof", "SAC").unwrap();
    let ppo = result.find("obstacle-reaching-6dof", "PPO").unwrap();
    let reinforce = result.find("obstacle-reaching-6dof", "REINFORCE").unwrap();

    // All metrics must be finite.
    for run in [cem, td3, sac, ppo, reinforce] {
        run.assert_finite();
    }

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();

    result.print_ranked(
        "obstacle-reaching-6dof",
        "Phase 6c: Obstacle avoidance (50ep/50env, 2-layer autograd)",
    );
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Findings-mode: no hard ordering assertions (see doc comment).
    // The only surviving assertion is the assert_finite() loop above,
    // which catches NaN/Inf regressions. Ordering is documented via
    // the print_reversal_check() and eprintln block, not asserted.
    eprintln!("\nPhase 6c findings (per-episode-total units, post Ch 41 PR 2b):");
    eprintln!(
        "  TD3={r_td3:.2}, SAC={r_sac:.2}, CEM={r_cem:.2}, \
         PPO={r_ppo:.2}, REINFORCE={r_reinforce:.2}"
    );
    eprintln!(
        "  Top-2 dones: TD3={}, SAC={}  (fast-termination-exploit \
         suspected if both ≈ 0 reward)",
        td3.total_dones(),
        sac.total_dones()
    );

    // Compare against reaching_6dof baseline (Test 9).
    // NOTE: the numbers below are pre-Ch-41-PR-2b reference values that
    // were reported under the unit-broken metric (CEM in per-step-mean
    // units, gradient methods in per-episode-total units). They are
    // preserved as a historical comparison, not as a live baseline.
    // Reward scales also differ between the two tasks (task-space
    // distance vs joint-space squared), so cross-task comparison is
    // two-ways broken regardless of the unit fix.
    eprintln!("\nreaching-6dof pre-PR-2b baseline (Test 9, unit-broken):");
    eprintln!("  CEM: -3.07, TD3: -4.08, SAC: -30.04, PPO: -9026, REINFORCE: -11980");
}
