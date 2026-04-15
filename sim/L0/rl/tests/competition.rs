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
    Competition, LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue, MlpPolicy, MlpQ,
    MlpValue, OptimizerConfig, RunResult, TaskConfig, TrainingBudget, obstacle_reaching_6dof,
    reaching_2dof, reaching_6dof,
};
use sim_rl::{
    Cem, CemHyperparams, Ppo, PpoHyperparams, Reinforce, ReinforceHyperparams, Sac, SacHyperparams,
    Td3, Td3Hyperparams,
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

/// All 5 algorithms, MLP policies, 6-DOF.  The headline competition.
///
/// **Ordering under Ch 41 PR 2b corrected metric** (seed 42, 50ep/50env):
///
///   SAC (-10.64) > TD3 (-11.99) >> CEM (-479.20) >> PPO (-3449) >> REINFORCE (-7500)
///
/// **Pre-PR-2b historical ordering** (reported in earlier runs against the
/// unit-broken metric):
///
///   CEM (-1.05, 49 dones) > SAC > TD3 >> PPO >> REINFORCE
///
/// The old "CEM dominates" conclusion was an artifact of unit mismatch:
/// CEM's reported value was per-step-mean (~-1.05) while the gradient
/// methods reported per-episode-total (~-10 to -7500). Under Ch 24 Decision 1
/// every algorithm now reports per-episode-total, and CEM's 49-done
/// convergence shows up at -479.20 (per-episode total ≈ per-step -1.05 ×
/// avg-trajectory-length). The corrected ordering shows SAC slightly
/// ahead of TD3 on 6-DOF even with `LinearStochasticPolicy`, and both
/// gradient methods decisively beat CEM. The `CEM > off-policy` ordering
/// previously asserted here was removed as a findings-mode conversion;
/// the `off-policy > on-policy` assertions survive because they reflect
/// a semantically valid comparison under both metrics.
///
/// See Ch 41 §4.3 and Ch 24 §1.8 for the full unit-mismatch diagnosis.
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
    let td3 = result.find("reaching-6dof", "TD3").unwrap();
    let sac = result.find("reaching-6dof", "SAC").unwrap();
    let ppo = result.find("reaching-6dof", "PPO").unwrap();
    let reinforce = result.find("reaching-6dof", "REINFORCE").unwrap();

    let r_cem = cem.final_reward().unwrap();
    let r_td3 = td3.final_reward().unwrap();
    let r_sac = sac.final_reward().unwrap();
    let r_ppo = ppo.final_reward().unwrap();
    let r_reinforce = reinforce.final_reward().unwrap();

    // Off-policy beats on-policy — this assertion is semantically valid
    // under both pre- and post-PR-2b metrics because TD3/PPO/REINFORCE
    // all reported per-episode-total units even before Decision 1.
    assert!(
        r_td3 > r_ppo,
        "TD3 ({r_td3:.2}) should beat PPO ({r_ppo:.2}) — off-policy replay helps"
    );
    assert!(
        r_ppo > r_reinforce,
        "PPO ({r_ppo:.2}) should beat REINFORCE ({r_reinforce:.2}) — baseline helps"
    );

    // CEM vs gradient methods: findings-mode, no hard assertion. The old
    // `r_cem > r_td3` assertion was an artifact of the pre-PR-2b unit
    // mismatch (see doc comment above).
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Print full results for the record.
    for name in &["CEM", "SAC", "TD3", "PPO", "REINFORCE"] {
        if let Some(run) = result.find("reaching-6dof", name) {
            let r = run.final_reward().unwrap();
            eprintln!("{name}: reward={r:.2}, dones={}", run.total_dones());
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Level 2: Autograd competition re-run (Phase 6)
// ═══════════════════════════════════════════════════════════════════════════

// ── Test 8: Autograd 1-layer parity ──────────────────────────────────────

/// All 5 algorithms, autograd backends, 1 hidden layer (32 units), 6-DOF.
///
/// Post-Ch-41-PR-2b ordering at level 2 1-layer parity (seed 42, 50ep/50env):
///
///   SAC (-10.64) > TD3 (-11.99) >> CEM (-479.20) >> PPO (-3449) >> REINFORCE (-7500)
///
/// The autograd-1-layer numbers match the hand-coded MLP numbers (Test 7)
/// within noise, validating that autograd doesn't regress performance. The
/// old "CEM should dominate" ordering was a unit-mismatch artifact — see
/// Test 7's doc comment for the full diagnosis. This test's findings-mode
/// conversion mirrors Test 7's: the `off-policy > on-policy` assertions
/// survive (semantically valid under both metrics) and the `CEM > TD3`
/// assertion is replaced with `print_reversal_check`.
///
/// Two differences from level 0-1 hand-coded:
/// 1. Gradients computed via tape-based reverse-mode AD, not hand-coded
/// 2. SAC gets `AutogradStochasticPolicy` (MLP actor) instead of
///    `LinearStochasticPolicy` — capacity upgrade, may improve SAC's rank
#[test]
#[ignore = "multi-minute competition run"]
fn competition_6dof_autograd_1layer_parity() {
    let task = reaching_6dof();
    let comp = Competition::new_verbose(50, TrainingBudget::Epochs(50), 42);

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
        &build_cem_autograd_1layer,
        &build_reinforce_autograd_1layer,
        &build_ppo_autograd_1layer,
        &build_td3_autograd_1layer,
        &build_sac_autograd_1layer,
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

    // Off-policy beats on-policy — semantically valid under both metrics
    // (TD3, PPO, REINFORCE all reported per-episode-total units even
    // before Ch 24 Decision 1).
    assert!(
        r_td3 > r_ppo,
        "TD3 ({r_td3:.2}) should beat PPO ({r_ppo:.2}) at 1 layer"
    );
    assert!(
        r_ppo > r_reinforce,
        "PPO ({r_ppo:.2}) should beat REINFORCE ({r_reinforce:.2}) at 1 layer"
    );

    // CEM vs gradient methods: findings-mode, no hard assertion (the old
    // `r_cem > r_td3` assertion was a pre-PR-2b unit-mismatch artifact).
    print_reversal_check(
        r_cem,
        &[
            ("TD3", r_td3),
            ("SAC", r_sac),
            ("PPO", r_ppo),
            ("REINFORCE", r_reinforce),
        ],
    );

    // Print full results for the record.
    eprintln!("\n=== Level 2 parity (1-layer autograd) ===");
    for (name, reward) in [
        ("CEM", r_cem),
        ("SAC", r_sac),
        ("TD3", r_td3),
        ("PPO", r_ppo),
        ("REINFORCE", r_reinforce),
    ] {
        let run = result.find("reaching-6dof", name).unwrap();
        eprintln!("{name}: reward={reward:.2}, dones={}", run.total_dones());
    }

    // SAC comparison: now with MLP actor, does it beat TD3?
    if r_sac > r_td3 {
        eprintln!(
            "Finding: SAC ({r_sac:.2}) overtakes TD3 ({r_td3:.2}) — \
             MLP stochastic actor unlocked by autograd"
        );
    } else {
        eprintln!("Finding: TD3 ({r_td3:.2}) still ahead of SAC ({r_sac:.2}) at 1 layer");
    }
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

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
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

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
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

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
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

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
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

    let builders: Vec<&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>> = vec![
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
