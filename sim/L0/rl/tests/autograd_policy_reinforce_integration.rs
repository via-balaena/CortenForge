//! Integration test exercising `AutogradPolicy` (Xavier-initialized,
//! 2-layer `ReLU`) through `Reinforce`'s full training loop.
//!
//! Originally lived inside `autograd_policy.rs`'s `mod tests` block.
//! When `autograd_policy.rs` moved to `sim-ml-chassis` during the
//! chassis/rl split, this test had to move with `Reinforce` — a chassis
//! test importing `Reinforce` would force a `sim-ml-chassis → sim-rl`
//! dev-dep cycle. It lives here instead, as a sim-rl integration test
//! against a chassis primitive.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use rand::SeedableRng;
use sim_ml_chassis::{
    Activation, Algorithm, AutogradPolicy, OptimizerConfig, TrainingBudget, reaching_2dof,
};
use sim_rl::{Reinforce, ReinforceHyperparams};

#[test]
fn relu_xavier_convergence_2dof() {
    let task = reaching_2dof();
    let mut rng = rand::rngs::StdRng::seed_from_u64(42);

    let policy = Box::new(AutogradPolicy::new_xavier(
        task.obs_dim(),
        &[32, 32],
        task.act_dim(),
        task.obs_scale(),
        Activation::Relu,
        &mut rng,
    ));

    let mut algo = Reinforce::new(
        policy,
        OptimizerConfig::adam(0.05),
        ReinforceHyperparams {
            gamma: 0.99,
            sigma_init: 0.5,
            sigma_decay: 0.95,
            sigma_min: 0.05,
            max_episode_steps: 300,
        },
    );

    let mut env = task.build_vec_env(20, 0).unwrap();
    let metrics = algo.train(&mut env, TrainingBudget::Epochs(20), 42, &|_| {});

    let first = metrics[0].mean_reward;
    let last = metrics[19].mean_reward;
    assert!(
        last > first,
        "2-layer ReLU+Xavier should improve: first={first:.1}, last={last:.1}",
    );
}
