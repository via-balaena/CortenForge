//! Smoke test: custom `TaskConfig::builder()` path through actual training.
//!
//! Proves the builder path works end-to-end — not just build + reset,
//! but real training with reward feedback. De-risks Phase 6c.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::Arc;

use sim_core::test_fixtures::reaching_2dof;
use sim_ml_chassis::{
    ActionSpace, Algorithm, LinearPolicy, ObservationSpace, TaskConfig, TrainingBudget,
};
use sim_rl::{Cem, CemHyperparams};

#[test]
fn custom_task_cem_trains() {
    // Builder-path smoke test: same Model as sim_rl::reaching_2dof, but routed
    // through TaskConfig::builder() with a custom reward to prove the bring-
    // your-own-Model API works end-to-end through actual training.
    let model = Arc::new(reaching_2dof());

    let obs = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("obs space");
    let act = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("act space");

    let target: [f64; 2] = [0.5, -0.8];

    let task = TaskConfig::builder("custom-reach", Arc::clone(&model))
        .observation_space(obs)
        .action_space(act)
        .obs_scale(vec![1.0, 1.0, 0.1, 0.1])
        .reward(move |_m, d| {
            let mut err_sq = 0.0;
            for (i, &tq) in target.iter().enumerate() {
                let e = d.qpos[i] - tq;
                err_sq = e.mul_add(e, err_sq);
            }
            -err_sq
        })
        .done(|_m, _d| false)
        .truncated(|_m, d| d.time > 2.0)
        .sub_steps(5)
        .build()
        .expect("task config");

    assert_eq!(task.obs_dim(), 4);
    assert_eq!(task.act_dim(), 2);

    let policy = Box::new(LinearPolicy::new(
        task.obs_dim(),
        task.act_dim(),
        task.obs_scale(),
    ));
    let mut cem = Cem::new(
        policy,
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 0.3,
            noise_decay: 0.95,
            noise_min: 0.01,
            max_episode_steps: 200,
        },
    );

    let mut env = task.build_vec_env(4, 0).expect("vec env");
    let metrics = cem.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {});

    assert_eq!(metrics.len(), 3, "expected 3 epochs");
    for m in &metrics {
        assert!(m.mean_reward.is_finite(), "reward must be finite");
    }
    // CEM should improve on a simple quadratic reward in 3 epochs.
    assert!(
        metrics[2].mean_reward > metrics[0].mean_reward,
        "reward should improve: epoch 0 = {}, epoch 2 = {}",
        metrics[0].mean_reward,
        metrics[2].mean_reward,
    );
}
