//! Smoke test: custom `TaskConfig::builder()` path through actual training.
//!
//! Proves the builder path works end-to-end — not just build + reset,
//! but real training with reward feedback. De-risks Phase 6c.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::Arc;

use sim_ml_bridge::{Cem, CemHyperparams};
use sim_ml_chassis::{
    ActionSpace, Algorithm, LinearPolicy, ObservationSpace, TaskConfig, TrainingBudget,
};

const MJCF: &str = r#"
<mujoco model="custom-2dof">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="upper" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 -1 0"
             limited="true" range="-3.14 3.14" damping="2.0"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.03" mass="0.5"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0"
               limited="true" range="-2.6 2.6" damping="1.0"/>
        <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.025" mass="0.3"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="shoulder" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="elbow" gear="5" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

#[test]
fn custom_task_cem_trains() {
    let model = Arc::new(sim_mjcf::load_model(MJCF).expect("valid MJCF"));

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
