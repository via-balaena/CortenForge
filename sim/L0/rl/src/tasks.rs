//! Stock task factories — fixture-backed reaching arms used across the RL
//! baselines and the `sim-ml` example suite.
//!
//! Models are constructed in pure Rust via `sim_core::test_fixtures`
//! (enabled here through the `test-fixtures` feature on `sim-core`). This
//! keeps `sim-rl`'s release graph free of the `sim-mjcf` → `mesh-io` →
//! `zip` chain, which in turn keeps every downstream consumer that
//! dev-deps `sim-rl` (sim-thermostat, sim-opt, examples) free of the
//! same chain. See L0 plan §3.2.
//!
//! ## Stock tasks
//!
//! - [`reaching_2dof()`] — 2-link planar arm, 4-dim obs, 2-dim act.
//! - [`reaching_6dof()`] — 3-segment arm with 6 joints, 12-dim obs, 6-dim act.
//! - [`obstacle_reaching_6dof()`] — 6-DOF arm with obstacle avoidance, 21-dim obs, 6-dim act.
//!
//! ## Custom tasks
//!
//! Use [`TaskConfig::builder()`](sim_ml_chassis::TaskConfig::builder) (re-exported
//! through `sim_ml_chassis` / `sim_rl`) to define new tasks from a pre-parsed
//! `Model`.

use std::sync::Arc;

use sim_core::BatchSim;
use sim_core::test_fixtures::reaching_6dof_obstacle as fixture_6dof_obstacle;
use sim_core::test_fixtures::{reaching_2dof as fixture_2dof, reaching_6dof as fixture_6dof};
use sim_ml_chassis::{ActionSpace, EnvError, ObservationSpace, TaskConfig, VecEnv};

// ── Stock task factories ────────────────────────────────────────────────────

/// 2-DOF reaching arm task — the stock "easy" task.
///
/// Two-link planar arm with shoulder and elbow joints.
/// - Observation: 4-dim (2 qpos + 2 qvel)
/// - Action: 2-dim (2 motor torques)
/// - Reward: negative squared joint-space error from target
/// - Done: fingertip within 5 cm of target AND velocity < 0.5
/// - Truncated: time > 3.0 s
///
/// Same arm as the CEM / REINFORCE / PPO examples.
///
/// # Panics
///
/// Panics if the hardcoded fixture's obs/act space build fails (indicates
/// a code bug — the fixture's structural shape is fixed at compile time).
// Fixture shape is compile-time fixed; obs/act build failure = author bug.
#[allow(clippy::panic)]
#[must_use]
pub fn reaching_2dof() -> TaskConfig {
    let model = Arc::new(fixture_2dof());

    let obs_space = match ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
    {
        Ok(s) => s,
        Err(e) => panic!("2-DOF obs space build failed: {e}"),
    };

    let act_space = match ActionSpace::builder().all_ctrl().build(&model) {
        Ok(s) => s,
        Err(e) => panic!("2-DOF act space build failed: {e}"),
    };

    let obs_dim = obs_space.dim();
    let act_dim = act_space.dim();

    let inv_pi = 1.0 / std::f64::consts::PI;
    let obs_scale = vec![inv_pi, inv_pi, 0.1, 0.1];

    // Target joint angles (IK solution for fingertip at [0.4, 0, 0.3]).
    let target_joints: [f64; 2] = [-0.242, 1.982];

    // Target end-effector position (for done condition — XZ plane only).
    let target_tip: [f64; 3] = [0.4, 0.0, 0.3];

    let sub_steps: usize = 5;

    let build_fn = move |n_envs: usize, _seed: u64| -> Result<VecEnv, EnvError> {
        let tq = target_joints;
        let gp = target_tip;
        VecEnv::builder(Arc::clone(&model), n_envs)
            .observation_space(obs_space.clone())
            .action_space(act_space.clone())
            .reward(move |_m, d| {
                let e0 = d.qpos[0] - tq[0];
                let e1 = d.qpos[1] - tq[1];
                -e0.mul_add(e0, e1 * e1)
            })
            .done(move |_m, d| {
                let tip = d.site_xpos[0];
                let dist = (tip.x - gp[0]).hypot(tip.z - gp[2]);
                let vel = d.qvel[0].hypot(d.qvel[1]);
                dist < 0.05 && vel < 0.5
            })
            .truncated(|_m, d| d.time > 3.0)
            .sub_steps(sub_steps)
            .build()
    };

    TaskConfig::from_build_fn("reaching-2dof", obs_dim, act_dim, obs_scale, build_fn)
}

/// 6-DOF reaching arm task — the stock "hard" task.
///
/// Three-segment arm with alternating pitch/yaw joints.
/// - Observation: 12-dim (6 qpos + 6 qvel)
/// - Action: 6-dim (6 motor torques)
/// - Reward: negative squared joint-space error from target
/// - Done: fingertip within 5 cm of target AND velocity < 1.0
/// - Truncated: time > 5.0 s
///
/// This task separates algorithms: CEM is sample-starved at 614 MLP params,
/// REINFORCE has high-variance gradients, PPO's learned baseline helps,
/// and off-policy methods (TD3/SAC) dominate via replay.
///
/// # Panics
///
/// Panics if the hardcoded fixture's obs/act space or FK setup fails
/// (indicates a code bug — the fixture's structural shape is fixed at
/// compile time).
// Fixture shape is compile-time fixed; obs/act/FK failure = author bug.
#[allow(clippy::panic)]
#[must_use]
pub fn reaching_6dof() -> TaskConfig {
    let model = Arc::new(fixture_6dof());

    let obs_space = match ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
    {
        Ok(s) => s,
        Err(e) => panic!("6-DOF obs space build failed: {e}"),
    };

    let act_space = match ActionSpace::builder().all_ctrl().build(&model) {
        Ok(s) => s,
        Err(e) => panic!("6-DOF act space build failed: {e}"),
    };

    let obs_dim = obs_space.dim();
    let act_dim = act_space.dim();

    let inv_pi = 1.0 / std::f64::consts::PI;
    let obs_scale = vec![
        inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, // qpos
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1, // qvel
    ];

    // Target joint configuration — moderate bend in all joints.
    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];

    // Compute target fingertip position via forward kinematics.
    #[allow(clippy::panic)]
    let target_tip = {
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        {
            let data = batch
                .env_mut(0)
                .unwrap_or_else(|| panic!("6-DOF FK: no env 0"));
            for (i, &q) in target_joints.iter().enumerate() {
                data.qpos[i] = q;
            }
            data.forward(&model)
                .unwrap_or_else(|e| panic!("6-DOF FK failed: {e}"));
        }
        let tip = batch
            .env(0)
            .unwrap_or_else(|| panic!("6-DOF FK: no env 0"))
            .site_xpos[0];
        [tip.x, tip.y, tip.z]
    };

    let sub_steps: usize = 5;

    let build_fn = move |n_envs: usize, _seed: u64| -> Result<VecEnv, EnvError> {
        let tq = target_joints;
        let gp = target_tip;
        VecEnv::builder(Arc::clone(&model), n_envs)
            .observation_space(obs_space.clone())
            .action_space(act_space.clone())
            .reward(move |_m, d| {
                let mut err_sq = 0.0;
                for (tq_i, &q) in tq.iter().enumerate() {
                    let e = d.qpos[tq_i] - q;
                    err_sq = e.mul_add(e, err_sq);
                }
                -err_sq
            })
            .done(move |_m, d| {
                let tip = d.site_xpos[0];
                let dx = tip.x - gp[0];
                let dy = tip.y - gp[1];
                let dz = tip.z - gp[2];
                let dist = dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt();
                let mut vel_sq = 0.0;
                for j in 0..6 {
                    vel_sq = d.qvel[j].mul_add(d.qvel[j], vel_sq);
                }
                dist < 0.05 && vel_sq.sqrt() < 1.0
            })
            .truncated(|_m, d| d.time > 5.0)
            .sub_steps(sub_steps)
            .build()
    };

    TaskConfig::from_build_fn("reaching-6dof", obs_dim, act_dim, obs_scale, build_fn)
}

/// 6-DOF obstacle-avoidance reaching task.
///
/// Same 3-segment arm as [`reaching_6dof()`], but a static obstacle sphere
/// sits between the rest configuration and the target. The agent must curve
/// around it.
///
/// - Observation: 21-dim (6 qpos + 6 qvel + 3 fingertip + 3 obstacle + 3 target)
/// - Action: 6-dim (6 motor torques)
/// - Reward: `-dist(fingertip, target) - λ * max(0, r_safe - dist(fingertip, obstacle))`
///   where λ=10.0, `r_safe`=0.12
/// - Done: fingertip within 5 cm of target AND velocity < 1.0
/// - Truncated: time > 5.0 s
///
/// The obstacle is a distance-penalty ghost — no contacts. This isolates
/// reward nonlinearity as the variable that breaks CEM dominance.
///
/// # Panics
///
/// Panics if the hardcoded fixture's obs/act space or FK setup fails
/// (indicates a code bug — the fixture's structural shape is fixed at
/// compile time).
// Fixture shape is compile-time fixed; obs/act/FK failure = author bug.
#[allow(clippy::panic)]
#[must_use]
pub fn obstacle_reaching_6dof() -> TaskConfig {
    let model = Arc::new(fixture_6dof_obstacle());

    // Safety: verify expected body/site counts.
    assert_eq!(model.nbody, 5, "obstacle fixture: expected 5 bodies");
    assert_eq!(model.nsite, 2, "obstacle fixture: expected 2 sites");

    // 21-dim obs: qpos(6) + qvel(6) + fingertip(3) + obstacle(3) + target(3).
    let obs_space = match ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .site_xpos(1..2) // fingertip (site 1)
        .xpos(4..5) // obstacle (body 4)
        .site_xpos(0..1) // target (site 0)
        .build(&model)
    {
        Ok(s) => s,
        Err(e) => panic!("obstacle 6-DOF obs space build failed: {e}"),
    };

    let act_space = match ActionSpace::builder().all_ctrl().build(&model) {
        Ok(s) => s,
        Err(e) => panic!("obstacle 6-DOF act space build failed: {e}"),
    };

    let obs_dim = obs_space.dim();
    let act_dim = act_space.dim();

    let inv_pi = 1.0 / std::f64::consts::PI;
    #[rustfmt::skip]
    let obs_scale = vec![
        inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, // qpos
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1,                   // qvel
        1.0, 1.0, 1.0,                                    // fingertip pos
        1.0, 1.0, 1.0,                                    // obstacle pos
        1.0, 1.0, 1.0,                                    // target pos
    ];

    // Compute target fingertip position via FK (same approach as reaching_6dof).
    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];
    // FK block runs on a fresh BatchSim built from a compile-time fixture;
    // every failure path here is an author bug, so panic is correct.
    #[allow(clippy::panic)]
    let target_tip = {
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        {
            let data = batch
                .env_mut(0)
                .unwrap_or_else(|| panic!("obstacle 6-DOF FK: no env 0"));
            for (i, &q) in target_joints.iter().enumerate() {
                data.qpos[i] = q;
            }
            data.forward(&model)
                .unwrap_or_else(|e| panic!("obstacle 6-DOF FK failed: {e}"));
        }
        batch
            .env(0)
            .unwrap_or_else(|| panic!("obstacle 6-DOF FK: no env 0"))
            .site_xpos[1] // fingertip is site 1 in obstacle MJCF
    };

    // Obstacle penalty parameters.
    let lambda: f64 = 10.0;
    let r_safe: f64 = 0.12;

    let sub_steps: usize = 5;

    let build_fn = move |n_envs: usize, _seed: u64| -> Result<VecEnv, EnvError> {
        let target = target_tip;
        let lam = lambda;
        let rs = r_safe;
        VecEnv::builder(Arc::clone(&model), n_envs)
            .observation_space(obs_space.clone())
            .action_space(act_space.clone())
            .reward(move |_m, d| {
                let fingertip = d.site_xpos[1];
                let obstacle = d.xpos[4];
                let dist_target = (fingertip - target).norm();
                let dist_obstacle = (fingertip - obstacle).norm();
                let penalty = lam * (rs - dist_obstacle).max(0.0);
                -dist_target - penalty
            })
            .done(move |_m, d| {
                let fingertip = d.site_xpos[1];
                let dist = (fingertip - target).norm();
                let mut vel_sq = 0.0;
                for j in 0..6 {
                    vel_sq = d.qvel[j].mul_add(d.qvel[j], vel_sq);
                }
                dist < 0.05 && vel_sq.sqrt() < 1.0
            })
            .truncated(|_m, d| d.time > 5.0)
            .sub_steps(sub_steps)
            .build()
    };

    TaskConfig::from_build_fn(
        "obstacle-reaching-6dof",
        obs_dim,
        act_dim,
        obs_scale,
        build_fn,
    )
}

// ── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::let_underscore_must_use
)]
mod tests {
    use super::*;
    use sim_ml_chassis::Tensor;

    // ── 2-DOF ─────────────────────────────────────────────────────────

    #[test]
    fn reaching_2dof_dims() {
        let task = reaching_2dof();
        assert_eq!(task.name(), "reaching-2dof");
        assert_eq!(task.obs_dim(), 4);
        assert_eq!(task.act_dim(), 2);
        assert_eq!(task.obs_scale().len(), 4);
    }

    #[test]
    fn reaching_2dof_build_and_reset() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[4, 4]);
    }

    #[test]
    fn reaching_2dof_step() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let _ = env.reset_all().unwrap();

        let actions = Tensor::zeros(&[4, 2]);
        let result = env.step(&actions).unwrap();
        assert_eq!(result.observations.shape(), &[4, 4]);
        assert_eq!(result.rewards.len(), 4);
        assert_eq!(result.dones.len(), 4);
        assert_eq!(result.truncateds.len(), 4);
    }

    #[test]
    fn reaching_2dof_multiple_builds() {
        let task = reaching_2dof();
        // build_vec_env can be called multiple times (Arc closure).
        let mut env1 = task.build_vec_env(2, 0).unwrap();
        let mut env2 = task.build_vec_env(8, 0).unwrap();
        assert_eq!(env1.reset_all().unwrap().shape(), &[2, 4]);
        assert_eq!(env2.reset_all().unwrap().shape(), &[8, 4]);
    }

    // ── 6-DOF ─────────────────────────────────────────────────────────

    #[test]
    fn reaching_6dof_dims() {
        let task = reaching_6dof();
        assert_eq!(task.name(), "reaching-6dof");
        assert_eq!(task.obs_dim(), 12);
        assert_eq!(task.act_dim(), 6);
        assert_eq!(task.obs_scale().len(), 12);
    }

    #[test]
    fn reaching_6dof_build_and_reset() {
        let task = reaching_6dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[4, 12]);
    }

    #[test]
    fn reaching_6dof_step() {
        let task = reaching_6dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let _ = env.reset_all().unwrap();

        let actions = Tensor::zeros(&[4, 6]);
        let result = env.step(&actions).unwrap();
        assert_eq!(result.observations.shape(), &[4, 12]);
        assert_eq!(result.rewards.len(), 4);
    }

    // ── Obstacle 6-DOF ───────────────────────────────────────────────

    #[test]
    fn obstacle_reaching_6dof_dims() {
        let task = obstacle_reaching_6dof();
        assert_eq!(task.name(), "obstacle-reaching-6dof");
        assert_eq!(task.obs_dim(), 21);
        assert_eq!(task.act_dim(), 6);
        assert_eq!(task.obs_scale().len(), 21);
    }

    #[test]
    fn obstacle_reaching_6dof_build_and_reset() {
        let task = obstacle_reaching_6dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[4, 21]);
    }

    #[test]
    fn obstacle_reaching_6dof_step() {
        let task = obstacle_reaching_6dof();
        let mut env = task.build_vec_env(4, 0).unwrap();
        let _ = env.reset_all().unwrap();

        let actions = Tensor::zeros(&[4, 6]);
        let result = env.step(&actions).unwrap();
        assert_eq!(result.observations.shape(), &[4, 21]);
        assert_eq!(result.rewards.len(), 4);
        assert_eq!(result.dones.len(), 4);
        assert_eq!(result.truncateds.len(), 4);
    }

    #[test]
    fn obstacle_reaching_6dof_reward_is_negative() {
        // At rest, fingertip is far from target — reward should be negative.
        let task = obstacle_reaching_6dof();
        let mut env = task.build_vec_env(1, 0).unwrap();
        let _ = env.reset_all().unwrap();
        let actions = Tensor::zeros(&[1, 6]);
        let result = env.step(&actions).unwrap();
        assert!(result.rewards[0] < 0.0, "reward should be negative at rest");
    }

    #[test]
    fn obstacle_penalty_fires_near_obstacle() {
        // At rest (qpos=0), fingertip is at (0.75, 0, 0).
        // Obstacle is at (0.730, 0.046, 0.030).
        // Distance ≈ 0.058m, which is < r_safe (0.12) — penalty should fire.
        let model = Arc::new(fixture_6dof_obstacle());
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        {
            let data = batch.env_mut(0).unwrap();
            data.forward(&model).unwrap();
        }
        let data = batch.env(0).unwrap();
        let fingertip = data.site_xpos[1];
        let obstacle = data.xpos[4];
        let dist = (fingertip - obstacle).norm();
        assert!(
            dist < 0.12,
            "rest-state fingertip should be within r_safe of obstacle, got {dist:.4}"
        );

        // Reward should include penalty (more negative than just -dist_target).
        let target = data.site_xpos[0]; // target site
        let dist_target = (fingertip - target).norm();
        let penalty = 10.0 * (0.12 - dist).max(0.0);
        let expected_reward = -dist_target - penalty;
        assert!(
            penalty > 0.0,
            "penalty should be positive at rest, got {penalty:.4}"
        );
        assert!(
            expected_reward < -dist_target,
            "reward with penalty ({expected_reward:.4}) should be worse than without ({:.4})",
            -dist_target
        );
    }

    #[test]
    fn obstacle_penalty_zero_when_far() {
        // Set joints to target config — fingertip should be near target
        // and far from obstacle (obstacle is between rest and target).
        let model = Arc::new(fixture_6dof_obstacle());
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        {
            let data = batch.env_mut(0).unwrap();
            let target_joints = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];
            for (i, &q) in target_joints.iter().enumerate() {
                data.qpos[i] = q;
            }
            data.forward(&model).unwrap();
        }
        let data = batch.env(0).unwrap();
        let fingertip = data.site_xpos[1];
        let obstacle = data.xpos[4];
        let dist = (fingertip - obstacle).norm();
        let penalty = 10.0_f64 * (0.12 - dist).max(0.0);
        assert!(
            penalty.abs() < 1e-10,
            "penalty should be zero at target config, got {penalty:.6} (dist={dist:.4})"
        );
    }

    #[test]
    fn obstacle_site_ordering_verified() {
        // Verify: site 0 = target (on worldbody), site 1 = fingertip (on seg3).
        let model = Arc::new(fixture_6dof_obstacle());
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        {
            let data = batch.env_mut(0).unwrap();
            data.forward(&model).unwrap();
        }
        let data = batch.env(0).unwrap();

        // Target site should be at the fixed position from the MJCF.
        let target = data.site_xpos[0];
        assert!((target.x - 0.681_474).abs() < 0.001);
        assert!((target.y - 0.154_033).abs() < 0.001);
        assert!((target.z - 0.101_028).abs() < 0.001);

        // Fingertip at rest should be at (0.75, 0, 0).
        let fingertip = data.site_xpos[1];
        assert!((fingertip.x - 0.75).abs() < 0.001);
        assert!(fingertip.y.abs() < 0.001);
        assert!(fingertip.z.abs() < 0.001);
    }
}
