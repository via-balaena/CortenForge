//! Stock reaching-arm task fixtures for test-scoped use.
//!
//! These mirror `sim-rl::tasks::{reaching_2dof, reaching_6dof}` but live
//! inside chassis so the chassis crate's own `#[cfg(test)]` suites (e.g.
//! `competition.rs`) can exercise the Competition runner on a realistic
//! `TaskConfig` without taking a cyclic dev-dep on `sim-rl` (which would
//! resolve `TaskConfig` to a distinct type at compile time).
//!
//! The module is compiled for chassis's own tests and, under the
//! `test-fixtures` feature, re-exported for downstream test-only crates
//! (e.g. `sim-opt`'s internal unit tests) that want the same env without
//! dev-depending on the higher-tier `sim-rl`. Gating lives on the `pub mod`
//! declaration in `lib.rs`; the feature forwards to `sim-core/test-fixtures`
//! so the underlying Model factories are available.
//!
//! Production stock factories with richer docstrings live in `sim-rl`.
//! Duplication here is bounded to test scope. Both halves now share the
//! same underlying `sim_core::test_fixtures::reaching_*` Model fixtures
//! so divergence is structurally bounded to the surrounding `TaskConfig`
//! reward/done/truncated wiring.

use std::sync::Arc;

use sim_core::BatchSim;
use sim_core::test_fixtures::{reaching_2dof as fixture_2dof, reaching_6dof as fixture_6dof};

use crate::error::EnvError;
use crate::space::{ActionSpace, ObservationSpace};
use crate::task::TaskConfig;
use crate::vec_env::VecEnv;

/// Stock 2-DOF reaching-arm task: qpos/qvel observation, all-ctrl action,
/// negative squared joint error reward, done on tip-near-target-and-slow.
/// Backed by `sim_core::test_fixtures::reaching_2dof`.
///
/// # Panics
///
/// Panics if the observation/action space build fails, which can only happen
/// on an author bug in the compile-time-fixed Model shape.
// Space builds use a compile-time-fixed Model shape; a build failure is an
// author bug, so panic on Err is the right contract (see `# Panics` above).
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

    let target_joints: [f64; 2] = [-0.242, 1.982];
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

/// Stock 6-DOF reaching-arm task: qpos/qvel observation, all-ctrl action,
/// negative squared joint error reward, done on tip-near-FK-target-and-slow.
/// Backed by `sim_core::test_fixtures::reaching_6dof`.
///
/// # Panics
///
/// Panics if the observation/action space build or the target-tip forward
/// kinematics fails, which can only happen on an author bug in the
/// compile-time-fixed Model shape.
// Space/FK builds use a compile-time-fixed Model shape; a failure is an author
// bug, so panic on Err is the right contract (see `# Panics` above).
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
        inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, inv_pi, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
    ];

    let target_joints: [f64; 6] = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1];

    // FK block runs on a fresh BatchSim built from a compile-time fixture;
    // every failure path here is an author bug, so panic is correct.
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
