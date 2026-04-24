//! Test-only stock task fixtures for chassis internal tests.
//!
//! These mirror `sim-rl::tasks::{reaching_2dof, reaching_6dof}` but live
//! inside chassis so the chassis crate's own `#[cfg(test)]` suites (e.g.
//! `competition.rs`) can exercise the Competition runner on a realistic
//! `TaskConfig` without taking a cyclic dev-dep on `sim-rl` (which would
//! resolve `TaskConfig` to a distinct type at compile time).
//!
//! Production stock factories with richer docstrings live in `sim-rl`.
//! Duplication here is bounded to test scope.

#![cfg(test)]

use std::sync::Arc;

use sim_core::BatchSim;

use crate::error::EnvError;
use crate::space::{ActionSpace, ObservationSpace};
use crate::task::TaskConfig;
use crate::vec_env::VecEnv;

const MJCF_2DOF: &str = r#"
<mujoco model="reaching-arm">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 -1 0"
             limited="true" range="-3.14159 3.14159" damping="2.0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0"
            size="0.03" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0"
               limited="true" range="-2.6 2.6" damping="1.0"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0"
              size="0.025" mass="0.3" rgba="0.85 0.4 0.2 1"/>
        <site name="fingertip" pos="0.4 0 0" size="0.015"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="10"
           ctrllimited="true" ctrlrange="-1 1"/>
    <motor name="elbow_motor" joint="elbow" gear="5"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

const MJCF_6DOF: &str = r#"
<mujoco model="reaching-arm-6dof">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="seg1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 -1 0" damping="2.0"
             limited="true" range="-3.14 3.14"/>
      <joint name="j2" type="hinge" axis="0 0 1" damping="1.5"
             limited="true" range="-1.57 1.57"/>
      <geom name="seg1_geom" type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <body name="seg2" pos="0.3 0 0">
        <joint name="j3" type="hinge" axis="0 -1 0" damping="1.5"
               limited="true" range="-2.6 2.6"/>
        <joint name="j4" type="hinge" axis="0 0 1" damping="1.0"
               limited="true" range="-1.57 1.57"/>
        <geom name="seg2_geom" type="capsule" fromto="0 0 0 0.25 0 0" size="0.025" mass="0.3"/>
        <body name="seg3" pos="0.25 0 0">
          <joint name="j5" type="hinge" axis="0 -1 0" damping="1.0"
                 limited="true" range="-2.6 2.6"/>
          <joint name="j6" type="hinge" axis="0 0 1" damping="0.5"
                 limited="true" range="-1.57 1.57"/>
          <geom name="seg3_geom" type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/>
          <site name="fingertip" pos="0.2 0 0" size="0.015"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j2" gear="8"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j3" gear="6"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j4" gear="5"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j5" gear="4"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j6" gear="3"  ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
"#;

// MJCF_2DOF is a compile-time constant; parse/build failures are author bugs,
// so panic on Err is the right contract — matches sim-rl::tasks.
#[allow(clippy::panic)]
#[must_use]
pub fn reaching_2dof() -> TaskConfig {
    let model = Arc::new(match sim_mjcf::load_model(MJCF_2DOF) {
        Ok(m) => m,
        Err(e) => panic!("hardcoded 2-DOF MJCF failed to parse: {e}"),
    });

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

// MJCF_6DOF is a compile-time constant; parse/build failures are author bugs,
// so panic on Err is the right contract — matches sim-rl::tasks.
#[allow(clippy::panic)]
#[must_use]
pub fn reaching_6dof() -> TaskConfig {
    let model = Arc::new(match sim_mjcf::load_model(MJCF_6DOF) {
        Ok(m) => m,
        Err(e) => panic!("hardcoded 6-DOF MJCF failed to parse: {e}"),
    });

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

    // FK block runs on a fresh BatchSim built from a compile-time MJCF; every
    // failure path here is an author bug, so panic is correct.
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
