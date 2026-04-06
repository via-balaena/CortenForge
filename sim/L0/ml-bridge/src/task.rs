//! Task configuration and stock task factories.
//!
//! A [`TaskConfig`] is a complete task definition — environment metadata plus
//! a [`VecEnv`] factory.  Algorithms use the metadata (dimensions, obs scales)
//! to construct their networks; the [`Competition`](crate::Competition) runner
//! calls [`build_vec_env`](TaskConfig::build_vec_env) to create fresh
//! environments for each run.
//!
//! ## Stock tasks
//!
//! - [`reaching_2dof()`] — 2-link planar arm, 4-dim obs, 2-dim act.
//! - [`reaching_6dof()`] — 3-segment arm with 6 joints, 12-dim obs, 6-dim act.
//!
//! ## Custom tasks
//!
//! Use [`TaskConfig::builder()`] to define new tasks from a pre-parsed
//! [`Model`].

use std::sync::Arc;

use sim_core::{BatchSim, Data, Model};

use crate::error::EnvError;
use crate::space::{ActionSpace, ObservationSpace};
use crate::vec_env::VecEnv;

// ── TaskConfig ──────────────────────────────────────────────────────────────

/// A complete task definition — environment + metadata for building algorithms.
///
/// Carries everything needed to (1) construct a [`VecEnv`] for training, and
/// (2) provide the algorithm with task-specific metadata (dimensions, obs
/// normalization scales).
///
/// Stock tasks: [`reaching_2dof()`], [`reaching_6dof()`].
/// Custom tasks: use [`TaskConfig::builder()`].
pub struct TaskConfig {
    name: String,
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    build_fn: Arc<dyn Fn(usize) -> Result<VecEnv, EnvError> + Send + Sync>,
}

// Manual Debug — build_fn doesn't implement Debug.
impl std::fmt::Debug for TaskConfig {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TaskConfig")
            .field("name", &self.name)
            .field("obs_dim", &self.obs_dim)
            .field("act_dim", &self.act_dim)
            .finish_non_exhaustive()
    }
}

// Clone — Arc<dyn Fn> is Clone.
impl Clone for TaskConfig {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            obs_dim: self.obs_dim,
            act_dim: self.act_dim,
            obs_scale: self.obs_scale.clone(),
            build_fn: Arc::clone(&self.build_fn),
        }
    }
}

impl TaskConfig {
    /// Task name (e.g., `"reaching-2dof"`).
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Observation dimension per environment.
    #[must_use]
    pub const fn obs_dim(&self) -> usize {
        self.obs_dim
    }

    /// Action dimension per environment.
    #[must_use]
    pub const fn act_dim(&self) -> usize {
        self.act_dim
    }

    /// Per-element observation normalization scales.
    ///
    /// Length equals [`obs_dim()`](Self::obs_dim).  Multiply raw observations
    /// element-wise by these scales before feeding into a policy.
    #[must_use]
    pub fn obs_scale(&self) -> &[f64] {
        &self.obs_scale
    }

    /// Construct a fresh [`VecEnv`] with `n_envs` parallel environments.
    ///
    /// Each call returns a new, independently resettable environment batch.
    /// The [`Competition`](crate::Competition) runner calls this once per
    /// (task, algorithm) pair.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if the internal `VecEnv` builder fails (should not
    /// happen for stock tasks).
    pub fn build_vec_env(&self, n_envs: usize) -> Result<VecEnv, EnvError> {
        (self.build_fn)(n_envs)
    }

    /// Start building a custom [`TaskConfig`] from a pre-parsed model.
    #[must_use]
    pub fn builder(name: impl Into<String>, model: Arc<Model>) -> TaskConfigBuilder {
        TaskConfigBuilder {
            name: name.into(),
            model,
            obs_space: None,
            act_space: None,
            obs_scale: None,
            reward_fn: None,
            done_fn: None,
            truncated_fn: None,
            sub_steps: 1,
        }
    }
}

// ── TaskConfigBuilder ───────────────────────────────────────────────────────

/// Builder for custom [`TaskConfig`] instances.
///
/// All fields except `sub_steps` (default 1) are required.
pub struct TaskConfigBuilder {
    name: String,
    model: Arc<Model>,
    obs_space: Option<ObservationSpace>,
    act_space: Option<ActionSpace>,
    obs_scale: Option<Vec<f64>>,
    reward_fn: Option<Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>>,
    done_fn: Option<Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    truncated_fn: Option<Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    sub_steps: usize,
}

impl TaskConfigBuilder {
    /// Set the observation space.
    #[must_use]
    pub fn observation_space(mut self, space: ObservationSpace) -> Self {
        self.obs_space = Some(space);
        self
    }

    /// Set the action space.
    #[must_use]
    pub fn action_space(mut self, space: ActionSpace) -> Self {
        self.act_space = Some(space);
        self
    }

    /// Set per-element observation normalization scales.
    ///
    /// Length must equal the observation space dimension.
    #[must_use]
    pub fn obs_scale(mut self, scale: Vec<f64>) -> Self {
        self.obs_scale = Some(scale);
        self
    }

    /// Set the reward function.
    #[must_use]
    pub fn reward(mut self, f: impl Fn(&Model, &Data) -> f64 + Send + Sync + 'static) -> Self {
        self.reward_fn = Some(Arc::new(f));
        self
    }

    /// Set the done (terminal) function.
    #[must_use]
    pub fn done(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self {
        self.done_fn = Some(Arc::new(f));
        self
    }

    /// Set the truncated (timeout) function.
    #[must_use]
    pub fn truncated(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self {
        self.truncated_fn = Some(Arc::new(f));
        self
    }

    /// Set the number of physics sub-steps per action (default 1).
    #[must_use]
    pub const fn sub_steps(mut self, n: usize) -> Self {
        self.sub_steps = n;
        self
    }

    /// Validate and build the [`TaskConfig`].
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if a required field is missing, `sub_steps` is 0,
    /// or `obs_scale` length doesn't match the observation dimension.
    pub fn build(self) -> Result<TaskConfig, EnvError> {
        let obs_space = self.obs_space.ok_or(EnvError::MissingField {
            field: "observation_space",
        })?;
        let act_space = self.act_space.ok_or(EnvError::MissingField {
            field: "action_space",
        })?;
        let obs_scale = self
            .obs_scale
            .ok_or(EnvError::MissingField { field: "obs_scale" })?;
        let reward_fn = self
            .reward_fn
            .ok_or(EnvError::MissingField { field: "reward" })?;
        let done_fn = self
            .done_fn
            .ok_or(EnvError::MissingField { field: "done" })?;
        let truncated_fn = self
            .truncated_fn
            .ok_or(EnvError::MissingField { field: "truncated" })?;

        if self.sub_steps == 0 {
            return Err(EnvError::ZeroSubSteps);
        }

        let obs_dim = obs_space.dim();
        let act_dim = act_space.dim();

        if obs_scale.len() != obs_dim {
            return Err(EnvError::ObsScaleMismatch {
                expected: obs_dim,
                actual: obs_scale.len(),
            });
        }

        let model = self.model;
        let sub_steps = self.sub_steps;

        let build_fn = Arc::new(move |n_envs: usize| -> Result<VecEnv, EnvError> {
            let r = Arc::clone(&reward_fn);
            let d = Arc::clone(&done_fn);
            let t = Arc::clone(&truncated_fn);
            VecEnv::builder(Arc::clone(&model), n_envs)
                .observation_space(obs_space.clone())
                .action_space(act_space.clone())
                .reward(move |m, data| r(m, data))
                .done(move |m, data| d(m, data))
                .truncated(move |m, data| t(m, data))
                .sub_steps(sub_steps)
                .build()
        });

        Ok(TaskConfig {
            name: self.name,
            obs_dim,
            act_dim,
            obs_scale,
            build_fn,
        })
    }
}

// ── Stock tasks ─────────────────────────────────────────────────────────────

// 2-DOF MJCF — identical to CEM / REINFORCE / PPO examples.
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

// 6-DOF MJCF — from spec. 3-segment planar arm with 6 hinge joints.
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
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <body name="seg2" pos="0.3 0 0">
        <joint name="j3" type="hinge" axis="0 -1 0" damping="1.5"
               limited="true" range="-2.6 2.6"/>
        <joint name="j4" type="hinge" axis="0 0 1" damping="1.0"
               limited="true" range="-1.57 1.57"/>
        <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.025" mass="0.3"/>
        <body name="seg3" pos="0.25 0 0">
          <joint name="j5" type="hinge" axis="0 -1 0" damping="1.0"
                 limited="true" range="-2.6 2.6"/>
          <joint name="j6" type="hinge" axis="0 0 1" damping="0.5"
                 limited="true" range="-1.57 1.57"/>
          <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/>
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
/// Panics if the hardcoded MJCF fails to parse (indicates a code bug).
#[allow(clippy::expect_used)]
#[must_use]
pub fn reaching_2dof() -> TaskConfig {
    let model = Arc::new(sim_mjcf::load_model(MJCF_2DOF).expect("hardcoded 2-DOF MJCF"));

    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("2-DOF obs space");

    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("2-DOF act space");

    let obs_dim = obs_space.dim();
    let act_dim = act_space.dim();

    let inv_pi = 1.0 / std::f64::consts::PI;
    let obs_scale = vec![inv_pi, inv_pi, 0.1, 0.1];

    // Target joint angles (IK solution for fingertip at [0.4, 0, 0.3]).
    let target_joints: [f64; 2] = [-0.242, 1.982];

    // Target end-effector position (for done condition — XZ plane only).
    let target_tip: [f64; 3] = [0.4, 0.0, 0.3];

    let sub_steps: usize = 5;

    let build_fn = Arc::new(move |n_envs: usize| -> Result<VecEnv, EnvError> {
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
    });

    TaskConfig {
        name: "reaching-2dof".into(),
        obs_dim,
        act_dim,
        obs_scale,
        build_fn,
    }
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
/// Panics if the hardcoded MJCF fails to parse (indicates a code bug).
#[allow(clippy::expect_used)]
#[must_use]
pub fn reaching_6dof() -> TaskConfig {
    let model = Arc::new(sim_mjcf::load_model(MJCF_6DOF).expect("hardcoded 6-DOF MJCF"));

    let obs_space = ObservationSpace::builder()
        .all_qpos()
        .all_qvel()
        .build(&model)
        .expect("6-DOF obs space");

    let act_space = ActionSpace::builder()
        .all_ctrl()
        .build(&model)
        .expect("6-DOF act space");

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
    let target_tip = {
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        let data = batch.env_mut(0).expect("env 0");
        for (i, &q) in target_joints.iter().enumerate() {
            data.qpos[i] = q;
        }
        data.forward(&model).expect("FK for 6-DOF target");
        let tip = batch.env(0).expect("env 0").site_xpos[0];
        [tip.x, tip.y, tip.z]
    };

    let sub_steps: usize = 5;

    let build_fn = Arc::new(move |n_envs: usize| -> Result<VecEnv, EnvError> {
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
    });

    TaskConfig {
        name: "reaching-6dof".into(),
        obs_dim,
        act_dim,
        obs_scale,
        build_fn,
    }
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
    use crate::tensor::Tensor;

    // ── TaskConfig traits ─────────────────────────────────────────────

    #[test]
    fn task_config_debug_clone() {
        let task = reaching_2dof();
        let task2 = task.clone();
        assert_eq!(task2.name(), "reaching-2dof");
        assert!(!format!("{task:?}").is_empty());
    }

    #[test]
    fn task_config_is_send_sync() {
        fn require<T: Send + Sync>() {}
        require::<TaskConfig>();
    }

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
        let mut env = task.build_vec_env(4).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[4, 4]);
    }

    #[test]
    fn reaching_2dof_step() {
        let task = reaching_2dof();
        let mut env = task.build_vec_env(4).unwrap();
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
        let mut env1 = task.build_vec_env(2).unwrap();
        let mut env2 = task.build_vec_env(8).unwrap();
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
        let mut env = task.build_vec_env(4).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[4, 12]);
    }

    #[test]
    fn reaching_6dof_step() {
        let task = reaching_6dof();
        let mut env = task.build_vec_env(4).unwrap();
        let _ = env.reset_all().unwrap();

        let actions = Tensor::zeros(&[4, 6]);
        let result = env.step(&actions).unwrap();
        assert_eq!(result.observations.shape(), &[4, 12]);
        assert_eq!(result.rewards.len(), 4);
    }

    // ── Builder ───────────────────────────────────────────────────────

    #[test]
    fn builder_missing_obs_space() {
        let model = Arc::new(sim_mjcf::load_model(MJCF_2DOF).expect("MJCF"));
        let result = TaskConfig::builder("test", model)
            .obs_scale(vec![1.0])
            .reward(|_, _| 0.0)
            .done(|_, _| false)
            .truncated(|_, _| false)
            .build();
        assert!(result.is_err());
    }

    #[test]
    fn builder_obs_scale_mismatch() {
        let model = Arc::new(sim_mjcf::load_model(MJCF_2DOF).expect("MJCF"));
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        // obs_dim is 4, but we pass 2 scales.
        let result = TaskConfig::builder("test", model)
            .observation_space(obs)
            .action_space(act)
            .obs_scale(vec![1.0, 1.0]) // wrong length
            .reward(|_, _| 0.0)
            .done(|_, _| false)
            .truncated(|_, _| false)
            .build();
        assert!(result.is_err());
    }

    #[test]
    fn builder_roundtrip() {
        let model = Arc::new(sim_mjcf::load_model(MJCF_2DOF).expect("MJCF"));
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let task = TaskConfig::builder("custom-task", Arc::clone(&model))
            .observation_space(obs)
            .action_space(act)
            .obs_scale(vec![1.0, 1.0, 0.1, 0.1])
            .reward(|_, _| 0.0)
            .done(|_, _| false)
            .truncated(|_m, d| d.time > 1.0)
            .sub_steps(3)
            .build()
            .unwrap();

        assert_eq!(task.name(), "custom-task");
        assert_eq!(task.obs_dim(), 4);
        assert_eq!(task.act_dim(), 2);

        let mut env = task.build_vec_env(2).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[2, 4]);
    }
}
