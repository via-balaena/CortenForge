//! Environment trait and [`SimEnv`] — closure-based single environment.
//!
//! [`SimEnv`] wraps a single `Model` + `Data` pair with observation/action
//! spaces, reward/done/truncated closures, and optional on-reset hooks.
//! It handles sub-stepping with early termination.

use std::sync::Arc;

use sim_core::{Data, Model, StepError};

use crate::error::{EnvError, ResetError};
use crate::space::{ActionSpace, ObservationSpace};
use crate::tensor::Tensor;

// ─── StepResult ─────────────────────────────────────────────────────────────

/// The output of a single environment step.
#[derive(Debug, Clone)]
pub struct StepResult {
    /// Observation after the step (or at terminal state).
    pub observation: Tensor,
    /// Reward for this step.  `f64` — reward precision matters for learning.
    pub reward: f64,
    /// Episode terminated (true terminal state — value is zero).
    pub done: bool,
    /// Episode truncated (time limit — value must be bootstrapped).
    pub truncated: bool,
}

// ─── Environment trait ──────────────────────────────────────────────────────

/// A single RL environment wrapping `Model` + `Data`.
///
/// Most users should use [`SimEnv`] (closure-based) rather than implementing
/// this trait directly.  The trait exists for cases where closures don't
/// suffice (multi-phase environments, complex state machines, etc.).
pub trait Environment {
    /// The observation space for this environment.
    fn observation_space(&self) -> &ObservationSpace;
    /// The action space for this environment.
    fn action_space(&self) -> &ActionSpace;
    /// Extract the current observation.
    fn observe(&self) -> Tensor;
    /// Step the environment with the given action.
    ///
    /// # Errors
    ///
    /// Returns [`StepError`] if physics integration fails.
    fn step(&mut self, action: &Tensor) -> Result<StepResult, StepError>;
    /// Reset the environment and return the initial observation.
    ///
    /// # Errors
    ///
    /// Returns [`ResetError`] if `forward()` fails after reset.
    fn reset(&mut self) -> Result<Tensor, ResetError>;
    /// Access the shared model.
    fn model(&self) -> &Model;
    /// Access the current data.
    fn data(&self) -> &Data;
}

// ─── SimEnv ─────────────────────────────────────────────────────────────────

/// A general-purpose [`Environment`] built from closures.
///
/// Handles observation extraction, action injection, stepping, and reset.
/// The user provides only the task-specific parts: reward, termination,
/// and optional reset customization (domain randomization).
///
/// ## Sub-stepping
///
/// `SimEnv` checks `done_fn` after each sub-step and breaks early if
/// the episode terminates — avoiding post-terminal corruption.  This is
/// cheap for a single env (one branch per sub-step).  Contrast with
/// [`VecEnv`](crate::VecEnv) which runs all sub-steps unconditionally
/// and evaluates done once at the final state.
///
/// Constructed via [`SimEnv::builder()`].
pub struct SimEnv {
    model: Arc<Model>,
    data: Data,
    obs_space: ObservationSpace,
    act_space: ActionSpace,
    reward_fn: Box<dyn Fn(&Model, &Data) -> f64>,
    done_fn: Box<dyn Fn(&Model, &Data) -> bool>,
    truncated_fn: Box<dyn Fn(&Model, &Data) -> bool>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data)>>,
    sub_steps: usize,
}

// Manual Debug — closures aren't Debug.
impl std::fmt::Debug for SimEnv {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SimEnv")
            .field("obs_dim", &self.obs_space.dim())
            .field("act_dim", &self.act_space.dim())
            .field("sub_steps", &self.sub_steps)
            .finish_non_exhaustive()
    }
}

impl SimEnv {
    /// Start building a `SimEnv`.
    #[must_use]
    pub fn builder(model: Arc<Model>) -> SimEnvBuilder {
        SimEnvBuilder {
            model,
            obs_space: None,
            act_space: None,
            reward_fn: None,
            done_fn: None,
            truncated_fn: None,
            on_reset_fn: None,
            sub_steps: 1,
        }
    }
}

impl Environment for SimEnv {
    fn observation_space(&self) -> &ObservationSpace {
        &self.obs_space
    }

    fn action_space(&self) -> &ActionSpace {
        &self.act_space
    }

    fn observe(&self) -> Tensor {
        self.obs_space.extract(&self.data)
    }

    fn step(&mut self, action: &Tensor) -> Result<StepResult, StepError> {
        // 1. Apply action
        self.act_space.apply(action, &mut self.data, &self.model);

        // 2. Sub-step with early termination
        for _ in 0..self.sub_steps {
            self.data.step(&self.model)?;
            if (self.done_fn)(&self.model, &self.data) {
                break;
            }
        }

        // 3–5. Evaluate reward, done, truncated
        let reward = (self.reward_fn)(&self.model, &self.data);
        let done = (self.done_fn)(&self.model, &self.data);
        let truncated = (self.truncated_fn)(&self.model, &self.data);

        // 6. Extract observation
        let observation = self.obs_space.extract(&self.data);

        // 7. Return
        Ok(StepResult {
            observation,
            reward,
            done,
            truncated,
        })
    }

    fn reset(&mut self) -> Result<Tensor, ResetError> {
        // 1. Standard Data reset
        self.data.reset(&self.model);

        // 2. Domain randomization hook
        if let Some(ref mut hook) = self.on_reset_fn {
            hook(&self.model, &mut self.data);
        }

        // 3. Recompute derived quantities
        self.data.forward(&self.model)?;

        // 4. Extract and return initial observation
        Ok(self.obs_space.extract(&self.data))
    }

    fn model(&self) -> &Model {
        &self.model
    }

    fn data(&self) -> &Data {
        &self.data
    }
}

impl SimEnv {
    /// Mutable access to the underlying `Data`.
    ///
    /// Prefer [`on_reset`](SimEnvBuilder::on_reset) for initial conditions
    /// and [`ActionSpace::apply`] for control inputs.  This escape hatch is
    /// for inspection, debugging, visualization, and advanced use cases
    /// (e.g., force injection between sub-steps) where you need direct
    /// state access.
    ///
    /// If you modify `qpos`, call `data.forward(model)` afterwards to
    /// recompute derived quantities (`xpos`, `xquat`, `sensordata`, etc.).
    pub const fn data_mut(&mut self) -> &mut Data {
        &mut self.data
    }
}

// ─── SimEnvBuilder ──────────────────────────────────────────────────────────

/// Builder for [`SimEnv`].
pub struct SimEnvBuilder {
    model: Arc<Model>,
    obs_space: Option<ObservationSpace>,
    act_space: Option<ActionSpace>,
    reward_fn: Option<Box<dyn Fn(&Model, &Data) -> f64>>,
    done_fn: Option<Box<dyn Fn(&Model, &Data) -> bool>>,
    truncated_fn: Option<Box<dyn Fn(&Model, &Data) -> bool>>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data)>>,
    sub_steps: usize,
}

impl SimEnvBuilder {
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

    /// Set the reward function.
    #[must_use]
    pub fn reward(mut self, f: impl Fn(&Model, &Data) -> f64 + 'static) -> Self {
        self.reward_fn = Some(Box::new(f));
        self
    }

    /// Set the done (terminal) function.
    #[must_use]
    pub fn done(mut self, f: impl Fn(&Model, &Data) -> bool + 'static) -> Self {
        self.done_fn = Some(Box::new(f));
        self
    }

    /// Set the truncated function.
    #[must_use]
    pub fn truncated(mut self, f: impl Fn(&Model, &Data) -> bool + 'static) -> Self {
        self.truncated_fn = Some(Box::new(f));
        self
    }

    /// Set the number of physics sub-steps per action.
    ///
    /// Default is 1.
    #[must_use]
    pub const fn sub_steps(mut self, n: usize) -> Self {
        self.sub_steps = n;
        self
    }

    /// Hook called after `Data::reset()`.
    ///
    /// Use for domain randomization: randomize initial joint angles, object
    /// positions, etc.  The closure is `FnMut` so it can own and advance
    /// its own RNG.
    #[must_use]
    pub fn on_reset(mut self, f: impl FnMut(&Model, &mut Data) + 'static) -> Self {
        self.on_reset_fn = Some(Box::new(f));
        self
    }

    /// Validate and build the [`SimEnv`].
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if a required field is missing or `sub_steps` is 0.
    pub fn build(self) -> Result<SimEnv, EnvError> {
        let obs_space = self.obs_space.ok_or(EnvError::MissingField {
            field: "observation_space",
        })?;
        let act_space = self.act_space.ok_or(EnvError::MissingField {
            field: "action_space",
        })?;
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

        let data = self.model.make_data();

        Ok(SimEnv {
            model: self.model,
            data,
            obs_space,
            act_space,
            reward_fn,
            done_fn,
            truncated_fn,
            on_reset_fn: self.on_reset_fn,
            sub_steps: self.sub_steps,
        })
    }
}

// ─── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::cast_possible_truncation,
    clippy::let_underscore_must_use
)]
mod tests {
    use super::*;

    /// Build a pendulum `SimEnv` with standard reward/done/truncated.
    fn make_pendulum_env(sub_steps: usize) -> SimEnv {
        let xml = r#"
        <mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
          <sensor>
            <jointpos joint="hinge" name="angle"/>
          </sensor>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));

        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, data| -data.qpos[0].powi(2))
            .done(|_m, data| data.qpos[0].abs() > 3.0)
            .truncated(|_m, data| data.time > 1.0)
            .sub_steps(sub_steps)
            .build()
            .unwrap()
    }

    // ── episode lifecycle ────────────────────────────────────────────────

    #[test]
    fn reset_returns_initial_observation() {
        let mut env = make_pendulum_env(1);
        let obs = env.reset().unwrap();
        assert_eq!(obs.shape(), &[2]); // nq=1 + nv=1
    }

    #[test]
    fn step_returns_step_result() {
        let mut env = make_pendulum_env(1);
        let _obs = env.reset().unwrap();
        let action = Tensor::zeros(&[1]);
        let result = env.step(&action).unwrap();
        assert_eq!(result.observation.shape(), &[2]);
        assert!(!result.done);
        assert!(!result.truncated);
    }

    #[test]
    fn step_advances_time() {
        let mut env = make_pendulum_env(1);
        env.reset().unwrap();
        let action = Tensor::zeros(&[1]);
        env.step(&action).unwrap();
        assert!(env.data().time > 0.0);
    }

    // ── reward / done / truncated ────────────────────────────────────────

    #[test]
    fn reward_is_correct() {
        let mut env = make_pendulum_env(1);
        env.reset().unwrap();
        // At rest, qpos[0] = 0, reward = -0^2 = 0
        let result = env.step(&Tensor::zeros(&[1])).unwrap();
        // After one step with zero torque, qpos is near zero → reward near 0
        assert!(result.reward.abs() < 0.1);
    }

    #[test]
    fn done_triggers_at_threshold() {
        let mut env = make_pendulum_env(1);
        env.reset().unwrap();
        // Manually set qpos beyond threshold
        env.data.qpos[0] = 4.0;
        let result = env.step(&Tensor::zeros(&[1])).unwrap();
        assert!(result.done);
    }

    #[test]
    fn truncated_triggers_at_time_limit() {
        let mut env = make_pendulum_env(1);
        env.reset().unwrap();
        // Manually set time beyond limit
        env.data.time = 2.0;
        let result = env.step(&Tensor::zeros(&[1])).unwrap();
        assert!(result.truncated);
    }

    // ── sub-stepping ─────────────────────────────────────────────────────

    #[test]
    fn sub_steps_advance_time_correctly() {
        let mut env1 = make_pendulum_env(1);
        let mut env5 = make_pendulum_env(5);
        env1.reset().unwrap();
        env5.reset().unwrap();

        let action = Tensor::zeros(&[1]);

        // 5 individual steps
        for _ in 0..5 {
            env1.step(&action).unwrap();
        }
        // 1 step with sub_steps=5
        env5.step(&action).unwrap();

        // Time should match (both did 5 physics steps)
        assert!((env1.data().time - env5.data().time).abs() < 1e-12);
    }

    #[test]
    fn sub_steps_produce_same_final_state() {
        let mut env1 = make_pendulum_env(1);
        let mut env5 = make_pendulum_env(5);
        env1.reset().unwrap();
        env5.reset().unwrap();

        let action = Tensor::from_slice(&[0.5_f32], &[1]);

        // 5 individual steps (action applied each time)
        for _ in 0..5 {
            env1.step(&action).unwrap();
        }
        // 1 step with sub_steps=5 (action applied once)
        env5.step(&action).unwrap();

        // qpos should match: both apply same ctrl then step 5 times
        // (action is applied once per env.step() call, so env1 re-applies
        // each iteration — but ctrl stays the same so result is identical)
        assert!(
            (env1.data().qpos[0] - env5.data().qpos[0]).abs() < 1e-12,
            "env1.qpos={}, env5.qpos={}",
            env1.data().qpos[0],
            env5.data().qpos[0]
        );
    }

    // ── early termination during sub-steps ───────────────────────────────

    #[test]
    fn early_termination_stops_sub_stepping() {
        // done_fn triggers at qpos > 3.0
        // We'll push the pendulum hard and use many sub-steps
        let mut env_many = make_pendulum_env(100);
        env_many.reset().unwrap();

        // Set qpos near threshold
        env_many.data.qpos[0] = 2.9;
        // Apply large torque to push past threshold
        let action = Tensor::from_slice(&[100.0_f32], &[1]);
        let result = env_many.step(&action).unwrap();

        // Should be done (crossed threshold during sub-steps)
        assert!(result.done);

        // Time should be less than 100 * 0.01 = 1.0 (early exit)
        // It should have stopped within a few sub-steps
        assert!(
            env_many.data().time < 0.5,
            "time={}, expected early termination",
            env_many.data().time
        );
    }

    // ── on_reset hook ────────────────────────────────────────────────────

    #[test]
    fn on_reset_hook_modifies_state() {
        let xml = r#"
        <mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));

        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let mut env = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .on_reset(|_m, data| {
                data.qpos[0] = 1.0; // Set initial angle
            })
            .build()
            .unwrap();

        let obs = env.reset().unwrap();
        // The observation should reflect qpos=1.0 (after forward() recomputed)
        assert!((obs.as_slice()[0] - 1.0_f64 as f32).abs() < 1e-5);
    }

    #[test]
    fn on_reset_forward_recomputes_derived() {
        let xml = r#"
        <mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body name="pendulum" pos="0 0 1">
              <joint name="hinge" type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="hinge" name="motor"/>
          </actuator>
          <sensor>
            <jointpos joint="hinge" name="angle"/>
          </sensor>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));

        // Observe sensordata (a derived quantity computed by forward())
        let obs_space = ObservationSpace::builder()
            .sensor("angle")
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let mut env = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .on_reset(|_m, data| {
                data.qpos[0] = 1.0; // set initial angle
            })
            .build()
            .unwrap();

        let obs = env.reset().unwrap();
        // Sensordata should reflect qpos=1.0 after forward() recomputed it
        assert!(
            (obs.as_slice()[0] - 1.0_f64 as f32).abs() < 1e-5,
            "sensor={}, expected ~1.0",
            obs.as_slice()[0]
        );
    }

    // ── observe() ────────────────────────────────────────────────────────

    #[test]
    fn observe_matches_extract() {
        let mut env = make_pendulum_env(1);
        env.reset().unwrap();
        env.step(&Tensor::zeros(&[1])).unwrap();

        let obs1 = env.observe();
        let obs2 = env.observation_space().extract(env.data());
        assert_eq!(obs1, obs2);
    }

    // ── model() / data() accessors ───────────────────────────────────────

    #[test]
    fn model_and_data_accessible() {
        let env = make_pendulum_env(1);
        assert_eq!(env.model().nq, 1);
        assert_eq!(env.data().qpos.len(), 1);
    }

    // ── builder validation ───────────────────────────────────────────────

    #[test]
    fn builder_missing_obs_space() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let err = SimEnv::builder(model)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            EnvError::MissingField {
                field: "observation_space"
            }
        ));
    }

    #[test]
    fn builder_missing_action_space() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();

        let err = SimEnv::builder(model)
            .observation_space(obs_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            EnvError::MissingField {
                field: "action_space"
            }
        ));
    }

    #[test]
    fn builder_missing_reward() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let err = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .build()
            .unwrap_err();
        assert!(matches!(err, EnvError::MissingField { field: "reward" }));
    }

    #[test]
    fn builder_missing_done() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let err = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .truncated(|_m, _d| false)
            .build()
            .unwrap_err();
        assert!(matches!(err, EnvError::MissingField { field: "done" }));
    }

    #[test]
    fn builder_missing_truncated() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let err = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .build()
            .unwrap_err();
        assert!(matches!(err, EnvError::MissingField { field: "truncated" }));
    }

    #[test]
    fn builder_zero_sub_steps() {
        let xml = r#"
        <mujoco>
          <worldbody>
            <body><joint name="j" type="hinge"/><geom size="0.1"/></body>
          </worldbody>
          <actuator><motor joint="j"/></actuator>
        </mujoco>
        "#;
        let model = Arc::new(sim_mjcf::load_model(xml).expect("valid MJCF"));
        let obs_space = ObservationSpace::builder()
            .all_qpos()
            .build(&model)
            .unwrap();
        let act_space = ActionSpace::builder().all_ctrl().build(&model).unwrap();

        let err = SimEnv::builder(model)
            .observation_space(obs_space)
            .action_space(act_space)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .sub_steps(0)
            .build()
            .unwrap_err();
        assert!(matches!(err, EnvError::ZeroSubSteps));
    }
}
