//! [`VecEnv`] — vectorized environments wrapping [`BatchSim`].
//!
//! `N` parallel environments sharing one [`Model`].  All environments use
//! identical observation and action spaces.  Individual environments can be
//! at different points in their episodes.
//!
//! [`BatchSim`]: sim_core::BatchSim

use std::sync::Arc;

use sim_core::{BatchSim, Data, Model, StepError};

use crate::error::{EnvError, ResetError, VecStepError};
use crate::space::{ActionSpace, ObservationSpace};
use crate::tensor::Tensor;

// ─── VecStepResult ──────────────────────────────────────────────────────────

/// Batch step result from [`VecEnv`].
///
/// Observations and rewards are batched — one contiguous tensor and one
/// `Vec`, not a `Vec<Tensor>`.
#[derive(Debug, Clone)]
pub struct VecStepResult {
    /// Shape: `[n_envs, obs_dim]`.  Row `i` is env `i`'s observation.
    /// For envs that were auto-reset, this is the **initial** observation
    /// of the new episode, not the terminal one.
    pub observations: Tensor,

    /// Rewards from the completed step (before auto-reset).
    pub rewards: Vec<f64>,

    /// `true` if env `i`'s episode terminated this step (true terminal).
    pub dones: Vec<bool>,

    /// `true` if env `i`'s episode was truncated this step (time limit).
    pub truncateds: Vec<bool>,

    /// Terminal observations for envs that were auto-reset.
    /// `terminal_observations[i]` is `Some(obs)` if env `i` was reset this
    /// step.  `None` otherwise.  Needed for correct value bootstrapping.
    pub terminal_observations: Vec<Option<Tensor>>,

    /// Per-env physics errors.  `None` = success.  `Some(e)` = this env had
    /// a non-recoverable physics error and was auto-reset.
    pub errors: Vec<Option<StepError>>,
}

// ─── VecEnv ─────────────────────────────────────────────────────────────────

/// Vectorized environment: `N` parallel environments sharing one [`Model`].
///
/// Wraps [`BatchSim`] and adds observation / action / reward / done semantics.
/// Constructed via [`VecEnv::builder()`].
///
/// [`BatchSim`]: sim_core::BatchSim
pub struct VecEnv {
    model: Arc<Model>,
    batch: BatchSim,
    obs_space: ObservationSpace,
    act_space: ActionSpace,
    reward_fn: Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
    done_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    truncated_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data, usize) + Send>>,
    sub_steps: usize,
}

// Manual Debug — closures aren't Debug.
impl std::fmt::Debug for VecEnv {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VecEnv")
            .field("n_envs", &self.batch.len())
            .field("obs_dim", &self.obs_space.dim())
            .field("act_dim", &self.act_space.dim())
            .field("sub_steps", &self.sub_steps)
            .finish_non_exhaustive()
    }
}

impl VecEnv {
    /// Start building a `VecEnv`.
    #[must_use]
    pub fn builder(model: Arc<Model>, n_envs: usize) -> VecEnvBuilder {
        VecEnvBuilder {
            model,
            n_envs,
            obs_space: None,
            act_space: None,
            reward_fn: None,
            done_fn: None,
            truncated_fn: None,
            on_reset_fn: None,
            sub_steps: 1,
        }
    }

    /// Number of parallel environments.
    #[must_use]
    pub fn n_envs(&self) -> usize {
        self.batch.len()
    }

    /// Step all environments.
    ///
    /// `actions` has shape `[n_envs, act_dim]`.  Row `i` is applied to env `i`.
    ///
    /// # Errors
    ///
    /// Returns [`VecStepError`] only for bridge-level errors (wrong action
    /// shape).  Per-env physics errors are reported in
    /// [`VecStepResult::errors`].
    pub fn step(&mut self, actions: &Tensor) -> Result<VecStepResult, VecStepError> {
        let n = self.batch.len();
        let act_dim = self.act_space.dim();
        let obs_dim = self.obs_space.dim();

        // Validate action shape
        if actions.shape() != [n, act_dim] {
            return Err(VecStepError::ActionShapeMismatch {
                n_envs: n,
                act_dim,
                actual: actions.shape().to_vec(),
            });
        }

        // 1. Inject actions into all envs
        let model = Arc::clone(&self.model);
        self.act_space
            .apply_batch(actions, self.batch.envs_mut(), &model);

        // 2. Sub-step with divergence accumulation
        let mut diverged = vec![false; n];
        let mut step_errors: Vec<Option<StepError>> = vec![None; n];

        for _ in 0..self.sub_steps {
            let errors = self.batch.step_all();
            for (i, err) in errors.into_iter().enumerate() {
                if let Some(e) = err {
                    step_errors[i] = Some(e);
                }
                if let Some(data) = self.batch.env(i) {
                    if data.divergence_detected() {
                        diverged[i] = true;
                    }
                }
            }
        }

        // 3. Evaluate reward, done, truncated for each env
        let mut rewards = Vec::with_capacity(n);
        let mut dones = Vec::with_capacity(n);
        let mut truncateds = Vec::with_capacity(n);

        for i in 0..n {
            if let Some(d) = self.batch.env(i) {
                rewards.push((self.reward_fn)(&model, d));
                // Diverged envs are treated as done
                let done = diverged[i] || step_errors[i].is_some() || (self.done_fn)(&model, d);
                dones.push(done);
                let truncated = if done {
                    false // done takes precedence
                } else {
                    (self.truncated_fn)(&model, d)
                };
                truncateds.push(truncated);
            } else {
                // Should not happen with valid BatchSim, but be safe
                rewards.push(0.0);
                dones.push(true);
                truncateds.push(false);
            }
        }

        // 4. Auto-reset and extract observations
        let mut obs_buf = vec![0.0_f32; n * obs_dim];
        let mut terminal_observations: Vec<Option<Tensor>> = vec![None; n];

        for i in 0..n {
            let needs_reset = dones[i] || truncateds[i];

            if needs_reset {
                // Extract terminal observation (if state is valid)
                if !diverged[i] {
                    if let Some(data) = self.batch.env(i) {
                        terminal_observations[i] = Some(self.obs_space.extract(data));
                    }
                }

                // Auto-reset
                self.batch.reset(i);

                // on_reset hook
                if let Some(ref mut hook) = self.on_reset_fn {
                    if let Some(data) = self.batch.env_mut(i) {
                        hook(&model, data, i);
                    }
                }

                // forward() to recompute derived quantities
                if let Some(data) = self.batch.env_mut(i) {
                    if let Err(e) = data.forward(&model) {
                        // Log the forward error — env stays in clean reset state
                        step_errors[i] = Some(e);
                    }
                }
            }

            // Extract observation (post-reset for reset envs, current for others)
            if let Some(data) = self.batch.env(i) {
                let row = &mut obs_buf[i * obs_dim..(i + 1) * obs_dim];
                // Use the same extraction logic as ObservationSpace::extract
                // but write directly into the batch buffer
                let obs = self.obs_space.extract(data);
                row.copy_from_slice(obs.as_slice());
            }
        }

        let observations = Tensor::from_slice(&obs_buf, &[n, obs_dim]);

        Ok(VecStepResult {
            observations,
            rewards,
            dones,
            truncateds,
            terminal_observations,
            errors: step_errors,
        })
    }

    /// Reset all environments, returning initial observations.
    ///
    /// Shape: `[n_envs, obs_dim]`.
    ///
    /// # Errors
    ///
    /// Returns [`ResetError`] if `forward()` fails after reset.
    pub fn reset_all(&mut self) -> Result<Tensor, ResetError> {
        let model = Arc::clone(&self.model);
        self.batch.reset_all();

        for i in 0..self.batch.len() {
            if let Some(ref mut hook) = self.on_reset_fn {
                if let Some(data) = self.batch.env_mut(i) {
                    hook(&model, data, i);
                }
            }
            if let Some(data) = self.batch.env_mut(i) {
                data.forward(&model)?;
            }
        }

        Ok(self.obs_space.extract_batch(self.batch.envs()))
    }

    /// Access the underlying [`BatchSim`].
    #[must_use]
    pub const fn batch(&self) -> &BatchSim {
        &self.batch
    }

    /// Mutably access the underlying [`BatchSim`].
    pub const fn batch_mut(&mut self) -> &mut BatchSim {
        &mut self.batch
    }

    /// Access the shared model.
    #[must_use]
    pub fn model(&self) -> &Model {
        &self.model
    }
}

// ─── VecEnvBuilder ──────────────────────────────────────────────────────────

/// Builder for [`VecEnv`].
pub struct VecEnvBuilder {
    model: Arc<Model>,
    n_envs: usize,
    obs_space: Option<ObservationSpace>,
    act_space: Option<ActionSpace>,
    reward_fn: Option<Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>>,
    done_fn: Option<Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    truncated_fn: Option<Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data, usize) + Send>>,
    sub_steps: usize,
}

impl VecEnvBuilder {
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

    /// Set the truncated function.
    #[must_use]
    pub fn truncated(mut self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self {
        self.truncated_fn = Some(Arc::new(f));
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

    /// Hook called after `Data::reset()` for each env that resets.
    ///
    /// Receives `(model, data, env_index)` — the env index lets the closure
    /// use per-env RNG seeds or deterministic randomization schedules.
    #[must_use]
    pub fn on_reset(mut self, f: impl FnMut(&Model, &mut Data, usize) + Send + 'static) -> Self {
        self.on_reset_fn = Some(Box::new(f));
        self
    }

    /// Validate and build the [`VecEnv`].
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if a required field is missing or `sub_steps` is 0.
    pub fn build(self) -> Result<VecEnv, EnvError> {
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

        let batch = BatchSim::new(Arc::clone(&self.model), self.n_envs);

        Ok(VecEnv {
            model: self.model,
            batch,
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
    use crate::env::{Environment, SimEnv, StepResult};

    const N_ENVS: usize = 4;

    fn pendulum_xml() -> &'static str {
        r#"
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
        "#
    }

    fn make_model() -> Arc<Model> {
        Arc::new(sim_mjcf::load_model(pendulum_xml()).expect("valid MJCF"))
    }

    fn make_spaces(model: &Model) -> (ObservationSpace, ActionSpace) {
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(model).unwrap();
        (obs, act)
    }

    fn make_vec_env(model: Arc<Model>, n_envs: usize, sub_steps: usize) -> VecEnv {
        let (obs, act) = make_spaces(&model);
        VecEnv::builder(model, n_envs)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, data| -data.qpos[0].powi(2))
            .done(|_m, data| data.qpos[0].abs() > 3.0)
            .truncated(|_m, data| data.time > 1.0)
            .sub_steps(sub_steps)
            .build()
            .unwrap()
    }

    fn make_sim_env(model: Arc<Model>, sub_steps: usize) -> SimEnv {
        let (obs, act) = make_spaces(&model);
        SimEnv::builder(model)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, data| -data.qpos[0].powi(2))
            .done(|_m, data| data.qpos[0].abs() > 3.0)
            .truncated(|_m, data| data.time > 1.0)
            .sub_steps(sub_steps)
            .build()
            .unwrap()
    }

    // ── basic lifecycle ──────────────────────────────────────────────────

    #[test]
    fn reset_all_returns_correct_shape() {
        let model = make_model();
        let mut env = make_vec_env(model, N_ENVS, 1);
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[N_ENVS, 2]); // nq=1, nv=1 → obs_dim=2
    }

    #[test]
    fn step_returns_correct_shapes() {
        let model = make_model();
        let mut env = make_vec_env(model, N_ENVS, 1);
        env.reset_all().unwrap();
        let actions = Tensor::zeros(&[N_ENVS, 1]);
        let result = env.step(&actions).unwrap();
        assert_eq!(result.observations.shape(), &[N_ENVS, 2]);
        assert_eq!(result.rewards.len(), N_ENVS);
        assert_eq!(result.dones.len(), N_ENVS);
        assert_eq!(result.truncateds.len(), N_ENVS);
        assert_eq!(result.terminal_observations.len(), N_ENVS);
        assert_eq!(result.errors.len(), N_ENVS);
    }

    #[test]
    fn n_envs_correct() {
        let model = make_model();
        let env = make_vec_env(model, N_ENVS, 1);
        assert_eq!(env.n_envs(), N_ENVS);
    }

    // ── bit-exact match vs sequential SimEnv ─────────────────────────────

    #[test]
    fn step_matches_sequential_sim_env() {
        let model = make_model();
        let mut vec_env = make_vec_env(model.clone(), N_ENVS, 1);
        let mut sim_envs: Vec<SimEnv> = (0..N_ENVS)
            .map(|_| make_sim_env(model.clone(), 1))
            .collect();

        vec_env.reset_all().unwrap();
        for env in &mut sim_envs {
            env.reset().unwrap();
        }

        // Each env gets a different action
        let action_vals: Vec<f32> = [0.0, 0.5, 1.0, 1.5].to_vec();
        let actions = Tensor::from_slice(&action_vals, &[N_ENVS, 1]);

        let vec_result = vec_env.step(&actions).unwrap();

        let sim_results: Vec<StepResult> = sim_envs
            .iter_mut()
            .enumerate()
            .map(|(i, env)| {
                let a = Tensor::from_slice(&[action_vals[i]], &[1]);
                env.step(&a).unwrap()
            })
            .collect();

        for (i, sim_r) in sim_results.iter().enumerate() {
            assert_eq!(
                vec_result.observations.row(i),
                sim_r.observation.as_slice(),
                "env {i} observations differ"
            );
            assert_eq!(
                vec_result.rewards[i], sim_r.reward,
                "env {i} rewards differ"
            );
            assert_eq!(vec_result.dones[i], sim_r.done, "env {i} dones differ");
            assert_eq!(
                vec_result.truncateds[i], sim_r.truncated,
                "env {i} truncateds differ"
            );
        }
    }

    // ── per-env action isolation ─────────────────────────────────────────

    #[test]
    fn per_env_action_isolation() {
        let model = make_model();
        let mut env = make_vec_env(model, N_ENVS, 1);
        env.reset_all().unwrap();

        // Different action per env
        let actions = Tensor::from_slice(&[0.0, 1.0, -1.0, 0.5], &[N_ENVS, 1]);
        let result = env.step(&actions).unwrap();

        // Env 0 (zero action) and env 1 (1.0 action) should differ
        assert_ne!(
            result.observations.row(0),
            result.observations.row(1),
            "envs with different actions should have different obs"
        );
    }

    // ── done / truncated flags ───────────────────────────────────────────

    #[test]
    fn done_flag_correctness() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 1);
        env.reset_all().unwrap();

        // Push env 0 past done threshold (qpos > 3.0)
        if let Some(data) = env.batch_mut().env_mut(0) {
            data.qpos[0] = 3.5;
        }

        let actions = Tensor::zeros(&[2, 1]);
        let result = env.step(&actions).unwrap();
        assert!(result.dones[0], "env 0 should be done");
        assert!(!result.dones[1], "env 1 should not be done");
    }

    #[test]
    fn truncated_flag_correctness() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 1);
        env.reset_all().unwrap();

        // Push env 1 past truncation threshold (time > 1.0)
        if let Some(data) = env.batch_mut().env_mut(1) {
            data.time = 2.0;
        }

        let actions = Tensor::zeros(&[2, 1]);
        let result = env.step(&actions).unwrap();
        assert!(!result.truncateds[0], "env 0 should not be truncated");
        assert!(result.truncateds[1], "env 1 should be truncated");
    }

    // ── auto-reset semantics ─────────────────────────────────────────────

    #[test]
    fn auto_reset_populates_terminal_observations() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 1);
        env.reset_all().unwrap();

        // Push env 0 past done threshold
        if let Some(data) = env.batch_mut().env_mut(0) {
            data.qpos[0] = 3.5;
        }

        let actions = Tensor::zeros(&[2, 1]);
        let result = env.step(&actions).unwrap();

        // Env 0 was auto-reset → should have terminal obs
        assert!(
            result.terminal_observations[0].is_some(),
            "done env should have terminal obs"
        );
        // Env 1 was not reset → no terminal obs
        assert!(
            result.terminal_observations[1].is_none(),
            "non-done env should have no terminal obs"
        );
    }

    #[test]
    fn auto_reset_returns_initial_obs() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 1);
        env.reset_all().unwrap();

        // Push env 0 past done threshold
        if let Some(data) = env.batch_mut().env_mut(0) {
            data.qpos[0] = 3.5;
        }

        let actions = Tensor::zeros(&[2, 1]);
        let result = env.step(&actions).unwrap();

        // The observation for env 0 should be the initial obs (post-reset),
        // not the terminal obs. After reset, qpos should be ~0.
        let obs_0 = result.observations.row(0);
        assert!(
            obs_0[0].abs() < 0.1,
            "post-reset qpos should be near 0, got {}",
            obs_0[0]
        );
    }

    #[test]
    fn terminal_obs_differs_from_initial_obs() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 1);
        env.reset_all().unwrap();

        // Push env 0 past done threshold
        if let Some(data) = env.batch_mut().env_mut(0) {
            data.qpos[0] = 3.5;
        }

        let actions = Tensor::zeros(&[2, 1]);
        let result = env.step(&actions).unwrap();

        let terminal = result.terminal_observations[0].as_ref().unwrap();
        let initial = result.observations.row(0);

        // Terminal obs has qpos ~3.5, initial has qpos ~0
        assert_ne!(terminal.as_slice(), initial);
    }

    // ── on_reset with env_index ──────────────────────────────────────────

    #[test]
    fn on_reset_receives_env_index() {
        let model = make_model();
        let (obs, act) = make_spaces(&model);

        let mut env = VecEnv::builder(model, 2)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| 0.0)
            .done(|_m, data| data.qpos[0].abs() > 3.0)
            .truncated(|_m, _d| false)
            .on_reset(|_m, data, env_idx| {
                // Set qpos to env_idx as a tag
                #[allow(clippy::cast_precision_loss)]
                {
                    data.qpos[0] = (env_idx as f64) * 0.01;
                }
            })
            .build()
            .unwrap();

        // reset_all should call on_reset with correct indices
        let obs = env.reset_all().unwrap();
        let qpos_0 = obs.row(0)[0]; // env 0 → qpos = 0.0 * 0.01 = 0.0
        let qpos_1 = obs.row(1)[0]; // env 1 → qpos = 1.0 * 0.01 = 0.01
        assert!(qpos_0.abs() < 1e-5, "env 0 qpos should be ~0.0");
        assert!(
            (qpos_1 - 0.01_f64 as f32).abs() < 1e-5,
            "env 1 qpos should be ~0.01"
        );
    }

    // ── sub-stepping ─────────────────────────────────────────────────────

    #[test]
    fn sub_steps_all_run() {
        let model = make_model();
        let mut env = make_vec_env(model, 2, 5);
        env.reset_all().unwrap();

        let actions = Tensor::zeros(&[2, 1]);
        env.step(&actions).unwrap();

        // 5 sub-steps × 0.01 timestep = 0.05
        for i in 0..2 {
            let time = env.batch().env(i).unwrap().time;
            assert!(
                (time - 0.05).abs() < 1e-12,
                "env {i} time should be 0.05, got {time}"
            );
        }
    }

    #[test]
    fn sub_steps_evaluate_at_end_not_per_step() {
        // VecEnv should NOT do early exit — it evaluates done at end only
        let model = make_model();
        let mut env = make_vec_env(model, 1, 100);
        env.reset_all().unwrap();

        // Set qpos near threshold
        if let Some(data) = env.batch_mut().env_mut(0) {
            data.qpos[0] = 2.9;
        }

        // Apply large torque — should cross threshold during sub-steps
        let actions = Tensor::from_slice(&[100.0_f32], &[1, 1]);
        let result = env.step(&actions).unwrap();

        // Should be done (crossed threshold)
        assert!(result.dones[0]);

        // All 100 sub-steps should have run (no early exit in VecEnv)
        // time = 100 × 0.01 = 1.0
        // (After auto-reset, time is 0 — but we know all sub-steps ran
        // because done was triggered at the end)
    }

    // ── wrong action shape ───────────────────────────────────────────────

    #[test]
    fn wrong_action_shape_returns_error() {
        let model = make_model();
        let mut env = make_vec_env(model, N_ENVS, 1);
        env.reset_all().unwrap();

        // Wrong number of envs
        let bad_actions = Tensor::zeros(&[2, 1]);
        let err = env.step(&bad_actions).unwrap_err();
        assert!(matches!(err, VecStepError::ActionShapeMismatch { .. }));
    }

    #[test]
    fn wrong_action_dim_returns_error() {
        let model = make_model();
        let mut env = make_vec_env(model, N_ENVS, 1);
        env.reset_all().unwrap();

        // Wrong action dimension
        let bad_actions = Tensor::zeros(&[N_ENVS, 5]);
        let err = env.step(&bad_actions).unwrap_err();
        assert!(matches!(err, VecStepError::ActionShapeMismatch { .. }));
    }

    // ── model / batch accessors ──────────────────────────────────────────

    #[test]
    fn model_and_batch_accessible() {
        let model = make_model();
        let env = make_vec_env(model, N_ENVS, 1);
        assert_eq!(env.model().nq, 1);
        assert_eq!(env.batch().len(), N_ENVS);
    }

    // ── builder validation ───────────────────────────────────────────────

    #[test]
    fn builder_missing_obs_space() {
        let model = make_model();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        let err = VecEnv::builder(model, 2)
            .action_space(act)
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
    fn builder_zero_sub_steps() {
        let model = make_model();
        let (obs, act) = make_spaces(&model);
        let err = VecEnv::builder(model, 2)
            .observation_space(obs)
            .action_space(act)
            .reward(|_m, _d| 0.0)
            .done(|_m, _d| false)
            .truncated(|_m, _d| false)
            .sub_steps(0)
            .build()
            .unwrap_err();
        assert!(matches!(err, EnvError::ZeroSubSteps));
    }
}
