//! Task configuration — the surface every algorithm binds against.
//!
//! A [`TaskConfig`] is a complete task definition — environment metadata plus
//! a [`VecEnv`] factory.  Algorithms use the metadata (dimensions, obs scales)
//! to construct their networks; the [`Competition`](crate::Competition) runner
//! calls [`build_vec_env`](TaskConfig::build_vec_env) to create fresh
//! environments for each run.
//!
//! ## Custom tasks
//!
//! Use [`TaskConfig::builder()`] to define new tasks from a pre-parsed
//! [`Model`], or [`TaskConfig::from_build_fn()`] for custom seeded
//! closures (stochastic-physics tasks).
//!
//! ## Stock tasks
//!
//! Stock MJCF-backed reaching arms (`reaching_2dof`, `reaching_6dof`,
//! `obstacle_reaching_6dof`) live in `sim-rl::tasks` so the algorithm
//! chassis stays free of the `sim-mjcf` → `mesh-io` → `zip` → `zstd-sys`
//! compile chain. Depend on `sim-rl` to get them.

use std::sync::Arc;

use sim_core::{Data, Model};

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
/// Custom tasks: use [`TaskConfig::builder()`] or [`TaskConfig::from_build_fn()`].
/// Stock MJCF reaching arms are provided by `sim-rl::tasks`.
pub struct TaskConfig {
    name: String,
    obs_dim: usize,
    act_dim: usize,
    obs_scale: Vec<f64>,
    // Seeded closure: accepts `(n_envs, seed)`.  Deterministic-physics stock
    // tasks ignore the seed via `_seed: u64`; stochastic-physics tasks
    // (e.g. those carrying a `LangevinThermostat`) consume it as their
    // per-replicate `master_seed` so `Competition::run_replicates` can vary
    // the physics noise sequence across replicates.  The seed parameter is
    // non-optional at the signature level so the "every stochastic task gets
    // a fresh per-replicate seed" invariant is enforced at compile time at
    // every call site, not by convention — see Ch 42 §2.4.
    build_fn: Arc<dyn Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync>,
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
    /// [`Competition::run_replicates`](crate::Competition::run_replicates)
    /// calls this once per `(task, builder, seed)` triple, threading the
    /// per-replicate seed through so stochastic-physics tasks can vary
    /// their physics noise sequence across replicates.  Deterministic stock
    /// tasks ignore the `seed` parameter; pass `0` from deterministic call
    /// sites that do not carry a replicate seed.
    ///
    /// # Errors
    ///
    /// Returns [`EnvError`] if the internal `VecEnv` builder fails (should not
    /// happen for stock tasks).
    pub fn build_vec_env(&self, n_envs: usize, seed: u64) -> Result<VecEnv, EnvError> {
        (self.build_fn)(n_envs, seed)
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

    /// Construct a [`TaskConfig`] from a custom seeded `build_fn` closure.
    ///
    /// Use this when the task carries stochastic physics that must vary
    /// per replicate — the closure receives the per-replicate seed from
    /// [`Competition::run_replicates`](crate::Competition::run_replicates)
    /// and threads it into e.g. a `LangevinThermostat`'s `master_seed`.
    ///
    /// Stock deterministic tasks should use [`TaskConfig::builder()`]
    /// instead; that path synthesizes its own `build_fn` and ignores
    /// the seed via `_seed: u64`.  This constructor is the
    /// custom-stochastic-task surface deferred from PR 2a in
    /// [Ch 42](../../../../docs/studies/ml_chassis_refactor/src/42-pr-3-sim-opt-rematch.md)
    /// §2 sub-decision (a).
    pub fn from_build_fn<F>(
        name: impl Into<String>,
        obs_dim: usize,
        act_dim: usize,
        obs_scale: Vec<f64>,
        build_fn: F,
    ) -> Self
    where
        F: Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync + 'static,
    {
        Self {
            name: name.into(),
            obs_dim,
            act_dim,
            obs_scale,
            build_fn: Arc::new(build_fn),
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

        // Custom tasks built via `TaskConfigBuilder` inherit the seeded-
        // closure shape.  The `_seed` parameter is accept-and-ignore for
        // deterministic custom tasks; a future custom-builder API surface
        // for stochastic tasks would consume the seed here.
        let build_fn = Arc::new(
            move |n_envs: usize, _seed: u64| -> Result<VecEnv, EnvError> {
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
            },
        );

        Ok(TaskConfig {
            name: self.name,
            obs_dim,
            act_dim,
            obs_scale,
            build_fn,
        })
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

    fn test_model() -> Arc<Model> {
        Arc::new(sim_core::test_fixtures::reaching_2dof())
    }

    // ── TaskConfig traits ─────────────────────────────────────────────

    #[test]
    fn task_config_debug_clone() {
        // from_build_fn gives a TaskConfig without needing MJCF / VecEnv —
        // build_fn is never called in this test.
        let task: TaskConfig =
            TaskConfig::from_build_fn("debug-clone", 4, 2, vec![1.0; 4], |_n_envs, _seed| {
                Err(EnvError::ZeroSubSteps)
            });
        let task2 = task.clone();
        assert_eq!(task2.name(), "debug-clone");
        assert!(!format!("{task:?}").is_empty());
    }

    #[test]
    fn task_config_is_send_sync() {
        fn require<T: Send + Sync>() {}
        require::<TaskConfig>();
    }

    // ── Builder ───────────────────────────────────────────────────────

    #[test]
    fn builder_missing_obs_space() {
        let model = test_model();
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
        let model = test_model();
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
        let model = test_model();
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

        let mut env = task.build_vec_env(2, 0).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[2, 4]);
    }

    // ── from_build_fn (custom seeded constructor) ─────────────────────

    /// Helper for `from_build_fn` tests: construct a trivial 2-DOF
    /// `VecEnv` for the given `n_envs`, ignoring the seed.  The seed
    /// is recorded into an outer channel by the closure that wraps
    /// this helper, not by the helper itself.
    fn build_trivial_2dof_vec_env(n_envs: usize) -> Result<VecEnv, EnvError> {
        let model = test_model();
        let obs = ObservationSpace::builder()
            .all_qpos()
            .all_qvel()
            .build(&model)
            .unwrap();
        let act = ActionSpace::builder().all_ctrl().build(&model).unwrap();
        VecEnv::builder(model, n_envs)
            .observation_space(obs)
            .action_space(act)
            .reward(|_, _| 0.0)
            .done(|_, _| false)
            .truncated(|_m, d| d.time > 1.0)
            .sub_steps(1)
            .build()
    }

    #[test]
    fn from_build_fn_metadata_roundtrip() {
        let task = TaskConfig::from_build_fn(
            "custom-seeded",
            4,
            2,
            vec![0.5, 0.5, 0.1, 0.1],
            |n_envs, _seed| build_trivial_2dof_vec_env(n_envs),
        );

        assert_eq!(task.name(), "custom-seeded");
        assert_eq!(task.obs_dim(), 4);
        assert_eq!(task.act_dim(), 2);
        assert_eq!(task.obs_scale(), &[0.5, 0.5, 0.1, 0.1]);

        // The closure builds a real VecEnv that resets cleanly.
        let mut env = task.build_vec_env(2, 0).unwrap();
        let obs = env.reset_all().unwrap();
        assert_eq!(obs.shape(), &[2, 4]);
    }

    #[test]
    fn from_build_fn_threads_seed_into_closure() {
        use std::sync::Mutex;

        // The closure captures an Arc<Mutex<Vec<u64>>> and pushes each
        // observed seed into it.  The test then checks that the seeds
        // recorded match the seeds passed to `build_vec_env`, proving
        // the per-replicate seed actually reaches the closure body —
        // which is the load-bearing invariant that the synthesized
        // `TaskConfigBuilder::build()` path violates.
        let observed_seeds: Arc<Mutex<Vec<u64>>> = Arc::new(Mutex::new(Vec::new()));
        let observed_for_closure = Arc::clone(&observed_seeds);

        let task = TaskConfig::from_build_fn(
            "seed-recorder",
            4,
            2,
            vec![1.0, 1.0, 1.0, 1.0],
            move |n_envs, seed| {
                observed_for_closure.lock().unwrap().push(seed);
                build_trivial_2dof_vec_env(n_envs)
            },
        );

        let _env_a = task.build_vec_env(2, 12_345).unwrap();
        let _env_b = task.build_vec_env(2, 67_890).unwrap();
        let _env_c = task.build_vec_env(4, 0).unwrap();

        let recorded: Vec<u64> = observed_seeds.lock().unwrap().clone();
        assert_eq!(recorded, vec![12_345, 67_890, 0]);
    }

    #[test]
    fn from_build_fn_propagates_env_error() {
        // A closure that always returns an EnvError should surface
        // that error through `build_vec_env`, matching the
        // `TaskConfigBuilder::build()` synthesized closure's error
        // propagation contract.
        let task = TaskConfig::from_build_fn("always-fails", 1, 1, vec![1.0], |_n_envs, _seed| {
            Err(EnvError::ZeroSubSteps)
        });

        let result = task.build_vec_env(2, 0);
        assert!(matches!(result, Err(EnvError::ZeroSubSteps)));
    }
}
