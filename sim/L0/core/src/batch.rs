//! Batched simulation: N independent environments sharing one [`Model`].
//!
//! All environments have identical physics (same [`Model`]), but independent
//! state (separate [`Data`]). Stepping is parallelized across CPU cores via
//! rayon when the `parallel` feature is enabled; sequential fallback when
//! disabled.
//!
//! This is a **pure physics batching** API — it knows nothing about rewards,
//! episodes, or RL semantics. Higher-level wrappers (Gymnasium, etc.) are
//! built on top by consumers, not inside sim-core.
//!
//! # Examples
//!
//! ```ignore
//! use sim_core::batch::BatchSim;
//! use sim_core::Model;
//! use std::sync::Arc;
//!
//! let model = Arc::new(Model::n_link_pendulum(3, 1.0, 0.1));
//! let mut batch = BatchSim::new(model, 64);
//!
//! // Set controls for each environment
//! for env in batch.envs_mut() {
//!     env.ctrl.fill(0.1);
//! }
//!
//! // Step all environments (parallel when `parallel` feature is enabled)
//! let errors = batch.step_all();
//!
//! // Read results
//! for (i, env) in batch.envs().enumerate() {
//!     println!("env {i}: qpos[0] = {}", env.qpos[0]);
//! }
//!
//! // Reset failed environments
//! let failed: Vec<bool> = errors.iter().map(|e| e.is_some()).collect();
//! batch.reset_where(&failed);
//! ```

use std::sync::Arc;

use crate::types::{Data, Model, StepError};

/// Batched simulation: N independent environments.
///
/// `BatchSim` has two construction paths:
///
/// 1. **Shared-model** via [`BatchSim::new`] — all environments share the
///    same [`Arc<Model>`] (same `nq`, `nv`, body tree, geom set). This is
///    the right choice for deterministic-physics batches (contacts,
///    control sweeps, non-stochastic rollouts).
///
/// 2. **Per-env** via [`BatchSim::new_per_env`] — each environment gets
///    its own [`Model`] built from a factory closure, with a per-env
///    stack installed via [`PerEnvStack::install_per_env`]. This is the
///    right choice for batches whose stochastic components (e.g.
///    [`LangevinThermostat`] in `sim-thermostat`) need per-env state to
///    avoid cross-env noise aliasing.
///
/// Each environment owns a full [`Data`] instance with its own heap
/// allocations (scratch buffers, contact vectors, warmstart `HashMap`).
/// Cross-environment parallelism comes from rayon task-level parallelism;
/// within-environment acceleration comes from sim-simd SIMD operations.
/// The two are orthogonal and compose naturally.
///
/// # Which path was used?
///
/// [`BatchSim::model`] and [`BatchSim::step_all`] handle both paths
/// transparently. Under the shared-model path, `model()` returns the one
/// shared [`Model`]; under the per-env path, `model()` returns the first
/// env's model (sound because all per-env models share the same shape —
/// the factory is expected to return structurally identical models). For
/// callers that need to distinguish, use [`BatchSim::is_per_env`].
pub struct BatchSim {
    /// Shared-model path: populated by [`BatchSim::new`]; `None` under
    /// [`BatchSim::new_per_env`].
    shared_model: Option<Arc<Model>>,
    /// Per-env path: one [`Model`] per env with `cb_passive` already
    /// installed. Empty under [`BatchSim::new`].
    per_env_models: Vec<Model>,
    /// One [`Data`] per env. Always populated, regardless of which
    /// construction path was used.
    envs: Vec<Data>,
}

impl BatchSim {
    /// Create a batch of `n` environments sharing the same [`Arc<Model>`].
    /// Each env is initialized via [`Model::make_data()`] (qpos = qpos0,
    /// qvel = 0, time = 0).
    ///
    /// Use this constructor for deterministic-physics batches. For
    /// batches whose per-env stochastic components need fresh state (e.g.
    /// `LangevinThermostat` under the C-3 chassis), prefer
    /// [`BatchSim::new_per_env`].
    #[must_use]
    pub fn new(model: Arc<Model>, n: usize) -> Self {
        let envs = (0..n).map(|_| model.make_data()).collect();
        Self {
            shared_model: Some(model),
            per_env_models: Vec::new(),
            envs,
        }
    }

    /// Create a batch of `n` environments, each constructed via a factory
    /// closure that returns a `(Model, Arc<S>)` pair where `S: PerEnvStack`.
    /// Each env's model has its `cb_passive` installed from its paired
    /// stack via [`PerEnvStack::install_per_env`].
    ///
    /// The `prototype` argument is the receiver for the
    /// `PerEnvStack::install_per_env` trait method — it is not held by
    /// the returned `BatchSim` and can be dropped after the call. The
    /// factory is the authoritative source of per-env stacks; the caller
    /// retains ownership of the `Arc<S>` handles it produces (via cloning
    /// inside the factory or externally) if it needs to call
    /// `disable_stochastic` per env later.
    ///
    /// Use this constructor when the batch needs per-env stochastic
    /// components (e.g. `LangevinThermostat` under C-3). For
    /// deterministic-physics batches or any case where all envs share
    /// the same callback-free `Model`, prefer [`BatchSim::new`].
    #[must_use]
    pub fn new_per_env<S, F>(prototype: &Arc<S>, n: usize, factory: F) -> Self
    where
        S: PerEnvStack,
        F: FnMut(usize) -> (Model, Arc<S>),
    {
        let batch = prototype.install_per_env(n, factory);
        let envs = batch.models.iter().map(Model::make_data).collect();
        // `batch.stacks` is intentionally dropped: the caller retains any
        // handles it needs via the factory closure's own scope. Holding
        // them on `BatchSim` would duplicate information the factory
        // already returns and force `BatchSim` to care about the stack
        // type parameter `S`, which it does not.
        drop(batch.stacks);
        Self {
            shared_model: None,
            per_env_models: batch.models,
            envs,
        }
    }

    /// `true` if this batch was constructed via [`BatchSim::new_per_env`];
    /// `false` if via [`BatchSim::new`].
    #[must_use]
    pub fn is_per_env(&self) -> bool {
        self.shared_model.is_none()
    }

    /// Number of environments.
    #[must_use]
    pub fn len(&self) -> usize {
        self.envs.len()
    }

    /// Whether the batch is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.envs.is_empty()
    }

    /// Reference to a representative [`Model`] for this batch.
    ///
    /// Under the shared-model path (constructed via [`BatchSim::new`]),
    /// this returns the one shared [`Model`]. Under the per-env path
    /// (constructed via [`BatchSim::new_per_env`]), this returns the
    /// first env's model — sound because all per-env models share the
    /// same shape (`nq`, `nv`, body tree, geom set) by factory contract;
    /// callers reading `model.nq`, `model.nv`, or other static fields
    /// get the correct values.
    ///
    /// # Panics
    ///
    /// Panics if called on an empty per-env batch (`new_per_env(_, 0, _)`
    /// returns a `BatchSim` with no first env to borrow a model from).
    /// Callers should check [`BatchSim::is_empty`] first if this is a
    /// concern.
    #[must_use]
    pub fn model(&self) -> &Model {
        match &self.shared_model {
            Some(m) => m,
            None => &self.per_env_models[0],
        }
    }

    // ==================== Environment Access ====================

    /// Immutable access to environment `i`.
    ///
    /// Returns `None` if `i >= len()`.
    #[must_use]
    pub fn env(&self, i: usize) -> Option<&Data> {
        self.envs.get(i)
    }

    /// Mutable access to environment `i`.
    ///
    /// Use this to set `ctrl`, `qfrc_applied`, `xfrc_applied`, or
    /// any other input field before calling [`step_all()`](Self::step_all).
    ///
    /// Returns `None` if `i >= len()`.
    pub fn env_mut(&mut self, i: usize) -> Option<&mut Data> {
        self.envs.get_mut(i)
    }

    /// Iterator over all environments (immutable).
    #[must_use]
    pub fn envs(&self) -> impl ExactSizeIterator<Item = &Data> {
        self.envs.iter()
    }

    /// Iterator over all environments (mutable).
    ///
    /// Primary mechanism for setting per-env controls before
    /// [`step_all()`](Self::step_all).
    pub fn envs_mut(&mut self) -> impl ExactSizeIterator<Item = &mut Data> {
        self.envs.iter_mut()
    }

    // ==================== Stepping ====================

    /// Step all environments by one timestep.
    ///
    /// Returns per-environment errors. `None` = success, `Some(e)` = that
    /// environment's step failed. NaN/divergence triggers auto-reset (§41 S8)
    /// — use `data.divergence_detected()` to check. Only non-recoverable
    /// errors (`CholeskyFailed`, `LuSingular`, `InvalidTimestep`) return `Some`.
    ///
    /// When the `parallel` feature is enabled, environments are stepped in
    /// parallel via rayon `par_iter_mut`. When disabled, environments are
    /// stepped sequentially (identical results, useful for debugging).
    ///
    /// # Determinism
    ///
    /// Output is independent of thread count and scheduling order. Each
    /// environment's step is a pure function of its own [`Data`] and the
    /// shared [`Model`]. No cross-environment communication occurs.
    pub fn step_all(&mut self) -> Vec<Option<StepError>> {
        if let Some(model) = &self.shared_model {
            #[cfg(feature = "parallel")]
            {
                use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
                self.envs
                    .par_iter_mut()
                    .map(|data| data.step(model).err())
                    .collect()
            }

            #[cfg(not(feature = "parallel"))]
            {
                self.envs
                    .iter_mut()
                    .map(|data| data.step(model).err())
                    .collect()
            }
        } else {
            // Per-env path — each env pairs with its own model. The
            // disjoint-field borrow (`&self.per_env_models` + `&mut
            // self.envs`) is sound because the two `Vec`s are separate
            // fields; the per-env parallel step walks index-paired
            // pairs without cross-env aliasing.
            let models = &self.per_env_models;
            #[cfg(feature = "parallel")]
            {
                use rayon::iter::{
                    IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator,
                };
                self.envs
                    .par_iter_mut()
                    .enumerate()
                    .map(|(i, data)| data.step(&models[i]).err())
                    .collect()
            }

            #[cfg(not(feature = "parallel"))]
            {
                self.envs
                    .iter_mut()
                    .enumerate()
                    .map(|(i, data)| data.step(&models[i]).err())
                    .collect()
            }
        }
    }

    // ==================== Reset ====================

    /// Reset environment `i` to initial state via [`Data::reset()`].
    ///
    /// Does **not** zero `qfrc_applied` / `xfrc_applied` (see [`Data::reset`]
    /// documentation). Callers must zero these explicitly if needed.
    ///
    /// Returns `None` if `i >= len()`.
    pub fn reset(&mut self, i: usize) -> Option<()> {
        // Destructure into disjoint field borrows so the model
        // reference (from `shared_model` or `per_env_models`) and the
        // `&mut Data` (from `envs`) can coexist.
        let Self {
            shared_model,
            per_env_models,
            envs,
        } = self;
        let data = envs.get_mut(i)?;
        let model: &Model = match shared_model {
            Some(m) => m,
            None => per_env_models.get(i)?,
        };
        data.reset(model);
        Some(())
    }

    /// Reset all environments where `mask[i]` is true.
    ///
    /// Environments where `mask[i]` is false are untouched.
    /// If `mask.len() < len()`, unaddressed environments are untouched.
    /// If `mask.len() > len()`, excess entries are ignored.
    pub fn reset_where(&mut self, mask: &[bool]) {
        let Self {
            shared_model,
            per_env_models,
            envs,
        } = self;
        for (i, data) in envs.iter_mut().enumerate() {
            if !mask.get(i).copied().unwrap_or(false) {
                continue;
            }
            let model: &Model = match shared_model {
                Some(m) => m,
                None => &per_env_models[i],
            };
            data.reset(model);
        }
    }

    /// Reset all environments to initial state.
    pub fn reset_all(&mut self) {
        let Self {
            shared_model,
            per_env_models,
            envs,
        } = self;
        for (i, data) in envs.iter_mut().enumerate() {
            let model: &Model = match shared_model {
                Some(m) => m,
                None => &per_env_models[i],
            };
            data.reset(model);
        }
    }

    // ==================== GPU Internals ====================

    /// Direct mutable slice access to environments.
    ///
    /// Used by GPU backend for rayon par_iter_mut() over envs.
    #[cfg(feature = "gpu-internals")]
    #[doc(hidden)]
    pub fn envs_as_mut_slice(&mut self) -> &mut [Data] {
        &mut self.envs
    }

    /// Clone of the shared model Arc.
    ///
    /// Used by GPU backend to pass model to parallel `forward()` calls.
    /// Only valid on shared-model batches (constructed via
    /// [`BatchSim::new`]). Per-env batches have no single `Arc<Model>`
    /// to hand out.
    ///
    /// # Panics
    ///
    /// Panics if called on a per-env batch.
    #[cfg(feature = "gpu-internals")]
    #[doc(hidden)]
    #[must_use]
    // The panic on per-env batches is the method contract — `model_arc`
    // is a GPU-internals accessor whose shape assumes one shared Arc.
    #[allow(clippy::expect_used)]
    pub fn model_arc(&self) -> &Arc<Model> {
        self.shared_model
            .as_ref()
            .expect("model_arc: only valid on shared-model batches")
    }
}

/// Trait exposing the `install_per_env` surface [`BatchSim::new_per_env`]
/// needs to wire per-env stacks into a batch.
///
/// The trait lives in `sim-core::batch` (rather than in the thermostat
/// crate) so that `sim-core` owns the contract without taking a reverse
/// dependency on `sim-thermostat`. Implementors live in downstream
/// crates — today, `sim-thermostat` implements it for `PassiveStack` —
/// and pass themselves as the `prototype` argument to
/// [`BatchSim::new_per_env`].
///
/// This trait is **not object-safe**: `install_per_env` is generic over
/// the factory closure type `F`, which prevents `dyn PerEnvStack` at the
/// call site. Use `S: PerEnvStack` as a generic bound instead (as
/// [`BatchSim::new_per_env`] does).
///
/// # Contract
///
/// Implementors build `n` independent `(Model, Arc<Self>)` pairs by
/// calling `build_one(i)` for each `i in 0..n`, install the resulting
/// stack onto each returned model, and collect the installed models
/// plus retained stacks into an [`EnvBatch`]. Each per-env model must
/// have its `cb_passive` ready to fire after `install_per_env` returns.
pub trait PerEnvStack: Send + Sync + 'static {
    /// Build `n` independent per-env `(Model, Arc<Self>)` pairs from
    /// `build_one` and install the resulting stacks onto each paired
    /// model, returning the installed models and retained stack handles.
    fn install_per_env<F>(self: &Arc<Self>, n: usize, build_one: F) -> EnvBatch<Self>
    where
        F: FnMut(usize) -> (Model, Arc<Self>);
}

/// A batch of `n` independent `(Model, Arc<S>)` pairs produced by
/// [`PerEnvStack::install_per_env`].
///
/// Each model has its `cb_passive` already installed, and the matching
/// stacks are retained in the same order as `models` so callers can
/// later call stack-type-specific operations (e.g. `disable_stochastic`)
/// per env.
pub struct EnvBatch<S: ?Sized> {
    /// One [`Model`] per env, with `cb_passive` already installed.
    pub models: Vec<Model>,
    /// One `Arc<S>` per env, in the same order as [`EnvBatch::models`].
    pub stacks: Vec<Arc<S>>,
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::needless_range_loop
)]
mod tests {
    use super::*;
    use crate::types::enums::ENABLE_ENERGY;

    fn pendulum_model() -> Model {
        let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
        model.enableflags |= ENABLE_ENERGY;
        model
    }

    // ==================== Construction ====================

    #[test]
    fn new_creates_n_environments() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(Arc::clone(&model), 8);
        assert_eq!(batch.len(), 8);
        assert!(!batch.is_empty());
    }

    #[test]
    fn new_zero_environments() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(model, 0);
        assert_eq!(batch.len(), 0);
        assert!(batch.is_empty());
    }

    #[test]
    fn envs_initialized_from_make_data() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(Arc::clone(&model), 4);
        let fresh = model.make_data();
        for env in batch.envs() {
            assert_eq!(env.qpos, fresh.qpos);
            assert_eq!(env.qvel, fresh.qvel);
            assert_eq!(env.time, 0.0);
        }
    }

    // ==================== Environment Access ====================

    #[test]
    fn env_returns_none_out_of_bounds() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(model, 3);
        assert!(batch.env(0).is_some());
        assert!(batch.env(2).is_some());
        assert!(batch.env(3).is_none());
        assert!(batch.env(usize::MAX).is_none());
    }

    #[test]
    fn env_mut_returns_none_out_of_bounds() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(model, 3);
        assert!(batch.env_mut(0).is_some());
        assert!(batch.env_mut(2).is_some());
        assert!(batch.env_mut(3).is_none());
    }

    #[test]
    fn envs_iterator_exact_size() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(model, 5);
        let iter = batch.envs();
        assert_eq!(iter.len(), 5);
        assert_eq!(iter.count(), 5);
    }

    #[test]
    fn envs_mut_can_set_ctrl() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 4);
        for (i, env) in batch.envs_mut().enumerate() {
            if model.nu > 0 {
                env.ctrl[0] = i as f64;
            }
        }
        // No actuators on n_link_pendulum, but the iteration works
        assert_eq!(batch.len(), 4);
    }

    #[test]
    fn model_accessor() {
        let model = Arc::new(pendulum_model());
        let batch = BatchSim::new(Arc::clone(&model), 2);
        assert_eq!(batch.model().nq, model.nq);
        assert_eq!(batch.model().nv, model.nv);
    }

    // ==================== Stepping ====================

    /// Acceptance criterion 1: Bit-exact determinism.
    /// Batched stepping produces identical results to sequential stepping.
    #[test]
    fn step_all_matches_sequential() {
        let model = Arc::new(pendulum_model());
        let n = 4;

        // Set up distinct initial states
        let mut batch = BatchSim::new(Arc::clone(&model), n);
        let mut sequential: Vec<Data> = (0..n).map(|_| model.make_data()).collect();

        for i in 0..n {
            let angle = (i as f64 + 1.0) * 0.3;
            batch.env_mut(i).unwrap().qpos[0] = angle;
            sequential[i].qpos[0] = angle;
        }

        // Step both 10 times
        for _ in 0..10 {
            let _errors = batch.step_all();
            for data in &mut sequential {
                data.step(&model).unwrap();
            }
        }

        // Compare all fields
        for i in 0..n {
            let batch_env = batch.env(i).unwrap();
            let seq_env = &sequential[i];
            assert_eq!(batch_env.qpos, seq_env.qpos, "env {i} qpos mismatch");
            assert_eq!(batch_env.qvel, seq_env.qvel, "env {i} qvel mismatch");
            assert_eq!(batch_env.qacc, seq_env.qacc, "env {i} qacc mismatch");
            assert_eq!(batch_env.time, seq_env.time, "env {i} time mismatch");
            assert_eq!(
                batch_env.sensordata, seq_env.sensordata,
                "env {i} sensordata mismatch"
            );
            assert_eq!(
                batch_env.energy_kinetic, seq_env.energy_kinetic,
                "env {i} energy_kinetic mismatch"
            );
            assert_eq!(
                batch_env.energy_potential, seq_env.energy_potential,
                "env {i} energy_potential mismatch"
            );
        }
    }

    /// Acceptance criterion 2: Error isolation.
    /// A StepError in one env does not affect others.
    #[test]
    fn nan_auto_reset_isolation() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 4);

        // Give env 0 a valid initial state, env 1 a NaN
        batch.env_mut(0).unwrap().qpos[0] = 0.5;
        batch.env_mut(1).unwrap().qpos[0] = f64::NAN;
        batch.env_mut(2).unwrap().qpos[0] = 0.7;
        batch.env_mut(3).unwrap().qpos[0] = 0.9;

        let errors = batch.step_all();

        // All envs succeed — NaN env auto-resets (§41 S8).
        for (i, e) in errors.iter().enumerate() {
            assert!(e.is_none(), "env {i} should succeed (got {e:?})");
        }

        // Env 1 should have detected divergence (auto-reset happened).
        assert!(
            batch.env(1).unwrap().divergence_detected(),
            "env 1 should auto-reset on NaN"
        );

        // Healthy envs should have advanced
        assert!(batch.env(0).unwrap().time > 0.0);
        assert!(batch.env(2).unwrap().time > 0.0);
        assert!(batch.env(3).unwrap().time > 0.0);
    }

    /// Acceptance criterion 3: Auto-reset on NaN.
    /// After §41 S8, NaN triggers auto-reset — second step succeeds from reset state.
    #[test]
    fn auto_reset_on_nan() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 2);

        // Inject NaN into env 0
        batch.env_mut(0).unwrap().qpos[0] = f64::NAN;
        batch.env_mut(1).unwrap().qpos[0] = 0.5;

        // First step — auto-resets env 0
        let errors1 = batch.step_all();
        assert!(errors1[0].is_none(), "env 0 should auto-reset, not error");
        assert!(errors1[1].is_none());
        assert!(
            batch.env(0).unwrap().divergence_detected(),
            "env 0 should have divergence flag set"
        );

        // Second step — env 0 runs from reset state, succeeds
        let errors2 = batch.step_all();
        assert!(
            errors2[0].is_none(),
            "env 0 should succeed from reset state"
        );
        assert!(errors2[1].is_none(), "env 1 should still succeed");
    }

    // ==================== Reset ====================

    /// Acceptance criterion 4: Reset correctness.
    #[test]
    fn reset_single_env() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 3);

        // Step all envs forward
        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        // All envs should have advanced
        assert!(batch.env(0).unwrap().time > 0.0);
        assert!(batch.env(1).unwrap().time > 0.0);
        assert!(batch.env(2).unwrap().time > 0.0);

        // Reset only env 1
        assert!(batch.reset(1).is_some());
        assert_eq!(batch.env(1).unwrap().time, 0.0);
        assert_eq!(batch.env(1).unwrap().qpos, model.qpos0);

        // Envs 0 and 2 should be untouched
        assert!(batch.env(0).unwrap().time > 0.0);
        assert!(batch.env(2).unwrap().time > 0.0);
    }

    #[test]
    fn reset_out_of_bounds_returns_none() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(model, 2);
        assert!(batch.reset(2).is_none());
        assert!(batch.reset(usize::MAX).is_none());
    }

    #[test]
    fn reset_where_selective() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 4);

        // Advance all envs
        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        // Reset envs 0 and 2 only
        batch.reset_where(&[true, false, true, false]);

        assert_eq!(batch.env(0).unwrap().time, 0.0, "env 0 should be reset");
        assert!(
            batch.env(1).unwrap().time > 0.0,
            "env 1 should NOT be reset"
        );
        assert_eq!(batch.env(2).unwrap().time, 0.0, "env 2 should be reset");
        assert!(
            batch.env(3).unwrap().time > 0.0,
            "env 3 should NOT be reset"
        );
    }

    #[test]
    fn reset_where_short_mask() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 4);

        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        // Mask shorter than env count — unaddressed envs untouched
        batch.reset_where(&[true]);

        assert_eq!(batch.env(0).unwrap().time, 0.0);
        assert!(batch.env(1).unwrap().time > 0.0);
        assert!(batch.env(2).unwrap().time > 0.0);
        assert!(batch.env(3).unwrap().time > 0.0);
    }

    #[test]
    fn reset_where_long_mask() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 2);

        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        // Mask longer than env count — excess entries ignored
        batch.reset_where(&[false, true, true, true, true]);

        assert!(batch.env(0).unwrap().time > 0.0);
        assert_eq!(batch.env(1).unwrap().time, 0.0);
    }

    #[test]
    fn reset_all_resets_everything() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 4);

        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        batch.reset_all();

        for (i, env) in batch.envs().enumerate() {
            assert_eq!(env.time, 0.0, "env {i} should be reset");
            assert_eq!(env.qpos, model.qpos0, "env {i} qpos should match qpos0");
        }
    }

    // ==================== Edge Cases ====================

    #[test]
    fn step_all_empty_batch() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(model, 0);
        let errors = batch.step_all();
        assert!(errors.is_empty());
    }

    #[test]
    fn step_all_single_env() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 1);
        batch.env_mut(0).unwrap().qpos[0] = 0.5;

        let errors = batch.step_all();
        assert_eq!(errors.len(), 1);
        assert!(errors[0].is_none());
        assert!(batch.env(0).unwrap().time > 0.0);
    }

    #[test]
    fn reset_where_empty_mask() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(Arc::clone(&model), 3);

        for env in batch.envs_mut() {
            env.qpos[0] = 1.0;
        }
        let _errors = batch.step_all();

        // Empty mask — nothing is reset
        batch.reset_where(&[]);

        for env in batch.envs() {
            assert!(env.time > 0.0, "no env should be reset");
        }
    }

    #[test]
    fn reset_all_empty_batch() {
        let model = Arc::new(pendulum_model());
        let mut batch = BatchSim::new(model, 0);
        batch.reset_all(); // Should not panic
    }
}
