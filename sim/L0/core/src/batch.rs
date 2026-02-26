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

/// Batched simulation: N independent environments sharing one [`Model`].
///
/// All environments have identical physics (same [`Model`]), but independent
/// state (separate [`Data`]). Stepping is parallelized across CPU cores via
/// rayon when the `parallel` feature is enabled; sequential fallback when
/// disabled.
///
/// # Design
///
/// Each environment owns a full [`Data`] instance with its own heap
/// allocations (scratch buffers, contact vectors, warmstart HashMap).
/// Cross-environment parallelism comes from rayon task-level parallelism;
/// within-environment acceleration comes from sim-simd SIMD operations.
/// The two are orthogonal and compose naturally.
///
/// # Single Model Constraint
///
/// All environments share the same [`Arc<Model>`] (same `nq`, `nv`, body
/// tree, geom set). Multi-model batching (different robots in the same
/// batch) is not supported.
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}

impl BatchSim {
    /// Create a batch of `n` environments, each initialized via
    /// [`Model::make_data()`] (qpos = qpos0, qvel = 0, time = 0).
    #[must_use]
    pub fn new(model: Arc<Model>, n: usize) -> Self {
        let envs = (0..n).map(|_| model.make_data()).collect();
        Self { model, envs }
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

    /// Shared model reference.
    #[must_use]
    pub fn model(&self) -> &Model {
        &self.model
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
        let model = &self.model;

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
    }

    // ==================== Reset ====================

    /// Reset environment `i` to initial state via [`Data::reset()`].
    ///
    /// Does **not** zero `qfrc_applied` / `xfrc_applied` (see [`Data::reset`]
    /// documentation). Callers must zero these explicitly if needed.
    ///
    /// Returns `None` if `i >= len()`.
    pub fn reset(&mut self, i: usize) -> Option<()> {
        self.envs.get_mut(i)?.reset(&self.model);
        Some(())
    }

    /// Reset all environments where `mask[i]` is true.
    ///
    /// Environments where `mask[i]` is false are untouched.
    /// If `mask.len() < len()`, unaddressed environments are untouched.
    /// If `mask.len() > len()`, excess entries are ignored.
    pub fn reset_where(&mut self, mask: &[bool]) {
        let model = &self.model;
        for (i, data) in self.envs.iter_mut().enumerate() {
            if mask.get(i).copied().unwrap_or(false) {
                data.reset(model);
            }
        }
    }

    /// Reset all environments to initial state.
    pub fn reset_all(&mut self) {
        let model = &self.model;
        for data in &mut self.envs {
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
    /// Used by GPU backend to pass model to parallel forward() calls.
    #[cfg(feature = "gpu-internals")]
    #[doc(hidden)]
    #[must_use]
    pub fn model_arc(&self) -> &Arc<Model> {
        &self.model
    }
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
