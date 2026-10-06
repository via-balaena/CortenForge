//! `PassiveStack` — composition layer for passive-force components.
//!
//! `PassiveStack` is the bridge between the spec's clean
//! `(&Model, &Data, &mut DVector<f64>)` trait shape ([`PassiveComponent`])
//! and the underlying `Fn(&Model, &mut Data)` shape that
//! `Model::cb_passive` actually uses. The stack:
//!
//! 1. Holds an ordered list of `Arc<dyn PassiveComponent>`s assembled
//!    via [`PassiveStackBuilder`].
//! 2. On `install`, registers a single `cb_passive` callback that
//!    iterates the components in order, performing the split-borrow
//!    dance once per step so component authors never see the mutable
//!    `Data` borrow.
//! 3. Exposes per-component stochastic gating via [`crate::Stochastic`] and
//!    the [`StochasticGuard`] RAII helper, so finite-difference and
//!    autograd contexts can wrap a block of code in
//!    [`PassiveStack::disable_stochastic`] and have every stochastic
//!    component in the stack temporarily produce only its
//!    deterministic forces.
//! 4. Supports parallel-environment construction via
//!    [`PassiveStack::install_per_env`], which builds N independent
//!    `(Model, PassiveStack)` pairs from a user-supplied factory.
//!    Decision-3 + N4 enforce that each env's stack is fresh (no
//!    aliased RNG state) via a `debug_assert!` + defensive
//!    `clear_passive_callback` pair.
//!
//! ## The split-borrow dance
//!
//! The core difficulty: `cb_passive` hands the closure
//! `&mut Data`, but the trait wants `&Data + &mut DVector<f64>`. We
//! resolve it with `std::mem::replace`:
//!
//! ```ignore
//! let mut qfrc_out = std::mem::replace(
//!     &mut data_inner.qfrc_passive,
//!     DVector::zeros(0),
//! );
//! {
//!     let data_ref: &Data = data_inner;
//!     for component in &stack_ref.components {
//!         component.apply(model_inner, data_ref, &mut qfrc_out);
//!     }
//! }
//! data_inner.qfrc_passive = qfrc_out;
//! ```
//!
//! `mem::replace` takes ownership of `qfrc_passive` (an O(1) `DVector`
//! pointer-swap, since `DVector::zeros(0)` allocates nothing), then
//! `data_inner` is no longer mutably borrowed and can be reborrowed
//! immutably as `&Data` for the trait calls. After the inner block
//! drops the immutable reborrow, the owned `qfrc_out` (now containing
//! the components' accumulated contributions on top of whatever
//! `mj_passive` had aggregated before `cb_passive` fired) is moved
//! back into `data_inner.qfrc_passive`. Total cost per step:
//! two `DVector` pointer swaps. No `unsafe`.

use std::sync::{Arc, Mutex, PoisonError};

use sim_core::batch::{EnvBatch, PerEnvStack};
use sim_core::{DVector, Data, Model};

use crate::component::{PassiveComponent, Stochastic};
use crate::error::ThermostatError;

/// Builder for [`PassiveStack`]. Construct via [`PassiveStack::builder`]
/// then chain `.with(component)` calls and finish with `.build()`.
pub struct PassiveStackBuilder {
    components: Vec<Arc<dyn PassiveComponent>>,
}

impl PassiveStackBuilder {
    /// Append a component to the stack. Components are applied in
    /// insertion order during each `cb_passive` invocation.
    #[must_use]
    pub fn with<C: PassiveComponent>(mut self, component: C) -> Self {
        self.components.push(Arc::new(component));
        self
    }

    /// Append a pre-wrapped `Arc<dyn PassiveComponent>` to the stack.
    ///
    /// Use this when the component is already type-erased (e.g. stored
    /// in a `Vec<Arc<dyn PassiveComponent>>` by a higher-level builder).
    #[must_use]
    pub fn with_arc(mut self, component: Arc<dyn PassiveComponent>) -> Self {
        self.components.push(component);
        self
    }

    /// Finalize the builder into an `Arc<PassiveStack>` ready to be
    /// `install`ed onto a `Model`.
    #[must_use]
    pub fn build(self) -> Arc<PassiveStack> {
        Arc::new(PassiveStack {
            components: self.components,
            disabled: Mutex::new(Disabled::default()),
        })
    }
}

/// An ordered, immutable composition of `PassiveComponent`s installed
/// onto a `Model` as a single `cb_passive` callback.
///
/// `PassiveStack` is always handed around as `Arc<PassiveStack>` —
/// the `cb_passive` callback closure captures a clone of the `Arc`,
/// and any caller that wants to call [`PassiveStack::disable_stochastic`]
/// later retains its own `Arc` handle. Both `install` and
/// `install_per_env` take `self: &Arc<Self>` (the standard
/// idiomatic-Rust pattern for "method on an Arc-wrapped type that
/// captures a clone of self into a callback") so the caller's handle
/// is retained automatically — no manual `Arc::clone` boilerplate at
/// the call site.
pub struct PassiveStack {
    components: Vec<Arc<dyn PassiveComponent>>,
    /// Live [`StochasticGuard`]s and the flags they will restore.
    disabled: Mutex<Disabled>,
}

/// How many [`StochasticGuard`]s are alive, and each component's active flag from before the
/// first of them (`prior[i]` for `components[i]`; read only for stochastic components).
#[derive(Default)]
struct Disabled {
    depth: usize,
    prior: Vec<bool>,
}

impl PassiveStack {
    /// Start building a new stack.
    #[must_use]
    pub fn builder() -> PassiveStackBuilder {
        PassiveStackBuilder {
            components: Vec::new(),
        }
    }

    /// Install this stack onto `model` as a single `cb_passive`
    /// callback. The closure captures a clone of `self` and replaces
    /// any prior `cb_passive` setting on `model`.
    ///
    /// `self: &Arc<Self>` — the caller retains its handle and can
    /// call [`PassiveStack::disable_stochastic`] or read the stack
    /// after `install` returns.
    ///
    /// # Panics
    ///
    /// Panics if a component refuses `model` (see [`Self::validate`]).
    /// [`Self::try_install`] returns the error instead.
    #[allow(clippy::panic)] // the documented refusal; try_install is the non-panicking path
    pub fn install(self: &Arc<Self>, model: &mut Model) {
        if let Err(e) = self.validate(model) {
            panic!("PassiveStack::install: {e}");
        }
        self.install_unchecked(model);
    }

    /// Install this stack onto `model` if every component accepts it and
    /// `model` has no passive callback yet.
    ///
    /// # Errors
    ///
    /// [`ThermostatError::AlreadyInstalled`] if `model.cb_passive` is set,
    /// or the first error from [`Self::validate`].
    pub fn try_install(self: &Arc<Self>, model: &mut Model) -> Result<(), ThermostatError> {
        if model.cb_passive.is_some() {
            return Err(ThermostatError::AlreadyInstalled);
        }
        self.validate(model)?;
        self.install_unchecked(model);
        Ok(())
    }

    /// Check `model` against every component in the stack, in order
    /// (see [`PassiveComponent::validate`]).
    ///
    /// # Errors
    ///
    /// Returns the first component's error.
    pub fn validate(&self, model: &Model) -> Result<(), ThermostatError> {
        self.components
            .iter()
            .try_for_each(|component| component.validate(model))
    }

    fn install_unchecked(self: &Arc<Self>, model: &mut Model) {
        let stack_ref = Arc::clone(self);
        model.set_passive_callback(move |model_inner, data_inner| {
            // Take qfrc_passive out of data via O(1) DVector pointer
            // swap so we can hand &Data + &mut DVector to components
            // without aliasing. See the module-level "split-borrow
            // dance" doc for the full reasoning.
            let mut qfrc_out = std::mem::replace(&mut data_inner.qfrc_passive, DVector::zeros(0));
            {
                let data_ref: &Data = data_inner;
                for component in &stack_ref.components {
                    component.apply(model_inner, data_ref, &mut qfrc_out);
                }
            }
            data_inner.qfrc_passive = qfrc_out;
        });
    }

    /// Set every stochastic component's active flag to `active`.
    /// Deterministic components (those that don't override
    /// `as_stochastic`) are left untouched.
    ///
    /// Prefer [`PassiveStack::disable_stochastic`] over
    /// `set_all_stochastic(false)` when the disable is scoped to a
    /// block — the RAII guard restores prior states on drop, which is
    /// exception-safe and avoids the "forgot to re-enable" footgun.
    pub fn set_all_stochastic(&self, active: bool) {
        for component in &self.components {
            if let Some(stoch) = component.as_stochastic() {
                stoch.set_stochastic_active(active);
            }
        }
    }

    /// Disable every stochastic component in the stack and return an
    /// RAII guard that restores their prior active flags on drop.
    ///
    /// Guards nest in any order: noise stays off until the LAST live
    /// guard drops, which restores the flags from before the FIRST.
    /// A [`Self::set_all_stochastic`] call made while a guard is alive
    /// is overwritten when the last guard drops.
    ///
    /// This is the chassis Decision-7 entry point for finite-difference
    /// and autograd contexts: wrap the FD perturbation block in
    /// `let _guard = stack.disable_stochastic();`, run the perturbed
    /// and baseline rollouts, drop the guard, and the stack returns to
    /// its prior stochastic state. Stochastic components produce only
    /// their deterministic forces inside the guarded block, so the FD
    /// difference recovers `∂F_det/∂qpos` exactly (state-independent
    /// noise is the only kind on the roadmap).
    #[must_use = "the StochasticGuard restores prior flags on drop; \
                  discarding it immediately re-enables noise — call \
                  set_all_stochastic(false) instead if that is desired"]
    pub fn disable_stochastic(self: &Arc<Self>) -> StochasticGuard {
        let mut disabled = self.disabled.lock().unwrap_or_else(PoisonError::into_inner);
        if disabled.depth == 0 {
            disabled.prior = self
                .components
                .iter()
                .map(|c| {
                    c.as_stochastic()
                        .is_some_and(Stochastic::is_stochastic_active)
                })
                .collect();
            self.set_all_stochastic(false);
        }
        disabled.depth += 1;
        drop(disabled);
        StochasticGuard {
            stack: Arc::clone(self),
        }
    }

    /// Restart every stochastic component's noise sequence from its first
    /// step (see [`Stochastic::reset_noise`]).
    pub fn reset_stochastic(&self) {
        for component in &self.components {
            if let Some(stoch) = component.as_stochastic() {
                stoch.reset_noise();
            }
        }
    }

    /// Read-only view of the components, useful for testing and for
    /// callers that need to enumerate the stack (e.g. building a
    /// per-component diagnostic report).
    #[must_use]
    pub fn components(&self) -> &[Arc<dyn PassiveComponent>] {
        &self.components
    }
}

/// `PassiveStack` implements the sim-core chassis entry point for
/// per-env batch construction.
///
/// `install_per_env` builds N independent `(Model, Arc<PassiveStack>)`
/// pairs by invoking `build_one(i)` for each `i in 0..n`, installs the
/// resulting stack onto each model via [`PassiveStack::install`], and
/// returns an [`EnvBatch<PassiveStack>`] holding the N installed
/// models and retained stack handles.
///
/// This is the chassis Decision-3 entry point for `BatchSim`-style
/// parallel-env runs: each env gets its own fresh stack with its own
/// step counter, so per-env independence is guaranteed by construction
/// (no aliased mutable state shared across envs; under C-3 the
/// `LangevinThermostat` counter lives on the per-env thermostat
/// instance).
///
/// # N4 defensive clear
///
/// `build_one` is expected to return a freshly-constructed `Model`
/// with no `cb_passive` already set. If a previous `cb_passive` is
/// detected on the returned model:
///
/// 1. In debug builds, a `debug_assert!` panics with a diagnostic
///    message — the user is misusing the API and should fix the
///    factory function.
/// 2. In release builds (where `debug_assert!` is a no-op), the prior
///    callback is silently `clear_passive_callback`'d before the new
///    stack is installed. This is the "fail loud in dev, behave
///    correctly in release" pattern.
impl PerEnvStack for PassiveStack {
    fn install_per_env<F>(self: &Arc<Self>, n: usize, mut build_one: F) -> EnvBatch<Self>
    where
        F: FnMut(usize) -> (Model, Arc<Self>),
    {
        // The prototype receiver (`&Arc<Self>`) is unused inside the
        // body: the per-env stacks are built by `build_one`, not by
        // cloning the prototype. The receiver exists so the call
        // reads as `prototype.install_per_env(...)` at the call site
        // and so future per-stack configuration can route through
        // the prototype without breaking the signature.
        let _ = self;
        let mut models = Vec::with_capacity(n);
        let mut stacks = Vec::with_capacity(n);
        for i in 0..n {
            let (mut model, stack) = build_one(i);
            // N4: catch misuse loud in dev, fix correctness in release.
            // The order matters — assert FIRST (so it can fire), then
            // defensive clear (so release builds stay correct).
            debug_assert!(
                model.cb_passive.is_none(),
                "install_per_env: build_one returned a Model that already has \
                 a cb_passive set. install_per_env will overwrite it (silently \
                 dropping the prior callback's captured state). Construct the \
                 Model fresh inside build_one and let install_per_env be the \
                 only callback installer.",
            );
            model.clear_passive_callback();
            stack.install(&mut model);
            models.push(model);
            stacks.push(stack);
        }
        EnvBatch { models, stacks }
    }
}

/// RAII guard returned by [`PassiveStack::disable_stochastic`].
///
/// While any guard on the stack is alive, every stochastic component in
/// it is inactive (produces only deterministic forces). When the last
/// live guard drops, the active flags from before the first guard are
/// restored, whatever order the guards drop in.
///
/// The guard is exception-safe: if the code inside the guarded block
/// panics, `Drop::drop` still runs and restores the prior states, so
/// the stack is never left in a partially-disabled state.
pub struct StochasticGuard {
    stack: Arc<PassiveStack>,
}

impl Drop for StochasticGuard {
    fn drop(&mut self) {
        let mut disabled = self
            .stack
            .disabled
            .lock()
            .unwrap_or_else(PoisonError::into_inner);
        disabled.depth -= 1;
        if disabled.depth == 0 {
            for (component, &prior) in self.stack.components.iter().zip(&disabled.prior) {
                if let Some(stoch) = component.as_stochastic() {
                    stoch.set_stochastic_active(prior);
                }
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use super::*;

    /// A no-op deterministic component used for builder/order tests.
    struct DummyDeterministic;
    impl PassiveComponent for DummyDeterministic {
        fn apply(&self, _model: &Model, _data: &Data, _qfrc_out: &mut DVector<f64>) {}
    }

    /// A counting deterministic component used for callback-firing
    /// tests in the integration suite (kept here for shape).
    struct CountingComponent {
        count: Arc<AtomicUsize>,
    }
    impl PassiveComponent for CountingComponent {
        fn apply(&self, _model: &Model, _data: &Data, _qfrc_out: &mut DVector<f64>) {
            self.count.fetch_add(1, Ordering::SeqCst);
        }
    }

    /// A stochastic component with a flag-only Stochastic impl, no
    /// real noise. Used for the gating dance tests.
    struct DummyStochastic {
        active: AtomicBool,
    }
    impl PassiveComponent for DummyStochastic {
        fn apply(&self, _model: &Model, _data: &Data, _qfrc_out: &mut DVector<f64>) {}
        fn as_stochastic(&self) -> Option<&dyn Stochastic> {
            Some(self)
        }
    }
    impl Stochastic for DummyStochastic {
        fn set_stochastic_active(&self, active: bool) {
            self.active.store(active, Ordering::SeqCst);
        }
        fn is_stochastic_active(&self) -> bool {
            self.active.load(Ordering::SeqCst)
        }
    }

    #[test]
    fn builder_starts_empty() {
        let stack = PassiveStack::builder().build();
        assert_eq!(stack.components().len(), 0);
    }

    #[test]
    fn builder_chain_preserves_insertion_order_and_count() {
        // Build a stack with three components and verify the count.
        // Per-slot identity is exercised by the integration tests
        // where the ORDER of forces is observable on data.qfrc_passive;
        // here we only assert the cardinality, which is sufficient
        // evidence given there is no API for reordering.
        let stack = PassiveStack::builder()
            .with(DummyDeterministic)
            .with(CountingComponent {
                count: Arc::new(AtomicUsize::new(0)),
            })
            .with(DummyDeterministic)
            .build();
        assert_eq!(stack.components().len(), 3);
    }

    #[test]
    fn disable_stochastic_flips_only_stochastic_components_and_restores_on_drop() {
        // Mixed stack: deterministic + stochastic + deterministic +
        // stochastic. The disable_stochastic guard should flip both
        // stochastic flags to false during its lifetime and restore
        // them to their prior values on drop.
        let stack = PassiveStack::builder()
            .with(DummyDeterministic)
            .with(DummyStochastic {
                active: AtomicBool::new(true),
            })
            .with(DummyDeterministic)
            .with(DummyStochastic {
                active: AtomicBool::new(true),
            })
            .build();

        // Pre-condition: both stochastic components are active.
        let stoch_views: Vec<&dyn Stochastic> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_stochastic())
            .collect();
        assert_eq!(stoch_views.len(), 2);
        assert!(stoch_views[0].is_stochastic_active());
        assert!(stoch_views[1].is_stochastic_active());

        {
            let _guard = stack.disable_stochastic();
            // Inside the guard: both flags are false.
            let stoch_views: Vec<&dyn Stochastic> = stack
                .components()
                .iter()
                .filter_map(|c| c.as_stochastic())
                .collect();
            assert!(!stoch_views[0].is_stochastic_active());
            assert!(!stoch_views[1].is_stochastic_active());
        }

        // After the guard drops: both flags restored to true.
        let stoch_views: Vec<&dyn Stochastic> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_stochastic())
            .collect();
        assert!(stoch_views[0].is_stochastic_active());
        assert!(stoch_views[1].is_stochastic_active());
    }

    #[test]
    fn disable_stochastic_preserves_already_disabled_components() {
        // Asymmetric pre-state: one stochastic component starts true,
        // the other starts false. After the guard drops, both should
        // return to their original state — not both true.
        let stack = PassiveStack::builder()
            .with(DummyStochastic {
                active: AtomicBool::new(true),
            })
            .with(DummyStochastic {
                active: AtomicBool::new(false),
            })
            .build();

        {
            let _guard = stack.disable_stochastic();
            let stoch_views: Vec<&dyn Stochastic> = stack
                .components()
                .iter()
                .filter_map(|c| c.as_stochastic())
                .collect();
            assert!(!stoch_views[0].is_stochastic_active());
            assert!(!stoch_views[1].is_stochastic_active());
        }

        let stoch_views: Vec<&dyn Stochastic> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_stochastic())
            .collect();
        assert!(stoch_views[0].is_stochastic_active());
        assert!(!stoch_views[1].is_stochastic_active());
    }

    #[test]
    fn set_all_stochastic_flips_all_stochastic_components() {
        let stack = PassiveStack::builder()
            .with(DummyStochastic {
                active: AtomicBool::new(true),
            })
            .with(DummyDeterministic)
            .with(DummyStochastic {
                active: AtomicBool::new(true),
            })
            .build();

        stack.set_all_stochastic(false);
        let stoch_views: Vec<&dyn Stochastic> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_stochastic())
            .collect();
        assert!(!stoch_views[0].is_stochastic_active());
        assert!(!stoch_views[1].is_stochastic_active());

        stack.set_all_stochastic(true);
        let stoch_views: Vec<&dyn Stochastic> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_stochastic())
            .collect();
        assert!(stoch_views[0].is_stochastic_active());
        assert!(stoch_views[1].is_stochastic_active());
    }

    #[test]
    fn install_registers_a_callback_on_the_model() {
        // The full install path (cb_passive fires on data.step) is
        // covered by the §8 integration test. This unit test only
        // verifies that install registers SOMETHING — that the
        // model.cb_passive Option transitions from None to Some.
        let mut model = sim_core::test_fixtures::sho_1d();
        assert!(model.cb_passive.is_none());

        let stack = PassiveStack::builder().with(DummyDeterministic).build();
        stack.install(&mut model);

        assert!(model.cb_passive.is_some());
    }

    #[test]
    fn install_callback_actually_invokes_each_component_per_forward() {
        // A counting component installed via stack.install — verify
        // that calling data.forward(&model) once causes the counter
        // to advance. cb_passive is documented as firing once per
        // mj_fwd_passive call, which forward() invokes once.
        let mut model = sim_core::test_fixtures::sho_1d();
        let mut data = model.make_data();

        let counter = Arc::new(AtomicUsize::new(0));
        let stack = PassiveStack::builder()
            .with(CountingComponent {
                count: Arc::clone(&counter),
            })
            .build();
        stack.install(&mut model);

        assert_eq!(counter.load(Ordering::SeqCst), 0);
        data.forward(&model).unwrap();
        assert_eq!(counter.load(Ordering::SeqCst), 1);
        data.forward(&model).unwrap();
        assert_eq!(counter.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn install_per_env_builds_n_envs_with_callbacks_set() {
        // Prototype is unused (the chassis Decision-3 anchor pattern).
        let prototype = PassiveStack::builder().with(DummyDeterministic).build();

        let batch = prototype.install_per_env(3, |_i| {
            let model = sim_core::test_fixtures::sho_1d();
            let stack = PassiveStack::builder().with(DummyDeterministic).build();
            (model, stack)
        });

        assert_eq!(batch.models.len(), 3);
        assert_eq!(batch.stacks.len(), 3);
        for (i, model) in batch.models.iter().enumerate() {
            assert!(
                model.cb_passive.is_some(),
                "env {i} should have cb_passive set after install_per_env",
            );
        }
    }

    // --- install-time validation ---

    use crate::{
        Diagnose, DoubleWellPotential, ExternalField, LangevinThermostat, OscillatingField,
        PairwiseCoupling, RatchetPotential,
    };

    fn one(component: impl PassiveComponent) -> Arc<PassiveStack> {
        PassiveStack::builder().with(component).build()
    }

    /// What `try_install` says about `component` on `model`.
    fn verdict(component: impl PassiveComponent, mut model: Model) -> Result<(), ThermostatError> {
        one(component).try_install(&mut model)
    }

    fn chain(n: usize) -> Model {
        sim_core::test_fixtures::bistable_chain(n)
    }

    /// A model whose DOFs 0–5 belong to one free joint.
    fn free_body() -> Model {
        sim_core::test_fixtures::free_body_diag(1.0, sim_core::Vector3::new(0.1, 0.1, 0.1))
    }

    #[test]
    fn components_accept_a_model_that_has_what_they_address() {
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 1), chain(2)),
            Ok(())
        );
        assert_eq!(verdict(PairwiseCoupling::chain(3, 0.5), chain(3)), Ok(()));
        assert_eq!(
            verdict(ExternalField::new(vec![0.1, 0.2]), chain(2)),
            Ok(())
        );
        assert_eq!(verdict(ExternalField::new(vec![0.1]), chain(2)), Ok(()));
        assert_eq!(
            verdict(OscillatingField::new(1.0, 1.0, 0.0, 1), chain(2)),
            Ok(())
        );
        let gamma = DVector::from_element(2, 0.1);
        assert_eq!(
            verdict(LangevinThermostat::new(gamma, 1.0, 1, 0), chain(2)),
            Ok(())
        );
        // The thermostat reads velocities only, so a free joint is fine.
        let gamma6 = DVector::from_element(6, 0.1);
        assert_eq!(
            verdict(LangevinThermostat::new(gamma6, 1.0, 1, 0), free_body()),
            Ok(())
        );
    }

    #[test]
    fn components_refuse_a_dof_the_model_lacks() {
        let out_of_range =
            |component, dof, nv| ThermostatError::DofOutOfRange { component, dof, nv };
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 2), chain(2)),
            Err(out_of_range("DoubleWellPotential", 2, 2))
        );
        assert_eq!(
            verdict(PairwiseCoupling::new(vec![1.0], vec![(0, 2)]), chain(2)),
            Err(out_of_range("PairwiseCoupling", 2, 2))
        );
        assert_eq!(
            verdict(ExternalField::new(vec![0.1; 3]), chain(2)),
            Err(out_of_range("ExternalField", 2, 2))
        );
        assert_eq!(
            verdict(OscillatingField::new(1.0, 1.0, 0.0, 2), chain(2)),
            Err(out_of_range("OscillatingField", 2, 2))
        );
        let gamma = DVector::from_element(3, 0.1);
        assert_eq!(
            verdict(LangevinThermostat::new(gamma, 1.0, 1, 0), chain(2)),
            Err(out_of_range("LangevinThermostat", 2, 2))
        );
    }

    #[test]
    fn position_readers_refuse_a_free_joint_dof() {
        let not_scalar = |component| ThermostatError::NotScalarJoint { component, dof: 0 };
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 0), free_body()),
            Err(not_scalar("DoubleWellPotential"))
        );
        assert_eq!(
            verdict(PairwiseCoupling::new(vec![1.0], vec![(0, 1)]), free_body()),
            Err(not_scalar("PairwiseCoupling"))
        );
        assert_eq!(
            verdict(
                RatchetPotential::new(1.0, 0.25, 0.0, 1.0, 0, 0),
                free_body()
            ),
            Err(not_scalar("RatchetPotential"))
        );
    }

    #[test]
    fn components_refuse_a_control_the_model_lacks() {
        // The chain has no actuators.
        assert_eq!(
            verdict(RatchetPotential::new(1.0, 0.25, 0.0, 1.0, 0, 0), chain(1)),
            Err(ThermostatError::CtrlOutOfRange {
                component: "RatchetPotential",
                ctrl: 0,
                nu: 0
            })
        );
        let gamma = DVector::from_element(1, 0.1);
        let thermostat = LangevinThermostat::new(gamma, 1.0, 1, 0).with_ctrl_temperature(0);
        assert_eq!(
            verdict(thermostat, chain(1)),
            Err(ThermostatError::CtrlOutOfRange {
                component: "LangevinThermostat",
                ctrl: 0,
                nu: 0
            })
        );
    }

    #[test]
    fn energy_helpers_refuse_dofs_without_a_position_coordinate() {
        let model = free_body();
        let data = model.make_data();
        assert_eq!(
            ExternalField::new(vec![0.1]).field_energy(&model, &data),
            Err(ThermostatError::NotScalarJoint {
                component: "ExternalField",
                dof: 0
            })
        );
        assert_eq!(
            PairwiseCoupling::new(vec![1.0], vec![(0, 1)]).coupling_energy(&model, &data),
            Err(ThermostatError::NotScalarJoint {
                component: "PairwiseCoupling",
                dof: 0
            })
        );
        let model = chain(2);
        let data = model.make_data();
        assert_eq!(
            ExternalField::new(vec![0.1; 3]).field_energy(&model, &data),
            Err(ThermostatError::DofOutOfRange {
                component: "ExternalField",
                dof: 2,
                nv: 2
            })
        );
    }

    #[test]
    fn try_install_refuses_a_model_that_has_a_passive_callback() {
        let mut model = chain(1);
        one(DummyDeterministic).install(&mut model);
        assert_eq!(
            one(DummyDeterministic).try_install(&mut model),
            Err(ThermostatError::AlreadyInstalled)
        );
    }

    #[test]
    #[should_panic(expected = "PassiveStack::install: DoubleWellPotential acts on DOF 2")]
    fn install_panics_on_a_refused_model() {
        one(DoubleWellPotential::new(1.0, 1.0, 2)).install(&mut chain(2));
    }

    #[test]
    #[should_panic(expected = "duplicate coupling: the pair (1, 0) appears twice")]
    fn pairwise_coupling_refuses_a_reversed_duplicate_edge() {
        let _coupling = PairwiseCoupling::new(vec![1.0, 1.0], vec![(0, 1), (1, 0)]);
    }

    // --- guards, noise reset, diagnostics, integrator ---

    fn thermostat_stack() -> Arc<PassiveStack> {
        one(LangevinThermostat::new(
            DVector::from_element(1, 0.5),
            1.0,
            7,
            0,
        ))
    }

    fn noise_on(stack: &PassiveStack) -> bool {
        stack.components()[0]
            .as_stochastic()
            .unwrap()
            .is_stochastic_active()
    }

    /// Guards dropped in the order they were taken, and in the other order, both keep noise
    /// off until the last one drops, then restore it.
    #[test]
    fn guards_keep_noise_off_until_the_last_drops_in_either_order() {
        for first_taken_drops_first in [false, true] {
            let stack = thermostat_stack();
            let a = stack.disable_stochastic();
            let b = stack.disable_stochastic();
            if first_taken_drops_first {
                drop(a);
                assert!(
                    !noise_on(&stack),
                    "noise came back with a guard still alive"
                );
                drop(b);
            } else {
                drop(b);
                assert!(
                    !noise_on(&stack),
                    "noise came back with a guard still alive"
                );
                drop(a);
            }
            assert!(
                noise_on(&stack),
                "noise stayed off after every guard dropped"
            );
        }
    }

    /// A component that was already off before the first guard stays off after the last.
    #[test]
    fn guards_restore_a_component_that_was_already_off() {
        let stack = thermostat_stack();
        stack.set_all_stochastic(false);
        let a = stack.disable_stochastic();
        let b = stack.disable_stochastic();
        drop(a);
        drop(b);
        assert!(!noise_on(&stack));
    }

    /// After `reset_stochastic`, the thermostat draws its first step's noise again.
    #[test]
    fn reset_stochastic_replays_the_noise() {
        let model = chain(1);
        let data = model.make_data();
        let stack = thermostat_stack();
        let draw = || {
            let mut qfrc = DVector::zeros(1);
            stack.components()[0].apply(&model, &data, &mut qfrc);
            qfrc[0]
        };
        let first = draw();
        let second = draw();
        assert_ne!(
            first.to_bits(),
            second.to_bits(),
            "two steps drew the same noise"
        );
        stack.reset_stochastic();
        assert_eq!(
            draw().to_bits(),
            first.to_bits(),
            "reset did not replay step 0"
        );
    }

    /// Every component in the crate answers `as_diagnose`.
    #[test]
    fn every_component_has_a_diagnostic_view() {
        let stack = PassiveStack::builder()
            .with(LangevinThermostat::new(
                DVector::from_element(1, 0.5),
                1.0,
                7,
                0,
            ))
            .with(DoubleWellPotential::new(1.0, 1.0, 0))
            .with(PairwiseCoupling::chain(2, 0.5))
            .with(ExternalField::new(vec![0.1]))
            .with(OscillatingField::new(1.0, 1.0, 0.0, 0))
            .with(RatchetPotential::new(1.0, 0.25, 0.0, 1.0, 0, 0))
            .build();
        let summaries: Vec<String> = stack
            .components()
            .iter()
            .filter_map(|c| c.as_diagnose().map(Diagnose::diagnostic_summary))
            .collect();
        assert_eq!(summaries.len(), 6);
        assert!(summaries[0].starts_with("LangevinThermostat"));
    }

    /// The thermostat refuses RK4; a deterministic component does not.
    #[test]
    fn the_thermostat_refuses_rk4() {
        let rk4 = || {
            let mut model = chain(1);
            model.integrator = sim_core::Integrator::RungeKutta4;
            model
        };
        assert!(matches!(
            verdict(
                LangevinThermostat::new(DVector::from_element(1, 0.5), 1.0, 7, 0),
                rk4()
            ),
            Err(ThermostatError::UnsupportedIntegrator {
                component: "LangevinThermostat",
                ..
            })
        ));
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 0), rk4()),
            Ok(())
        );
    }
}
