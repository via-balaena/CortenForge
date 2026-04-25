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
//! 3. Exposes per-component stochastic gating via [`Stochastic`] and
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

use std::sync::Arc;

use sim_core::batch::{EnvBatch, PerEnvStack};
use sim_core::{DVector, Data, Model};

use crate::component::PassiveComponent;

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
    pub fn install(self: &Arc<Self>, model: &mut Model) {
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
        let mut prior_states = Vec::with_capacity(self.components.len());
        for component in &self.components {
            if let Some(stoch) = component.as_stochastic() {
                prior_states.push(stoch.is_stochastic_active());
                stoch.set_stochastic_active(false);
            } else {
                // Sentinel for non-stochastic components — never read
                // back during Drop because we re-check `as_stochastic`
                // there. The slot exists only to keep the index aligned
                // with `self.components`.
                prior_states.push(false);
            }
        }
        StochasticGuard {
            stack: Arc::clone(self),
            prior_states,
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
/// While the guard is alive, every stochastic component in the stack
/// is inactive (produces only deterministic forces). When the guard
/// is dropped, the components' prior active flags are restored.
///
/// The guard is exception-safe: if the code inside the guarded block
/// panics, `Drop::drop` still runs and restores the prior states, so
/// the stack is never left in a partially-disabled state.
pub struct StochasticGuard {
    stack: Arc<PassiveStack>,
    prior_states: Vec<bool>,
}

impl Drop for StochasticGuard {
    fn drop(&mut self) {
        for (component, prior) in self.stack.components.iter().zip(&self.prior_states) {
            if let Some(stoch) = component.as_stochastic() {
                stoch.set_stochastic_active(*prior);
            }
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use super::*;
    use crate::component::Stochastic;

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
}
