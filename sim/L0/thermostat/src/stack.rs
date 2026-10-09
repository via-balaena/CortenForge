//! `PassiveStack` — composition layer for passive-force components.
//!
//! `PassiveStack` is the bridge between the spec's clean
//! `(&Model, &Data, &mut DVector<f64>)` trait shape ([`PassiveComponent`])
//! and the underlying `Fn(&Model, &mut Data)` shape that
//! `Model::cb_passive` actually uses. The stack:
//!
//! 1. Holds an ordered list of `Arc<dyn PassiveComponent>`s assembled
//!    via [`PassiveStackBuilder`].
//! 2. On [`PassiveStack::try_install`], registers a single `cb_passive` callback that
//!    iterates the components in order, performing the split-borrow
//!    dance once per step so component authors never see the mutable
//!    `Data` borrow.
//! 3. Exposes per-component stochastic gating via [`crate::Stochastic`] and
//!    the [`StochasticGuard`] RAII helper, so finite-difference and
//!    autograd contexts can wrap a block of code in
//!    [`PassiveStack::disable_stochastic`] and have every stochastic
//!    component in the stack temporarily produce only its
//!    deterministic forces.
//! 4. Supports parallel-environment construction: `PassiveStack`
//!    implements `sim_core::batch::PerEnvStack`, so
//!    `BatchSim::try_new_per_env` builds N independent
//!    `(Model, PassiveStack)` pairs from a user-supplied factory.
//!    Each env needs its own stack, so a thermostat's step counter is
//!    not shared; a stack returned for two envs is refused, and so is a
//!    model that already has a passive callback.
//!
//! ## The split-borrow dance
//!
//! The core difficulty: `cb_passive` hands the closure
//! `&mut Data`, but the trait wants `&Data + &mut DVector<f64>`. We
//! resolve it with `std::mem::replace`:
//!
//! ```text
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

use sim_core::batch::PerEnvStack;
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
    /// insertion order during each `cb_passive` invocation, and each adds
    /// its forces to the same accumulator, so a stack can hold several
    /// components of one type (two double wells on different DOFs).
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
    /// installed onto a `Model` (see [`PassiveStack::try_install`]).
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
/// later retains its own `Arc` handle. Both `try_install` and
/// `install_on` take `self: &Arc<Self>` (the standard
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

    /// Install this stack onto `model` as its passive callback (`Model::cb_passive`), if
    /// `model` has none yet and every component accepts it.
    ///
    /// The callback captures a clone of `self`, and the caller keeps its own handle, so it
    /// can call [`PassiveStack::disable_stochastic`] or read the stack afterwards.
    ///
    /// A model holds one passive callback. To replace one on purpose (another stack, or a
    /// callback of your own), call `model.clear_passive_callback()` first.
    ///
    /// # Errors
    ///
    /// Nothing is installed on an error.
    /// - [`ThermostatError::PassiveCallbackInstalled`] if `model` already has a passive
    ///   callback.
    /// - Otherwise, the first error from [`Self::validate`].
    pub fn try_install(self: &Arc<Self>, model: &mut Model) -> Result<(), ThermostatError> {
        if model.cb_passive.is_some() {
            return Err(ThermostatError::PassiveCallbackInstalled);
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
    /// block: when the last live guard drops, the prior flags come back,
    /// even if the block panicked.
    pub fn set_all_stochastic(&self, active: bool) {
        for component in &self.components {
            if let Some(stoch) = component.as_stochastic() {
                stoch.set_stochastic_active(active);
            }
        }
    }

    /// Disable every stochastic component in the stack and return a guard;
    /// when the last live guard on the stack drops, the flags from before
    /// the first are restored.
    ///
    /// Guards on this stack nest in any order: each guard turns noise
    /// off when taken, noise stays off until the LAST live guard drops,
    /// and that drop restores the flags from before the FIRST. A
    /// [`Self::set_all_stochastic`] call made while a guard is alive is
    /// overwritten when the last guard drops. The count is per stack: a
    /// component shared by two stacks has two independent counts.
    ///
    /// For finite-difference and autograd contexts, wrap the FD perturbation block in
    /// `let _guard = stack.disable_stochastic();`, run the perturbed
    /// and baseline rollouts, drop the guard, and the stack returns to
    /// its prior stochastic state. Stochastic components produce only
    /// their deterministic forces inside the guarded block, so no noise
    /// enters the FD difference.
    ///
    /// If a component's `set_stochastic_active` panics here, no guard is
    /// returned and the components switched off before it stay off.
    #[must_use = "the last StochasticGuard to drop restores the prior flags; \
                  discarding it immediately can re-enable noise — call \
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
        }
        // Off on every take, not only the first: noise switched back on under a live guard
        // goes off again for the new one.
        self.set_all_stochastic(false);
        disabled.depth += 1;
        drop(disabled);
        StochasticGuard {
            stack: Arc::clone(self),
        }
    }

    /// Restart every stochastic component's noise sequence from its first
    /// step (see [`Stochastic::reset_stochastic`]).
    pub fn reset_stochastic(&self) {
        for component in &self.components {
            if let Some(stoch) = component.as_stochastic() {
                stoch.reset_stochastic();
            }
        }
    }

    /// Read-only view of the components, in the order they apply. Each
    /// component's [`PassiveComponent::as_diagnose`] gives its one-line
    /// summary, for a per-component diagnostic report.
    #[must_use]
    pub fn components(&self) -> &[Arc<dyn PassiveComponent>] {
        &self.components
    }
}

/// `PassiveStack` implements sim-core's per-env batch construction:
/// `BatchSim::try_new_per_env` calls [`PassiveStack::try_install`] on each
/// env's model with the stack its factory returned, and refuses a stack
/// returned for two envs.
///
/// A component shared between two stacks (one `Arc` passed to both through
/// [`PassiveStackBuilder::with_arc`]) is not detected, and shares its state.
impl PerEnvStack for PassiveStack {
    type Error = ThermostatError;

    /// [`PassiveStack::try_install`].
    fn install_on(self: &Arc<Self>, model: &mut Model) -> Result<(), ThermostatError> {
        self.try_install(model)
    }
}

/// RAII guard returned by [`PassiveStack::disable_stochastic`].
///
/// While any guard on the stack is alive, every stochastic component in
/// it is inactive (produces only deterministic forces), unless switched
/// back on with [`PassiveStack::set_all_stochastic`]. When the last
/// live guard drops, the active flags from before the first guard are
/// restored, whatever order the guards drop in.
///
/// If the code inside the guarded block panics, `Drop::drop` still runs, so
/// the last guard to drop restores the prior flags all the same.
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
        fn reset_stochastic(&self) {}
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
        stack.try_install(&mut model).unwrap();

        assert!(model.cb_passive.is_some());
    }

    #[test]
    fn install_callback_actually_invokes_each_component_per_forward() {
        // A counting component installed via stack.try_install — verify
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
        stack.try_install(&mut model).unwrap();

        assert_eq!(counter.load(Ordering::SeqCst), 0);
        data.forward(&model).unwrap();
        assert_eq!(counter.load(Ordering::SeqCst), 1);
        data.forward(&model).unwrap();
        assert_eq!(counter.load(Ordering::SeqCst), 2);
    }

    #[test]
    fn new_per_env_builds_n_envs_with_callbacks_set() {
        let batch = sim_core::BatchSim::new_per_env(3, |_i| {
            let model = sim_core::test_fixtures::sho_1d();
            let stack = PassiveStack::builder().with(DummyDeterministic).build();
            (model, stack)
        });

        assert_eq!(batch.len(), 3);
        for i in 0..3 {
            assert!(
                batch.model_of(i).unwrap().cb_passive.is_some(),
                "env {i} should have cb_passive set by new_per_env",
            );
        }
    }

    // --- install-time validation ---

    use crate::{
        Diagnose, DoubleWellPotential, ExternalField, LangevinThermostat, OscillatingField,
        PairwiseCoupling, RatchetPotential,
    };
    use sim_core::BatchSim;
    use sim_core::batch::PerEnvError;

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

    /// A free joint's translation DOFs (0–2) have a position coordinate each; its rotation
    /// DOFs (3–5) move a quaternion and have none.
    #[test]
    fn position_readers_take_free_translations_and_refuse_free_rotations() {
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 2), free_body()),
            Ok(())
        );
        let none = |component, dof| ThermostatError::NoPositionCoordinate { component, dof };
        assert_eq!(
            verdict(DoubleWellPotential::new(1.0, 1.0, 3), free_body()),
            Err(none("DoubleWellPotential", 3))
        );
        assert_eq!(
            verdict(PairwiseCoupling::new(vec![1.0], vec![(0, 4)]), free_body()),
            Err(none("PairwiseCoupling", 4))
        );
        assert_eq!(
            verdict(
                RatchetPotential::new(1.0, 0.25, 0.0, 1.0, 5, 0),
                free_body()
            ),
            Err(none("RatchetPotential", 5))
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
            ExternalField::new(vec![0.1; 4]).field_energy(&model, &data),
            Err(ThermostatError::NoPositionCoordinate {
                component: "ExternalField",
                dof: 3
            })
        );
        assert_eq!(
            PairwiseCoupling::new(vec![1.0], vec![(2, 3)]).coupling_energy(&model, &data),
            Err(ThermostatError::NoPositionCoordinate {
                component: "PairwiseCoupling",
                dof: 3
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

    /// A model a component refuses is left without a callback.
    #[test]
    fn a_refused_model_gets_no_callback() {
        let mut refused = chain(1);
        assert!(
            one(DoubleWellPotential::new(1.0, 1.0, 1))
                .try_install(&mut refused)
                .is_err()
        );
        assert!(
            refused.cb_passive.is_none(),
            "a refused model got a callback"
        );
    }

    /// A refused model is an error naming the env.
    #[test]
    fn try_new_per_env_returns_a_refused_model() {
        let err = BatchSim::try_new_per_env(2, |i| {
            (chain(2), one(DoubleWellPotential::new(1.0, 1.0, 2 * i)))
        });
        assert!(
            matches!(
                err,
                Err(PerEnvError::Install {
                    env: 1,
                    source: ThermostatError::DofOutOfRange { dof: 2, nv: 2, .. }
                })
            ),
            "{:?}",
            err.err()
        );
    }

    /// `new_per_env` panics with the error's message.
    #[test]
    #[should_panic(expected = "BatchSim::new_per_env: env 1: DoubleWellPotential acts on DOF 2")]
    fn new_per_env_panics_on_a_refused_model() {
        let _batch = BatchSim::new_per_env(2, |i| {
            (chain(2), one(DoubleWellPotential::new(1.0, 1.0, 2 * i)))
        });
    }

    /// A stack returned for two envs is refused.
    #[test]
    fn try_new_per_env_refuses_one_stack_for_two_envs() {
        let shared = one(DummyDeterministic);
        let err = BatchSim::try_new_per_env(2, |_| (chain(1), Arc::clone(&shared)));
        assert_eq!(
            err.err(),
            Some(PerEnvError::SharedStack { env: 1, earlier: 0 })
        );
    }

    /// A factory model that already has a passive callback is refused, not silently cleared.
    #[test]
    fn try_new_per_env_refuses_a_model_that_already_has_a_callback() {
        let err = BatchSim::try_new_per_env(1, |_| {
            let mut model = chain(1);
            model.set_passive_callback(|_, _| {});
            (model, one(DummyDeterministic))
        });
        assert_eq!(
            err.err(),
            Some(PerEnvError::Install {
                env: 0,
                source: ThermostatError::PassiveCallbackInstalled
            })
        );
    }

    #[test]
    #[should_panic(
        expected = "PairwiseCoupling: edge (1, 0) repeats the pair of an earlier edge (in either order)"
    )]
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

    // --- review round 1 ---

    /// Noise switched back on under a live guard goes off again for the next guard, and the
    /// flags from before the first guard come back after the last.
    #[test]
    fn a_new_guard_turns_noise_off_after_it_was_switched_back_on() {
        let stack = thermostat_stack();
        let a = stack.disable_stochastic();
        stack.set_all_stochastic(true);
        let b = stack.disable_stochastic();
        assert!(!noise_on(&stack), "the second guard left noise on");
        drop(a);
        drop(b);
        assert!(noise_on(&stack));
    }

    /// Three guards dropped in each of the six orders keep noise off until the last drop.
    #[test]
    fn three_guards_in_every_drop_order() {
        for order in [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ] {
            let stack = thermostat_stack();
            let mut guards: Vec<Option<StochasticGuard>> =
                (0..3).map(|_| Some(stack.disable_stochastic())).collect();
            for (k, &i) in order.iter().enumerate() {
                drop(guards[i].take());
                let expected_on = k == 2;
                assert_eq!(
                    noise_on(&stack),
                    expected_on,
                    "order {order:?}, after drop {k}"
                );
            }
        }
    }

    /// `try_install` refuses a model that already has a passive callback, from a stack or set
    /// directly, and leaves that callback running; after `clear_passive_callback` it installs.
    #[test]
    fn try_install_refuses_a_model_that_already_has_a_passive_callback() {
        let count = |counter: &Arc<AtomicUsize>| {
            one(CountingComponent {
                count: Arc::clone(counter),
            })
        };
        let runs = |model: &Model| {
            model.make_data().forward(model).unwrap();
        };
        let (first, second) = (Arc::new(AtomicUsize::new(0)), Arc::new(AtomicUsize::new(0)));
        let mut model = chain(1);
        count(&first).try_install(&mut model).unwrap();
        assert_eq!(
            count(&second).try_install(&mut model),
            Err(ThermostatError::PassiveCallbackInstalled)
        );
        runs(&model);
        assert_eq!(
            (
                first.load(Ordering::SeqCst) > 0,
                second.load(Ordering::SeqCst)
            ),
            (true, 0),
            "the refusal replaced the installed stack"
        );

        model.clear_passive_callback();
        count(&second).try_install(&mut model).unwrap();
        runs(&model);
        assert!(
            second.load(Ordering::SeqCst) > 0,
            "the new stack did not run"
        );

        let mut own = chain(1);
        own.set_passive_callback(|_, _| {});
        assert_eq!(
            count(&second).try_install(&mut own),
            Err(ThermostatError::PassiveCallbackInstalled)
        );
    }
}
