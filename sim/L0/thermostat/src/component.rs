//! Passive-component trait contracts.
//!
//! This module defines the two traits that any passive force injector
//! installed onto a `Model` via `cb_passive` must implement:
//!
//! - [`PassiveComponent`] is the M5 contract — it gives a component
//!   immutable access to `Model`/`Data` and a single `&mut DVector<f64>`
//!   accumulator to write its per-DOF contribution into. Mutable access
//!   to `Data` is **uncompilable** at this layer, not just discouraged;
//!   the [`crate::PassiveStack`](crate) wrapper drives the split-borrow
//!   dance against the underlying `cb_passive: Fn(&Model, &mut Data)`
//!   shape so component authors never have to.
//!
//! - [`Stochastic`] is the Decision-7 gating opt-in trait. A component
//!   that injects random forces (`LangevinThermostat`, future
//!   colored-noise / GLE / Brownian-motor components) implements
//!   `Stochastic` and reports its current active flag via
//!   [`PassiveComponent::as_stochastic`]. Finite-difference and autograd
//!   contexts call [`crate::PassiveStack::disable_stochastic`](crate)
//!   to wrap the stochastic contribution off via an RAII guard, so the
//!   FD perturbation block recovers `∂F_det/∂qpos` exactly even though
//!   the component would normally be writing FDT noise into
//!   `qfrc_passive`.
//!
//! Both traits are `Send + Sync` because the resulting `cb_passive`
//! callback is stored in an `Arc<dyn Fn + Send + Sync>` on `Model`,
//! and `Model` is itself `Clone + Send + Sync` for `BatchSim` parallel
//! environments.

use sim_core::{DVector, Data, Model};

/// A passive force injector that writes into a per-DOF accumulator.
///
/// Implementors are bolted onto a `Model` indirectly via
/// [`crate::PassiveStack::install`](crate). The stack drives the split
/// from the underlying `Fn(&Model, &mut Data)` `cb_passive` shape into
/// the trait's `(&Model, &Data, &mut DVector<f64>)` shape, so an
/// implementor never observes the mutable `Data` borrow.
///
/// # The M5 contract
///
/// `apply` reads `model` and `data` immutably and accumulates its
/// per-DOF contribution **with `+=`**, never `=`, into `qfrc_out`.
/// `qfrc_out` is the same vector as `data.qfrc_passive` at runtime —
/// the stack temporarily takes ownership of it via `std::mem::take`
/// across the trait call so this signature is sound, and restores it
/// before the callback returns. Components must not assume `qfrc_out`
/// is zero on entry; earlier components in the same stack may have
/// already written, and `mj_passive` itself populates the buffer with
/// aggregated spring/damper/fluid/gravcomp contributions before
/// `cb_passive` fires.
///
/// # Object safety and trait bounds
///
/// `Send + Sync + 'static` are required because the component is
/// erased into `Arc<dyn PassiveComponent>` inside the stack and that
/// `Arc` is then captured by the `cb_passive` callback closure, which
/// is itself stored in `Model::cb_passive: Option<Callback<dyn Fn +
/// Send + Sync>>`. The full chain is verified by the unit tests in
/// this file.
pub trait PassiveComponent: Send + Sync + 'static {
    /// Read `model`/`data` and accumulate the component's per-DOF
    /// contribution into `qfrc_out` with `+=`.
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);

    /// Optional introspection hook for the [`Stochastic`] gating
    /// trait. Components that inject randomness override this to
    /// return `Some(self)`; deterministic components leave the default
    /// `None`. The stack uses this to implement
    /// [`crate::PassiveStack::disable_stochastic`](crate) without
    /// requiring `dyn` downcasting.
    fn as_stochastic(&self) -> Option<&dyn Stochastic> {
        None
    }
}

/// Decision-7 gating opt-in for stochastic passive components.
///
/// A component that writes random forces into `qfrc_out` implements
/// `Stochastic` and exposes a flag the stack can flip on or off
/// via interior mutability. When inactive, the component's `apply`
/// must produce only its **deterministic** contribution (e.g. just
/// `−γ·v` for a Langevin thermostat — the FDT noise term drops to
/// zero).
///
/// The flag is per-component, not per-step, so a single
/// [`crate::PassiveStack::disable_stochastic`](crate) call can wrap an
/// arbitrary block of code (an FD perturbation loop, an autograd
/// rollout, a derivative test) where every stochastic component in
/// the stack must produce exactly its deterministic forces. The RAII
/// guard returned by `disable_stochastic` restores the prior flag
/// values on drop, so the wrapping is exception-safe.
///
/// # Why `&self` and not `&mut self`
///
/// The stack hands components out as `Arc<dyn PassiveComponent>` —
/// shared, immutable references — so any mutable state on the
/// component must use interior mutability. `LangevinThermostat` uses
/// an `AtomicBool` for the active flag and a `Mutex<ChaCha8Rng>` for
/// the RNG; both are compatible with `&self`-only access.
pub trait Stochastic: Send + Sync {
    /// Set the active flag. `true` re-enables the stochastic part of
    /// `apply`; `false` disables it (deterministic-only forces).
    fn set_stochastic_active(&self, active: bool);

    /// Read the current active flag.
    fn is_stochastic_active(&self) -> bool;
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

    use sim_core::DVector;

    use super::*;

    /// A no-op deterministic component for trait-shape testing.
    struct DummyDeterministic {
        call_count: AtomicUsize,
    }

    impl PassiveComponent for DummyDeterministic {
        fn apply(&self, _model: &Model, _data: &Data, _qfrc_out: &mut DVector<f64>) {
            self.call_count.fetch_add(1, Ordering::SeqCst);
        }
        // as_stochastic uses the default — returns None.
    }

    /// A no-op stochastic component that toggles its active flag via
    /// an `AtomicBool`, mirroring the `LangevinThermostat` pattern.
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
    fn deterministic_default_as_stochastic_is_none() {
        let comp = DummyDeterministic {
            call_count: AtomicUsize::new(0),
        };
        assert!(
            comp.as_stochastic().is_none(),
            "deterministic components should leave as_stochastic default = None",
        );
    }

    #[test]
    fn stochastic_override_returns_self() {
        let comp = DummyStochastic {
            active: AtomicBool::new(true),
        };
        let stochastic_view = comp.as_stochastic().unwrap();
        assert!(stochastic_view.is_stochastic_active());
        stochastic_view.set_stochastic_active(false);
        assert!(!stochastic_view.is_stochastic_active());
        stochastic_view.set_stochastic_active(true);
        assert!(stochastic_view.is_stochastic_active());
    }

    #[test]
    fn passive_component_is_object_safe() {
        // Compile-time test: PassiveComponent and Stochastic must be
        // dyn-compatible. If either grew a `Self`-typed parameter or
        // a generic method, this would fail to compile.
        let _det: Box<dyn PassiveComponent> = Box::new(DummyDeterministic {
            call_count: AtomicUsize::new(0),
        });
        let stoch_box: Box<dyn PassiveComponent> = Box::new(DummyStochastic {
            active: AtomicBool::new(true),
        });
        let _stoch_view: &dyn Stochastic = stoch_box.as_stochastic().unwrap();
    }

    #[test]
    fn send_sync_static_bounds_hold() {
        // Compile-time test: PassiveComponent must be Send + Sync +
        // 'static so it can be Arc'd into a cb_passive callback.
        fn assert_send_sync_static<T: Send + Sync + 'static + ?Sized>() {}
        assert_send_sync_static::<dyn PassiveComponent>();
        assert_send_sync_static::<DummyDeterministic>();
        assert_send_sync_static::<DummyStochastic>();
    }
}
