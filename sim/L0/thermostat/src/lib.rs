//! # sim-thermostat — Langevin thermostat + passive-component framework
//!
//! `sim-thermostat` is the bolt-on layer that turns a deterministic CortenForge
//! simulation into a stochastic thermal one. It provides a small framework for
//! composing passive forces (`PassiveComponent` + `PassiveStack`) plus a
//! production `LangevinThermostat` implementation that ships the
//! fluctuation–dissipation pair `(−γ·v, σ·z)` into `data.qfrc_passive` via the
//! `cb_passive` user-callback hook.
//!
//! ## Architecture
//!
//! The crate has three layers:
//!
//! 1. **Component contracts** ([`PassiveComponent`], [`Stochastic`],
//!    [`Diagnose`]) — small traits that anything bolting onto a `Model` via
//!    `cb_passive` must implement. The `apply` signature
//!    `(&self, &Model, &Data, &mut DVector<f64>)` enforces M5: a passive
//!    component reads `Data` immutably and writes only to a per-DOF
//!    accumulator. Mutable access to `Data` is **uncompilable**, not just
//!    discouraged.
//! 2. **Composition** ([`PassiveStack`], [`PassiveStackBuilder`],
//!    [`StochasticGuard`], [`EnvBatch`]) — a builder-style stack that
//!    `install`s as a single `cb_passive` callback. The stack drives the
//!    split-borrow dance between `Fn(&Model, &mut Data)` (the real
//!    `cb_passive` shape) and the trait's `&Data + &mut DVector<f64>` shape,
//!    so component authors never touch raw borrowing.
//! 3. **Production component** ([`LangevinThermostat`]) — Euler-Maruyama
//!    Langevin thermostat with FDT-paired noise, an owned `ChaCha8Rng` (bit-
//!    stable across `rand`/`rand_chacha` versions), and integration with the
//!    `Stochastic` gating trait so finite-difference / autograd contexts can
//!    reversibly switch noise off via [`PassiveStack::disable_stochastic`].
//!
//! `sim-core` does **not** depend on any `rand` crate — that property is the
//! load-bearing reason this crate exists as a sibling crate rather than as a
//! `sim-core` module. Stochasticity is opt-in by depending on this crate.
//!
//! ## Quick start
//!
//! ```ignore
//! use sim_core::DVector;
//! use sim_thermostat::{LangevinThermostat, PassiveStack};
//!
//! let mut model = sim_mjcf::load_model(SHO_XML)?;
//! let mut data  = model.make_data();
//!
//! PassiveStack::builder()
//!     .with(LangevinThermostat::new(
//!         DVector::from_element(model.nv, 0.1),
//!         1.0,
//!         42,
//!     ))
//!     .build()
//!     .install(&mut model);
//!
//! for _ in 0..n_steps {
//!     data.step(&model)?;
//! }
//! # Ok::<(), sim_mjcf::LoadError>(())
//! ```
//!
//! ## See also
//!
//! - [`docs/thermo_computing/02_foundations/chassis_design.md`](https://github.com/bigmark222/cortenforge/blob/main/docs/thermo_computing/02_foundations/chassis_design.md)
//!   — the seven bolt-pattern decisions this crate inherits.
//! - [`docs/thermo_computing/03_phases/01_langevin_thermostat.md`](https://github.com/bigmark222/cortenforge/blob/main/docs/thermo_computing/03_phases/01_langevin_thermostat.md)
//!   — the Phase 1 spec this crate implements.

mod component;
mod diagnose;
mod langevin;
mod stack;

pub mod test_utils;
