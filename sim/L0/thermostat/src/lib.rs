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
//!    `(&self, &Model, &Data, &mut DVector<f64>)` enforces that a passive
//!    component reads `Data` immutably and writes only to a per-DOF
//!    accumulator. Mutable access to `Data` is **uncompilable**, not just
//!    discouraged.
//! 2. **Composition** ([`PassiveStack`], [`PassiveStackBuilder`],
//!    [`StochasticGuard`], plus `sim_core::batch::EnvBatch`) — a
//!    builder-style stack that
//!    installs (`try_install`) as a single `cb_passive` callback. The stack drives the
//!    split-borrow dance between `Fn(&Model, &mut Data)` (the real
//!    `cb_passive` shape) and the trait's `&Data + &mut DVector<f64>` shape,
//!    so component authors never touch raw borrowing.
//! 3. **Production components** (e.g. [`LangevinThermostat`],
//!    [`DoubleWellPotential`], [`PairwiseCoupling`], [`ExternalField`],
//!    [`OscillatingField`], [`RatchetPotential`], [`ColoredDriveSim`],
//!    [`GibbsSampler`], [`IsingLearner`]) — the building blocks for
//!    thermodynamic computing simulations. [`IsingProblem`] puts an Ising
//!    problem or a QUBO onto a coupled array of double wells, and
//!    [`SpinLatch`] keeps the lowest-energy configuration a run reads.
//!
//! `sim-core` does **not** depend on any `rand` crate — that property is the
//! load-bearing reason this crate exists as a sibling crate rather than as a
//! `sim-core` module. Stochasticity is opt-in by depending on this crate.
//!
//! ## Quick start
//!
//! ```
//! use sim_core::DVector;
//! use sim_thermostat::{LangevinThermostat, PassiveStack};
//!
//! // Bring your own Model; see "The model" below.
//! let mut model = sim_core::test_fixtures::bistable_chain(1);
//! let mut data = model.make_data();
//!
//! PassiveStack::builder()
//!     .with(LangevinThermostat::new(
//!         DVector::from_element(model.nv, 0.1),
//!         1.0,
//!         42,
//!         0,
//!     ))
//!     .build()
//!     .try_install(&mut model)?;
//!
//! for _ in 0..1_000 {
//!     data.step(&model)?;
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## The model
//!
//! The components act on DOFs by index (entry `i` on DOF `i`) and read
//! positions through each DOF's joint, so the usual model is one slide joint
//! per element, with no gravity or contacts, under the Euler integrator.
//! `sim_therm_env::generate_mjcf` writes such a model as MJCF for
//! `sim_mjcf::load_model`; both are reachable through the `sim` crate, whose
//! docs run an annealing example end to end. The examples here use
//! `sim_core::test_fixtures::bistable_chain`, which needs sim-core's
//! `test-fixtures` feature.

mod baoab;
mod colored_drive;
mod component;
mod diagnose;
mod double_well;
mod error;
mod external_field;
mod gibbs;
pub mod ising;
mod ising_learner;
mod ising_problem;
mod langevin;
mod oscillating_field;
mod pairwise_coupling;
mod params;
pub mod prf;
mod ratchet;
mod reference_integrator;
mod stack;
mod well_state;

pub mod test_utils;

pub use baoab::Baoab1D;
pub use colored_drive::ColoredDriveSim;
pub use component::{PassiveComponent, Stochastic};
pub use diagnose::Diagnose;
pub use double_well::DoubleWellPotential;
pub use error::ThermostatError;
pub use external_field::ExternalField;
pub use gibbs::GibbsSampler;
pub use ising_learner::{IsingLearner, IsingTarget, LearnerConfig, LearningRecord};
pub use ising_problem::{IsingProblem, SpinLatch};
pub use langevin::LangevinThermostat;
pub use oscillating_field::OscillatingField;
pub use pairwise_coupling::PairwiseCoupling;
pub use ratchet::RatchetPotential;
pub use stack::{PassiveStack, PassiveStackBuilder, StochasticGuard};
pub use well_state::WellState;
