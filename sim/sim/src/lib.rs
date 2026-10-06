//! Headless simulation & differentiable co-design toolkit.
//!
//! This umbrella crate re-exports the headless CortenForge simulation spine
//! under one dependency, providing a unified entry point for physics
//! simulation, differentiable rollouts, and gradient-based co-design. Every
//! re-exported crate is Bevy-free and GPU-free (`sim-soft` is pulled without
//! its `gpu-probe` feature), so this toolkit runs in CLI tools, WASM, servers,
//! or headless training loops.
//!
//! # Module Organization
//!
//! ## Foundation
//! - [`types`] — shared simulation types: `BodyId`, `Pose`, `SimulationConfig`, `SimError`.
//!
//! ## Physics engines
//! - [`core`] — rigid-body dynamics (MuJoCo-compatible forward/inverse).
//! - [`soft`] — backward-Euler soft-body FEM with implicit-function-theorem gradients.
//! - [`coupling`] — staggered forward soft↔rigid coupling with one
//!   `tape.backward` across both engines.
//!
//! ## Model I/O
//! - [`mjcf`] — MJCF (MuJoCo XML) model loading.
//! - [`urdf`] — URDF model loading.
//!
//! ## Learning & optimization
//! - [`ml_chassis`] — autograd / `Policy` / `VecEnv` training chassis.
//! - [`rl`] — reinforcement-learning algorithms (CEM/PPO/TD3/SAC).
//! - [`opt`] — black-box optimizers (SA / parallel tempering).
//! - [`thermostat`] — thermodynamic-computing primitives.
//! - [`therm_env`] — thermodynamic training environments.
//!
//! # Quick Start
//!
//! ```
//! use sim::core::MjStage;
//! use sim::coupling::StaggeredCoupling;
//! // Reach every engine through one dependency: `sim::core`, `sim::soft`,
//! // `sim::coupling`, `sim::rl`, …
//! ```
//!
//! # Annealing a QUBO on a thermodynamic circuit
//!
//! Load one slide particle per variable from MJCF, put a QUBO on the array
//! as coupled double wells, cool it through a temperature control, and keep
//! the lowest-energy configuration it visits. The QUBO here is a maximum
//! independent set on the path 0–1–2: each pick earns −1, two neighbours
//! picked together cost 2, so the answer is `x = (1, 0, 1)` at `E = −2`.
//!
//! ```
//! use sim::core::DVector;
//! use sim::mjcf::load_model;
//! use sim::therm_env::generate_mjcf;
//! use sim::thermostat::{IsingProblem, LangevinThermostat, PassiveStack, SpinLatch};
//!
//! let (problem, offset) =
//!     IsingProblem::from_qubo(&[-1.0, -1.0, -1.0], &[(0, 1), (1, 2)], &[2.0, 2.0]);
//!
//! // Three slide particles, a 1 ms step, and one control channel for the temperature.
//! let mut model = load_model(&generate_mjcf(3, 1, 0.001, (0.0, 10.0)))?;
//! problem
//!     .add_components(PassiveStack::builder(), 3.0, 1.0)
//!     .with(LangevinThermostat::new(DVector::from_element(3, 1.0), 1.0, 7, 0).with_ctrl_temperature(0))
//!     .build()
//!     .try_install(&mut model)?;
//!
//! let mut data = model.make_data();
//! let mut latch = SpinLatch::new(problem, 0.5);
//! let steps = 100_000;
//! for step in 0..steps {
//!     // Cool linearly from 5 kT to 0.
//!     data.ctrl[0] = 5.0 * f64::from(steps - step) / f64::from(steps);
//!     data.step(&model)?;
//!     latch.observe(&model, &data)?;
//! }
//!
//! let (spins, energy) = latch.best().ok_or("no configuration was read")?;
//! let x: Vec<f64> = spins.iter().map(|s| (1.0 + s) / 2.0).collect();
//! assert_eq!(x, [1.0, 0.0, 1.0]);
//! assert_eq!(energy + offset, -2.0);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

// =============================================================================
// Re-exports
// =============================================================================

/// Shared simulation types: `BodyId`, `Pose`, `SimulationConfig`, `SimError`.
pub use sim_types as types;

/// Rigid-body dynamics (MuJoCo-compatible forward/inverse).
pub use sim_core as core;

/// Backward-Euler soft-body FEM with implicit-function-theorem gradients.
pub use sim_soft as soft;

/// The L1 keystone: staggered forward soft↔rigid coupling.
pub use sim_coupling as coupling;

/// MJCF (MuJoCo XML) model loading.
pub use sim_mjcf as mjcf;

/// URDF model loading.
pub use sim_urdf as urdf;

/// Autograd / `Policy` / `VecEnv` training chassis.
pub use sim_ml_chassis as ml_chassis;

/// Reinforcement-learning algorithms (CEM/PPO/TD3/SAC).
pub use sim_rl as rl;

/// Black-box optimizers (SA / parallel tempering / rematch).
pub use sim_opt as opt;

/// Thermodynamic-computing primitives.
pub use sim_thermostat as thermostat;

/// Thermodynamic training environments.
pub use sim_therm_env as therm_env;

// =============================================================================
// Prelude
// =============================================================================

/// Common imports for headless simulation.
///
/// # Usage
///
/// ```
/// use sim::prelude::*;
/// ```
pub mod prelude {
    // Foundation types
    pub use sim_types::{BodyId, Pose, SimError, SimulationConfig};

    // The coupling keystone driver
    pub use sim_coupling::StaggeredCoupling;
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn module_reexports_are_accessible() {
        // Zero-assumption reachability: naming the re-exported type paths is
        // enough to prove the umbrella wires every crate through. Note the
        // `sim_core as core` re-export shadows `::core`, so use `std::mem`.
        assert!(std::mem::size_of::<types::SimulationConfig>() < usize::MAX);
        assert!(std::mem::size_of::<types::SimError>() < usize::MAX);
        assert!(std::mem::size_of::<coupling::CoupledStep>() < usize::MAX);
    }

    #[test]
    fn prelude_imports_resolve() {
        use prelude::*;
        assert!(std::mem::size_of::<SimulationConfig>() < usize::MAX);
        assert!(std::mem::size_of::<BodyId>() < usize::MAX);
    }
}
