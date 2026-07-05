//! Headless simulation & differentiable co-design toolkit.
//!
//! This umbrella crate re-exports the headless CortenForge simulation spine
//! under one dependency, providing a unified entry point for physics
//! simulation, differentiable rollouts, and gradient-based co-design. Every
//! re-exported crate is Bevy-free and GPU-free (`sim-soft` is pulled without
//! its `gpu-probe` feature), so this toolkit runs in CLI tools, WASM, servers,
//! or headless training loops.
//!
//! Bevy/GUI crates (`sim/L1/bevy*`) and the wgpu-backed `sim-gpu` are
//! deliberately excluded — depend on those directly when you need rendering.
//!
//! # Module Organization
//!
//! ## Foundation
//! - [`types`] — shared simulation types: `BodyId`, `Pose`, `SimulationConfig`, `SimError`.
//!
//! ## Physics engines
//! - [`core`] — rigid-body dynamics (MuJoCo-compatible forward/inverse).
//! - [`soft`] — backward-Euler soft-body FEM with implicit-function-theorem gradients.
//! - [`coupling`] — the L1 keystone: staggered forward soft↔rigid coupling with
//!   one `tape.backward` across both engines.
//!
//! ## Model I/O
//! - [`mjcf`] — MJCF (MuJoCo XML) model loading.
//! - [`urdf`] — URDF model loading.
//!
//! ## Learning & optimization
//! - [`ml_chassis`] — autograd / `Policy` / `VecEnv` training chassis.
//! - [`rl`] — reinforcement-learning algorithms (CEM/PPO/TD3/SAC).
//! - [`opt`] — black-box optimizers (SA / parallel tempering / rematch).
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

#![doc(html_root_url = "https://docs.rs/sim/1.0.0")]

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
