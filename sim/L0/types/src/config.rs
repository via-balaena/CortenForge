//! Configuration types for simulation.
//!
//! This module provides configuration types that control how the simulation
//! runs: timestep, solver settings, integration method, etc.

use crate::dynamics::Gravity;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Main configuration for a simulation.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SimulationConfig {
    /// Fixed timestep for physics integration (seconds).
    pub timestep: f64,
    /// Gravity configuration.
    pub gravity: Gravity,
    /// Solver configuration.
    pub solver: SolverConfig,
    /// Maximum simulation time (None for unlimited).
    pub max_time: Option<f64>,
    /// Whether to detect and report contacts.
    pub enable_contacts: bool,
    /// Whether to compute and report system energy/momentum.
    pub compute_diagnostics: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            timestep: 1.0 / 240.0, // 240 Hz - typical for physics
            gravity: Gravity::earth(),
            solver: SolverConfig::default(),
            max_time: None,
            enable_contacts: true,
            compute_diagnostics: false,
        }
    }
}

impl SimulationConfig {
    /// Create a new simulation config with the given timestep.
    #[must_use]
    pub fn with_timestep(timestep: f64) -> Self {
        Self {
            timestep,
            ..Default::default()
        }
    }

    /// Create a configuration for real-time simulation (60 Hz).
    #[must_use]
    pub fn realtime() -> Self {
        Self {
            timestep: 1.0 / 60.0,
            ..Default::default()
        }
    }

    /// Create a configuration for high-fidelity simulation (1000 Hz).
    #[must_use]
    pub fn high_fidelity() -> Self {
        Self {
            timestep: 1.0 / 1000.0,
            solver: SolverConfig::high_accuracy(),
            compute_diagnostics: true,
            ..Default::default()
        }
    }

    /// Create a configuration for fast, low-fidelity simulation (30 Hz).
    #[must_use]
    pub fn fast() -> Self {
        Self {
            timestep: 1.0 / 30.0,
            solver: SolverConfig::fast(),
            ..Default::default()
        }
    }

    /// Set the gravity.
    #[must_use]
    pub fn gravity(mut self, gravity: Gravity) -> Self {
        self.gravity = gravity;
        self
    }

    /// Disable gravity (zero-G environment).
    #[must_use]
    pub fn zero_gravity(mut self) -> Self {
        self.gravity = Gravity::zero();
        self
    }

    /// Set the solver configuration.
    #[must_use]
    pub fn solver(mut self, solver: SolverConfig) -> Self {
        self.solver = solver;
        self
    }

    /// Set the maximum simulation time.
    #[must_use]
    pub fn max_time(mut self, max_time: f64) -> Self {
        self.max_time = Some(max_time);
        self
    }

    /// Enable diagnostic computation.
    #[must_use]
    pub fn with_diagnostics(mut self) -> Self {
        self.compute_diagnostics = true;
        self
    }

    /// Disable contact detection.
    #[must_use]
    pub fn without_contacts(mut self) -> Self {
        self.enable_contacts = false;
        self
    }

    /// Validate the configuration.
    pub fn validate(&self) -> crate::Result<()> {
        if !self.timestep.is_finite() || self.timestep <= 0.0 {
            return Err(crate::SimError::InvalidTimestep(self.timestep));
        }

        if self.timestep > 1.0 {
            return Err(crate::SimError::invalid_config(
                "timestep > 1 second is likely an error",
            ));
        }

        self.solver.validate()?;

        Ok(())
    }

    /// Get the frequency in Hz.
    #[must_use]
    pub fn frequency(&self) -> f64 {
        1.0 / self.timestep
    }
}

/// Configuration for the physics solver.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SolverConfig {
    /// Number of velocity iterations for constraint solving.
    pub velocity_iterations: usize,
    /// Number of position iterations for constraint solving.
    pub position_iterations: usize,
    /// Contact tolerance (penetration below this is acceptable).
    pub contact_tolerance: f64,
    /// Velocity threshold below which objects are considered at rest.
    ///
    /// Both linear velocity (m/s) and angular velocity (rad/s) must be
    /// below this threshold for the body to be considered stationary.
    pub sleep_threshold: f64,
    /// Time (in seconds) a body must remain below the sleep threshold
    /// before it is put to sleep.
    ///
    /// This hysteresis prevents bodies from sleeping immediately after
    /// a brief moment of low velocity, which helps avoid jittering.
    pub sleep_time_threshold: f64,
    /// Whether to allow bodies to sleep (optimization).
    pub allow_sleeping: bool,
    /// Coefficient of restitution (bounciness) for contacts.
    pub default_restitution: f64,
    /// Coefficient of friction for contacts.
    pub default_friction: f64,
    /// Configuration for parallel execution (requires `parallel` feature).
    pub parallel: ParallelConfig,
}

/// Configuration for parallel execution.
///
/// When the `parallel` feature is enabled in sim-core, these settings
/// control multi-threaded constraint solving and body integration.
///
/// # Note
///
/// Parallel execution provides speedups for scenes with multiple independent
/// constraint islands. For single-island scenes (e.g., a single articulated
/// robot), the overhead may not be worth it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ParallelConfig {
    /// Enable parallel constraint solving across independent islands.
    ///
    /// When enabled, constraint islands are solved concurrently using rayon.
    /// This provides significant speedups for scenes with multiple
    /// independent mechanisms (e.g., multiple robots, scattered objects).
    pub parallel_constraints: bool,

    /// Enable parallel body integration.
    ///
    /// When enabled, body position/velocity integration is parallelized.
    /// This is beneficial for scenes with many bodies.
    pub parallel_integration: bool,

    /// Minimum number of active islands to trigger parallel constraint solving.
    ///
    /// Parallel overhead is not worth it for single-island scenes.
    /// Increase this value if you observe slowdowns on small scenes.
    pub min_islands_for_parallel: usize,

    /// Minimum total bodies across all islands to trigger parallel integration.
    ///
    /// Small scenes don't benefit from parallel integration overhead.
    pub min_bodies_for_parallel: usize,
}

impl Default for ParallelConfig {
    fn default() -> Self {
        Self {
            // Disabled by default because parallel constraint solving uses the Newton
            // solver which may have slightly different behavior than the default solver.
            // Enable explicitly for performance in multi-island scenes.
            parallel_constraints: false,
            // Body integration is sequential anyway (HashMap limitation)
            parallel_integration: false,
            min_islands_for_parallel: 2,
            min_bodies_for_parallel: 8,
        }
    }
}

impl ParallelConfig {
    /// Create a configuration with parallelism disabled.
    #[must_use]
    pub const fn sequential() -> Self {
        Self {
            parallel_constraints: false,
            parallel_integration: false,
            min_islands_for_parallel: 2,
            min_bodies_for_parallel: 8,
        }
    }

    /// Create a configuration optimized for many islands.
    #[must_use]
    pub const fn many_islands() -> Self {
        Self {
            parallel_constraints: true,
            parallel_integration: true,
            min_islands_for_parallel: 2,
            min_bodies_for_parallel: 4,
        }
    }

    /// Set the minimum islands threshold.
    #[must_use]
    pub const fn with_min_islands(mut self, min: usize) -> Self {
        self.min_islands_for_parallel = min;
        self
    }

    /// Set the minimum bodies threshold.
    #[must_use]
    pub const fn with_min_bodies(mut self, min: usize) -> Self {
        self.min_bodies_for_parallel = min;
        self
    }
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 4,
            contact_tolerance: 0.001,
            sleep_threshold: 0.01,
            sleep_time_threshold: 0.5, // 0.5 seconds before sleeping
            allow_sleeping: true,
            default_restitution: 0.3,
            default_friction: 0.5,
            parallel: ParallelConfig::default(),
        }
    }
}

impl SolverConfig {
    /// Create a high-accuracy solver configuration.
    #[must_use]
    pub fn high_accuracy() -> Self {
        Self {
            velocity_iterations: 16,
            position_iterations: 8,
            contact_tolerance: 0.0001,
            sleep_threshold: 0.001,
            sleep_time_threshold: 1.0, // Longer time before sleeping for accuracy
            allow_sleeping: false,
            // Disable parallel for high accuracy (determinism)
            parallel: ParallelConfig::sequential(),
            ..Default::default()
        }
    }

    /// Create a fast solver configuration.
    #[must_use]
    pub fn fast() -> Self {
        Self {
            velocity_iterations: 4,
            position_iterations: 2,
            contact_tolerance: 0.005,
            sleep_threshold: 0.05,
            sleep_time_threshold: 0.2, // Faster sleep for performance
            allow_sleeping: true,
            // Enable parallel with lower thresholds for max performance
            parallel: ParallelConfig::many_islands(),
            ..Default::default()
        }
    }

    /// Set the number of solver iterations.
    #[must_use]
    pub fn iterations(mut self, velocity: usize, position: usize) -> Self {
        self.velocity_iterations = velocity;
        self.position_iterations = position;
        self
    }

    /// Set default material properties.
    #[must_use]
    pub fn materials(mut self, restitution: f64, friction: f64) -> Self {
        self.default_restitution = restitution.clamp(0.0, 1.0);
        self.default_friction = friction.max(0.0);
        self
    }

    /// Disable sleeping (all bodies always active).
    #[must_use]
    pub fn no_sleeping(mut self) -> Self {
        self.allow_sleeping = false;
        self
    }

    /// Validate the solver configuration.
    pub fn validate(&self) -> crate::Result<()> {
        if self.velocity_iterations == 0 {
            return Err(crate::SimError::invalid_config(
                "velocity_iterations must be at least 1",
            ));
        }

        if self.contact_tolerance < 0.0 {
            return Err(crate::SimError::invalid_config(
                "contact_tolerance cannot be negative",
            ));
        }

        if self.default_restitution < 0.0 || self.default_restitution > 1.0 {
            return Err(crate::SimError::invalid_config(
                "restitution must be between 0 and 1",
            ));
        }

        if self.default_friction < 0.0 {
            return Err(crate::SimError::invalid_config(
                "friction cannot be negative",
            ));
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_default_config() {
        let config = SimulationConfig::default();
        assert!(config.validate().is_ok());
        assert_relative_eq!(config.timestep, 1.0 / 240.0, epsilon = 1e-10);
        assert!(config.enable_contacts);
    }

    #[test]
    fn test_config_presets() {
        let realtime = SimulationConfig::realtime();
        assert_relative_eq!(realtime.timestep, 1.0 / 60.0, epsilon = 1e-10);

        let hifi = SimulationConfig::high_fidelity();
        assert_relative_eq!(hifi.timestep, 1.0 / 1000.0, epsilon = 1e-10);
        assert!(hifi.compute_diagnostics);

        let fast = SimulationConfig::fast();
        assert_relative_eq!(fast.timestep, 1.0 / 30.0, epsilon = 1e-10);
    }

    #[test]
    fn test_config_builder() {
        let config = SimulationConfig::with_timestep(0.001)
            .zero_gravity()
            .with_diagnostics()
            .max_time(10.0);

        assert_relative_eq!(config.timestep, 0.001, epsilon = 1e-10);
        assert_relative_eq!(config.gravity.acceleration.norm(), 0.0, epsilon = 1e-10);
        assert!(config.compute_diagnostics);
        assert_eq!(config.max_time, Some(10.0));
    }

    #[test]
    fn test_config_validation() {
        let mut config = SimulationConfig::default();
        assert!(config.validate().is_ok());

        config.timestep = -0.01;
        assert!(config.validate().is_err());

        config.timestep = 0.0;
        assert!(config.validate().is_err());

        config.timestep = f64::NAN;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_solver_config() {
        let solver = SolverConfig::default();
        assert!(solver.validate().is_ok());

        let hifi = SolverConfig::high_accuracy();
        assert!(!hifi.allow_sleeping);

        let fast = SolverConfig::fast();
        assert_eq!(fast.velocity_iterations, 4);
    }

    #[test]
    fn test_solver_validation() {
        let mut solver = SolverConfig::default();
        assert!(solver.validate().is_ok());

        solver.velocity_iterations = 0;
        assert!(solver.validate().is_err());

        solver.velocity_iterations = 8;
        solver.default_restitution = 1.5;
        assert!(solver.validate().is_err());

        solver.default_restitution = 0.5;
        solver.default_friction = -0.1;
        assert!(solver.validate().is_err());
    }

    #[test]
    fn test_frequency() {
        let config = SimulationConfig::with_timestep(0.01);
        assert_relative_eq!(config.frequency(), 100.0, epsilon = 1e-10);
    }
}
