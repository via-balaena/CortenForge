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
    /// Integration method for dynamics.
    pub integration: IntegrationMethod,
    /// Number of velocity iterations for constraint solving.
    pub velocity_iterations: usize,
    /// Number of position iterations for constraint solving.
    pub position_iterations: usize,
    /// Contact tolerance (penetration below this is acceptable).
    pub contact_tolerance: f64,
    /// Velocity threshold below which objects are considered at rest.
    pub sleep_threshold: f64,
    /// Whether to allow bodies to sleep (optimization).
    pub allow_sleeping: bool,
    /// Coefficient of restitution (bounciness) for contacts.
    pub default_restitution: f64,
    /// Coefficient of friction for contacts.
    pub default_friction: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            integration: IntegrationMethod::SemiImplicitEuler,
            velocity_iterations: 8,
            position_iterations: 4,
            contact_tolerance: 0.001,
            sleep_threshold: 0.01,
            allow_sleeping: true,
            default_restitution: 0.3,
            default_friction: 0.5,
        }
    }
}

impl SolverConfig {
    /// Create a high-accuracy solver configuration.
    #[must_use]
    pub fn high_accuracy() -> Self {
        Self {
            integration: IntegrationMethod::RungeKutta4,
            velocity_iterations: 16,
            position_iterations: 8,
            contact_tolerance: 0.0001,
            sleep_threshold: 0.001,
            allow_sleeping: false,
            ..Default::default()
        }
    }

    /// Create a fast solver configuration.
    #[must_use]
    pub fn fast() -> Self {
        Self {
            integration: IntegrationMethod::SemiImplicitEuler,
            velocity_iterations: 4,
            position_iterations: 2,
            contact_tolerance: 0.005,
            sleep_threshold: 0.05,
            allow_sleeping: true,
            ..Default::default()
        }
    }

    /// Set the integration method.
    #[must_use]
    pub fn integration(mut self, method: IntegrationMethod) -> Self {
        self.integration = method;
        self
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

/// Integration method for dynamics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum IntegrationMethod {
    /// Explicit Euler (first-order, unstable for stiff systems).
    ExplicitEuler,
    /// Semi-implicit Euler (symplectic, good for games).
    SemiImplicitEuler,
    /// Velocity Verlet (second-order, symplectic).
    VelocityVerlet,
    /// 4th-order Runge-Kutta (high accuracy, expensive).
    RungeKutta4,
}

impl IntegrationMethod {
    /// Get the order of accuracy for this method.
    #[must_use]
    pub const fn order(self) -> usize {
        match self {
            Self::ExplicitEuler | Self::SemiImplicitEuler => 1,
            Self::VelocityVerlet => 2,
            Self::RungeKutta4 => 4,
        }
    }

    /// Check if this method is symplectic (energy-preserving).
    #[must_use]
    pub const fn is_symplectic(self) -> bool {
        matches!(self, Self::SemiImplicitEuler | Self::VelocityVerlet)
    }

    /// Get approximate relative computational cost (1 = cheapest).
    #[must_use]
    pub const fn relative_cost(self) -> usize {
        match self {
            Self::ExplicitEuler | Self::SemiImplicitEuler => 1,
            Self::VelocityVerlet => 2,
            Self::RungeKutta4 => 4,
        }
    }
}

impl std::fmt::Display for IntegrationMethod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ExplicitEuler => write!(f, "Explicit Euler"),
            Self::SemiImplicitEuler => write!(f, "Semi-Implicit Euler"),
            Self::VelocityVerlet => write!(f, "Velocity Verlet"),
            Self::RungeKutta4 => write!(f, "RK4"),
        }
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
        assert_eq!(hifi.integration, IntegrationMethod::RungeKutta4);
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
    fn test_integration_method() {
        assert_eq!(IntegrationMethod::ExplicitEuler.order(), 1);
        assert_eq!(IntegrationMethod::RungeKutta4.order(), 4);

        assert!(IntegrationMethod::SemiImplicitEuler.is_symplectic());
        assert!(IntegrationMethod::VelocityVerlet.is_symplectic());
        assert!(!IntegrationMethod::ExplicitEuler.is_symplectic());
        assert!(!IntegrationMethod::RungeKutta4.is_symplectic());
    }

    #[test]
    fn test_frequency() {
        let config = SimulationConfig::with_timestep(0.01);
        assert_relative_eq!(config.frequency(), 100.0, epsilon = 1e-10);
    }
}
