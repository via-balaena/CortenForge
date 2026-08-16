//! Configuration types for simulation.
//!
//! This module provides configuration types that control how the simulation
//! runs: timestep, solver settings, integration method, etc.

use nalgebra::Vector3;

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

/// Gravity configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Gravity {
    /// Acceleration due to gravity (m/s²).
    pub acceleration: Vector3<f64>,
}

impl Default for Gravity {
    fn default() -> Self {
        Self::earth()
    }
}

impl Gravity {
    /// Standard Earth gravity (9.81 m/s² in -Z direction).
    #[must_use]
    pub fn earth() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -9.81),
        }
    }

    /// Moon gravity (1.62 m/s² in -Z direction).
    #[must_use]
    pub fn moon() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -1.62),
        }
    }

    /// Mars gravity (3.71 m/s² in -Z direction).
    #[must_use]
    pub fn mars() -> Self {
        Self {
            acceleration: Vector3::new(0.0, 0.0, -3.71),
        }
    }

    /// Zero gravity (microgravity).
    #[must_use]
    pub fn zero() -> Self {
        Self {
            acceleration: Vector3::zeros(),
        }
    }

    /// Custom gravity vector.
    #[must_use]
    pub fn custom(acceleration: Vector3<f64>) -> Self {
        Self { acceleration }
    }

    /// Compute the gravitational force on a body.
    #[must_use]
    pub fn force_on_mass(&self, mass: f64) -> Vector3<f64> {
        self.acceleration * mass
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_gravity() {
        let g = Gravity::earth();
        assert_relative_eq!(g.acceleration.z, -9.81, epsilon = 1e-10);

        let force = g.force_on_mass(2.0);
        assert_relative_eq!(force.z, -19.62, epsilon = 1e-10);
    }

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

    /// ★ The fourth of `SolverConfig::validate`'s branches — the other three
    /// are covered by `test_solver_validation`, this one was not, so the
    /// guard could have been deleted with the suite still green.
    #[test]
    fn solver_validation_rejects_a_negative_contact_tolerance() {
        let solver = SolverConfig {
            contact_tolerance: -1e-6,
            ..SolverConfig::default()
        };

        assert!(
            solver.validate().is_err(),
            "a negative contact tolerance is not a usable collision margin"
        );
    }

    /// A timestep above one second is rejected as a likely unit error, which
    /// is a different branch from the non-finite/non-positive guard above it.
    #[test]
    fn config_validation_rejects_an_implausibly_large_timestep() {
        let config = SimulationConfig::with_timestep(2.0);
        assert!(config.validate().is_err(), "2 s per step is a unit mistake");

        // Right at the boundary the config is still accepted — pins that the
        // guard is `> 1.0` and not `>= 1.0`.
        assert!(SimulationConfig::with_timestep(1.0).validate().is_ok());
    }

    /// ★★ `SimulationConfig::validate` DELEGATES to the solver's validation.
    ///
    /// Without this, dropping the `self.solver.validate()?` line leaves every
    /// other validation test passing while an invalid solver sails through the
    /// only check a consumer actually calls.
    #[test]
    fn config_validation_rejects_an_invalid_solver() {
        let solver = SolverConfig {
            velocity_iterations: 0,
            ..SolverConfig::default()
        };
        let config = SimulationConfig::default().solver(solver);

        assert!(
            config.validate().is_err(),
            "an invalid solver must fail the parent config's validation"
        );
    }

    /// ★ `materials()` SANITISES rather than trusting its caller: restitution
    /// is clamped into `[0, 1]` and friction to non-negative. So the builder
    /// path cannot produce a config that `validate` would reject — worth
    /// pinning, because the two functions would otherwise disagree about what
    /// counts as a legal value.
    #[test]
    fn materials_clamps_into_the_range_validation_demands() {
        let solver = SolverConfig::default().materials(5.0, -3.0);

        assert_relative_eq!(solver.default_restitution, 1.0, epsilon = 1e-12);
        assert_relative_eq!(solver.default_friction, 0.0, epsilon = 1e-12);
        assert!(
            solver.validate().is_ok(),
            "values that arrived through materials() must always validate"
        );

        let low = SolverConfig::default().materials(-1.0, 0.5);
        assert_relative_eq!(low.default_restitution, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn solver_builders_set_what_they_name() {
        let solver = SolverConfig::default().iterations(12, 7).no_sleeping();

        assert_eq!(solver.velocity_iterations, 12);
        assert_eq!(solver.position_iterations, 7);
        assert!(!solver.allow_sleeping);
    }

    #[test]
    fn config_builders_set_what_they_name() {
        let config = SimulationConfig::default()
            .gravity(Gravity::mars())
            .without_contacts();

        assert!(!config.enable_contacts);
        assert_relative_eq!(config.gravity.acceleration.z, -3.71, epsilon = 1e-10);
    }

    /// The remaining gravity presets, and that `force_on_mass` scales an
    /// arbitrary vector rather than only the -Z earth case already covered.
    #[test]
    fn gravity_presets_and_force_on_an_arbitrary_vector() {
        assert_relative_eq!(Gravity::moon().acceleration.z, -1.62, epsilon = 1e-10);
        assert_relative_eq!(Gravity::mars().acceleration.z, -3.71, epsilon = 1e-10);
        assert_relative_eq!(Gravity::zero().acceleration.norm(), 0.0, epsilon = 1e-12);

        let sideways = Gravity::custom(Vector3::new(1.0, -2.0, 3.0));
        assert_relative_eq!(
            sideways.force_on_mass(4.0),
            Vector3::new(4.0, -8.0, 12.0),
            epsilon = 1e-12
        );
    }
}
