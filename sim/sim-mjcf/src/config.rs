//! Conversion from MJCF options to simulation configuration.
//!
//! This module provides conversion functions to transform MJCF `<option>` elements
//! into `sim-types` configuration structures that can be used by the simulation.

use nalgebra::Vector3;
use sim_types::{Gravity, IntegrationMethod, SimulationConfig, SolverConfig};

use crate::types::{MjcfFlag, MjcfIntegrator, MjcfOption, MjcfSolverType};

/// Convert MJCF integrator to sim-types integration method.
impl From<MjcfIntegrator> for IntegrationMethod {
    fn from(integrator: MjcfIntegrator) -> Self {
        match integrator {
            MjcfIntegrator::Euler => IntegrationMethod::SemiImplicitEuler,
            MjcfIntegrator::RK4 => IntegrationMethod::RungeKutta4,
            MjcfIntegrator::Implicit => IntegrationMethod::ImplicitVelocity,
            MjcfIntegrator::ImplicitFast => IntegrationMethod::ImplicitFast,
        }
    }
}

/// Convert MJCF option to simulation configuration.
impl From<&MjcfOption> for SimulationConfig {
    fn from(option: &MjcfOption) -> Self {
        // Convert gravity
        let gravity = if option.flag.gravity {
            Gravity::custom(option.gravity)
        } else {
            Gravity::zero()
        };

        // Build solver configuration
        let solver = SolverConfig {
            integration: option.integrator.into(),
            velocity_iterations: option.iterations,
            position_iterations: option.iterations / 2, // Heuristic: half of velocity iters
            contact_tolerance: option.tolerance,
            sleep_threshold: 0.01, // Default - not in MJCF
            sleep_time_threshold: 0.5,
            allow_sleeping: option.flag.island, // MJCF 'island' flag controls sleeping
            default_restitution: 0.3,           // Default - set per-geom in MJCF
            default_friction: 0.5,              // Default - set per-geom in MJCF
        };

        SimulationConfig {
            timestep: option.timestep,
            gravity,
            solver,
            max_time: None,
            enable_contacts: option.flag.contact,
            compute_diagnostics: option.flag.energy,
        }
    }
}

impl From<MjcfOption> for SimulationConfig {
    fn from(option: MjcfOption) -> Self {
        SimulationConfig::from(&option)
    }
}

/// Extended solver configuration that captures more MJCF-specific settings.
///
/// This structure holds additional settings from MJCF `<option>` that don't map
/// directly to the standard `SolverConfig` but are useful for advanced simulation.
#[derive(Debug, Clone)]
pub struct ExtendedSolverConfig {
    /// Base simulation config.
    pub base: SimulationConfig,

    /// Solver algorithm type.
    pub solver_type: MjcfSolverType,

    /// Line-search iterations for CG/Newton solvers.
    pub ls_iterations: usize,

    /// No-slip solver iterations (0 = disabled).
    pub noslip_iterations: usize,

    /// CCD iterations.
    pub ccd_iterations: usize,

    /// Friction-to-normal impedance ratio.
    pub impratio: f64,

    /// Maximum number of contacts (0 = unlimited).
    pub nconmax: usize,

    /// Maximum number of constraint rows (0 = unlimited).
    pub njmax: usize,

    /// Wind velocity for aerodynamic effects.
    pub wind: Vector3<f64>,

    /// Magnetic field direction.
    pub magnetic: Vector3<f64>,

    /// Medium density for drag.
    pub density: f64,

    /// Medium viscosity.
    pub viscosity: f64,

    /// Global contact margin override (negative = use per-geom).
    pub contact_margin_override: Option<f64>,

    /// Global solimp override.
    pub solimp_override: Option<[f64; 5]>,

    /// Global solref override.
    pub solref_override: Option<[f64; 2]>,

    /// Global friction override.
    pub friction_override: Option<[f64; 5]>,

    /// Simulation flags.
    pub flags: MjcfFlag,
}

impl From<&MjcfOption> for ExtendedSolverConfig {
    fn from(option: &MjcfOption) -> Self {
        Self {
            base: SimulationConfig::from(option),
            solver_type: option.solver,
            ls_iterations: option.ls_iterations,
            noslip_iterations: option.noslip_iterations,
            ccd_iterations: option.ccd_iterations,
            impratio: option.impratio,
            nconmax: option.nconmax,
            njmax: option.njmax,
            wind: option.wind,
            magnetic: option.magnetic,
            density: option.density,
            viscosity: option.viscosity,
            contact_margin_override: option.effective_margin(),
            solimp_override: option.o_solimp,
            solref_override: option.o_solref,
            friction_override: option.o_friction,
            flags: option.flag,
        }
    }
}

impl From<MjcfOption> for ExtendedSolverConfig {
    fn from(option: MjcfOption) -> Self {
        ExtendedSolverConfig::from(&option)
    }
}

impl Default for ExtendedSolverConfig {
    fn default() -> Self {
        Self::from(&MjcfOption::default())
    }
}

impl ExtendedSolverConfig {
    /// Check if warm-starting is enabled.
    #[must_use]
    pub fn warmstart_enabled(&self) -> bool {
        self.flags.warmstart
    }

    /// Check if CCD (continuous collision detection) is enabled.
    #[must_use]
    pub fn ccd_enabled(&self) -> bool {
        self.ccd_iterations > 0 && self.flags.nativeccd
    }

    /// Check if aerodynamic effects are enabled.
    #[must_use]
    pub fn aero_enabled(&self) -> bool {
        self.density > 0.0 || self.wind.norm() > 1e-10
    }

    /// Get effective contact margin if override is set.
    #[must_use]
    pub fn effective_contact_margin(&self) -> Option<f64> {
        self.contact_margin_override
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_default_option_to_config() {
        let option = MjcfOption::default();
        let config: SimulationConfig = option.into();

        assert_relative_eq!(config.timestep, 0.002, epsilon = 1e-10);
        assert_relative_eq!(config.gravity.acceleration.z, -9.81, epsilon = 1e-10);
        assert!(config.enable_contacts);
    }

    #[test]
    fn test_disabled_gravity() {
        let mut option = MjcfOption::default();
        option.flag.gravity = false;

        let config: SimulationConfig = option.into();
        assert_relative_eq!(config.gravity.acceleration.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_disabled_contacts() {
        let mut option = MjcfOption::default();
        option.flag.contact = false;

        let config: SimulationConfig = option.into();
        assert!(!config.enable_contacts);
    }

    #[test]
    fn test_integrator_conversion() {
        assert_eq!(
            IntegrationMethod::from(MjcfIntegrator::Euler),
            IntegrationMethod::SemiImplicitEuler
        );
        assert_eq!(
            IntegrationMethod::from(MjcfIntegrator::RK4),
            IntegrationMethod::RungeKutta4
        );
        assert_eq!(
            IntegrationMethod::from(MjcfIntegrator::Implicit),
            IntegrationMethod::ImplicitVelocity
        );
        assert_eq!(
            IntegrationMethod::from(MjcfIntegrator::ImplicitFast),
            IntegrationMethod::ImplicitFast
        );
    }

    #[test]
    fn test_extended_config() {
        let mut option = MjcfOption::default();
        option.solver = MjcfSolverType::PGS;
        option.ls_iterations = 100;
        option.wind = Vector3::new(1.0, 0.0, 0.0);
        option.density = 1.2; // Air density

        let ext_config: ExtendedSolverConfig = option.into();

        assert_eq!(ext_config.solver_type, MjcfSolverType::PGS);
        assert_eq!(ext_config.ls_iterations, 100);
        assert!(ext_config.aero_enabled());
        assert!(ext_config.warmstart_enabled());
    }

    #[test]
    fn test_margin_override() {
        let mut option = MjcfOption::default();

        // Default is negative (no override)
        let config: ExtendedSolverConfig = (&option).into();
        assert!(config.effective_contact_margin().is_none());

        // Set positive override
        option.o_margin = 0.001;
        let config: ExtendedSolverConfig = option.into();
        assert_eq!(config.effective_contact_margin(), Some(0.001));
    }
}
