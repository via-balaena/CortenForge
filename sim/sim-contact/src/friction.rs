//! Friction cone model with convex projection.
//!
//! This module implements the friction cone constraint used in contact dynamics.
//! The friction force is constrained to lie within a cone:
//!
//! ```text
//! |F_tangent| ≤ μ * F_normal
//! ```
//!
//! For computational efficiency, we approximate the circular friction cone with
//! a pyramidal cone, which can be solved with linear constraints instead of
//! second-order cone programming.

use nalgebra::Vector3;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Friction cone model for computing tangential friction forces.
///
/// The friction cone constrains the tangential force to satisfy:
/// ```text
/// |F_t| ≤ μ * F_n
/// ```
///
/// Where:
/// - `F_t` is the tangential (friction) force
/// - `F_n` is the normal force magnitude
/// - `μ` is the friction coefficient
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct FrictionCone {
    /// Coulomb friction coefficient.
    pub mu: f64,
}

impl FrictionCone {
    /// Create a new friction cone with the given coefficient.
    #[must_use]
    pub fn new(mu: f64) -> Self {
        Self { mu: mu.max(0.0) }
    }

    /// Create a frictionless cone (μ = 0).
    #[must_use]
    pub fn frictionless() -> Self {
        Self { mu: 0.0 }
    }

    /// Project a tangential force onto the friction cone.
    ///
    /// Given a desired tangential force and the normal force magnitude,
    /// returns the force clamped to lie within the friction cone.
    ///
    /// # Arguments
    ///
    /// * `tangent_force` - Desired tangential force vector
    /// * `normal_magnitude` - Magnitude of the normal force (must be non-negative)
    ///
    /// # Returns
    ///
    /// The tangential force projected onto the friction cone.
    #[must_use]
    pub fn project(&self, tangent_force: Vector3<f64>, normal_magnitude: f64) -> Vector3<f64> {
        if normal_magnitude <= 0.0 || self.mu <= 0.0 {
            return Vector3::zeros();
        }

        let max_friction = self.mu * normal_magnitude;
        let tangent_magnitude = tangent_force.norm();

        if tangent_magnitude <= max_friction {
            // Inside cone, no projection needed
            tangent_force
        } else if tangent_magnitude < 1e-10 {
            // Effectively zero tangent force
            Vector3::zeros()
        } else {
            // Project to cone boundary
            tangent_force * (max_friction / tangent_magnitude)
        }
    }

    /// Check if a tangential force is within the friction cone.
    #[must_use]
    pub fn contains(&self, tangent_force: &Vector3<f64>, normal_magnitude: f64) -> bool {
        if normal_magnitude <= 0.0 {
            return tangent_force.norm() < 1e-10;
        }
        tangent_force.norm() <= self.mu * normal_magnitude + 1e-10
    }

    /// Compute the maximum static friction force.
    #[must_use]
    pub fn max_friction(&self, normal_magnitude: f64) -> f64 {
        self.mu * normal_magnitude.max(0.0)
    }

    /// Get the cone half-angle in radians.
    #[must_use]
    pub fn half_angle(&self) -> f64 {
        self.mu.atan()
    }
}

/// Different friction models for tangential force computation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FrictionModel {
    /// Coulomb friction with a single coefficient.
    ///
    /// Uses the same coefficient for static and kinetic friction.
    Coulomb {
        /// Friction coefficient.
        mu: f64,
    },

    /// Separate static and kinetic friction coefficients.
    ///
    /// Static friction is used when velocity is below the threshold,
    /// kinetic friction is used otherwise. There's a smooth transition.
    StaticKinetic {
        /// Static friction coefficient (typically higher).
        mu_static: f64,
        /// Kinetic friction coefficient.
        mu_kinetic: f64,
        /// Velocity threshold for transition (m/s).
        velocity_threshold: f64,
    },

    /// Viscous friction model (force proportional to velocity).
    ///
    /// Useful for simulating fluid drag or lubricated contacts.
    Viscous {
        /// Viscous friction coefficient (N·s/m).
        viscosity: f64,
        /// Optional Coulomb component.
        mu_coulomb: f64,
    },

    /// Combined static, kinetic, and viscous friction.
    ///
    /// Most realistic model for general contacts.
    Combined {
        /// Static friction coefficient.
        mu_static: f64,
        /// Kinetic friction coefficient.
        mu_kinetic: f64,
        /// Viscous coefficient.
        viscosity: f64,
        /// Stribeck velocity for smooth transition.
        stribeck_velocity: f64,
    },
}

impl Default for FrictionModel {
    fn default() -> Self {
        Self::Coulomb { mu: 0.5 }
    }
}

impl FrictionModel {
    /// Create a simple Coulomb friction model.
    #[must_use]
    pub fn coulomb(mu: f64) -> Self {
        Self::Coulomb { mu }
    }

    /// Create a static/kinetic friction model.
    #[must_use]
    pub fn static_kinetic(mu_static: f64, mu_kinetic: f64) -> Self {
        Self::StaticKinetic {
            mu_static,
            mu_kinetic,
            velocity_threshold: 0.01, // Default 1 cm/s
        }
    }

    /// Create a viscous friction model.
    #[must_use]
    pub fn viscous(viscosity: f64) -> Self {
        Self::Viscous {
            viscosity,
            mu_coulomb: 0.0,
        }
    }

    /// Create a combined friction model (most realistic).
    #[must_use]
    pub fn combined(mu_static: f64, mu_kinetic: f64, viscosity: f64) -> Self {
        Self::Combined {
            mu_static,
            mu_kinetic,
            viscosity,
            stribeck_velocity: 0.01,
        }
    }

    /// Compute the friction force given the contact conditions.
    ///
    /// # Arguments
    ///
    /// * `tangent_velocity` - Relative tangential velocity (sliding direction)
    /// * `normal_force` - Normal force magnitude (positive, pushing apart)
    ///
    /// # Returns
    ///
    /// The friction force vector (opposes motion).
    #[must_use]
    pub fn compute_force(&self, tangent_velocity: Vector3<f64>, normal_force: f64) -> Vector3<f64> {
        if normal_force <= 0.0 {
            return Vector3::zeros();
        }

        let speed = tangent_velocity.norm();
        if speed < 1e-10 {
            // No sliding - could apply static friction up to limit
            // but for force-based model, we return zero
            return Vector3::zeros();
        }

        let direction = -tangent_velocity / speed; // Opposes motion

        let force_magnitude = match self {
            Self::Coulomb { mu } => mu * normal_force,

            Self::StaticKinetic {
                mu_static,
                mu_kinetic,
                velocity_threshold,
            } => {
                // Smooth transition using tanh
                let t = (speed / velocity_threshold).min(10.0);
                let blend = t.tanh();
                let mu = mu_static * (1.0 - blend) + mu_kinetic * blend;
                mu * normal_force
            }

            Self::Viscous {
                viscosity,
                mu_coulomb,
            } => mu_coulomb * normal_force + viscosity * speed,

            Self::Combined {
                mu_static,
                mu_kinetic,
                viscosity,
                stribeck_velocity,
            } => {
                // Stribeck curve: smooth transition from static to kinetic
                // f(v) = mu_kinetic + (mu_static - mu_kinetic) * exp(-|v|/v_s)
                let stribeck =
                    mu_kinetic + (mu_static - mu_kinetic) * (-speed / stribeck_velocity).exp();
                stribeck * normal_force + viscosity * speed
            }
        };

        direction * force_magnitude
    }

    /// Get the effective friction coefficient at a given velocity.
    #[must_use]
    pub fn effective_mu(&self, speed: f64) -> f64 {
        match self {
            Self::Coulomb { mu } => *mu,

            Self::StaticKinetic {
                mu_static,
                mu_kinetic,
                velocity_threshold,
            } => {
                let t = (speed / velocity_threshold).min(10.0);
                let blend = t.tanh();
                mu_static * (1.0 - blend) + mu_kinetic * blend
            }

            Self::Viscous { mu_coulomb, .. } => *mu_coulomb,

            Self::Combined {
                mu_static,
                mu_kinetic,
                stribeck_velocity,
                ..
            } => mu_kinetic + (mu_static - mu_kinetic) * (-speed / stribeck_velocity).exp(),
        }
    }

    /// Create a friction cone for the current effective friction.
    #[must_use]
    pub fn cone_at_velocity(&self, speed: f64) -> FrictionCone {
        FrictionCone::new(self.effective_mu(speed))
    }
}

/// Regularized friction for implicit integration.
///
/// The standard Coulomb friction model has a discontinuity at zero velocity
/// which can cause numerical issues. This provides a smooth approximation.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RegularizedFriction {
    /// Friction coefficient.
    pub mu: f64,
    /// Regularization velocity (m/s).
    ///
    /// Below this velocity, friction increases linearly rather than
    /// being constant. This creates a smooth function suitable for
    /// implicit integration.
    pub regularization_velocity: f64,
}

impl RegularizedFriction {
    /// Create a new regularized friction model.
    #[must_use]
    pub fn new(mu: f64, regularization_velocity: f64) -> Self {
        Self {
            mu,
            regularization_velocity: regularization_velocity.max(1e-6),
        }
    }

    /// Default regularization for typical robotics applications.
    #[must_use]
    pub fn default_robotics(mu: f64) -> Self {
        Self::new(mu, 0.001) // 1 mm/s regularization
    }

    /// Compute the friction force.
    ///
    /// The force smoothly transitions from linear (viscous-like) at low
    /// velocities to constant (Coulomb-like) at high velocities.
    #[must_use]
    pub fn compute_force(&self, tangent_velocity: Vector3<f64>, normal_force: f64) -> Vector3<f64> {
        if normal_force <= 0.0 {
            return Vector3::zeros();
        }

        let speed = tangent_velocity.norm();
        if speed < 1e-12 {
            return Vector3::zeros();
        }

        let direction = -tangent_velocity / speed;

        // Smooth regularization: f(v) = mu * F_n * tanh(v / v_reg)
        // This gives linear behavior near zero and saturates to mu * F_n
        let normalized_speed = speed / self.regularization_velocity;
        let scale = normalized_speed.tanh();

        direction * (self.mu * normal_force * scale)
    }

    /// Compute the effective viscosity at zero velocity.
    ///
    /// This is the slope of the force-velocity curve at v=0, which is
    /// useful for implicit integration (it's the "friction damping").
    #[must_use]
    pub fn zero_velocity_viscosity(&self, normal_force: f64) -> f64 {
        // d/dv [mu * F_n * tanh(v/v_reg)] at v=0
        // = mu * F_n / v_reg * sech²(0)
        // = mu * F_n / v_reg
        self.mu * normal_force / self.regularization_velocity
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_friction_cone_inside() {
        let cone = FrictionCone::new(0.5);
        let force = Vector3::new(10.0, 0.0, 0.0);
        let normal_magnitude = 100.0; // Max friction = 50

        let projected = cone.project(force, normal_magnitude);
        assert_eq!(projected, force); // Inside cone, unchanged
    }

    #[test]
    fn test_friction_cone_outside() {
        let cone = FrictionCone::new(0.5);
        let force = Vector3::new(100.0, 0.0, 0.0);
        let normal_magnitude = 100.0; // Max friction = 50

        let projected = cone.project(force, normal_magnitude);
        assert_relative_eq!(projected.norm(), 50.0, epsilon = 1e-10);
        assert!(projected.x > 0.0); // Same direction
    }

    #[test]
    fn test_friction_cone_contains() {
        let cone = FrictionCone::new(0.5);
        let normal = 100.0;

        assert!(cone.contains(&Vector3::new(49.0, 0.0, 0.0), normal));
        assert!(!cone.contains(&Vector3::new(51.0, 0.0, 0.0), normal));
    }

    #[test]
    fn test_friction_cone_frictionless() {
        let cone = FrictionCone::frictionless();
        let force = Vector3::new(100.0, 0.0, 0.0);

        let projected = cone.project(force, 100.0);
        assert_relative_eq!(projected.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_coulomb_friction() {
        let model = FrictionModel::coulomb(0.5);
        let velocity = Vector3::new(1.0, 0.0, 0.0);
        let normal = 100.0;

        let force = model.compute_force(velocity, normal);

        // Should oppose motion
        assert!(force.x < 0.0);
        // Magnitude should be mu * normal
        assert_relative_eq!(force.norm(), 50.0, epsilon = 1e-10);
    }

    #[test]
    fn test_static_kinetic_friction() {
        let model = FrictionModel::static_kinetic(0.6, 0.4);

        // At low velocity, closer to static
        let mu_low = model.effective_mu(0.001);
        // At high velocity, closer to kinetic
        let mu_high = model.effective_mu(1.0);

        assert!(mu_low > mu_high);
        assert!(mu_low <= 0.6);
        assert!(mu_high >= 0.4);
    }

    #[test]
    fn test_combined_friction() {
        let model = FrictionModel::combined(0.6, 0.4, 10.0);
        let velocity = Vector3::new(0.1, 0.0, 0.0);
        let normal = 100.0;

        let force = model.compute_force(velocity, normal);

        // Should have both Coulomb and viscous components
        assert!(force.norm() > 0.4 * normal); // More than just kinetic Coulomb
    }

    #[test]
    fn test_regularized_friction_smooth() {
        let friction = RegularizedFriction::new(0.5, 0.01);
        let normal = 100.0;

        // At zero velocity, force should be zero
        let f0 = friction.compute_force(Vector3::zeros(), normal);
        assert_relative_eq!(f0.norm(), 0.0, epsilon = 1e-10);

        // At small velocity, force should be small
        let f_small = friction.compute_force(Vector3::new(0.001, 0.0, 0.0), normal);
        assert!(f_small.norm() < 50.0);

        // At large velocity, force should approach mu * normal
        let f_large = friction.compute_force(Vector3::new(1.0, 0.0, 0.0), normal);
        assert_relative_eq!(f_large.norm(), 50.0, epsilon = 0.1);
    }

    #[test]
    fn test_regularized_friction_viscosity() {
        let friction = RegularizedFriction::new(0.5, 0.01);
        let normal = 100.0;

        let viscosity = friction.zero_velocity_viscosity(normal);

        // Should be mu * normal / v_reg = 0.5 * 100 / 0.01 = 5000
        assert_relative_eq!(viscosity, 5000.0, epsilon = 1e-10);
    }

    #[test]
    fn test_friction_zero_normal() {
        let cone = FrictionCone::new(0.5);
        let model = FrictionModel::coulomb(0.5);
        let friction = RegularizedFriction::new(0.5, 0.01);

        let velocity = Vector3::new(1.0, 0.0, 0.0);

        // All should return zero force for zero normal
        assert_eq!(cone.project(velocity, 0.0), Vector3::zeros());
        assert_eq!(model.compute_force(velocity, 0.0), Vector3::zeros());
        assert_eq!(friction.compute_force(velocity, 0.0), Vector3::zeros());
    }
}
