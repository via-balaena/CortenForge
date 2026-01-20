//! Friction cone model with convex projection.
//!
//! This module implements the friction cone constraint used in contact dynamics.
//! The friction force is constrained to lie within a cone:
//!
//! ```text
//! |F_tangent| ≤ μ * F_normal
//! ```
//!
//! # Elliptic Friction Cones
//!
//! MuJoCo uses elliptic friction cones by default, which allow anisotropic
//! friction (different friction coefficients in different tangent directions).
//! The elliptic cone constraint is:
//!
//! ```text
//! f_n ≥ 0,  (f_t1/μ_1)² + (f_t2/μ_2)² ≤ f_n²
//! ```
//!
//! Where:
//! - `f_n` is the normal force (must be non-negative)
//! - `f_t1`, `f_t2` are tangential force components
//! - `μ_1`, `μ_2` are friction coefficients along the two tangent axes
//!
//! When `μ_1 = μ_2`, this reduces to the standard circular (isotropic) cone.

use nalgebra::{Vector2, Vector3};

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

    /// Convert to an elliptic cone with isotropic friction.
    #[must_use]
    pub fn to_elliptic(&self) -> EllipticFrictionCone {
        EllipticFrictionCone::isotropic(self.mu)
    }
}

/// Elliptic friction cone model for anisotropic friction.
///
/// MuJoCo uses elliptic friction cones by default, which allow different
/// friction coefficients along two orthogonal tangent directions. The cone
/// constraint is:
///
/// ```text
/// f_n ≥ 0,  (f_t1/μ_1)² + (f_t2/μ_2)² ≤ f_n²
/// ```
///
/// This generalizes the circular (isotropic) friction cone where `μ_1 = μ_2`.
///
/// # Use Cases
///
/// - **Treaded surfaces**: Different friction along/across treads
/// - **Brushed metal**: Anisotropic surface texture
/// - **Fabric contact**: Warp vs weft directions
/// - **Grooved surfaces**: Like vinyl records or machined parts
///
/// # Example
///
/// ```
/// use sim_contact::EllipticFrictionCone;
/// use nalgebra::Vector3;
///
/// // Anisotropic friction: μ_x = 0.8, μ_y = 0.4
/// let cone = EllipticFrictionCone::new(0.8, 0.4);
///
/// // Tangent force with components in both directions
/// let tangent = Vector3::new(10.0, 10.0, 0.0);
/// let normal_mag = 100.0;
///
/// // Project onto the elliptic cone
/// let projected = cone.project(tangent, normal_mag);
///
/// // Force is now within the elliptic cone
/// assert!(cone.contains(&projected, normal_mag));
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct EllipticFrictionCone {
    /// Friction coefficient along the primary tangent direction (typically X).
    pub mu_1: f64,
    /// Friction coefficient along the secondary tangent direction (typically Y).
    pub mu_2: f64,
}

impl EllipticFrictionCone {
    /// Create a new elliptic friction cone with anisotropic friction.
    ///
    /// # Arguments
    ///
    /// * `mu_1` - Friction coefficient along the first tangent axis
    /// * `mu_2` - Friction coefficient along the second tangent axis
    #[must_use]
    pub fn new(mu_1: f64, mu_2: f64) -> Self {
        Self {
            mu_1: mu_1.max(0.0),
            mu_2: mu_2.max(0.0),
        }
    }

    /// Create an isotropic (circular) friction cone.
    ///
    /// This is equivalent to `FrictionCone` but using the elliptic formulation.
    #[must_use]
    pub fn isotropic(mu: f64) -> Self {
        Self::new(mu, mu)
    }

    /// Create a frictionless cone.
    #[must_use]
    pub fn frictionless() -> Self {
        Self {
            mu_1: 0.0,
            mu_2: 0.0,
        }
    }

    /// Check if this cone is isotropic (circular).
    #[must_use]
    pub fn is_isotropic(&self) -> bool {
        (self.mu_1 - self.mu_2).abs() < 1e-10
    }

    /// Get the average friction coefficient.
    #[must_use]
    pub fn average_mu(&self) -> f64 {
        (self.mu_1 + self.mu_2) / 2.0
    }

    /// Get the friction coefficient along a direction in the tangent plane.
    ///
    /// # Arguments
    ///
    /// * `angle` - Angle from the primary tangent axis (radians)
    #[must_use]
    pub fn mu_at_angle(&self, angle: f64) -> f64 {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        // The effective friction at angle θ for an ellipse is:
        // 1/μ_eff² = cos²θ/μ_1² + sin²θ/μ_2²
        // μ_eff = 1/√(cos²θ/μ_1² + sin²θ/μ_2²)
        if self.mu_1 < 1e-10 || self.mu_2 < 1e-10 {
            return 0.0;
        }
        let inv_sq =
            (cos_a * cos_a) / (self.mu_1 * self.mu_1) + (sin_a * sin_a) / (self.mu_2 * self.mu_2);
        if inv_sq < 1e-20 {
            return self.mu_1.max(self.mu_2);
        }
        1.0 / inv_sq.sqrt()
    }

    /// Project a tangential force onto the elliptic friction cone.
    ///
    /// Uses the closed-form solution for projecting onto an ellipsoid boundary.
    /// The projection minimizes the Euclidean distance to the original force
    /// while satisfying the cone constraint.
    ///
    /// # Arguments
    ///
    /// * `tangent_force` - Desired tangential force vector (X,Y components used)
    /// * `normal_magnitude` - Magnitude of the normal force (must be non-negative)
    ///
    /// # Returns
    ///
    /// The tangential force projected onto the elliptic friction cone.
    #[must_use]
    pub fn project(&self, tangent_force: Vector3<f64>, normal_magnitude: f64) -> Vector3<f64> {
        if normal_magnitude <= 0.0 || (self.mu_1 <= 0.0 && self.mu_2 <= 0.0) {
            return Vector3::zeros();
        }

        // Extract 2D tangent components (assuming Z is normal)
        let f_t = Vector2::new(tangent_force.x, tangent_force.y);

        // Handle edge cases
        if f_t.norm() < 1e-12 {
            return Vector3::zeros();
        }

        // Semi-axes of the ellipse are μ_i * f_n
        let a = self.mu_1 * normal_magnitude; // Semi-axis in X direction
        let b = self.mu_2 * normal_magnitude; // Semi-axis in Y direction

        // Check if already inside the ellipse: (x/a)² + (y/b)² ≤ 1
        let ellipse_value = if a > 1e-10 && b > 1e-10 {
            (f_t.x / a).powi(2) + (f_t.y / b).powi(2)
        } else if a > 1e-10 {
            // b ≈ 0, force must be along X axis only
            (f_t.x / a).powi(2)
                + if f_t.y.abs() > 1e-10 {
                    f64::INFINITY
                } else {
                    0.0
                }
        } else if b > 1e-10 {
            // a ≈ 0, force must be along Y axis only
            (if f_t.x.abs() > 1e-10 {
                f64::INFINITY
            } else {
                0.0
            }) + (f_t.y / b).powi(2)
        } else {
            // Both axes are zero - frictionless
            return Vector3::zeros();
        };

        if ellipse_value <= 1.0 + 1e-10 {
            // Inside or on the ellipse boundary
            return tangent_force;
        }

        // Need to project onto ellipse boundary
        // For projection onto an ellipse, we solve for the closest point
        // using an iterative Newton method on the parametric boundary.
        let projected_2d = self.project_to_ellipse_boundary(f_t, a, b);

        Vector3::new(projected_2d.x, projected_2d.y, 0.0)
    }

    /// Project a 2D point onto an ellipse boundary using Newton's method.
    ///
    /// Finds the point on the ellipse (x/a)² + (y/b)² = 1 closest to `point`.
    fn project_to_ellipse_boundary(&self, point: Vector2<f64>, a: f64, b: f64) -> Vector2<f64> {
        // Handle degenerate cases
        if a < 1e-10 {
            // Ellipse degenerates to line segment on Y axis
            return Vector2::new(0.0, point.y.clamp(-b, b));
        }
        if b < 1e-10 {
            // Ellipse degenerates to line segment on X axis
            return Vector2::new(point.x.clamp(-a, a), 0.0);
        }

        // Use the parametric approach: find t such that the closest point is
        // (a*cos(t), b*sin(t))
        //
        // The condition for the closest point is that the vector from the point
        // to the ellipse is perpendicular to the ellipse tangent.
        // This leads to: a*sin(t)*(x - a*cos(t)) = b*cos(t)*(y - b*sin(t))
        //
        // We use Newton iteration on this nonlinear equation.

        // Initial guess: direction to point, normalized to ellipse
        let angle_guess = point.y.atan2(point.x);
        let mut t = angle_guess;

        // Newton iteration
        for _ in 0..10 {
            let cos_t = t.cos();
            let sin_t = t.sin();

            // Residual: perpendicularity condition
            // f(t) = a*sin(t)*(px - a*cos(t)) - b*cos(t)*(py - b*sin(t))
            //      = a*sin(t)*px - a²*sin(t)*cos(t) - b*cos(t)*py + b²*cos(t)*sin(t)
            //      = a*sin(t)*px - b*cos(t)*py + (b² - a²)*sin(t)*cos(t)
            let f = a * sin_t * point.x - b * cos_t * point.y + (b * b - a * a) * sin_t * cos_t;

            // Derivative: df/dt
            // df/dt = a*cos(t)*px + b*sin(t)*py + (b² - a²)*(cos²(t) - sin²(t))
            let df = a * cos_t * point.x
                + b * sin_t * point.y
                + (b * b - a * a) * (cos_t * cos_t - sin_t * sin_t);

            if df.abs() < 1e-12 {
                break;
            }

            let dt = -f / df;
            t += dt;

            if dt.abs() < 1e-10 {
                break;
            }
        }

        Vector2::new(a * t.cos(), b * t.sin())
    }

    /// Check if a tangential force is within the elliptic friction cone.
    #[must_use]
    pub fn contains(&self, tangent_force: &Vector3<f64>, normal_magnitude: f64) -> bool {
        if normal_magnitude <= 0.0 {
            return tangent_force.norm() < 1e-10;
        }

        let a = self.mu_1 * normal_magnitude;
        let b = self.mu_2 * normal_magnitude;

        if a < 1e-10 && b < 1e-10 {
            return tangent_force.norm() < 1e-10;
        }

        let f_t = Vector2::new(tangent_force.x, tangent_force.y);

        // Check ellipse: (x/a)² + (y/b)² ≤ 1
        let mut ellipse_value = 0.0;
        if a > 1e-10 {
            ellipse_value += (f_t.x / a).powi(2);
        } else if f_t.x.abs() > 1e-10 {
            return false; // Non-zero X with zero X-friction
        }
        if b > 1e-10 {
            ellipse_value += (f_t.y / b).powi(2);
        } else if f_t.y.abs() > 1e-10 {
            return false; // Non-zero Y with zero Y-friction
        }

        ellipse_value <= 1.0 + 1e-10
    }

    /// Compute the maximum friction force along each axis.
    #[must_use]
    pub fn max_friction(&self, normal_magnitude: f64) -> (f64, f64) {
        let n = normal_magnitude.max(0.0);
        (self.mu_1 * n, self.mu_2 * n)
    }

    /// Convert to a circular cone using the average friction coefficient.
    #[must_use]
    pub fn to_circular(&self) -> FrictionCone {
        FrictionCone::new(self.average_mu())
    }

    /// Create tangent basis vectors from a contact normal.
    ///
    /// Returns two orthogonal unit vectors in the tangent plane.
    /// These define the directions for `mu_1` and `mu_2` respectively.
    #[must_use]
    pub fn compute_tangent_basis(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        // Choose a vector not parallel to normal
        let up = if normal.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };

        // First tangent: perpendicular to normal
        let t1 = normal.cross(&up).normalize();

        // Second tangent: perpendicular to both
        let t2 = normal.cross(&t1).normalize();

        (t1, t2)
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

    // ==================== Elliptic Friction Cone Tests ====================

    #[test]
    fn test_elliptic_cone_isotropic_equivalent() {
        // When mu_1 == mu_2, elliptic should behave like circular
        let circular = FrictionCone::new(0.5);
        let elliptic = EllipticFrictionCone::isotropic(0.5);

        let force = Vector3::new(60.0, 0.0, 0.0);
        let normal = 100.0; // Max friction = 50

        let proj_circ = circular.project(force, normal);
        let proj_ell = elliptic.project(force, normal);

        assert_relative_eq!(proj_circ.norm(), proj_ell.norm(), epsilon = 1e-6);
        assert_relative_eq!(proj_circ.x, proj_ell.x, epsilon = 1e-6);
    }

    #[test]
    fn test_elliptic_cone_inside() {
        let cone = EllipticFrictionCone::new(0.5, 0.5);
        let force = Vector3::new(10.0, 10.0, 0.0);
        let normal = 100.0; // Max friction along axes = 50

        let projected = cone.project(force, normal);
        // Force is inside, should be unchanged
        assert_relative_eq!(projected.x, force.x, epsilon = 1e-10);
        assert_relative_eq!(projected.y, force.y, epsilon = 1e-10);
    }

    #[test]
    fn test_elliptic_cone_outside_project() {
        let cone = EllipticFrictionCone::new(0.5, 0.5);
        let force = Vector3::new(100.0, 0.0, 0.0);
        let normal = 100.0; // Max friction = 50

        let projected = cone.project(force, normal);

        // Should be projected to boundary
        assert_relative_eq!(projected.norm(), 50.0, epsilon = 1e-6);
        assert!(projected.x > 0.0); // Same direction
    }

    #[test]
    fn test_elliptic_cone_anisotropic() {
        // μ_x = 0.8, μ_y = 0.4 (2:1 ratio)
        let cone = EllipticFrictionCone::new(0.8, 0.4);
        let normal = 100.0;

        // Force along X axis should allow up to 80 N
        let force_x = Vector3::new(100.0, 0.0, 0.0);
        let proj_x = cone.project(force_x, normal);
        assert_relative_eq!(proj_x.x, 80.0, epsilon = 1e-6);
        assert_relative_eq!(proj_x.y, 0.0, epsilon = 1e-6);

        // Force along Y axis should allow up to 40 N
        let force_y = Vector3::new(0.0, 100.0, 0.0);
        let proj_y = cone.project(force_y, normal);
        assert_relative_eq!(proj_y.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(proj_y.y, 40.0, epsilon = 1e-6);
    }

    #[test]
    fn test_elliptic_cone_diagonal_projection() {
        // μ_x = 0.8, μ_y = 0.4
        let cone = EllipticFrictionCone::new(0.8, 0.4);
        let normal = 100.0;

        // Force at 45 degrees, outside the ellipse
        let force = Vector3::new(100.0, 100.0, 0.0);
        let projected = cone.project(force, normal);

        // Should be on the ellipse boundary
        assert!(cone.contains(&projected, normal));

        // Should maintain roughly the same direction (same quadrant)
        assert!(projected.x > 0.0);
        assert!(projected.y > 0.0);
    }

    #[test]
    fn test_elliptic_cone_contains() {
        let cone = EllipticFrictionCone::new(0.8, 0.4);
        let normal = 100.0;

        // Inside the ellipse
        assert!(cone.contains(&Vector3::new(40.0, 20.0, 0.0), normal));

        // On the boundary along X axis
        assert!(cone.contains(&Vector3::new(80.0, 0.0, 0.0), normal));

        // On the boundary along Y axis
        assert!(cone.contains(&Vector3::new(0.0, 40.0, 0.0), normal));

        // Outside along X
        assert!(!cone.contains(&Vector3::new(81.0, 0.0, 0.0), normal));

        // Outside along Y
        assert!(!cone.contains(&Vector3::new(0.0, 41.0, 0.0), normal));
    }

    #[test]
    fn test_elliptic_cone_frictionless() {
        let cone = EllipticFrictionCone::frictionless();
        let force = Vector3::new(100.0, 50.0, 0.0);

        let projected = cone.project(force, 100.0);
        assert_relative_eq!(projected.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_elliptic_cone_zero_normal() {
        let cone = EllipticFrictionCone::new(0.8, 0.4);
        let force = Vector3::new(10.0, 5.0, 0.0);

        let projected = cone.project(force, 0.0);
        assert_eq!(projected, Vector3::zeros());
    }

    #[test]
    fn test_elliptic_cone_mu_at_angle() {
        let cone = EllipticFrictionCone::new(0.8, 0.4);

        // At 0 degrees (along X), mu should be 0.8
        let mu_0 = cone.mu_at_angle(0.0);
        assert_relative_eq!(mu_0, 0.8, epsilon = 1e-6);

        // At 90 degrees (along Y), mu should be 0.4
        let mu_90 = cone.mu_at_angle(std::f64::consts::FRAC_PI_2);
        assert_relative_eq!(mu_90, 0.4, epsilon = 1e-6);

        // At 45 degrees, mu should be between 0.4 and 0.8
        let mu_45 = cone.mu_at_angle(std::f64::consts::FRAC_PI_4);
        assert!(mu_45 > 0.4 && mu_45 < 0.8);
    }

    #[test]
    fn test_elliptic_cone_max_friction() {
        let cone = EllipticFrictionCone::new(0.8, 0.4);
        let normal = 100.0;

        let (max_x, max_y) = cone.max_friction(normal);
        assert_relative_eq!(max_x, 80.0, epsilon = 1e-10);
        assert_relative_eq!(max_y, 40.0, epsilon = 1e-10);
    }

    #[test]
    fn test_elliptic_cone_is_isotropic() {
        let isotropic = EllipticFrictionCone::isotropic(0.5);
        let anisotropic = EllipticFrictionCone::new(0.8, 0.4);

        assert!(isotropic.is_isotropic());
        assert!(!anisotropic.is_isotropic());
    }

    #[test]
    fn test_elliptic_to_circular_conversion() {
        let elliptic = EllipticFrictionCone::new(0.8, 0.4);
        let circular = elliptic.to_circular();

        // Average mu should be (0.8 + 0.4) / 2 = 0.6
        assert_relative_eq!(circular.mu, 0.6, epsilon = 1e-10);
    }

    #[test]
    fn test_circular_to_elliptic_conversion() {
        let circular = FrictionCone::new(0.5);
        let elliptic = circular.to_elliptic();

        assert_relative_eq!(elliptic.mu_1, 0.5, epsilon = 1e-10);
        assert_relative_eq!(elliptic.mu_2, 0.5, epsilon = 1e-10);
        assert!(elliptic.is_isotropic());
    }

    #[test]
    fn test_elliptic_tangent_basis() {
        // Normal pointing up (Z)
        let normal = Vector3::z();
        let (t1, t2) = EllipticFrictionCone::compute_tangent_basis(&normal);

        // Tangents should be perpendicular to normal
        assert_relative_eq!(t1.dot(&normal), 0.0, epsilon = 1e-10);
        assert_relative_eq!(t2.dot(&normal), 0.0, epsilon = 1e-10);

        // Tangents should be perpendicular to each other
        assert_relative_eq!(t1.dot(&t2), 0.0, epsilon = 1e-10);

        // Tangents should be unit vectors
        assert_relative_eq!(t1.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(t2.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_elliptic_projection_preserves_containment() {
        // Any projected force should always be inside or on the cone
        let cone = EllipticFrictionCone::new(0.7, 0.3);
        let normal = 100.0;

        // Test various force directions and magnitudes
        let test_forces = vec![
            Vector3::new(1000.0, 0.0, 0.0),
            Vector3::new(0.0, 1000.0, 0.0),
            Vector3::new(500.0, 500.0, 0.0),
            Vector3::new(-200.0, 300.0, 0.0),
            Vector3::new(100.0, -400.0, 0.0),
            Vector3::new(-300.0, -200.0, 0.0),
        ];

        for force in test_forces {
            let projected = cone.project(force, normal);
            assert!(
                cone.contains(&projected, normal),
                "Projected force {:?} not contained in cone",
                projected
            );
        }
    }
}
