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
use std::f64::consts::PI;

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

/// Torsional friction model for resistance to spinning.
///
/// Torsional (spinning) friction opposes rotation about the contact normal.
/// This is modeled in MuJoCo as part of "condim 4-6" contacts.
///
/// The torsional friction torque is:
/// ```text
/// τ_torsion = -μ_torsion * r_contact * F_n * sign(ω_n)
/// ```
///
/// Where:
/// - `μ_torsion` is the torsional friction coefficient (dimensionless)
/// - `r_contact` is the effective contact radius
/// - `F_n` is the normal force magnitude
/// - `ω_n` is the angular velocity about the contact normal
///
/// # MuJoCo condim Reference
///
/// - condim 1: Frictionless (only normal force)
/// - condim 3: Tangential friction (adds 2D friction)
/// - condim 4: Adds torsional friction (spinning resistance)
/// - condim 6: Adds rolling friction (rolling resistance)
///
/// # Example
///
/// ```
/// use sim_contact::TorsionalFriction;
/// use nalgebra::Vector3;
///
/// // Create torsional friction model
/// let friction = TorsionalFriction::new(0.05, 0.01); // μ=0.05, radius=1cm
///
/// // Compute torque opposing spinning
/// let normal = Vector3::z();
/// let angular_velocity = Vector3::new(0.0, 0.0, 5.0); // Spinning at 5 rad/s
/// let normal_force = 100.0; // 100 N
///
/// let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);
/// assert!(torque.z < 0.0); // Torque opposes spinning
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TorsionalFriction {
    /// Torsional friction coefficient (dimensionless).
    ///
    /// Typical values: 0.01 - 0.1
    pub mu: f64,

    /// Effective contact radius (meters).
    ///
    /// For a sphere, this is approximately 0.1-0.5 times the sphere radius.
    /// For a flat contact, this is the radius of the contact patch.
    pub contact_radius: f64,

    /// Regularization angular velocity (rad/s).
    ///
    /// Below this velocity, friction increases linearly rather than
    /// being constant. This avoids the discontinuity at zero angular velocity.
    pub regularization_velocity: f64,
}

impl TorsionalFriction {
    /// Create a new torsional friction model.
    ///
    /// # Arguments
    ///
    /// * `mu` - Torsional friction coefficient (dimensionless)
    /// * `contact_radius` - Effective contact radius in meters
    #[must_use]
    pub fn new(mu: f64, contact_radius: f64) -> Self {
        Self {
            mu: mu.max(0.0),
            contact_radius: contact_radius.max(1e-6),
            regularization_velocity: 0.01, // 0.01 rad/s default
        }
    }

    /// Create torsional friction with custom regularization.
    #[must_use]
    pub fn with_regularization(mut self, velocity: f64) -> Self {
        self.regularization_velocity = velocity.max(1e-6);
        self
    }

    /// Create from MuJoCo-style parameters.
    ///
    /// MuJoCo specifies torsional friction as a coefficient that gets
    /// multiplied by the contact patch radius.
    #[must_use]
    pub fn from_mujoco(friction_coef: f64, contact_radius: f64) -> Self {
        Self::new(friction_coef, contact_radius)
    }

    /// Compute the torsional friction torque.
    ///
    /// # Arguments
    ///
    /// * `normal` - Contact normal (unit vector)
    /// * `angular_velocity` - Relative angular velocity (ω_A - ω_B)
    /// * `normal_force` - Normal force magnitude (positive)
    ///
    /// # Returns
    ///
    /// Torque vector opposing the spinning motion.
    #[must_use]
    pub fn compute_torque(
        &self,
        normal: &Vector3<f64>,
        angular_velocity: &Vector3<f64>,
        normal_force: f64,
    ) -> Vector3<f64> {
        if normal_force <= 0.0 || self.mu <= 0.0 {
            return Vector3::zeros();
        }

        // Extract the spinning component (rotation about normal)
        let omega_n = angular_velocity.dot(normal);

        if omega_n.abs() < 1e-12 {
            return Vector3::zeros();
        }

        // Maximum torsional torque: μ * r * F_n
        let max_torque = self.mu * self.contact_radius * normal_force;

        // Apply regularization (smooth near zero velocity)
        let normalized_speed = omega_n.abs() / self.regularization_velocity;
        let scale = normalized_speed.tanh();

        // Torque opposes spinning direction
        let sign = -omega_n.signum();
        let torque_magnitude = max_torque * scale;

        normal * (torque_magnitude * sign)
    }

    /// Get the maximum torsional torque for a given normal force.
    #[must_use]
    pub fn max_torque(&self, normal_force: f64) -> f64 {
        self.mu * self.contact_radius * normal_force.max(0.0)
    }

    /// Check if a given torque is within the torsional friction limit.
    #[must_use]
    pub fn torque_within_limit(&self, torque: f64, normal_force: f64) -> bool {
        torque.abs() <= self.max_torque(normal_force) + 1e-10
    }

    /// Project a torque onto the torsional friction cone.
    #[must_use]
    pub fn project_torque(&self, torque: f64, normal_force: f64) -> f64 {
        let max = self.max_torque(normal_force);
        torque.clamp(-max, max)
    }
}

impl Default for TorsionalFriction {
    fn default() -> Self {
        Self {
            mu: 0.01,             // Small default torsional friction
            contact_radius: 0.01, // 1cm contact radius
            regularization_velocity: 0.01,
        }
    }
}

/// Rolling friction model for resistance to rolling motion.
///
/// Rolling friction opposes rotation perpendicular to the contact normal.
/// This is modeled in MuJoCo as part of "condim 6-10" contacts.
///
/// The rolling friction torque is:
/// ```text
/// τ_roll = -μ_roll * r * F_n * normalize(ω_tangent)
/// ```
///
/// Where:
/// - `μ_roll` is the rolling friction coefficient (dimensionless)
/// - `r` is the rolling radius
/// - `F_n` is the normal force magnitude
/// - `ω_tangent` is the angular velocity component perpendicular to normal
///
/// # Physical Interpretation
///
/// Rolling friction arises from:
/// - Deformation of the rolling body and surface
/// - Hysteresis in the contact materials
/// - Surface roughness
///
/// # Example
///
/// ```
/// use sim_contact::RollingFriction;
/// use nalgebra::Vector3;
///
/// // Create rolling friction model
/// let friction = RollingFriction::new(0.02, 0.05); // μ=0.02, radius=5cm
///
/// // Ball rolling in +X direction
/// let normal = Vector3::z();
/// let angular_velocity = Vector3::new(0.0, 10.0, 0.0); // Rolling about Y
/// let normal_force = 50.0;
///
/// let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);
/// assert!(torque.y < 0.0); // Torque opposes rolling
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RollingFriction {
    /// Rolling friction coefficient (dimensionless).
    ///
    /// Typical values:
    /// - Hard rubber on concrete: 0.01-0.02
    /// - Ball bearings: 0.001-0.005
    /// - Soft rubber: 0.02-0.05
    pub mu: f64,

    /// Rolling radius (meters).
    ///
    /// For a sphere, this is the sphere radius.
    /// For a cylinder, this is the cylinder radius.
    pub radius: f64,

    /// Regularization angular velocity (rad/s).
    pub regularization_velocity: f64,
}

impl RollingFriction {
    /// Create a new rolling friction model.
    ///
    /// # Arguments
    ///
    /// * `mu` - Rolling friction coefficient (dimensionless)
    /// * `radius` - Rolling radius in meters
    #[must_use]
    pub fn new(mu: f64, radius: f64) -> Self {
        Self {
            mu: mu.max(0.0),
            radius: radius.max(1e-6),
            regularization_velocity: 0.01,
        }
    }

    /// Create rolling friction with custom regularization.
    #[must_use]
    pub fn with_regularization(mut self, velocity: f64) -> Self {
        self.regularization_velocity = velocity.max(1e-6);
        self
    }

    /// Create from MuJoCo-style parameters.
    #[must_use]
    pub fn from_mujoco(friction_coef: f64, radius: f64) -> Self {
        Self::new(friction_coef, radius)
    }

    /// Compute the rolling friction torque.
    ///
    /// # Arguments
    ///
    /// * `normal` - Contact normal (unit vector)
    /// * `angular_velocity` - Relative angular velocity (ω_A - ω_B)
    /// * `normal_force` - Normal force magnitude (positive)
    ///
    /// # Returns
    ///
    /// Torque vector opposing the rolling motion.
    #[must_use]
    pub fn compute_torque(
        &self,
        normal: &Vector3<f64>,
        angular_velocity: &Vector3<f64>,
        normal_force: f64,
    ) -> Vector3<f64> {
        if normal_force <= 0.0 || self.mu <= 0.0 {
            return Vector3::zeros();
        }

        // Extract the rolling component (rotation perpendicular to normal)
        let omega_n = angular_velocity.dot(normal);
        let omega_tangent = angular_velocity - normal * omega_n;
        let omega_tangent_mag = omega_tangent.norm();

        if omega_tangent_mag < 1e-12 {
            return Vector3::zeros();
        }

        // Maximum rolling torque: μ * r * F_n
        let max_torque = self.mu * self.radius * normal_force;

        // Apply regularization
        let normalized_speed = omega_tangent_mag / self.regularization_velocity;
        let scale = normalized_speed.tanh();

        // Torque opposes rolling direction
        let direction = -omega_tangent / omega_tangent_mag;
        direction * (max_torque * scale)
    }

    /// Compute the rolling resistance force.
    ///
    /// This is an alternative formulation that computes the equivalent
    /// linear force that would be needed to overcome rolling friction.
    ///
    /// F_roll = μ * F_n / r
    #[must_use]
    pub fn compute_resistance_force(&self, normal_force: f64) -> f64 {
        if self.radius > 1e-10 {
            self.mu * normal_force / self.radius
        } else {
            0.0
        }
    }

    /// Get the maximum rolling torque for a given normal force.
    #[must_use]
    pub fn max_torque(&self, normal_force: f64) -> f64 {
        self.mu * self.radius * normal_force.max(0.0)
    }

    /// Project a torque vector onto the rolling friction cone.
    #[must_use]
    pub fn project_torque(&self, torque: Vector3<f64>, normal_force: f64) -> Vector3<f64> {
        let max = self.max_torque(normal_force);
        let mag = torque.norm();

        if mag <= max {
            torque
        } else if mag > 1e-10 {
            torque * (max / mag)
        } else {
            Vector3::zeros()
        }
    }
}

impl Default for RollingFriction {
    fn default() -> Self {
        Self {
            mu: 0.01,
            radius: 0.05, // 5cm default radius
            regularization_velocity: 0.01,
        }
    }
}

/// Pyramidal (linearized) friction cone for constraint-based solvers.
///
/// The elliptic/circular friction cone is a quadratic constraint that can be
/// difficult for linear solvers. Pyramidal cones approximate the cone with
/// a finite number of flat faces, creating a polyhedral approximation.
///
/// # Advantages
///
/// - Linear constraints (suitable for LCP/QP solvers)
/// - Exact projection (no iteration needed)
/// - Can be solved with complementarity methods
///
/// # Trade-offs
///
/// - Less accurate than elliptic cones
/// - More constraints to solve (num_faces vs 1)
/// - Slight anisotropy in friction directions
///
/// # MuJoCo Context
///
/// MuJoCo uses elliptic cones by default, but pyramidal cones are common in
/// other physics engines (Bullet, ODE) and are still supported as an alternative.
///
/// # Example
///
/// ```
/// use sim_contact::PyramidalFrictionCone;
/// use nalgebra::Vector3;
///
/// // 8-face pyramid (common choice)
/// let cone = PyramidalFrictionCone::new(0.5, 8);
///
/// // Project a force onto the cone
/// let force = Vector3::new(100.0, 0.0, 0.0);
/// let normal_mag = 100.0; // Max friction = 50
///
/// let projected = cone.project(force, normal_mag);
/// assert!(cone.contains(&projected, normal_mag));
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct PyramidalFrictionCone {
    /// Friction coefficient.
    pub mu: f64,

    /// Number of faces in the pyramid (must be >= 3).
    ///
    /// Common choices:
    /// - 4: Box approximation (fastest, least accurate)
    /// - 8: Octagonal pyramid (good balance)
    /// - 16: Hexadecagonal (high accuracy)
    pub num_faces: usize,

    /// Precomputed face normals in the tangent plane.
    face_directions: Vec<Vector2<f64>>,

    /// Precomputed face angles for containment tests.
    face_angles: Vec<f64>,
}

impl PyramidalFrictionCone {
    /// Create a new pyramidal friction cone.
    ///
    /// # Arguments
    ///
    /// * `mu` - Friction coefficient
    /// * `num_faces` - Number of faces (clamped to 3-64)
    #[must_use]
    pub fn new(mu: f64, num_faces: usize) -> Self {
        let num_faces = num_faces.clamp(3, 64);

        // Compute face directions evenly distributed around the circle
        let mut face_directions = Vec::with_capacity(num_faces);
        let mut face_angles = Vec::with_capacity(num_faces);

        for i in 0..num_faces {
            let angle = 2.0 * PI * (i as f64) / (num_faces as f64);
            face_angles.push(angle);
            face_directions.push(Vector2::new(angle.cos(), angle.sin()));
        }

        Self {
            mu: mu.max(0.0),
            num_faces,
            face_directions,
            face_angles,
        }
    }

    /// Create a 4-face (box) friction approximation.
    ///
    /// This is the fastest but least accurate approximation.
    #[must_use]
    pub fn box_approximation(mu: f64) -> Self {
        Self::new(mu, 4)
    }

    /// Create an 8-face (octagonal) friction approximation.
    ///
    /// Good balance of accuracy and speed.
    #[must_use]
    pub fn octagonal(mu: f64) -> Self {
        Self::new(mu, 8)
    }

    /// Create a high-accuracy pyramidal cone.
    #[must_use]
    pub fn high_accuracy(mu: f64) -> Self {
        Self::new(mu, 16)
    }

    /// Get the face directions in the tangent plane.
    #[must_use]
    pub fn face_directions(&self) -> &[Vector2<f64>] {
        &self.face_directions
    }

    /// Compute the effective friction coefficient at a given angle.
    ///
    /// Due to the polygonal approximation, the effective friction varies
    /// slightly with direction.
    #[must_use]
    pub fn effective_mu_at_angle(&self, angle: f64) -> f64 {
        // The pyramidal cone inscribes the circular cone
        // Effective mu is reduced at face boundaries
        let half_face_angle = PI / (self.num_faces as f64);

        // Find the angle relative to the nearest face center
        let normalized_angle = angle.rem_euclid(2.0 * PI);
        let face_center_angle = 2.0 * PI / (self.num_faces as f64);
        let offset_from_center = (normalized_angle % face_center_angle) - half_face_angle;

        // Effective mu = mu * cos(offset)
        self.mu * offset_from_center.abs().cos()
    }

    /// Project a tangential force onto the pyramidal friction cone.
    ///
    /// The pyramidal cone circumscribes the circular cone (vertices touch the circle).
    /// This is the standard convention used in physics engines.
    #[must_use]
    pub fn project(&self, tangent_force: Vector3<f64>, normal_magnitude: f64) -> Vector3<f64> {
        if normal_magnitude <= 0.0 || self.mu <= 0.0 {
            return Vector3::zeros();
        }

        let f_t = Vector2::new(tangent_force.x, tangent_force.y);
        let f_t_mag = f_t.norm();

        if f_t_mag < 1e-12 {
            return Vector3::zeros();
        }

        // Check if already inside
        if self.contains_2d(&f_t, normal_magnitude) {
            return tangent_force;
        }

        // Find the maximum friction in the direction of the force
        let max_friction_in_direction = self.max_friction_in_direction(&f_t, normal_magnitude);

        // Scale to that maximum
        let scale = max_friction_in_direction / f_t_mag;
        Vector3::new(tangent_force.x * scale, tangent_force.y * scale, 0.0)
    }

    /// Check if a 2D force is within the pyramidal cone.
    fn contains_2d(&self, force: &Vector2<f64>, normal_magnitude: f64) -> bool {
        let f_mag = force.norm();
        if f_mag < 1e-10 {
            return true;
        }

        // For each face, check if the force is on the interior side
        // A pyramidal cone that circumscribes the circle has vertices at distance μ * f_n
        // and face normals at angles between vertices
        let face_angle = 2.0 * PI / (self.num_faces as f64);
        let half_face_angle = face_angle / 2.0;

        // Distance from origin to face edge (inscribed circle radius)
        let inscribed_radius = self.mu * normal_magnitude * half_face_angle.cos();

        // Check against each face - the force projected onto the face normal
        // must be less than the inscribed radius
        for dir in &self.face_directions {
            // Face normal is perpendicular to face direction (pointing outward)
            // The face normal points in the direction of the face
            let face_normal = *dir;
            let projection = force.dot(&face_normal);
            if projection > inscribed_radius + 1e-10 {
                return false;
            }
        }

        true
    }

    /// Compute the maximum friction in a given direction.
    fn max_friction_in_direction(&self, direction: &Vector2<f64>, normal_magnitude: f64) -> f64 {
        let dir_mag = direction.norm();
        if dir_mag < 1e-12 {
            return 0.0;
        }

        let dir_normalized = direction / dir_mag;

        // Find the angle of the direction
        let angle = dir_normalized.y.atan2(dir_normalized.x);
        let face_angle = 2.0 * PI / (self.num_faces as f64);
        let half_face_angle = face_angle / 2.0;

        // The inscribed radius gives us the distance to the face
        let inscribed_radius = self.mu * normal_magnitude * half_face_angle.cos();

        // Find angle offset from nearest face direction
        let normalized_angle = angle.rem_euclid(2.0 * PI);
        let face_idx = ((normalized_angle + half_face_angle) / face_angle).floor();
        let face_center_angle = face_idx * face_angle;
        let offset_from_face = normalized_angle - face_center_angle;

        // The max friction at this angle is inscribed_radius / cos(offset)
        // But we need to handle the case where offset is large
        let cos_offset = offset_from_face.cos().abs().max(1e-10);
        inscribed_radius / cos_offset
    }

    /// Check if a tangential force is within the pyramidal friction cone.
    #[must_use]
    pub fn contains(&self, tangent_force: &Vector3<f64>, normal_magnitude: f64) -> bool {
        if normal_magnitude <= 0.0 {
            return tangent_force.norm() < 1e-10;
        }

        let f_t = Vector2::new(tangent_force.x, tangent_force.y);
        self.contains_2d(&f_t, normal_magnitude)
    }

    /// Generate the linear constraint matrices for an LCP/QP solver.
    ///
    /// Returns the matrix A and vector b such that A * f ≤ b represents
    /// the friction cone constraint.
    ///
    /// Each row corresponds to one face of the pyramid.
    #[must_use]
    pub fn constraint_matrix(&self, normal_magnitude: f64) -> (Vec<[f64; 2]>, Vec<f64>) {
        let half_face_angle = PI / (self.num_faces as f64);
        let max_value = self.mu * normal_magnitude * half_face_angle.cos();

        let mut a_rows = Vec::with_capacity(self.num_faces);
        let mut b = Vec::with_capacity(self.num_faces);

        for dir in &self.face_directions {
            // Face normal (perpendicular to direction, both orientations)
            let normal = Vector2::new(-dir.y, dir.x);
            a_rows.push([normal.x, normal.y]);
            b.push(max_value);
        }

        (a_rows, b)
    }

    /// Convert to an elliptic cone with approximately equivalent friction.
    ///
    /// Uses the inscribed circle radius.
    #[must_use]
    pub fn to_elliptic(&self) -> EllipticFrictionCone {
        // The inscribed circle has radius mu * cos(π/n)
        let effective_mu = self.mu * (PI / (self.num_faces as f64)).cos();
        EllipticFrictionCone::isotropic(effective_mu)
    }
}

impl Default for PyramidalFrictionCone {
    fn default() -> Self {
        Self::octagonal(0.5)
    }
}

/// Combined friction model supporting torsional and rolling components.
///
/// This combines tangential friction, torsional friction, and rolling friction
/// into a unified model, similar to MuJoCo's "condim 6" contacts.
///
/// # MuJoCo Contact Dimensionality
///
/// - condim 1: Normal force only (frictionless)
/// - condim 3: Normal + 2D tangential friction
/// - condim 4: condim 3 + torsional (spinning) friction
/// - condim 6: condim 4 + 2D rolling friction
///
/// # Example
///
/// ```
/// use sim_contact::{CompleteFrictionModel, TorsionalFriction, RollingFriction, EllipticFrictionCone};
/// use nalgebra::Vector3;
///
/// // Full friction model with all components
/// let friction = CompleteFrictionModel::new(
///     EllipticFrictionCone::isotropic(0.5),
///     Some(TorsionalFriction::new(0.03, 0.01)),
///     Some(RollingFriction::new(0.02, 0.05)),
/// );
///
/// let normal = Vector3::z();
/// let tangent_vel = Vector3::new(0.1, 0.0, 0.0);
/// let angular_vel = Vector3::new(0.0, 1.0, 0.5); // Rolling + spinning
/// let normal_force = 100.0;
///
/// let result = friction.compute_force_and_torque(
///     &normal,
///     &tangent_vel,
///     &angular_vel,
///     normal_force,
/// );
///
/// assert!(result.friction.x < 0.0); // Opposes sliding
/// assert!(result.torsional_torque.z < 0.0); // Opposes spinning
/// assert!(result.rolling_torque.y < 0.0); // Opposes rolling
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CompleteFrictionModel {
    /// Tangential friction cone (sliding resistance).
    pub tangential: EllipticFrictionCone,

    /// Optional torsional friction (spinning resistance).
    pub torsional: Option<TorsionalFriction>,

    /// Optional rolling friction (rolling resistance).
    pub rolling: Option<RollingFriction>,
}

/// Result of computing complete friction forces and torques.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CompleteFrictionResult {
    /// Tangential friction force (in world coordinates).
    pub friction: Vector3<f64>,

    /// Torsional friction torque (about contact normal).
    pub torsional_torque: Vector3<f64>,

    /// Rolling friction torque (perpendicular to normal).
    pub rolling_torque: Vector3<f64>,
}

impl CompleteFrictionResult {
    /// Get the total torque (torsional + rolling).
    #[must_use]
    pub fn total_torque(&self) -> Vector3<f64> {
        self.torsional_torque + self.rolling_torque
    }

    /// Check if all components are effectively zero.
    #[must_use]
    pub fn is_zero(&self, epsilon: f64) -> bool {
        self.friction.norm() < epsilon
            && self.torsional_torque.norm() < epsilon
            && self.rolling_torque.norm() < epsilon
    }
}

impl CompleteFrictionModel {
    /// Create a new complete friction model.
    #[must_use]
    pub fn new(
        tangential: EllipticFrictionCone,
        torsional: Option<TorsionalFriction>,
        rolling: Option<RollingFriction>,
    ) -> Self {
        Self {
            tangential,
            torsional,
            rolling,
        }
    }

    /// Create a model with only tangential friction (MuJoCo condim 3).
    #[must_use]
    pub fn tangential_only(mu: f64) -> Self {
        Self {
            tangential: EllipticFrictionCone::isotropic(mu),
            torsional: None,
            rolling: None,
        }
    }

    /// Create a model with tangential and torsional friction (MuJoCo condim 4).
    #[must_use]
    pub fn with_torsional(mu: f64, torsional_mu: f64, contact_radius: f64) -> Self {
        Self {
            tangential: EllipticFrictionCone::isotropic(mu),
            torsional: Some(TorsionalFriction::new(torsional_mu, contact_radius)),
            rolling: None,
        }
    }

    /// Create a complete model with all friction types (MuJoCo condim 6).
    #[must_use]
    pub fn complete(
        mu: f64,
        torsional_mu: f64,
        contact_radius: f64,
        rolling_mu: f64,
        rolling_radius: f64,
    ) -> Self {
        Self {
            tangential: EllipticFrictionCone::isotropic(mu),
            torsional: Some(TorsionalFriction::new(torsional_mu, contact_radius)),
            rolling: Some(RollingFriction::new(rolling_mu, rolling_radius)),
        }
    }

    /// Get the MuJoCo-style contact dimensionality.
    ///
    /// - 1: Frictionless
    /// - 3: Tangential only
    /// - 4: Tangential + torsional
    /// - 6: All friction types
    #[must_use]
    pub fn condim(&self) -> u8 {
        if self.tangential.mu_1 <= 1e-10 && self.tangential.mu_2 <= 1e-10 {
            1
        } else if self.torsional.is_none() {
            3
        } else if self.rolling.is_none() {
            4
        } else {
            6
        }
    }

    /// Compute all friction forces and torques.
    ///
    /// # Arguments
    ///
    /// * `normal` - Contact normal (unit vector pointing from B to A)
    /// * `tangent_velocity` - Relative tangential velocity at contact point
    /// * `angular_velocity` - Relative angular velocity (ω_A - ω_B)
    /// * `normal_force` - Normal force magnitude (positive)
    #[must_use]
    pub fn compute_force_and_torque(
        &self,
        normal: &Vector3<f64>,
        tangent_velocity: &Vector3<f64>,
        angular_velocity: &Vector3<f64>,
        normal_force: f64,
    ) -> CompleteFrictionResult {
        let mut result = CompleteFrictionResult::default();

        if normal_force <= 0.0 {
            return result;
        }

        // Tangential friction
        let speed = tangent_velocity.norm();
        if speed > 1e-10 {
            let direction = -*tangent_velocity / speed;
            let desired_force =
                direction * (self.tangential.mu_1.max(self.tangential.mu_2) * normal_force * 2.0);
            result.friction = self.tangential.project(desired_force, normal_force);
        }

        // Torsional friction
        if let Some(ref torsional) = self.torsional {
            result.torsional_torque =
                torsional.compute_torque(normal, angular_velocity, normal_force);
        }

        // Rolling friction
        if let Some(ref rolling) = self.rolling {
            result.rolling_torque = rolling.compute_torque(normal, angular_velocity, normal_force);
        }

        result
    }

    /// Create presets for common scenarios.
    #[must_use]
    pub fn rubber_ball(radius: f64) -> Self {
        Self::complete(
            0.8,          // High tangential friction
            0.03,         // Moderate torsional
            radius * 0.1, // Contact radius ~10% of ball radius
            0.02,         // Moderate rolling resistance
            radius,       // Rolling radius = ball radius
        )
    }

    /// Preset for a wheel on pavement.
    #[must_use]
    pub fn wheel(radius: f64) -> Self {
        Self::complete(
            0.7,           // Good traction
            0.01,          // Low torsional (tires grip)
            radius * 0.05, // Small contact patch
            0.01,          // Low rolling resistance
            radius,        // Rolling radius = wheel radius
        )
    }

    /// Preset for a box sliding on a surface.
    #[must_use]
    pub fn sliding_box() -> Self {
        Self::with_torsional(
            0.5,  // Moderate sliding friction
            0.05, // Some torsional resistance
            0.02, // Small contact patch
        )
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

    // ==================== Torsional Friction Tests ====================

    #[test]
    fn test_torsional_friction_opposes_spinning() {
        let friction = TorsionalFriction::new(0.05, 0.01);
        let normal = Vector3::z();
        let angular_velocity = Vector3::new(0.0, 0.0, 5.0); // Spinning at 5 rad/s
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);

        // Torque should oppose spinning (negative Z since omega is positive Z)
        assert!(torque.z < 0.0);
    }

    #[test]
    fn test_torsional_friction_magnitude() {
        let mu = 0.05;
        let contact_radius = 0.01;
        let friction = TorsionalFriction::new(mu, contact_radius).with_regularization(0.001); // Small regularization
        let normal = Vector3::z();
        let angular_velocity = Vector3::new(0.0, 0.0, 10.0); // Fast spin
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);

        // At high velocity, torque should approach max: μ * r * F_n = 0.05 * 0.01 * 100 = 0.05
        let expected_max = mu * contact_radius * normal_force;
        assert_relative_eq!(torque.norm(), expected_max, epsilon = 0.001);
    }

    #[test]
    fn test_torsional_friction_regularization() {
        let friction = TorsionalFriction::new(0.05, 0.01).with_regularization(0.1);
        let normal = Vector3::z();
        let normal_force = 100.0;

        // At very low angular velocity, torque should be small (regularized)
        let slow_omega = Vector3::new(0.0, 0.0, 0.01);
        let slow_torque = friction.compute_torque(&normal, &slow_omega, normal_force);

        // At high angular velocity, torque should be near maximum
        let fast_omega = Vector3::new(0.0, 0.0, 1.0);
        let fast_torque = friction.compute_torque(&normal, &fast_omega, normal_force);

        assert!(slow_torque.norm() < fast_torque.norm());
    }

    #[test]
    fn test_torsional_friction_zero_normal_force() {
        let friction = TorsionalFriction::new(0.05, 0.01);
        let normal = Vector3::z();
        let angular_velocity = Vector3::new(0.0, 0.0, 5.0);

        let torque = friction.compute_torque(&normal, &angular_velocity, 0.0);
        assert_relative_eq!(torque.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_torsional_friction_no_spinning() {
        let friction = TorsionalFriction::new(0.05, 0.01);
        let normal = Vector3::z();
        // Angular velocity perpendicular to normal (rolling, not spinning)
        let angular_velocity = Vector3::new(5.0, 0.0, 0.0);
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);
        assert_relative_eq!(torque.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_torsional_friction_max_torque() {
        let friction = TorsionalFriction::new(0.05, 0.01);
        let normal_force = 100.0;

        let max = friction.max_torque(normal_force);
        assert_relative_eq!(max, 0.05, epsilon = 1e-10); // 0.05 * 0.01 * 100
    }

    // ==================== Rolling Friction Tests ====================

    #[test]
    fn test_rolling_friction_opposes_rolling() {
        let friction = RollingFriction::new(0.02, 0.05);
        let normal = Vector3::z();
        // Rolling about Y axis (ball moving in +X direction)
        let angular_velocity = Vector3::new(0.0, 10.0, 0.0);
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);

        // Torque should oppose rolling (negative Y)
        assert!(torque.y < 0.0);
        assert_relative_eq!(torque.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(torque.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_rolling_friction_magnitude() {
        let mu = 0.02;
        let radius = 0.05;
        let friction = RollingFriction::new(mu, radius).with_regularization(0.001);
        let normal = Vector3::z();
        let angular_velocity = Vector3::new(0.0, 10.0, 0.0);
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);

        // At high velocity, torque ≈ μ * r * F_n = 0.02 * 0.05 * 100 = 0.1
        let expected_max = mu * radius * normal_force;
        assert_relative_eq!(torque.norm(), expected_max, epsilon = 0.01);
    }

    #[test]
    fn test_rolling_friction_no_rolling() {
        let friction = RollingFriction::new(0.02, 0.05);
        let normal = Vector3::z();
        // Pure spinning (no rolling component)
        let angular_velocity = Vector3::new(0.0, 0.0, 5.0);
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);
        assert_relative_eq!(torque.norm(), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_rolling_friction_2d_rolling() {
        let friction = RollingFriction::new(0.02, 0.05);
        let normal = Vector3::z();
        // Rolling in both X and Y directions
        let angular_velocity = Vector3::new(3.0, 4.0, 0.0);
        let normal_force = 100.0;

        let torque = friction.compute_torque(&normal, &angular_velocity, normal_force);

        // Torque should oppose the rolling direction
        // Direction should be opposite to angular velocity in XY plane
        assert!(torque.x * angular_velocity.x < 0.0);
        assert!(torque.y * angular_velocity.y < 0.0);
    }

    #[test]
    fn test_rolling_friction_resistance_force() {
        let friction = RollingFriction::new(0.02, 0.05);
        let normal_force = 100.0;

        let resistance = friction.compute_resistance_force(normal_force);
        // F = μ * F_n / r = 0.02 * 100 / 0.05 = 40
        assert_relative_eq!(resistance, 40.0, epsilon = 1e-10);
    }

    // ==================== Pyramidal Friction Cone Tests ====================

    #[test]
    fn test_pyramidal_cone_creation() {
        let cone = PyramidalFrictionCone::new(0.5, 8);
        assert_eq!(cone.num_faces, 8);
        assert_eq!(cone.face_directions.len(), 8);
        assert_relative_eq!(cone.mu, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_pyramidal_cone_presets() {
        let box_cone = PyramidalFrictionCone::box_approximation(0.5);
        let octagonal = PyramidalFrictionCone::octagonal(0.5);
        let high_accuracy = PyramidalFrictionCone::high_accuracy(0.5);

        assert_eq!(box_cone.num_faces, 4);
        assert_eq!(octagonal.num_faces, 8);
        assert_eq!(high_accuracy.num_faces, 16);
    }

    #[test]
    fn test_pyramidal_cone_inside() {
        let cone = PyramidalFrictionCone::octagonal(0.5);
        let force = Vector3::new(10.0, 0.0, 0.0);
        let normal_mag = 100.0; // Max friction ≈ 50 * cos(π/8) ≈ 46

        // Force well inside the cone
        assert!(cone.contains(&force, normal_mag));
    }

    #[test]
    fn test_pyramidal_cone_projection() {
        let cone = PyramidalFrictionCone::octagonal(0.5);
        let force = Vector3::new(100.0, 0.0, 0.0);
        let normal_mag = 100.0;

        let projected = cone.project(force, normal_mag);

        // Should be inside after projection
        assert!(cone.contains(&projected, normal_mag));
        // Magnitude should be reduced
        assert!(projected.norm() < force.norm());
    }

    #[test]
    fn test_pyramidal_cone_projection_preserves_containment() {
        let cone = PyramidalFrictionCone::octagonal(0.5);
        let normal_mag = 100.0;

        let test_forces = vec![
            Vector3::new(1000.0, 0.0, 0.0),
            Vector3::new(0.0, 1000.0, 0.0),
            Vector3::new(500.0, 500.0, 0.0),
            Vector3::new(-200.0, 300.0, 0.0),
        ];

        for force in test_forces {
            let projected = cone.project(force, normal_mag);
            assert!(
                cone.contains(&projected, normal_mag),
                "Projected force {:?} not contained in pyramid",
                projected
            );
        }
    }

    #[test]
    fn test_pyramidal_cone_inscribes_circle() {
        // The pyramidal cone should always be inside the circular cone
        let pyramid = PyramidalFrictionCone::octagonal(0.5);
        let circle = FrictionCone::new(0.5);
        let normal_mag = 100.0;

        // Test forces at pyramid boundary
        for i in 0..16 {
            let angle = 2.0 * PI * (i as f64) / 16.0;
            // Force at the circular cone boundary
            let force = Vector3::new(50.0 * angle.cos(), 50.0 * angle.sin(), 0.0);

            // This should be on/outside the pyramid boundary
            // (pyramid inscribes the circle)
            let in_circle = circle.contains(&force, normal_mag);
            let in_pyramid = pyramid.contains(&force, normal_mag);

            // If it's in the pyramid, it must be in the circle
            if in_pyramid {
                assert!(in_circle);
            }
        }
    }

    #[test]
    fn test_pyramidal_cone_to_elliptic() {
        let pyramid = PyramidalFrictionCone::octagonal(0.5);
        let elliptic = pyramid.to_elliptic();

        // Effective mu should be reduced by cos(π/8)
        let expected_mu = 0.5 * (PI / 8.0).cos();
        assert_relative_eq!(elliptic.mu_1, expected_mu, epsilon = 1e-10);
        assert_relative_eq!(elliptic.mu_2, expected_mu, epsilon = 1e-10);
    }

    #[test]
    fn test_pyramidal_cone_constraint_matrix() {
        let cone = PyramidalFrictionCone::new(0.5, 4);
        let normal_mag = 100.0;

        let (a, b) = cone.constraint_matrix(normal_mag);

        assert_eq!(a.len(), 4);
        assert_eq!(b.len(), 4);

        // All b values should be equal (symmetric cone)
        for b_val in &b {
            assert_relative_eq!(*b_val, b[0], epsilon = 1e-10);
        }
    }

    // ==================== Complete Friction Model Tests ====================

    #[test]
    fn test_complete_friction_model_creation() {
        let model = CompleteFrictionModel::complete(0.5, 0.03, 0.01, 0.02, 0.05);

        assert_eq!(model.condim(), 6);
        assert!(model.torsional.is_some());
        assert!(model.rolling.is_some());
    }

    #[test]
    fn test_complete_friction_model_condim() {
        let frictionless = CompleteFrictionModel::tangential_only(0.0);
        let tangential = CompleteFrictionModel::tangential_only(0.5);
        let with_torsional = CompleteFrictionModel::with_torsional(0.5, 0.03, 0.01);
        let complete = CompleteFrictionModel::complete(0.5, 0.03, 0.01, 0.02, 0.05);

        assert_eq!(frictionless.condim(), 1);
        assert_eq!(tangential.condim(), 3);
        assert_eq!(with_torsional.condim(), 4);
        assert_eq!(complete.condim(), 6);
    }

    #[test]
    fn test_complete_friction_model_compute() {
        let model = CompleteFrictionModel::complete(0.5, 0.03, 0.01, 0.02, 0.05);
        let normal = Vector3::z();
        let tangent_vel = Vector3::new(1.0, 0.0, 0.0);
        let angular_vel = Vector3::new(0.0, 5.0, 2.0); // Rolling + spinning
        let normal_force = 100.0;

        let result =
            model.compute_force_and_torque(&normal, &tangent_vel, &angular_vel, normal_force);

        // Friction should oppose sliding
        assert!(result.friction.x < 0.0);

        // Torsional should oppose spinning
        assert!(result.torsional_torque.z < 0.0);

        // Rolling should oppose rolling
        assert!(result.rolling_torque.y < 0.0);
    }

    #[test]
    fn test_complete_friction_zero_normal_force() {
        let model = CompleteFrictionModel::complete(0.5, 0.03, 0.01, 0.02, 0.05);
        let normal = Vector3::z();
        let tangent_vel = Vector3::new(1.0, 0.0, 0.0);
        let angular_vel = Vector3::new(0.0, 5.0, 2.0);

        let result = model.compute_force_and_torque(&normal, &tangent_vel, &angular_vel, 0.0);

        assert!(result.is_zero(1e-10));
    }

    #[test]
    fn test_complete_friction_presets() {
        let ball = CompleteFrictionModel::rubber_ball(0.1);
        let wheel = CompleteFrictionModel::wheel(0.3);
        let box_model = CompleteFrictionModel::sliding_box();

        assert_eq!(ball.condim(), 6);
        assert_eq!(wheel.condim(), 6);
        assert_eq!(box_model.condim(), 4);
    }

    #[test]
    fn test_complete_friction_result_total_torque() {
        let torsional = Vector3::new(0.0, 0.0, 1.0);
        let rolling = Vector3::new(1.0, 1.0, 0.0);

        let result = CompleteFrictionResult {
            friction: Vector3::zeros(),
            torsional_torque: torsional,
            rolling_torque: rolling,
        };

        let total = result.total_torque();
        assert_relative_eq!(total, torsional + rolling, epsilon = 1e-10);
    }
}
