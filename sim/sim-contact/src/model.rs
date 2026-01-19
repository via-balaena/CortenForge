//! Compliant contact model implementation.
//!
//! This module provides the core contact force computation using a MuJoCo-inspired
//! compliant (spring-damper) model rather than impulse-based collision resolution.
//!
//! # Why Compliant Contacts?
//!
//! Traditional rigid-body physics engines use impulse-based methods that:
//! - Create velocity discontinuities
//! - Require careful tuning of iteration counts
//! - Can be sensitive to contact point ordering
//!
//! Compliant contacts instead model the contact as a stiff spring-damper:
//! - Forces vary smoothly with penetration
//! - Energy dissipation is explicit
//! - Parameters map to measurable material properties
//! - More stable for stiff contacts and small timesteps
//!
//! # MuJoCo Contact Model
//!
//! MuJoCo's contact model (which this is inspired by) uses:
//!
//! ```text
//! F_normal = k * d^p + c * ḋ
//! ```
//!
//! Where:
//! - `d` = penetration depth
//! - `ḋ` = penetration velocity (approaching = positive)
//! - `k` = stiffness
//! - `p` = stiffness power (1.0 = linear, >1 = superlinear)
//! - `c` = damping
//!
//! The superlinear stiffness (p > 1) is important because it allows:
//! - Softer contact at small penetrations (better stability)
//! - Stiffer contact at large penetrations (prevents deep interpenetration)

use nalgebra::Vector3;

use crate::{
    ContactForce, ContactManifold, ContactParams, ContactPoint, FrictionCone, FrictionModel,
    RegularizedFriction,
};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// The compliant contact model.
///
/// This struct holds contact parameters and provides methods to compute
/// contact forces given contact geometry and relative velocities.
///
/// # Example
///
/// ```
/// use sim_contact::{ContactModel, ContactParams, ContactPoint, ContactForce};
/// use nalgebra::{Point3, Vector3};
/// use sim_types::BodyId;
///
/// let model = ContactModel::new(ContactParams::rubber_on_concrete());
///
/// let contact = ContactPoint::new(
///     Point3::origin(),
///     Vector3::z(),     // Normal pointing up
///     0.001,            // 1mm penetration
///     BodyId::new(0),
///     BodyId::new(1),
/// );
///
/// // Relative velocity: sliding in X, approaching in Z
/// let rel_velocity = Vector3::new(0.1, 0.0, -0.5);
///
/// let force = model.compute_force(&contact, &rel_velocity);
///
/// // Normal force pushes up (separating)
/// assert!(force.normal.z > 0.0);
/// // Friction opposes X sliding
/// assert!(force.friction.x < 0.0);
/// ```
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactModel {
    /// Contact parameters (stiffness, damping, friction, etc.)
    params: ContactParams,

    /// Friction model to use.
    friction_model: FrictionModelType,

    /// Whether to use regularized friction (smoother, better for implicit integration).
    use_regularized_friction: bool,

    /// Regularization velocity for smooth friction.
    regularization_velocity: f64,
}

/// Which friction model to use for tangential forces.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum FrictionModelType {
    /// Simple Coulomb friction (discontinuous at zero velocity).
    Coulomb,
    /// Regularized Coulomb (smooth, good for implicit integration).
    #[default]
    RegularizedCoulomb,
    /// Stribeck curve with static/kinetic transition.
    Stribeck,
}

impl ContactModel {
    /// Create a new contact model with the given parameters.
    #[must_use]
    pub fn new(params: ContactParams) -> Self {
        Self {
            params,
            friction_model: FrictionModelType::RegularizedCoulomb,
            use_regularized_friction: true,
            regularization_velocity: 0.001, // 1 mm/s
        }
    }

    /// Create a contact model with default parameters.
    #[must_use]
    pub fn default_model() -> Self {
        Self::new(ContactParams::default())
    }

    /// Set the friction model type.
    #[must_use]
    pub fn with_friction_model(mut self, model: FrictionModelType) -> Self {
        self.friction_model = model;
        self.use_regularized_friction = matches!(model, FrictionModelType::RegularizedCoulomb);
        self
    }

    /// Set the regularization velocity for smooth friction.
    #[must_use]
    pub fn with_regularization_velocity(mut self, velocity: f64) -> Self {
        self.regularization_velocity = velocity.max(1e-6);
        self
    }

    /// Get a reference to the contact parameters.
    #[must_use]
    pub fn params(&self) -> &ContactParams {
        &self.params
    }

    /// Update the contact parameters.
    pub fn set_params(&mut self, params: ContactParams) {
        self.params = params;
    }

    /// Compute the contact force for a single contact point.
    ///
    /// # Arguments
    ///
    /// * `contact` - The contact point geometry
    /// * `relative_velocity` - Velocity of body A minus velocity of body B
    ///   at the contact point (in world coordinates)
    ///
    /// # Returns
    ///
    /// The contact force that should be applied to body A.
    /// The negation should be applied to body B.
    #[must_use]
    pub fn compute_force(
        &self,
        contact: &ContactPoint,
        relative_velocity: &Vector3<f64>,
    ) -> ContactForce {
        // If no penetration, no force
        if contact.penetration <= self.params.contact_margin {
            return ContactForce::zero();
        }

        // Effective penetration (after margin)
        let d = contact.penetration - self.params.contact_margin;

        // Decompose velocity into normal and tangent components
        let (v_n, v_t) = contact.decompose_velocity(relative_velocity);

        // --- Normal Force ---
        // F_n = k * d^p + c * (-v_n)
        // Note: v_n positive = separating, so we use -v_n for "approaching velocity"
        let normal_magnitude = self.compute_normal_force_magnitude(d, -v_n);

        // Ensure non-negative (contacts can only push, not pull)
        let normal_magnitude = normal_magnitude.max(0.0);

        let normal_force = contact.normal * normal_magnitude;

        // --- Tangential (Friction) Force ---
        let friction_force = self.compute_friction_force(&v_t, normal_magnitude);

        ContactForce::new(normal_force, friction_force, contact.position)
    }

    /// Compute the normal force magnitude using the compliant contact model.
    ///
    /// ```text
    /// F_n = k * d^p + c * v_approach
    /// ```
    fn compute_normal_force_magnitude(&self, penetration: f64, approach_velocity: f64) -> f64 {
        if penetration <= 0.0 {
            return 0.0;
        }

        // Spring force: k * d^p
        let spring_force = self.params.stiffness * penetration.powf(self.params.stiffness_power);

        // Damping force: c * v_approach
        // Only apply damping when approaching (v > 0) or always for stability
        let damping_force = self.params.damping * approach_velocity;

        // Total normal force
        let total = spring_force + damping_force;

        // Clamp to non-negative (no adhesion)
        total.max(0.0)
    }

    /// Compute the friction force given tangent velocity and normal force.
    fn compute_friction_force(
        &self,
        tangent_velocity: &Vector3<f64>,
        normal_magnitude: f64,
    ) -> Vector3<f64> {
        if normal_magnitude <= 0.0 {
            return Vector3::zeros();
        }

        match self.friction_model {
            FrictionModelType::Coulomb => {
                // Standard Coulomb friction with cone projection
                let cone = FrictionCone::new(self.params.friction_coefficient);

                let speed = tangent_velocity.norm();
                if speed < 1e-10 {
                    Vector3::zeros()
                } else {
                    // Friction opposes motion
                    let direction = -tangent_velocity / speed;
                    let max_force = cone.max_friction(normal_magnitude);
                    direction * max_force
                }
            }

            FrictionModelType::RegularizedCoulomb => {
                // Smooth friction model
                let friction = RegularizedFriction::new(
                    self.params.friction_coefficient,
                    self.regularization_velocity,
                );
                friction.compute_force(*tangent_velocity, normal_magnitude)
            }

            FrictionModelType::Stribeck => {
                // Full Stribeck curve
                let model = FrictionModel::combined(
                    self.params.friction_coefficient * 1.2, // Static slightly higher
                    self.params.friction_coefficient,
                    0.0, // No viscous component by default
                );
                model.compute_force(*tangent_velocity, normal_magnitude)
            }
        }
    }

    /// Compute forces for all contacts in a manifold.
    ///
    /// Returns the total force and torque about a reference point (typically COM).
    #[must_use]
    pub fn compute_manifold_force(
        &self,
        manifold: &ContactManifold,
        relative_velocity_fn: impl Fn(&ContactPoint) -> Vector3<f64>,
    ) -> ManifoldForceResult {
        let mut total_normal = Vector3::zeros();
        let mut total_friction = Vector3::zeros();
        let mut total_torque = Vector3::zeros();
        let mut forces = Vec::with_capacity(manifold.len());

        for contact in manifold.iter() {
            let rel_vel = relative_velocity_fn(contact);
            let force = self.compute_force(contact, &rel_vel);

            total_normal += force.normal;
            total_friction += force.friction;

            // Compute torque about manifold centroid
            if let Some(centroid) = manifold.centroid() {
                total_torque += force.torque_about(&centroid);
            }

            forces.push(force);
        }

        ManifoldForceResult {
            forces,
            total_normal,
            total_friction,
            total_torque,
        }
    }

    /// Compute the Jacobian of normal force with respect to penetration.
    ///
    /// This is useful for implicit integration:
    /// ```text
    /// dF/dd = k * p * d^(p-1)
    /// ```
    #[must_use]
    pub fn normal_force_jacobian(&self, penetration: f64) -> f64 {
        if penetration <= 0.0 {
            return 0.0;
        }

        self.params.stiffness
            * self.params.stiffness_power
            * penetration.powf(self.params.stiffness_power - 1.0)
    }

    /// Compute the effective "stiffness" at a given penetration.
    ///
    /// This accounts for the nonlinear stiffness power.
    #[must_use]
    pub fn effective_stiffness(&self, penetration: f64) -> f64 {
        self.normal_force_jacobian(penetration)
    }

    /// Compute the characteristic frequency of the contact.
    ///
    /// This is useful for choosing timesteps:
    /// ```text
    /// omega = sqrt(k_eff / m)
    /// dt < 2 / omega for stability
    /// ```
    #[must_use]
    pub fn characteristic_frequency(&self, mass: f64, penetration: f64) -> f64 {
        let k_eff = self
            .effective_stiffness(penetration)
            .max(self.params.stiffness);
        (k_eff / mass).sqrt()
    }

    /// Recommend a timestep for stable explicit integration.
    ///
    /// Uses the CFL-like condition: dt < 2 / omega
    /// With safety factor for damping.
    #[must_use]
    pub fn recommended_timestep(&self, mass: f64, max_penetration: f64) -> f64 {
        let omega = self.characteristic_frequency(mass, max_penetration);
        let dt_critical = 2.0 / omega;

        // Apply safety factor (0.5 for explicit Euler, could be higher for symplectic)
        let safety = 0.5;

        // Also consider damping (critical damping = 2*sqrt(k*m))
        let damping_ratio = self.params.damping / (2.0 * (self.params.stiffness * mass).sqrt());
        let damping_factor = if damping_ratio > 1.0 {
            1.0 / damping_ratio.sqrt()
        } else {
            1.0
        };

        dt_critical * safety * damping_factor
    }

    /// Estimate contact energy stored in penetration.
    ///
    /// Useful for energy balance checks.
    #[must_use]
    pub fn potential_energy(&self, penetration: f64) -> f64 {
        if penetration <= 0.0 {
            return 0.0;
        }

        // Integral of F_spring = k * d^p from 0 to d
        // = k * d^(p+1) / (p+1)
        let p = self.params.stiffness_power;
        self.params.stiffness * penetration.powf(p + 1.0) / (p + 1.0)
    }
}

/// Result of computing forces for a contact manifold.
#[derive(Debug, Clone)]
pub struct ManifoldForceResult {
    /// Individual contact forces.
    pub forces: Vec<ContactForce>,
    /// Sum of all normal forces.
    pub total_normal: Vector3<f64>,
    /// Sum of all friction forces.
    pub total_friction: Vector3<f64>,
    /// Sum of all torques about manifold centroid.
    pub total_torque: Vector3<f64>,
}

impl ManifoldForceResult {
    /// Get the total force (normal + friction).
    #[must_use]
    pub fn total_force(&self) -> Vector3<f64> {
        self.total_normal + self.total_friction
    }

    /// Get the number of contact forces.
    #[must_use]
    pub fn len(&self) -> usize {
        self.forces.len()
    }

    /// Check if there are no contact forces.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.forces.is_empty()
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
    use nalgebra::Point3;
    use sim_types::BodyId;

    fn make_contact(penetration: f64) -> ContactPoint {
        ContactPoint {
            position: Point3::origin(),
            normal: Vector3::z(),
            penetration,
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        }
    }

    #[test]
    fn test_no_force_when_separated() {
        let model = ContactModel::new(ContactParams::default());
        let contact = make_contact(-0.01); // Separated

        let force = model.compute_force(&contact, &Vector3::zeros());
        assert!(force.is_zero(1e-10));
    }

    #[test]
    fn test_normal_force_increases_with_penetration() {
        let model = ContactModel::new(ContactParams::default());

        let contact1 = make_contact(0.001);
        let contact2 = make_contact(0.002);

        let force1 = model.compute_force(&contact1, &Vector3::zeros());
        let force2 = model.compute_force(&contact2, &Vector3::zeros());

        assert!(force2.normal.norm() > force1.normal.norm());
    }

    #[test]
    fn test_damping_increases_force_when_approaching() {
        let model = ContactModel::new(ContactParams::default());
        let contact = make_contact(0.001);

        // At rest
        let force_rest = model.compute_force(&contact, &Vector3::zeros());

        // Approaching (negative z velocity = moving into contact)
        let force_approach = model.compute_force(&contact, &Vector3::new(0.0, 0.0, -1.0));

        assert!(force_approach.normal.z > force_rest.normal.z);
    }

    #[test]
    fn test_damping_decreases_force_when_separating() {
        let model = ContactModel::new(ContactParams::default());
        let contact = make_contact(0.01); // Larger penetration for more spring force

        // At rest
        let force_rest = model.compute_force(&contact, &Vector3::zeros());

        // Separating (positive z velocity)
        let force_separate = model.compute_force(&contact, &Vector3::new(0.0, 0.0, 1.0));

        // Force should be less when separating (damping opposes motion)
        assert!(force_separate.normal.z < force_rest.normal.z);
    }

    #[test]
    fn test_friction_opposes_sliding() {
        let model = ContactModel::new(ContactParams::default().with_friction(0.5));
        let contact = make_contact(0.01);

        // Sliding in +X direction
        let force = model.compute_force(&contact, &Vector3::new(1.0, 0.0, 0.0));

        // Friction should be in -X direction
        assert!(force.friction.x < 0.0);
    }

    #[test]
    fn test_friction_cone_limit() {
        let mu = 0.5;
        let model = ContactModel::new(ContactParams::default().with_friction(mu));
        let contact = make_contact(0.01);

        // Fast sliding
        let force = model.compute_force(&contact, &Vector3::new(10.0, 0.0, 0.0));

        // Friction should be bounded by cone
        let normal_mag = force.normal.norm();
        let friction_mag = force.friction.norm();

        // Allow small tolerance for regularization
        assert!(friction_mag <= mu * normal_mag + 0.1);
    }

    #[test]
    fn test_nonlinear_stiffness() {
        // Power > 1 means stiffer at larger penetrations
        // Use zero margin to get exact power-law behavior
        let params = ContactParams {
            stiffness: 100_000.0,
            stiffness_power: 2.0,
            contact_margin: 0.0, // No margin for exact test
            ..ContactParams::default()
        };
        let model = ContactModel::new(params);

        let contact1 = make_contact(0.01);
        let contact2 = make_contact(0.02);

        let force1 = model.compute_force(&contact1, &Vector3::zeros());
        let force2 = model.compute_force(&contact2, &Vector3::zeros());

        // With power=2, doubling penetration should quadruple force
        let ratio = force2.normal.z / force1.normal.z;
        assert_relative_eq!(ratio, 4.0, epsilon = 0.01);
    }

    #[test]
    fn test_potential_energy() {
        let model = ContactModel::new(ContactParams::default().with_stiffness_power(1.0));

        // For linear spring: E = 0.5 * k * d^2
        // Our formula: E = k * d^2 / 2 (when p=1, integral is k*d^2/2)
        let d = 0.01;
        let energy = model.potential_energy(d);
        let expected = model.params().stiffness * d * d / 2.0;

        assert_relative_eq!(energy, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_characteristic_frequency() {
        let model = ContactModel::new(ContactParams::default());
        let mass = 1.0;
        let penetration = 0.001;

        let omega = model.characteristic_frequency(mass, penetration);

        // Should be sqrt(k/m) for linear spring
        let expected = (model.params().stiffness / mass).sqrt();
        assert!(omega >= expected * 0.5); // At least half (nonlinearity might change it)
    }

    #[test]
    fn test_recommended_timestep() {
        let model = ContactModel::new(ContactParams::default());
        let mass = 1.0;
        let max_penetration = 0.01;

        let dt = model.recommended_timestep(mass, max_penetration);

        // Should be positive and reasonable
        assert!(dt > 0.0);
        assert!(dt < 0.01); // Should be sub-centisecond for stiff contacts
    }

    #[test]
    fn test_manifold_force() {
        let model = ContactModel::new(ContactParams::default());

        let p1 = ContactPoint {
            position: Point3::new(-1.0, 0.0, 0.0),
            normal: Vector3::z(),
            penetration: 0.01,
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        };
        let p2 = ContactPoint {
            position: Point3::new(1.0, 0.0, 0.0),
            normal: Vector3::z(),
            penetration: 0.01,
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        };

        let manifold = ContactManifold::from_points(vec![p1, p2]).unwrap();

        let result = model.compute_manifold_force(&manifold, |_| Vector3::zeros());

        assert_eq!(result.len(), 2);
        assert!(result.total_normal.z > 0.0);
    }

    #[test]
    fn test_different_friction_models() {
        let contact = make_contact(0.01);
        let velocity = Vector3::new(0.1, 0.0, 0.0);

        let coulomb = ContactModel::new(ContactParams::default())
            .with_friction_model(FrictionModelType::Coulomb);
        let regularized = ContactModel::new(ContactParams::default())
            .with_friction_model(FrictionModelType::RegularizedCoulomb);
        let stribeck = ContactModel::new(ContactParams::default())
            .with_friction_model(FrictionModelType::Stribeck);

        let f1 = coulomb.compute_force(&contact, &velocity);
        let f2 = regularized.compute_force(&contact, &velocity);
        let f3 = stribeck.compute_force(&contact, &velocity);

        // All should have friction opposing motion
        assert!(f1.friction.x < 0.0);
        assert!(f2.friction.x < 0.0);
        assert!(f3.friction.x < 0.0);
    }
}
