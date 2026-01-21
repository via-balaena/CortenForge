//! Compliant contact model for physics simulation.
//!
//! This crate provides a MuJoCo-inspired contact model that uses spring-damper
//! forces instead of impulse-based collision resolution. This approach offers:
//!
//! - **Stability**: Smooth force response, no velocity discontinuities
//! - **Tunability**: Physical parameters map to real-world material properties
//! - **Domain randomization**: Contact parameters can be varied for sim-to-real
//! - **Determinism**: Fixed iteration counts, no convergence-based termination
//!
//! # Contact Model
//!
//! The normal contact force follows MuJoCo's soft contact model:
//!
//! ```text
//! F_n = k * d^p + c * ḋ
//! ```
//!
//! Where:
//! - `d` = penetration depth (positive when overlapping)
//! - `ḋ` = penetration velocity (positive when approaching)
//! - `k` = stiffness (N/m^p)
//! - `p` = stiffness power (typically 1.0-2.0)
//! - `c` = damping coefficient (N·s/m)
//!
//! The tangential friction force uses a friction cone:
//!
//! ```text
//! |F_t| ≤ μ * F_n
//! ```
//!
//! Where `μ` is the friction coefficient.
//!
//! # Example
//!
//! ```
//! use sim_contact::{ContactModel, ContactParams, ContactPoint, ContactForce};
//! use nalgebra::{Point3, Vector3};
//!
//! // Create a contact model with physical parameters
//! let model = ContactModel::new(ContactParams::rubber_on_concrete());
//!
//! // Define a contact point (e.g., from narrow-phase detection)
//! let contact = ContactPoint {
//!     position: Point3::new(0.0, 0.0, 0.0),
//!     normal: Vector3::new(0.0, 0.0, 1.0),  // Pointing up
//!     penetration: 0.001,                     // 1mm penetration
//!     body_a: sim_types::BodyId::new(0),
//!     body_b: sim_types::BodyId::new(1),
//! };
//!
//! // Compute contact force given relative velocity
//! let relative_velocity = Vector3::new(0.1, 0.0, -0.5);  // Sliding + approaching
//! let force = model.compute_force(&contact, &relative_velocity);
//!
//! // Normal force pushes bodies apart, friction opposes sliding
//! assert!(force.normal.z > 0.0);  // Repulsive
//! assert!(force.friction.x < 0.0);  // Opposes +X sliding
//! ```
//!
//! # Domain Randomization
//!
//! Contact parameters can be sampled from distributions for sim-to-real transfer:
//!
//! ```
//! use sim_contact::{ContactParams, DomainRandomization};
//!
//! let base = ContactParams::default();
//! let randomizer = DomainRandomization::default();
//!
//! // Sample randomized parameters (use your own RNG)
//! // let randomized = randomizer.sample(&base, &mut rng);
//! ```
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops
//! - Hardware control code
//! - Analysis tools
//! - Other physics engines

#![doc(html_root_url = "https://docs.rs/sim-contact/0.1.0")]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![warn(missing_docs)]
#![allow(clippy::missing_const_for_fn)]

mod contact;
mod friction;
mod model;
mod params;
mod solver;

pub use contact::{ContactForce, ContactManifold, ContactPoint};
pub use friction::{
    CompleteFrictionModel, CompleteFrictionResult, EllipticFrictionCone, FrictionCone,
    FrictionModel, PyramidalFrictionCone, RegularizedFriction, RollingFriction, TorsionalFriction,
};
pub use model::{ContactModel, FrictionModelType};
pub use params::{ContactParams, DomainRandomization, MaterialPair, SurfaceMaterial};
pub use solver::{ContactSolver, ContactSolverConfig};

// Re-export types needed for contact computation
pub use sim_types::{BodyId, Pose, RigidBodyState, Twist, Vector3};

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use nalgebra::Point3;

    #[test]
    fn test_basic_contact_force() {
        let model = ContactModel::new(ContactParams::default());

        let contact = ContactPoint {
            position: Point3::origin(),
            normal: Vector3::z(),
            penetration: 0.001,
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        };

        // Approaching at 0.1 m/s
        let velocity = Vector3::new(0.0, 0.0, -0.1);
        let force = model.compute_force(&contact, &velocity);

        // Should have positive normal force (repulsive)
        assert!(force.normal.z > 0.0);
    }

    #[test]
    fn test_no_force_when_separating() {
        let model = ContactModel::new(ContactParams::default());

        let contact = ContactPoint {
            position: Point3::origin(),
            normal: Vector3::z(),
            penetration: 0.0001, // Tiny penetration
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        };

        // Separating quickly
        let velocity = Vector3::new(0.0, 0.0, 1.0);
        let force = model.compute_force(&contact, &velocity);

        // Force should be small or zero (damping reduces it)
        assert!(force.normal.z >= 0.0); // Never attractive
    }

    #[test]
    fn test_friction_opposes_motion() {
        let model = ContactModel::new(ContactParams::default());

        let contact = ContactPoint {
            position: Point3::origin(),
            normal: Vector3::z(),
            penetration: 0.01, // 1cm penetration for significant normal force
            body_a: BodyId::new(0),
            body_b: BodyId::new(1),
        };

        // Sliding in +X direction
        let velocity = Vector3::new(1.0, 0.0, 0.0);
        let force = model.compute_force(&contact, &velocity);

        // Friction should oppose the motion
        assert!(force.friction.x < 0.0);

        // Friction magnitude bounded by cone
        let normal_mag = force.normal.norm();
        let friction_mag = force.friction.norm();
        let mu = model.params().friction_coefficient;
        assert!(friction_mag <= mu * normal_mag + 1e-10);
    }

    #[test]
    fn test_domain_randomization_bounds() {
        let _base = ContactParams::default();
        let randomizer = DomainRandomization::default();

        // Check that bounds are reasonable
        assert!(randomizer.stiffness_range.0 > 0.0);
        assert!(randomizer.stiffness_range.1 > randomizer.stiffness_range.0);
        assert!(randomizer.friction_range.0 >= 0.0);
        assert!(randomizer.friction_range.1 <= 2.0);
    }
}
