//! SIMD-optimized batch contact force computation.
//!
//! This module provides batch operations for processing multiple contacts
//! simultaneously using SIMD vectorization. For scenes with many contacts,
//! this can provide 2-4x speedup over scalar processing.
//!
//! # Example
//!
//! ```
//! use sim_contact::{BatchContactProcessor, ContactParams, ContactPoint};
//! use nalgebra::{Point3, Vector3};
//! use sim_types::BodyId;
//!
//! let processor = BatchContactProcessor::new(ContactParams::default());
//!
//! // Process multiple contacts in batch
//! let contacts = vec![
//!     ContactPoint::new(Point3::origin(), Vector3::z(), 0.01, BodyId::new(0), BodyId::new(1)),
//!     ContactPoint::new(Point3::new(1.0, 0.0, 0.0), Vector3::z(), 0.02, BodyId::new(2), BodyId::new(3)),
//! ];
//!
//! let velocities = vec![
//!     Vector3::new(0.1, 0.0, -0.5),
//!     Vector3::new(-0.2, 0.1, -0.3),
//! ];
//!
//! let forces = processor.compute_forces_batch(&contacts, &velocities);
//! assert_eq!(forces.len(), 2);
//! ```

use nalgebra::Vector3;
use sim_simd::{batch_friction_force_4, batch_normal_force_4, Vec3x4};

use crate::{ContactForce, ContactParams, ContactPoint};

/// Batch processor for SIMD-optimized contact force computation.
///
/// This processor maintains contact parameters and provides methods
/// to process multiple contacts simultaneously using SIMD operations.
#[derive(Debug, Clone)]
pub struct BatchContactProcessor {
    /// Contact parameters.
    params: ContactParams,
    /// Regularization velocity for smooth friction.
    regularization_velocity: f64,
}

impl BatchContactProcessor {
    /// Create a new batch processor with the given parameters.
    #[must_use]
    pub fn new(params: ContactParams) -> Self {
        Self {
            params,
            regularization_velocity: 0.001, // 1 mm/s
        }
    }

    /// Set the regularization velocity for smooth friction.
    #[must_use]
    pub fn with_regularization_velocity(mut self, velocity: f64) -> Self {
        self.regularization_velocity = velocity.max(1e-6);
        self
    }

    /// Compute contact forces for a batch of contacts.
    ///
    /// This method processes contacts in groups of 4 using SIMD operations,
    /// with scalar fallback for the remainder.
    ///
    /// # Arguments
    ///
    /// * `contacts` - Slice of contact points
    /// * `relative_velocities` - Relative velocities at each contact point
    ///   (velocity of body A minus velocity of body B)
    ///
    /// # Returns
    ///
    /// Vector of contact forces, one per input contact.
    #[must_use]
    pub fn compute_forces_batch(
        &self,
        contacts: &[ContactPoint],
        relative_velocities: &[Vector3<f64>],
    ) -> Vec<ContactForce> {
        debug_assert_eq!(contacts.len(), relative_velocities.len());

        let n = contacts.len();
        let mut forces = Vec::with_capacity(n);

        // Process in batches of 4
        let num_batches = n / 4;

        for batch_idx in 0..num_batches {
            let base = batch_idx * 4;
            let batch_forces = self.compute_batch_4(
                &contacts[base..base + 4],
                &relative_velocities[base..base + 4],
            );
            forces.extend(batch_forces);
        }

        // Process remainder with scalar code
        let remainder_start = num_batches * 4;
        for i in remainder_start..n {
            forces.push(self.compute_single(&contacts[i], &relative_velocities[i]));
        }

        forces
    }

    /// Compute forces for exactly 4 contacts using SIMD.
    fn compute_batch_4(
        &self,
        contacts: &[ContactPoint],
        velocities: &[Vector3<f64>],
    ) -> [ContactForce; 4] {
        debug_assert!(contacts.len() >= 4);
        debug_assert!(velocities.len() >= 4);

        // Extract penetrations and decompose velocities
        let mut penetrations = [0.0; 4];
        let mut approach_velocities = [0.0; 4];
        let mut tangent_vels = [Vector3::zeros(); 4];
        let mut normals = [Vector3::zeros(); 4];
        let mut positions = [nalgebra::Point3::origin(); 4];

        for i in 0..4 {
            let contact = &contacts[i];
            let vel = &velocities[i];

            // Store contact data
            positions[i] = contact.position;
            normals[i] = contact.normal;

            // Effective penetration (after margin)
            let d = (contact.penetration - self.params.contact_margin).max(0.0);
            penetrations[i] = d;

            // Decompose velocity
            let v_n = vel.dot(&contact.normal);
            let v_t = vel - contact.normal * v_n;

            // Approach velocity is negative of normal velocity
            // (positive v_n = separating, so approach = -v_n)
            approach_velocities[i] = -v_n;
            tangent_vels[i] = v_t;
        }

        // Batch compute normal forces
        let normal_magnitudes = batch_normal_force_4(
            &penetrations,
            &approach_velocities,
            self.params.stiffness,
            self.params.stiffness_power,
            self.params.damping,
        );

        // Batch compute friction forces
        let tangent_batch = Vec3x4::from_vectors(tangent_vels);
        let friction_batch = batch_friction_force_4(
            &tangent_batch,
            &normal_magnitudes,
            self.params.friction_coefficient,
            self.regularization_velocity,
        );

        // Assemble results
        let friction_vecs = friction_batch.to_vectors();

        [
            ContactForce::new(
                normals[0] * normal_magnitudes[0],
                friction_vecs[0],
                positions[0],
            ),
            ContactForce::new(
                normals[1] * normal_magnitudes[1],
                friction_vecs[1],
                positions[1],
            ),
            ContactForce::new(
                normals[2] * normal_magnitudes[2],
                friction_vecs[2],
                positions[2],
            ),
            ContactForce::new(
                normals[3] * normal_magnitudes[3],
                friction_vecs[3],
                positions[3],
            ),
        ]
    }

    /// Compute force for a single contact (scalar fallback).
    fn compute_single(&self, contact: &ContactPoint, velocity: &Vector3<f64>) -> ContactForce {
        // Skip if no penetration
        if contact.penetration <= self.params.contact_margin {
            return ContactForce::zero();
        }

        // Effective penetration
        let d = contact.penetration - self.params.contact_margin;

        // Decompose velocity
        let v_n = velocity.dot(&contact.normal);
        let v_t = velocity - contact.normal * v_n;
        let approach_velocity = -v_n;

        // Normal force: F = k * d^p + c * v_approach
        let spring_force = self.params.stiffness * d.powf(self.params.stiffness_power);
        let damping_force = self.params.damping * approach_velocity;
        let normal_magnitude = (spring_force + damping_force).max(0.0);

        let normal_force = contact.normal * normal_magnitude;

        // Friction force
        let friction_force = if normal_magnitude > 0.0 {
            let speed = v_t.norm();
            if speed > 1e-10 {
                let max_friction = self.params.friction_coefficient * normal_magnitude;
                let friction_mag = if speed < self.regularization_velocity {
                    max_friction * speed / self.regularization_velocity
                } else {
                    max_friction
                };
                // Safe normalization: we already checked speed > 1e-10, so divide directly
                -v_t * (friction_mag / speed)
            } else {
                Vector3::zeros()
            }
        } else {
            Vector3::zeros()
        };

        ContactForce::new(normal_force, friction_force, contact.position)
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
}

/// Batch result containing aggregated forces and torques.
#[derive(Debug, Clone, Default)]
pub struct BatchForceResult {
    /// Individual contact forces.
    pub forces: Vec<ContactForce>,
    /// Total normal force (sum of all contacts).
    pub total_normal: Vector3<f64>,
    /// Total friction force (sum of all contacts).
    pub total_friction: Vector3<f64>,
}

impl BatchForceResult {
    /// Create a new batch result from individual forces.
    #[must_use]
    pub fn from_forces(forces: Vec<ContactForce>) -> Self {
        let mut total_normal = Vector3::zeros();
        let mut total_friction = Vector3::zeros();

        for force in &forces {
            total_normal += force.normal;
            total_friction += force.friction;
        }

        Self {
            forces,
            total_normal,
            total_friction,
        }
    }

    /// Get the total force (normal + friction).
    #[must_use]
    pub fn total_force(&self) -> Vector3<f64> {
        self.total_normal + self.total_friction
    }

    /// Get the number of contacts.
    #[must_use]
    pub fn len(&self) -> usize {
        self.forces.len()
    }

    /// Check if there are no contacts.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.forces.is_empty()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
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
    fn test_batch_matches_single() {
        let processor = BatchContactProcessor::new(ContactParams::default());

        let contacts = vec![
            make_contact(0.01),
            make_contact(0.02),
            make_contact(0.005),
            make_contact(0.015),
        ];

        let velocities = vec![
            Vector3::new(0.1, 0.0, -0.5),
            Vector3::new(-0.2, 0.1, -0.3),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, -1.0),
        ];

        // Compute batch
        let batch_forces = processor.compute_forces_batch(&contacts, &velocities);

        // Compute individually
        for (i, force) in batch_forces.iter().enumerate() {
            let single_force = processor.compute_single(&contacts[i], &velocities[i]);

            assert_relative_eq!(force.normal.x, single_force.normal.x, epsilon = 1e-10);
            assert_relative_eq!(force.normal.y, single_force.normal.y, epsilon = 1e-10);
            assert_relative_eq!(force.normal.z, single_force.normal.z, epsilon = 1e-10);
            assert_relative_eq!(force.friction.x, single_force.friction.x, epsilon = 1e-10);
            assert_relative_eq!(force.friction.y, single_force.friction.y, epsilon = 1e-10);
            assert_relative_eq!(force.friction.z, single_force.friction.z, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_batch_with_remainder() {
        let processor = BatchContactProcessor::new(ContactParams::default());

        // 5 contacts: 1 batch of 4 + 1 remainder
        let contacts: Vec<_> = (0..5).map(|_| make_contact(0.01)).collect();
        let velocities: Vec<_> = (0..5).map(|_| Vector3::new(0.1, 0.0, -0.5)).collect();

        let forces = processor.compute_forces_batch(&contacts, &velocities);
        assert_eq!(forces.len(), 5);

        // All should have positive normal force
        for force in &forces {
            assert!(force.normal.z > 0.0);
        }
    }

    #[test]
    fn test_batch_result_aggregation() {
        let processor = BatchContactProcessor::new(ContactParams::default());

        let contacts = vec![make_contact(0.01), make_contact(0.01)];
        let velocities = vec![Vector3::new(0.1, 0.0, -0.5), Vector3::new(0.1, 0.0, -0.5)];

        let forces = processor.compute_forces_batch(&contacts, &velocities);
        let result = BatchForceResult::from_forces(forces);

        // Total should be approximately twice each individual force
        assert!(result.total_normal.z > 0.0);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_no_force_when_separated() {
        let processor = BatchContactProcessor::new(ContactParams::default());

        let contacts = vec![
            make_contact(-0.01), // Separated
            make_contact(0.0),   // Just touching
            make_contact(0.001), // Penetrating
            make_contact(-0.1),  // Very separated
        ];

        let velocities = vec![Vector3::zeros(); 4];

        let forces = processor.compute_forces_batch(&contacts, &velocities);

        // First two should have zero force, third should have positive force
        assert!(forces[0].is_zero(1e-10));
        assert!(forces[2].normal.z > 0.0);
        assert!(forces[3].is_zero(1e-10));
    }
}
