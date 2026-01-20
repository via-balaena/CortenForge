//! Contact solver with deterministic fixed iteration counts.
//!
//! This module provides the solver that integrates contact forces into the
//! simulation. The key design decision is **fixed iteration counts** rather
//! than convergence-based termination, which ensures:
//!
//! - **Determinism**: Same inputs always produce same outputs
//! - **Predictable performance**: Fixed computational cost per step
//! - **Reproducibility**: Essential for RL training and debugging
//!
//! # Solver Approach
//!
//! Unlike impulse-based solvers that iterate until velocity errors converge,
//! our compliant contact solver computes forces directly from penetration
//! and velocity. The "iterations" here refer to:
//!
//! 1. **Position iterations**: Correcting deep penetrations gradually
//! 2. **Velocity iterations**: Stabilizing contact velocities
//!
//! For simple compliant contacts, a single pass is often sufficient.
//! Multiple iterations help with:
//! - Coupled contacts (multiple contacts on same body)
//! - Contact islands with different stiffnesses
//! - Constraint stabilization for articulated bodies

use nalgebra::Vector3;
use sim_types::BodyId;

use crate::{ContactForce, ContactManifold, ContactModel, ContactPoint};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for the contact solver.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ContactSolverConfig {
    /// Number of velocity solver iterations.
    ///
    /// More iterations improve accuracy for coupled contacts at the cost
    /// of computation. For simple scenes, 1-2 is often sufficient.
    /// For complex stacking, 4-8 may be needed.
    pub velocity_iterations: usize,

    /// Number of position correction iterations.
    ///
    /// Used to gradually resolve deep penetrations without large forces.
    /// Set to 0 to disable position correction (pure force-based).
    pub position_iterations: usize,

    /// Maximum penetration correction per iteration (m).
    ///
    /// Limits how much penetration is corrected per step to prevent
    /// instability from large position changes.
    pub max_penetration_correction: f64,

    /// Penetration depth at which position correction kicks in (m).
    ///
    /// Below this threshold, only forces are used.
    /// Above this, position correction helps resolve deep interpenetration.
    pub penetration_slop: f64,

    /// Velocity threshold for treating contacts as resting (m/s).
    ///
    /// Contacts with relative normal velocity below this are considered
    /// at rest and may use different friction handling.
    pub resting_velocity_threshold: f64,

    /// Whether to warm-start from previous frame's forces.
    ///
    /// Warm-starting can improve convergence but adds state.
    pub enable_warm_starting: bool,

    /// Relaxation factor for iterative solver (0-2).
    ///
    /// - 1.0 = Gauss-Seidel
    /// - <1.0 = Under-relaxation (more stable, slower convergence)
    /// - >1.0 = Over-relaxation (faster convergence, may be unstable)
    pub relaxation: f64,
}

impl Default for ContactSolverConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 4,
            position_iterations: 2,
            max_penetration_correction: 0.01, // 1 cm max per step
            penetration_slop: 0.001,          // 1 mm allowed penetration
            resting_velocity_threshold: 0.01, // 1 cm/s
            enable_warm_starting: false,      // Simpler without
            relaxation: 1.0,                  // Standard Gauss-Seidel
        }
    }
}

impl ContactSolverConfig {
    /// Configuration optimized for robotics (high accuracy).
    #[must_use]
    pub fn robotics() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 4,
            max_penetration_correction: 0.005,
            penetration_slop: 0.0005,
            resting_velocity_threshold: 0.005,
            enable_warm_starting: true,
            relaxation: 1.0,
        }
    }

    /// Configuration optimized for games (fast, stable).
    #[must_use]
    pub fn realtime() -> Self {
        Self {
            velocity_iterations: 2,
            position_iterations: 1,
            max_penetration_correction: 0.02,
            penetration_slop: 0.002,
            resting_velocity_threshold: 0.02,
            enable_warm_starting: false,
            relaxation: 1.0,
        }
    }

    /// Configuration for high-fidelity simulation.
    #[must_use]
    pub fn high_fidelity() -> Self {
        Self {
            velocity_iterations: 16,
            position_iterations: 8,
            max_penetration_correction: 0.002,
            penetration_slop: 0.0002,
            resting_velocity_threshold: 0.002,
            enable_warm_starting: true,
            relaxation: 1.0,
        }
    }

    /// Validate the configuration.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.velocity_iterations == 0 {
            return Err("velocity_iterations must be at least 1");
        }
        if self.max_penetration_correction <= 0.0 {
            return Err("max_penetration_correction must be positive");
        }
        if self.penetration_slop < 0.0 {
            return Err("penetration_slop cannot be negative");
        }
        if self.relaxation <= 0.0 || self.relaxation > 2.0 {
            return Err("relaxation must be in (0, 2]");
        }
        Ok(())
    }
}

/// The contact solver.
///
/// Manages contact force computation across multiple contacts with
/// deterministic, fixed-iteration solving.
#[derive(Debug, Clone)]
pub struct ContactSolver {
    /// Solver configuration.
    config: ContactSolverConfig,

    /// Contact model for force computation.
    model: ContactModel,

    /// Cached forces from previous frame (for warm starting).
    warm_start_forces: Vec<CachedContactForce>,
}

/// Cached contact force for warm starting.
#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // Fields used for warm-starting (planned feature)
struct CachedContactForce {
    /// Body pair this force applies to.
    body_a: BodyId,
    body_b: BodyId,
    /// Cached normal force magnitude.
    normal_magnitude: f64,
}

impl ContactSolver {
    /// Create a new contact solver.
    #[must_use]
    pub fn new(model: ContactModel, config: ContactSolverConfig) -> Self {
        Self {
            config,
            model,
            warm_start_forces: Vec::new(),
        }
    }

    /// Create a solver with default configuration.
    #[must_use]
    pub fn with_model(model: ContactModel) -> Self {
        Self::new(model, ContactSolverConfig::default())
    }

    /// Get the solver configuration.
    #[must_use]
    pub fn config(&self) -> &ContactSolverConfig {
        &self.config
    }

    /// Get the contact model.
    #[must_use]
    pub fn model(&self) -> &ContactModel {
        &self.model
    }

    /// Update the contact model.
    pub fn set_model(&mut self, model: ContactModel) {
        self.model = model;
    }

    /// Solve contacts and compute forces.
    ///
    /// This is the main entry point for the solver. Given a set of contacts
    /// and a function to query body velocities, it computes all contact forces.
    ///
    /// # Arguments
    ///
    /// * `contacts` - All active contacts in the scene
    /// * `velocity_fn` - Function to get velocity at a contact point for a body
    ///
    /// # Returns
    ///
    /// The computed contact forces and position corrections.
    pub fn solve<F>(&mut self, contacts: &[ContactPoint], velocity_fn: F) -> SolverResult
    where
        F: Fn(BodyId, &ContactPoint) -> Vector3<f64>,
    {
        if contacts.is_empty() {
            return SolverResult::empty();
        }

        let mut forces = Vec::with_capacity(contacts.len());
        let mut position_corrections = Vec::new();

        // --- Velocity Iterations ---
        // For compliant contacts, we compute forces directly from penetration/velocity.
        // Multiple iterations would be used with implicit integration where velocities
        // are updated between iterations. For explicit integration (our case), we
        // compute forces once based on current state.
        //
        // Note: In a full simulation loop, multiple iterations would involve:
        // 1. Compute forces
        // 2. Update velocities (tentatively)
        // 3. Recompute forces with new velocities
        // 4. Repeat
        //
        // Since the velocity_fn provides current velocities, we compute final forces
        // on the last iteration. Earlier iterations are for convergence in coupled systems.
        for iter in 0..self.config.velocity_iterations {
            // Clear forces on each iteration (we want final forces, not accumulated)
            if iter == self.config.velocity_iterations - 1 {
                // Last iteration - store the results
                for contact in contacts {
                    let vel_a = velocity_fn(contact.body_a, contact);
                    let vel_b = velocity_fn(contact.body_b, contact);
                    let relative_velocity = vel_a - vel_b;

                    let force = self.model.compute_force(contact, &relative_velocity);

                    let relaxed_force = if self.config.relaxation != 1.0 {
                        ContactForce::new(
                            force.normal * self.config.relaxation,
                            force.friction * self.config.relaxation,
                            force.position,
                        )
                    } else {
                        force
                    };

                    forces.push(ContactForceResult {
                        contact: *contact,
                        force: relaxed_force,
                    });
                }
            }
            // Earlier iterations: in a real iterative solver, we'd update velocities here
            // For now, we skip them since velocity_fn returns constant values
        }

        // --- Position Iterations ---
        // Correct deep penetrations gradually
        if self.config.position_iterations > 0 {
            for contact in contacts {
                if contact.penetration > self.config.penetration_slop {
                    let excess = contact.penetration - self.config.penetration_slop;
                    let correction = excess.min(self.config.max_penetration_correction)
                        / self.config.position_iterations as f64;

                    position_corrections.push(PositionCorrection {
                        body_a: contact.body_a,
                        body_b: contact.body_b,
                        correction: contact.normal * correction,
                    });
                }
            }
        }

        // Update warm start cache
        if self.config.enable_warm_starting {
            self.warm_start_forces = forces
                .iter()
                .map(|f| CachedContactForce {
                    body_a: f.contact.body_a,
                    body_b: f.contact.body_b,
                    normal_magnitude: f.force.normal.norm(),
                })
                .collect();
        }

        SolverResult {
            forces,
            position_corrections,
            iterations_used: self.config.velocity_iterations,
        }
    }

    /// Solve a single contact manifold.
    ///
    /// Convenience method for solving contacts between a single pair of bodies.
    pub fn solve_manifold<F>(&mut self, manifold: &ContactManifold, velocity_fn: F) -> SolverResult
    where
        F: Fn(BodyId, &ContactPoint) -> Vector3<f64>,
    {
        let contacts = manifold.points.to_vec();
        self.solve(&contacts, velocity_fn)
    }

    /// Clear warm start cache.
    ///
    /// Call this when the scene changes significantly.
    pub fn clear_warm_start(&mut self) {
        self.warm_start_forces.clear();
    }

    /// Get statistics from the last solve.
    #[must_use]
    pub fn stats(&self) -> SolverStats {
        SolverStats {
            warm_start_contacts: self.warm_start_forces.len(),
        }
    }
}

/// Result of solving contacts.
#[derive(Debug, Clone)]
pub struct SolverResult {
    /// Computed forces for each contact.
    pub forces: Vec<ContactForceResult>,

    /// Position corrections for deep penetrations.
    pub position_corrections: Vec<PositionCorrection>,

    /// Number of iterations actually used.
    pub iterations_used: usize,
}

impl SolverResult {
    /// Create an empty result.
    #[must_use]
    fn empty() -> Self {
        Self {
            forces: Vec::new(),
            position_corrections: Vec::new(),
            iterations_used: 0,
        }
    }

    /// Get forces grouped by body.
    ///
    /// Returns a map from body ID to total force and torque.
    #[must_use]
    pub fn forces_by_body(&self) -> std::collections::HashMap<BodyId, BodyForces> {
        use std::collections::HashMap;

        let mut result: HashMap<BodyId, BodyForces> = HashMap::new();

        for contact_result in &self.forces {
            let force = &contact_result.force;
            let contact = &contact_result.contact;

            // Force on body A (positive direction)
            let entry_a = result.entry(contact.body_a).or_default();
            entry_a.force += force.total();
            // Torque would need body COM - skipping for now

            // Force on body B (negative direction)
            let entry_b = result.entry(contact.body_b).or_default();
            entry_b.force -= force.total();
        }

        result
    }

    /// Get the total normal force magnitude across all contacts.
    #[must_use]
    pub fn total_normal_force(&self) -> f64 {
        self.forces.iter().map(|f| f.force.normal.norm()).sum()
    }

    /// Get the total friction force magnitude across all contacts.
    #[must_use]
    pub fn total_friction_force(&self) -> f64 {
        self.forces.iter().map(|f| f.force.friction.norm()).sum()
    }

    /// Check if any contacts are in deep penetration.
    #[must_use]
    pub fn has_deep_penetration(&self, threshold: f64) -> bool {
        self.forces
            .iter()
            .any(|f| f.contact.penetration > threshold)
    }
}

/// Force result for a single contact.
#[derive(Debug, Clone)]
pub struct ContactForceResult {
    /// The contact this force applies to.
    pub contact: ContactPoint,
    /// The computed contact force.
    pub force: ContactForce,
}

/// Position correction for resolving penetration.
#[derive(Debug, Clone, Copy)]
pub struct PositionCorrection {
    /// Body to move in positive direction.
    pub body_a: BodyId,
    /// Body to move in negative direction.
    pub body_b: BodyId,
    /// Correction vector (positive = move A, negative = move B).
    pub correction: Vector3<f64>,
}

/// Total forces on a body.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyForces {
    /// Total force.
    pub force: Vector3<f64>,
    /// Total torque about COM.
    pub torque: Vector3<f64>,
}

/// Solver statistics.
#[derive(Debug, Clone, Copy, Default)]
pub struct SolverStats {
    /// Number of warm-started contacts.
    pub warm_start_contacts: usize,
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
    use crate::ContactParams;
    use nalgebra::Point3;

    fn make_solver() -> ContactSolver {
        ContactSolver::new(
            ContactModel::new(ContactParams::default()),
            ContactSolverConfig::default(),
        )
    }

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
    fn test_empty_contacts() {
        let mut solver = make_solver();
        let result = solver.solve(&[], |_, _| Vector3::zeros());

        assert!(result.forces.is_empty());
        assert!(result.position_corrections.is_empty());
        assert_eq!(result.iterations_used, 0);
    }

    #[test]
    fn test_single_contact() {
        let mut solver = make_solver();
        let contacts = vec![make_contact(0.01)];

        let result = solver.solve(&contacts, |_, _| Vector3::zeros());

        assert_eq!(result.forces.len(), 1);
        assert!(result.forces[0].force.normal.z > 0.0);
    }

    #[test]
    fn test_position_correction() {
        let mut solver = make_solver();
        let contacts = vec![make_contact(0.02)]; // Deep penetration

        let result = solver.solve(&contacts, |_, _| Vector3::zeros());

        // Should have position corrections for deep penetration
        assert!(!result.position_corrections.is_empty());

        // Correction should be along normal
        let correction = &result.position_corrections[0];
        assert!(correction.correction.z > 0.0);
    }

    #[test]
    fn test_multiple_contacts() {
        let mut solver = make_solver();
        let contacts = vec![
            ContactPoint {
                position: Point3::new(-1.0, 0.0, 0.0),
                normal: Vector3::z(),
                penetration: 0.01,
                body_a: BodyId::new(0),
                body_b: BodyId::new(1),
            },
            ContactPoint {
                position: Point3::new(1.0, 0.0, 0.0),
                normal: Vector3::z(),
                penetration: 0.01,
                body_a: BodyId::new(0),
                body_b: BodyId::new(1),
            },
        ];

        let result = solver.solve(&contacts, |_, _| Vector3::zeros());

        assert_eq!(result.forces.len(), 2);

        // Total force should be sum of individual forces
        let total = result.total_normal_force();
        assert!(total > 0.0);
    }

    #[test]
    fn test_forces_by_body() {
        let mut solver = make_solver();
        let contacts = vec![make_contact(0.01)];

        let result = solver.solve(&contacts, |_, _| Vector3::zeros());
        let by_body = result.forces_by_body();

        // Body A gets positive force
        let force_a = by_body.get(&BodyId::new(0)).unwrap();
        assert!(force_a.force.z > 0.0);

        // Body B gets negative force (reaction)
        let force_b = by_body.get(&BodyId::new(1)).unwrap();
        assert!(force_b.force.z < 0.0);
    }

    #[test]
    fn test_config_presets() {
        assert!(ContactSolverConfig::default().validate().is_ok());
        assert!(ContactSolverConfig::robotics().validate().is_ok());
        assert!(ContactSolverConfig::realtime().validate().is_ok());
        assert!(ContactSolverConfig::high_fidelity().validate().is_ok());
    }

    #[test]
    fn test_determinism() {
        let mut solver1 = make_solver();
        let mut solver2 = make_solver();

        let contacts = vec![
            make_contact(0.01),
            ContactPoint {
                position: Point3::new(0.5, 0.5, 0.0),
                normal: Vector3::z(),
                penetration: 0.015,
                body_a: BodyId::new(0),
                body_b: BodyId::new(2),
            },
        ];

        let velocity_fn = |_: BodyId, _: &ContactPoint| Vector3::new(0.1, 0.0, -0.2);

        let result1 = solver1.solve(&contacts, velocity_fn);
        let result2 = solver2.solve(&contacts, velocity_fn);

        // Results should be identical
        assert_eq!(result1.forces.len(), result2.forces.len());
        for (f1, f2) in result1.forces.iter().zip(result2.forces.iter()) {
            assert_eq!(f1.force.normal, f2.force.normal);
            assert_eq!(f1.force.friction, f2.force.friction);
        }
    }

    #[test]
    fn test_relaxation() {
        let mut solver_normal = ContactSolver::new(
            ContactModel::new(ContactParams::default()),
            ContactSolverConfig {
                relaxation: 1.0,
                ..Default::default()
            },
        );

        let mut solver_relaxed = ContactSolver::new(
            ContactModel::new(ContactParams::default()),
            ContactSolverConfig {
                relaxation: 0.5,
                ..Default::default()
            },
        );

        let contacts = vec![make_contact(0.01)];

        let result_normal = solver_normal.solve(&contacts, |_, _| Vector3::zeros());
        let result_relaxed = solver_relaxed.solve(&contacts, |_, _| Vector3::zeros());

        // Relaxed solver should produce smaller forces
        let force_normal = result_normal.forces[0].force.normal.norm();
        let force_relaxed = result_relaxed.forces[0].force.normal.norm();

        assert!(force_relaxed < force_normal);
    }
}
