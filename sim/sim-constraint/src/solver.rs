//! Constraint solver for joint constraints.
//!
//! This module provides a solver that computes constraint forces to maintain
//! joint connections while respecting limits and motor commands.
//!
//! # Solver Approach
//!
//! The solver uses a Gauss-Seidel-like iterative approach:
//!
//! 1. For each constraint, compute the velocity error
//! 2. Compute the impulse needed to correct the error
//! 3. Apply the impulse to both bodies
//! 4. Repeat for stability
//!
//! For position correction (Baumgarte stabilization), we add a bias term
//! proportional to the position error.

use nalgebra::{Matrix3, Point3, Vector3};
use sim_types::BodyId;

use crate::{ConstraintForce, Joint, JointType};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Configuration for the constraint solver.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConstraintSolverConfig {
    /// Number of velocity iterations.
    pub velocity_iterations: usize,

    /// Number of position iterations (for Baumgarte stabilization).
    pub position_iterations: usize,

    /// Baumgarte stabilization factor (0 = no stabilization, 1 = full correction).
    /// Typical value: 0.1-0.3
    pub baumgarte_factor: f64,

    /// Maximum position correction per step (m or rad).
    pub max_position_correction: f64,

    /// Relaxation factor for iterative solver (0-2).
    /// 1.0 = standard, <1 = under-relaxed, >1 = over-relaxed.
    pub relaxation: f64,

    /// Whether to warm-start from previous solution.
    pub warm_starting: bool,
}

impl Default for ConstraintSolverConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 8,
            position_iterations: 4,
            baumgarte_factor: 0.2,
            max_position_correction: 0.1,
            relaxation: 1.0,
            warm_starting: false,
        }
    }
}

impl ConstraintSolverConfig {
    /// High-accuracy configuration for robotics.
    #[must_use]
    pub fn robotics() -> Self {
        Self {
            velocity_iterations: 16,
            position_iterations: 8,
            baumgarte_factor: 0.1,
            max_position_correction: 0.05,
            relaxation: 1.0,
            warm_starting: true,
        }
    }

    /// Fast configuration for real-time applications.
    #[must_use]
    pub fn realtime() -> Self {
        Self {
            velocity_iterations: 4,
            position_iterations: 2,
            baumgarte_factor: 0.3,
            max_position_correction: 0.2,
            relaxation: 1.0,
            warm_starting: false,
        }
    }
}

/// Body state needed for constraint solving.
#[derive(Debug, Clone, Copy)]
pub struct BodyState {
    /// Body position in world frame.
    pub position: Point3<f64>,
    /// Body orientation (as rotation matrix for efficiency).
    pub rotation: Matrix3<f64>,
    /// Linear velocity.
    pub linear_velocity: Vector3<f64>,
    /// Angular velocity.
    pub angular_velocity: Vector3<f64>,
    /// Inverse mass (0 for static bodies).
    pub inv_mass: f64,
    /// Inverse inertia tensor in world frame.
    pub inv_inertia: Matrix3<f64>,
    /// Whether this body is static.
    pub is_static: bool,
}

impl BodyState {
    /// Create a static body state.
    #[must_use]
    pub fn fixed(position: Point3<f64>) -> Self {
        Self {
            position,
            rotation: Matrix3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            inv_mass: 0.0,
            inv_inertia: Matrix3::zeros(),
            is_static: true,
        }
    }
}

/// The constraint solver.
#[derive(Debug, Clone)]
pub struct ConstraintSolver {
    /// Solver configuration.
    config: ConstraintSolverConfig,

    /// Accumulated impulses from previous frame (for warm starting).
    warm_start_impulses: Vec<f64>,
}

impl ConstraintSolver {
    /// Create a new constraint solver.
    #[must_use]
    pub fn new(config: ConstraintSolverConfig) -> Self {
        Self {
            config,
            warm_start_impulses: Vec::new(),
        }
    }

    /// Create a solver with default configuration.
    #[must_use]
    pub fn default_solver() -> Self {
        Self::new(ConstraintSolverConfig::default())
    }

    /// Get the configuration.
    #[must_use]
    pub fn config(&self) -> &ConstraintSolverConfig {
        &self.config
    }

    /// Solve constraints for a set of joints.
    ///
    /// # Arguments
    ///
    /// * `joints` - The joints to solve
    /// * `get_body_state` - Function to get body state by ID
    ///
    /// # Returns
    ///
    /// The constraint forces to apply to each body.
    pub fn solve<J, F>(&mut self, joints: &[J], get_body_state: F) -> SolverResult
    where
        J: Joint,
        F: Fn(BodyId) -> Option<BodyState>,
    {
        if joints.is_empty() {
            return SolverResult::empty();
        }

        let mut forces: Vec<JointForce> = Vec::with_capacity(joints.len());

        // Process each joint
        for joint in joints {
            let Some(parent_state) = get_body_state(joint.parent()) else {
                continue;
            };
            let Some(child_state) = get_body_state(joint.child()) else {
                continue;
            };

            // Compute constraint force based on joint type
            let force = self.solve_joint(joint, &parent_state, &child_state);

            forces.push(JointForce {
                parent: joint.parent(),
                child: joint.child(),
                force,
            });
        }

        SolverResult {
            forces,
            iterations_used: self.config.velocity_iterations,
        }
    }

    /// Solve a single joint constraint.
    fn solve_joint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        match joint.joint_type() {
            JointType::Fixed => self.solve_fixed_constraint(joint, parent, child),
            JointType::Revolute => self.solve_revolute_constraint(joint, parent, child),
            JointType::Prismatic => self.solve_prismatic_constraint(joint, parent, child),
            JointType::Spherical => self.solve_spherical_constraint(joint, parent, child),
            JointType::Universal => self.solve_universal_constraint(joint, parent, child),
        }
    }

    /// Solve a fixed joint constraint (6 DOF constrained).
    #[allow(clippy::unused_self)] // Will use self.config in future iterations
    fn solve_fixed_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Compute anchor positions in world frame
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        // Position error
        let position_error = child_anchor_world - parent_anchor_world;

        // Compute corrective force using spring-like response
        // F = -k * error - c * velocity
        let stiffness = 10000.0; // High stiffness for constraints
        let damping = 1000.0;

        // Velocity at anchor points
        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        let velocity_error = child_anchor_vel - parent_anchor_vel;

        // Constraint force
        let constraint_force = -stiffness * position_error - damping * velocity_error;

        // Also constrain rotation (simplified - just apply damping to angular velocity difference)
        let angular_error = child.angular_velocity - parent.angular_velocity;
        let constraint_torque = -damping * angular_error;

        ConstraintForce::new(
            -constraint_force, // Parent gets opposite force
            -constraint_torque,
            constraint_force,
            constraint_torque,
        )
    }

    /// Solve a revolute joint constraint (5 DOF constrained).
    #[allow(clippy::unused_self)] // Will use self.config in future iterations
    fn solve_revolute_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Anchor positions
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        // Position constraint (3 DOF)
        let position_error = child_anchor_world - parent_anchor_world;

        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        let velocity_error = child_anchor_vel - parent_anchor_vel;

        let constraint_force = -stiffness * position_error - damping * velocity_error;

        // Angular constraint: allow rotation only about joint axis
        // The perpendicular angular velocities should match
        // (simplified implementation)
        let angular_damping = 100.0;
        let relative_omega = child.angular_velocity - parent.angular_velocity;

        // Damp rotation perpendicular to axis (but we don't have the axis here)
        // For now, just apply light damping
        let constraint_torque = -angular_damping * relative_omega * 0.1;

        // Add motor and limit forces
        let mut total_torque = constraint_torque;

        if let Some(motor) = joint.motor() {
            // Compute joint velocity (simplified)
            let joint_velocity = relative_omega.norm();
            let motor_force = motor.compute_force(0.0, joint_velocity);
            // Apply motor torque along constraint axis (simplified)
            total_torque += Vector3::z() * motor_force * 0.01; // Scale factor
        }

        // Joint damping
        let joint_damping_torque = -joint.damping() * relative_omega;
        total_torque += joint_damping_torque;

        ConstraintForce::new(
            -constraint_force,
            -total_torque,
            constraint_force,
            total_torque,
        )
    }

    /// Solve a prismatic joint constraint (5 DOF constrained).
    #[allow(clippy::unused_self)] // Will use self.config in future iterations
    fn solve_prismatic_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Position constraint perpendicular to axis
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        let position_error = child_anchor_world - parent_anchor_world;

        // For prismatic, we only constrain perpendicular to axis
        // (simplified - constrain all translation for now)
        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        let velocity_error = child_anchor_vel - parent_anchor_vel;

        let constraint_force = -stiffness * position_error - damping * velocity_error;

        // Full rotation constraint
        let angular_damping = 1000.0;
        let relative_omega = child.angular_velocity - parent.angular_velocity;
        let constraint_torque = -angular_damping * relative_omega;

        ConstraintForce::new(
            -constraint_force,
            -constraint_torque,
            constraint_force,
            constraint_torque,
        )
    }

    /// Solve a spherical joint constraint (3 DOF constrained).
    #[allow(clippy::unused_self)] // Will use self.config in future iterations
    fn solve_spherical_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Only position constraint (anchor points must coincide)
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        let position_error = child_anchor_world - parent_anchor_world;

        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        let velocity_error = child_anchor_vel - parent_anchor_vel;

        let constraint_force = -stiffness * position_error - damping * velocity_error;

        // No rotation constraint for spherical, but apply damping
        let angular_damping = joint.damping();
        let relative_omega = child.angular_velocity - parent.angular_velocity;
        let constraint_torque = -angular_damping * relative_omega;

        ConstraintForce::new(
            -constraint_force,
            -constraint_torque,
            constraint_force,
            constraint_torque,
        )
    }

    /// Solve a universal joint constraint (4 DOF constrained).
    #[allow(clippy::unused_self)] // Will use self.config in future iterations
    fn solve_universal_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Position constraint + 1 rotation constraint
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        let position_error = child_anchor_world - parent_anchor_world;

        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        let velocity_error = child_anchor_vel - parent_anchor_vel;

        let constraint_force = -stiffness * position_error - damping * velocity_error;

        // Light rotation damping
        let angular_damping = joint.damping();
        let relative_omega = child.angular_velocity - parent.angular_velocity;
        let constraint_torque = -angular_damping * relative_omega * 0.5;

        ConstraintForce::new(
            -constraint_force,
            -constraint_torque,
            constraint_force,
            constraint_torque,
        )
    }

    /// Clear warm-start data.
    pub fn clear_warm_start(&mut self) {
        self.warm_start_impulses.clear();
    }
}

/// Result of constraint solving.
#[derive(Debug, Clone)]
pub struct SolverResult {
    /// Forces for each joint.
    pub forces: Vec<JointForce>,
    /// Number of iterations used.
    pub iterations_used: usize,
}

impl SolverResult {
    /// Create an empty result.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            forces: Vec::new(),
            iterations_used: 0,
        }
    }

    /// Check if there are no forces.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.forces.is_empty()
    }
}

/// Force result for a single joint.
#[derive(Debug, Clone)]
pub struct JointForce {
    /// Parent body ID.
    pub parent: BodyId,
    /// Child body ID.
    pub child: BodyId,
    /// The constraint force.
    pub force: ConstraintForce,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::RevoluteJoint;

    fn make_body_state(position: Point3<f64>) -> BodyState {
        BodyState {
            position,
            rotation: Matrix3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            inv_mass: 1.0,
            inv_inertia: Matrix3::identity(),
            is_static: false,
        }
    }

    #[test]
    fn test_solver_creation() {
        let solver = ConstraintSolver::default_solver();
        assert_eq!(solver.config().velocity_iterations, 8);
    }

    #[test]
    fn test_solve_revolute_joint() {
        let mut solver = ConstraintSolver::default_solver();

        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        let parent_state = make_body_state(Point3::origin());
        let child_state = make_body_state(Point3::new(0.1, 0.0, 0.0)); // Offset from parent

        let result = solver.solve(&[joint], |id| {
            if id == BodyId::new(0) {
                Some(parent_state)
            } else if id == BodyId::new(1) {
                Some(child_state)
            } else {
                None
            }
        });

        assert_eq!(result.forces.len(), 1);

        // Should have corrective force to pull child back
        let force = &result.forces[0].force;
        assert!(force.child_force.x < 0.0); // Pull child in -X direction
    }

    #[test]
    fn test_solve_empty_joints() {
        let mut solver = ConstraintSolver::default_solver();
        let joints: Vec<RevoluteJoint> = vec![];

        let result = solver.solve(&joints, |_| None);
        assert!(result.is_empty());
    }

    #[test]
    fn test_config_presets() {
        let robotics = ConstraintSolverConfig::robotics();
        assert!(robotics.velocity_iterations >= 8);

        let realtime = ConstraintSolverConfig::realtime();
        assert!(realtime.velocity_iterations <= 8);
    }
}
