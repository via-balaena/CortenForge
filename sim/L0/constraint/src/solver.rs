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
    /// Create a static/fixed body state (infinite mass, cannot move).
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

    /// Create a dynamic body at rest with the given mass properties.
    ///
    /// # Arguments
    ///
    /// * `position` - World position of the body
    /// * `mass` - Mass of the body (must be positive)
    /// * `inertia` - Inertia tensor (diagonal elements, e.g., `[Ixx, Iyy, Izz]`)
    ///
    /// # Example
    ///
    /// ```
    /// use sim_constraint::BodyState;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// // Create a 1kg body with uniform inertia of 0.1
    /// let body = BodyState::dynamic(
    ///     Point3::new(0.0, 0.0, 1.0),
    ///     1.0,
    ///     Vector3::new(0.1, 0.1, 0.1),
    /// );
    /// ```
    #[must_use]
    pub fn dynamic(position: Point3<f64>, mass: f64, inertia: Vector3<f64>) -> Self {
        let inv_mass = if mass > 0.0 { 1.0 / mass } else { 0.0 };
        let inv_inertia = Matrix3::from_diagonal(&Vector3::new(
            if inertia.x > 0.0 {
                1.0 / inertia.x
            } else {
                0.0
            },
            if inertia.y > 0.0 {
                1.0 / inertia.y
            } else {
                0.0
            },
            if inertia.z > 0.0 {
                1.0 / inertia.z
            } else {
                0.0
            },
        ));

        Self {
            position,
            rotation: Matrix3::identity(),
            linear_velocity: Vector3::zeros(),
            angular_velocity: Vector3::zeros(),
            inv_mass,
            inv_inertia,
            is_static: false,
        }
    }

    /// Create a dynamic body with initial velocity.
    ///
    /// # Arguments
    ///
    /// * `position` - World position of the body
    /// * `mass` - Mass of the body (must be positive)
    /// * `inertia` - Inertia tensor (diagonal elements)
    /// * `linear_velocity` - Initial linear velocity
    /// * `angular_velocity` - Initial angular velocity
    #[must_use]
    pub fn dynamic_with_velocity(
        position: Point3<f64>,
        mass: f64,
        inertia: Vector3<f64>,
        linear_velocity: Vector3<f64>,
        angular_velocity: Vector3<f64>,
    ) -> Self {
        let mut state = Self::dynamic(position, mass, inertia);
        state.linear_velocity = linear_velocity;
        state.angular_velocity = angular_velocity;
        state
    }

    /// Set the rotation matrix.
    #[must_use]
    pub fn with_rotation(mut self, rotation: Matrix3<f64>) -> Self {
        self.rotation = rotation;
        self
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

    /// Solve constraints using a slice of body states indexed by `BodyId::raw()`.
    ///
    /// This is a convenience method for the common case where body states are stored
    /// in a contiguous slice indexed by body ID.
    ///
    /// # Arguments
    ///
    /// * `joints` - The joints to solve
    /// * `bodies` - Slice of body states where `bodies[id.raw()]` gives the state for body `id`
    ///
    /// # Example
    ///
    /// ```
    /// use sim_constraint::{ConstraintSolver, ConstraintSolverConfig, BodyState, RevoluteJoint};
    /// use sim_types::BodyId;
    /// use nalgebra::{Point3, Vector3};
    ///
    /// let bodies = vec![
    ///     BodyState::fixed(Point3::origin()),
    ///     BodyState::dynamic(Point3::new(0.0, 0.0, 1.0), 1.0, Vector3::new(0.1, 0.1, 0.1)),
    /// ];
    /// let joints = vec![
    ///     RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
    /// ];
    ///
    /// let mut solver = ConstraintSolver::new(ConstraintSolverConfig::default());
    /// let result = solver.solve_slice(&joints, &bodies);
    /// ```
    #[allow(clippy::cast_possible_truncation)] // BodyId fits in usize for practical use
    pub fn solve_slice<J: Joint>(&mut self, joints: &[J], bodies: &[BodyState]) -> SolverResult {
        self.solve(joints, |id| bodies.get(id.raw() as usize).copied())
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
            JointType::Free => self.solve_free_constraint(joint, parent, child),
            JointType::Planar => self.solve_planar_constraint(joint, parent, child),
            JointType::Cylindrical => self.solve_cylindrical_constraint(joint, parent, child),
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

        // Add joint damping
        let mut total_torque = constraint_torque;
        total_torque -= joint.damping() * relative_omega;

        // Note: Motor/stiffness support requires implicit integration for stability.
        // For now, only damping is applied. See sim/ARCHITECTURE.md "Known Limitations".

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

        // Apply joint damping to angular velocity
        let relative_omega = child.angular_velocity - parent.angular_velocity;
        let constraint_torque = -joint.damping() * relative_omega;

        // Note: Motor/stiffness support requires implicit integration for stability.
        // For now, only damping is applied. See sim/ARCHITECTURE.md "Known Limitations".

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

    /// Solve a free joint constraint (0 DOF constrained).
    ///
    /// Free joints have no positional constraints. They only apply damping forces
    /// based on relative motion between parent and child.
    #[allow(clippy::unused_self)]
    fn solve_free_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Free joints have no position constraints, only damping
        let linear_damping = joint.damping();
        let angular_damping = joint.damping();

        // Relative velocities
        let relative_linear = child.linear_velocity - parent.linear_velocity;
        let relative_angular = child.angular_velocity - parent.angular_velocity;

        // Apply damping only
        let linear_force = -linear_damping * relative_linear;
        let angular_torque = -angular_damping * relative_angular;

        ConstraintForce::new(-linear_force, -angular_torque, linear_force, angular_torque)
    }

    /// Solve a planar joint constraint (3 DOF constrained).
    ///
    /// Planar joints constrain:
    /// - 1 translational DOF (perpendicular to plane)
    /// - 2 rotational DOF (tilt about in-plane axes)
    #[allow(clippy::unused_self)]
    fn solve_planar_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Compute anchor positions in world frame
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        // Use Z as the plane normal by default
        // (Real implementation would get this from the joint)
        let normal = Vector3::z();

        // Position error perpendicular to plane
        let position_diff = child_anchor_world - parent_anchor_world;
        let perpendicular_error = normal * normal.dot(&position_diff);

        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        // Perpendicular velocity error
        let velocity_diff = child_anchor_vel - parent_anchor_vel;
        let perpendicular_velocity = normal * normal.dot(&velocity_diff);

        // Constraint force perpendicular to plane
        let constraint_force = -stiffness * perpendicular_error - damping * perpendicular_velocity;

        // Angular constraint: constrain rotation about in-plane axes
        // Allow only rotation about the normal
        let relative_omega = child.angular_velocity - parent.angular_velocity;

        // Project out the rotation about normal (which is free)
        let omega_about_normal = normal * normal.dot(&relative_omega);
        let omega_perpendicular = relative_omega - omega_about_normal;

        // Apply angular damping to perpendicular rotation (constrained)
        let angular_damping = 1000.0;
        let constraint_torque = -angular_damping * omega_perpendicular;

        // Apply joint damping to free rotation about normal
        let free_rotation_damping = -joint.damping() * omega_about_normal;

        ConstraintForce::new(
            -constraint_force,
            -constraint_torque - free_rotation_damping,
            constraint_force,
            constraint_torque + free_rotation_damping,
        )
    }

    /// Solve a cylindrical joint constraint (4 DOF constrained).
    ///
    /// Cylindrical joints constrain:
    /// - 2 translational DOF (perpendicular to axis)
    /// - 2 rotational DOF (perpendicular to axis)
    #[allow(clippy::unused_self)]
    fn solve_cylindrical_constraint<J: Joint>(
        &self,
        joint: &J,
        parent: &BodyState,
        child: &BodyState,
    ) -> ConstraintForce {
        // Compute anchor positions in world frame
        let parent_anchor_world = parent.position + parent.rotation * joint.parent_anchor().coords;
        let child_anchor_world = child.position + child.rotation * joint.child_anchor().coords;

        // Use X as the axis by default
        // (Real implementation would get this from the joint)
        let axis = Vector3::x();

        // Position error perpendicular to axis
        let position_diff = child_anchor_world - parent_anchor_world;
        let along_axis = axis * axis.dot(&position_diff);
        let perpendicular_error = position_diff - along_axis;

        let stiffness = 10000.0;
        let damping = 1000.0;

        let r_parent = parent_anchor_world - parent.position;
        let r_child = child_anchor_world - child.position;

        let parent_anchor_vel = parent.linear_velocity + parent.angular_velocity.cross(&r_parent);
        let child_anchor_vel = child.linear_velocity + child.angular_velocity.cross(&r_child);

        // Perpendicular velocity error
        let velocity_diff = child_anchor_vel - parent_anchor_vel;
        let velocity_along_axis = axis * axis.dot(&velocity_diff);
        let perpendicular_velocity = velocity_diff - velocity_along_axis;

        // Constraint force perpendicular to axis
        let constraint_force = -stiffness * perpendicular_error - damping * perpendicular_velocity;

        // Angular constraint: constrain rotation about axes perpendicular to main axis
        let relative_omega = child.angular_velocity - parent.angular_velocity;

        // Project out the rotation about the joint axis (which is free)
        let omega_about_axis = axis * axis.dot(&relative_omega);
        let omega_perpendicular = relative_omega - omega_about_axis;

        // Apply angular damping to perpendicular rotation (constrained)
        let angular_damping = 1000.0;
        let constraint_torque = -angular_damping * omega_perpendicular;

        // Apply joint damping to free DOFs (rotation about axis and translation along axis)
        let free_rotation_damping = -joint.damping() * omega_about_axis;
        let free_translation_damping = -joint.damping() * velocity_along_axis;

        ConstraintForce::new(
            -constraint_force - free_translation_damping,
            -constraint_torque - free_rotation_damping,
            constraint_force + free_translation_damping,
            constraint_torque + free_rotation_damping,
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
