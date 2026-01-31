//! Common types for constraint computation.

use nalgebra::{Matrix3, Point3, Vector3};
use sim_types::BodyId;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// State of a joint (generalized coordinates).
///
/// For a 1-DOF joint (revolute/prismatic), only `q[0]` is used.
/// For multi-DOF joints, more elements are used.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointState {
    /// Generalized position coordinates.
    /// - Revolute: angle in radians
    /// - Prismatic: displacement in meters
    /// - Spherical: quaternion (stored as 4 values, but 3 DOF)
    pub position: [f64; 4],

    /// Number of active DOF.
    pub dof: usize,
}

impl JointState {
    /// Create a zero state with given DOF.
    #[must_use]
    pub fn zero(dof: usize) -> Self {
        Self {
            position: [0.0; 4],
            dof,
        }
    }

    /// Create a 1-DOF state.
    #[must_use]
    pub fn single(q: f64) -> Self {
        Self {
            position: [q, 0.0, 0.0, 0.0],
            dof: 1,
        }
    }

    /// Create a 2-DOF state.
    #[must_use]
    pub fn double(q1: f64, q2: f64) -> Self {
        Self {
            position: [q1, q2, 0.0, 0.0],
            dof: 2,
        }
    }

    /// Create a 3-DOF state.
    #[must_use]
    pub fn triple(q1: f64, q2: f64, q3: f64) -> Self {
        Self {
            position: [q1, q2, q3, 0.0],
            dof: 3,
        }
    }

    /// Get the first (or only) coordinate.
    #[must_use]
    pub fn q(&self) -> f64 {
        self.position[0]
    }

    /// Get coordinate by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<f64> {
        if index < self.dof {
            Some(self.position[index])
        } else {
            None
        }
    }

    /// Set coordinate by index.
    pub fn set(&mut self, index: usize, value: f64) {
        if index < self.dof {
            self.position[index] = value;
        }
    }

    /// Clamp state to limits.
    #[must_use]
    pub fn clamp(&self, lower: &[f64], upper: &[f64]) -> Self {
        let mut result = *self;
        for i in 0..self.dof.min(lower.len()).min(upper.len()) {
            result.position[i] = self.position[i].clamp(lower[i], upper[i]);
        }
        result
    }
}

impl Default for JointState {
    fn default() -> Self {
        Self::zero(1)
    }
}

/// Velocity of a joint (generalized velocities).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct JointVelocity {
    /// Generalized velocity coordinates.
    /// - Revolute: angular velocity in rad/s
    /// - Prismatic: linear velocity in m/s
    pub velocity: [f64; 4],

    /// Number of active DOF.
    pub dof: usize,
}

impl JointVelocity {
    /// Create a zero velocity with given DOF.
    #[must_use]
    pub fn zero(dof: usize) -> Self {
        Self {
            velocity: [0.0; 4],
            dof,
        }
    }

    /// Create a 1-DOF velocity.
    #[must_use]
    pub fn single(v: f64) -> Self {
        Self {
            velocity: [v, 0.0, 0.0, 0.0],
            dof: 1,
        }
    }

    /// Create a 2-DOF velocity.
    #[must_use]
    pub fn double(v1: f64, v2: f64) -> Self {
        Self {
            velocity: [v1, v2, 0.0, 0.0],
            dof: 2,
        }
    }

    /// Get the first (or only) velocity.
    #[must_use]
    pub fn v(&self) -> f64 {
        self.velocity[0]
    }

    /// Get velocity by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<f64> {
        if index < self.dof {
            Some(self.velocity[index])
        } else {
            None
        }
    }

    /// Set velocity by index.
    pub fn set(&mut self, index: usize, value: f64) {
        if index < self.dof {
            self.velocity[index] = value;
        }
    }

    /// Clamp velocity to limits.
    #[must_use]
    pub fn clamp(&self, max_velocity: &[f64]) -> Self {
        let mut result = *self;
        let len = self.dof.min(max_velocity.len());
        for (i, &max_vel) in max_velocity.iter().enumerate().take(len) {
            let limit = max_vel.abs();
            result.velocity[i] = self.velocity[i].clamp(-limit, limit);
        }
        result
    }
}

impl Default for JointVelocity {
    fn default() -> Self {
        Self::zero(1)
    }
}

/// Force/torque applied by a constraint.
///
/// This represents the Lagrange multiplier times the constraint Jacobian,
/// giving the actual force/torque vectors to apply to each body.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConstraintForce {
    /// Force on parent body (at constraint anchor).
    pub parent_force: Vector3<f64>,

    /// Torque on parent body.
    pub parent_torque: Vector3<f64>,

    /// Force on child body (at constraint anchor).
    pub child_force: Vector3<f64>,

    /// Torque on child body.
    pub child_torque: Vector3<f64>,
}

impl ConstraintForce {
    /// Create a zero constraint force.
    #[must_use]
    pub fn zero() -> Self {
        Self::default()
    }

    /// Create from parent and child wrenches.
    #[must_use]
    pub fn new(
        parent_force: Vector3<f64>,
        parent_torque: Vector3<f64>,
        child_force: Vector3<f64>,
        child_torque: Vector3<f64>,
    ) -> Self {
        Self {
            parent_force,
            parent_torque,
            child_force,
            child_torque,
        }
    }

    /// Total force magnitude (for diagnostics).
    #[must_use]
    pub fn total_force_magnitude(&self) -> f64 {
        self.parent_force.norm() + self.child_force.norm()
    }

    /// Total torque magnitude (for diagnostics).
    #[must_use]
    pub fn total_torque_magnitude(&self) -> f64 {
        self.parent_torque.norm() + self.child_torque.norm()
    }

    /// Scale all forces/torques.
    #[must_use]
    pub fn scale(&self, factor: f64) -> Self {
        Self {
            parent_force: self.parent_force * factor,
            parent_torque: self.parent_torque * factor,
            child_force: self.child_force * factor,
            child_torque: self.child_torque * factor,
        }
    }

    /// Add another constraint force.
    #[must_use]
    pub fn add(&self, other: &Self) -> Self {
        Self {
            parent_force: self.parent_force + other.parent_force,
            parent_torque: self.parent_torque + other.parent_torque,
            child_force: self.child_force + other.child_force,
            child_torque: self.child_torque + other.child_torque,
        }
    }
}

/// Body state needed for constraint solving.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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

/// Force result for a single joint.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    use approx::assert_relative_eq;

    #[test]
    fn test_joint_state() {
        let state = JointState::single(1.5);
        assert_eq!(state.dof, 1);
        assert_relative_eq!(state.q(), 1.5, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_state_clamp() {
        let state = JointState::single(2.0);
        let clamped = state.clamp(&[-1.0], &[1.0]);
        assert_relative_eq!(clamped.q(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_velocity() {
        let vel = JointVelocity::double(1.0, -2.0);
        assert_eq!(vel.dof, 2);
        assert_relative_eq!(vel.v(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(vel.get(1).unwrap_or(0.0), -2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_constraint_force() {
        let f1 = ConstraintForce::new(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::new(-1.0, 0.0, 0.0),
            Vector3::zeros(),
        );

        let f2 = f1.scale(2.0);
        assert_relative_eq!(f2.parent_force.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(f2.child_force.x, -2.0, epsilon = 1e-10);
    }
}
