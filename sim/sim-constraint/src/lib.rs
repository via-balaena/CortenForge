//! Joint constraints and motors for articulated body simulation.
//!
//! This crate provides constraint-based joints for connecting rigid bodies
//! into articulated structures like robot arms, legged robots, and mechanisms.
//!
//! # Joint Types
//!
//! - [`RevoluteJoint`]: Single-axis rotation (hinges, elbows)
//! - [`PrismaticJoint`]: Single-axis translation (linear actuators)
//! - [`FixedJoint`]: Rigid connection (welded parts)
//! - [`SphericalJoint`] - Ball-and-socket (3 rotational DOF)
//! - [`UniversalJoint`]: Two perpendicular rotation axes
//!
//! # Joint Features
//!
//! Each joint can have:
//! - **Limits**: Position bounds with soft/hard enforcement
//! - **Motors**: Position or velocity control with torque limits
//! - **Damping**: Viscous friction at the joint
//! - **Spring**: Passive stiffness toward a rest position
//!
//! # Constraint Solvers
//!
//! Two solvers are available:
//!
//! - [`ConstraintSolver`]: Gauss-Seidel iterative solver (8-16 iterations typical)
//! - [`NewtonConstraintSolver`]: Newton-Raphson solver with analytical Jacobians
//!   (2-3 iterations typical, faster convergence for stiff systems)
//!
//! # Constraint Islands
//!
//! For performance optimization, use [`ConstraintIslands`] to automatically detect
//! independent groups of bodies that can be solved separately:
//!
//! ```
//! use sim_constraint::{ConstraintIslands, NewtonConstraintSolver, RevoluteJoint};
//! use sim_types::BodyId;
//! use nalgebra::Vector3;
//!
//! let joints = vec![
//!     RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z()),
//!     // Disconnected pair forms separate island
//!     RevoluteJoint::new(BodyId::new(2), BodyId::new(3), Vector3::z()),
//! ];
//!
//! // Automatic island detection and solving
//! let mut solver = NewtonConstraintSolver::default();
//! // solver.solve_with_islands(&joints, get_body_state, dt);
//! ```
//!
//! # Constraint Formulation
//!
//! Joints are formulated as bilateral constraints:
//!
//! ```text
//! C(q) = 0        (position constraint)
//! J * v = 0       (velocity constraint, J = dC/dq)
//! ```
//!
//! The solver computes constraint forces (Lagrange multipliers) to satisfy
//! these constraints while respecting joint limits and motor commands.
//!
//! # Example
//!
//! ```
//! use sim_constraint::{RevoluteJoint, JointLimits, JointMotor, MotorMode};
//! use sim_types::BodyId;
//! use nalgebra::Vector3;
//!
//! // Create a revolute joint (hinge) between two bodies
//! let joint = RevoluteJoint::new(
//!     BodyId::new(1),  // Parent body
//!     BodyId::new(2),  // Child body
//!     Vector3::z(),    // Rotation axis (Z)
//! )
//! .with_limits(JointLimits::symmetric(std::f64::consts::FRAC_PI_2))
//! .with_motor(JointMotor::velocity(1.0, 10.0));  // 1 rad/s, max 10 Nm
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

#![doc(html_root_url = "https://docs.rs/sim-constraint/0.1.0")]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![warn(missing_docs)]
#![allow(clippy::missing_const_for_fn)]

mod islands;
mod joint;
mod limits;
mod motor;
#[cfg(feature = "muscle")]
mod muscle;
mod newton;
mod solver;
mod types;

pub use islands::{ConstraintIslands, Island, IslandStatistics};
pub use joint::{
    FixedJoint, Joint, JointDof, JointType, PrismaticJoint, RevoluteJoint, SphericalJoint,
    UniversalJoint,
};
pub use limits::{JointLimits, LimitState, LimitStiffness};
pub use motor::{JointMotor, MotorMode};
#[cfg(feature = "muscle")]
pub use muscle::{MuscleCommands, MuscleJoint, MuscleJointBuilder};
pub use newton::{NewtonConstraintSolver, NewtonSolverConfig, NewtonSolverResult};
pub use solver::{BodyState, ConstraintSolver, ConstraintSolverConfig, JointForce, SolverResult};
pub use types::{ConstraintForce, JointState, JointVelocity};

// Re-export types needed for constraint computation
pub use sim_types::{BodyId, Pose, Vector3};

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_revolute_joint_creation() {
        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z());

        assert_eq!(joint.parent(), BodyId::new(0));
        assert_eq!(joint.child(), BodyId::new(1));
        assert_eq!(joint.dof(), 1);
    }

    #[test]
    fn test_joint_limits() {
        let limits = JointLimits::new(-1.0, 1.0);
        assert!(limits.contains(0.5));
        assert!(!limits.contains(1.5));

        let symmetric = JointLimits::symmetric(2.0);
        assert_relative_eq!(symmetric.lower(), -2.0, epsilon = 1e-10);
        assert_relative_eq!(symmetric.upper(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_motor_modes() {
        let vel_motor = JointMotor::velocity(1.0, 10.0);
        assert!(matches!(vel_motor.mode(), MotorMode::Velocity { .. }));

        let pos_motor = JointMotor::position(0.5, 100.0, 10.0);
        assert!(matches!(pos_motor.mode(), MotorMode::Position { .. }));
    }

    #[test]
    fn test_joint_with_limits_and_motor() {
        let joint = RevoluteJoint::new(BodyId::new(0), BodyId::new(1), Vector3::z())
            .with_limits(JointLimits::symmetric(std::f64::consts::FRAC_PI_2))
            .with_motor(JointMotor::velocity(1.0, 10.0))
            .with_damping(0.1);

        assert!(joint.limits().is_some());
        assert!(joint.motor().is_some());
        assert_relative_eq!(joint.damping(), 0.1, epsilon = 1e-10);
    }
}
