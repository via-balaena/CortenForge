//! Core types for physics simulation.
//!
//! This crate provides the foundational types for building physics simulations:
//!
//! - [`RigidBodyState`] - Position, orientation, velocity of rigid bodies
//! - [`JointState`] - Position, velocity of articulated joints
//! - [`Action`] - Control inputs (forces, torques, target positions)
//! - [`Observation`] - Sensor observations from simulation
//! - [`SimulationConfig`] - Timestep, solver settings
//!
//! # Design Philosophy
//!
//! These types are **pure data**. They have no behavior, no physics, no integration.
//! They're the common language between:
//!
//! - Physics engines (sim-core, Avian, external)
//! - Control policies (ML models, classical controllers)
//! - Sensor simulation (generating observations from state)
//! - Logging and replay (serialized state trajectories)
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops
//! - Hardware control code
//! - Analysis tools
//! - Other engines
//!
//! # Coordinate System
//!
//! Consistent with the CortenForge ecosystem:
//!
//! - X: right
//! - Y: forward
//! - Z: up
//! - Right-handed
//!
//! # Example
//!
//! ```
//! use sim_types::{RigidBodyState, Pose, Twist};
//! use nalgebra::{Point3, UnitQuaternion, Vector3};
//!
//! // Create a body at rest at the origin
//! let state = RigidBodyState::new(
//!     Pose::from_position(Point3::new(0.0, 0.0, 1.0)),
//!     Twist::zero(),
//! );
//!
//! assert_eq!(state.pose.position.z, 1.0);
//! assert!(state.twist.linear.norm() < 1e-10);
//! ```

#![doc(html_root_url = "https://docs.rs/sim-types/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
// Allow certain clippy lints that are overly pedantic for type definitions
#![allow(
    clippy::missing_const_for_fn,     // Many methods can't be const due to nalgebra
    clippy::suboptimal_flops,          // mul_add style changes aren't always clearer
    clippy::cast_precision_loss,       // usize to f64 is fine for counts
    clippy::missing_errors_doc,        // Error docs added where non-obvious
)]

mod body;
mod config;
mod dynamics;
mod error;
mod joint;
mod observation;

pub use body::{BodyId, MassProperties, Pose, RigidBodyState, Twist};
pub use config::{IntegrationMethod, SimulationConfig, SolverConfig};
pub use dynamics::{Action, ActionType, ExternalForce, Gravity, JointCommand, JointCommandType};
pub use error::SimError;
pub use joint::{JointAxis, JointId, JointLimits, JointState, JointStateExtended, JointType};
pub use observation::{
    ContactInfo, ContactStats, Observation, ObservationType, PoseObservation, SensorObservation,
    VelocityObservation,
};

// Re-export math types for convenience
pub use glam::{Quat, Vec3};
pub use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

/// Result type for simulation operations.
pub type Result<T> = std::result::Result<T, SimError>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn test_rigid_body_state() {
        let pose = Pose::from_position(Point3::new(1.0, 2.0, 3.0));
        let twist = Twist::new(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros());
        let state = RigidBodyState::new(pose, twist);

        assert_eq!(state.pose.position.x, 1.0);
        assert_eq!(state.twist.linear.x, 1.0);
    }

    #[test]
    fn test_pose_transform() {
        let pose = Pose::from_position_rotation(
            Point3::new(1.0, 0.0, 0.0),
            UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );

        // Transform a point
        let local = Point3::new(1.0, 0.0, 0.0);
        let world = pose.transform_point(&local);

        // After 90 degree rotation around Z, (1,0,0) in local becomes (0,1,0) in rotated frame
        // Plus translation of (1,0,0)
        assert!((world.x - 1.0).abs() < 1e-10);
        assert!((world.y - 1.0).abs() < 1e-10);
    }
}
