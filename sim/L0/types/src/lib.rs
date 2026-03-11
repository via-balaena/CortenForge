//! Foundation types for the sim-core simulation pipeline.
//!
//! This crate provides the foundational types shared across simulation crates:
//!
//! - [`BodyId`] - Unique identifier for rigid bodies
//! - [`Pose`] - Position and orientation in 3D space
//! - [`SimulationConfig`] - Timestep, solver, and gravity settings
//! - [`Gravity`] - Gravity configuration
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
//! use sim_types::{Pose, SimulationConfig, Gravity};
//! use nalgebra::Point3;
//!
//! let pose = Pose::from_position(Point3::new(0.0, 0.0, 1.0));
//! assert_eq!(pose.position.z, 1.0);
//!
//! let config = SimulationConfig::with_timestep(0.001).gravity(Gravity::moon());
//! assert!(config.validate().is_ok());
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
mod error;

pub use body::{BodyId, Pose};
pub use config::{Gravity, ParallelConfig, SimulationConfig, SolverConfig};
pub use error::SimError;

// Re-export math types for convenience
pub use nalgebra::{Isometry3, Point3, UnitQuaternion, Vector3};

/// Result type for simulation operations.
pub type Result<T> = std::result::Result<T, SimError>;

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

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
