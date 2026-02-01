//! Tendon and cable simulation for cable-driven robots and biomechanics.
//!
//! This crate provides comprehensive tendon/cable modeling for:
//!
//! - **Cable-driven robots**: Tendon-driven manipulators, exoskeletons
//! - **Biomechanical simulation**: Muscle-tendon units, ligaments
//! - **Pulley systems**: Cable routing, mechanical advantage
//! - **Soft robotics**: Continuum robots, compliant actuators
//!
//! # Tendon Types
//!
//! ## Fixed Tendons (MuJoCo-style)
//!
//! Fixed tendons couple multiple joints through linear relationships:
//!
//! ```text
//! L = L₀ + Σᵢ cᵢ qᵢ
//! ```
//!
//! Where `cᵢ` are coupling coefficients and `qᵢ` are joint positions.
//! This is useful for:
//! - Differential drives
//! - Synergistic hand control
//! - Underactuated mechanisms
//!
//! ## Spatial Tendons
//!
//! Spatial tendons route through 3D space via attachment points:
//!
//! ```text
//!     Body A          Body B          Body C
//!       ●───────────────●───────────────●
//!    origin         via point       insertion
//! ```
//!
//! The total length is the sum of segment lengths between attachment points,
//! computed in world coordinates from body transforms.
//!
//! ## Wrapping Geometry
//!
//! Tendons can wrap around geometric obstacles:
//!
//! - **Sphere wrapping**: For joint articulation (e.g., knee, shoulder)
//! - **Cylinder wrapping**: For pulleys and cable guides
//!
//! # Pulley Systems
//!
//! Pulley systems model mechanical advantage and cable routing:
//!
//! ```text
//!           Fixed
//!             ●
//!            /|\
//!           / | \
//!      r   /  |  \  r
//!         ●   |   ●
//!        Pulley   Pulley
//!           \ | /
//!            \|/
//!             ●
//!           Load
//! ```
//!
//! # Cable Properties
//!
//! Cables have:
//! - **Stiffness**: Spring constant when stretched
//! - **Damping**: Viscous damping for stability
//! - **Rest length**: Unstretched length
//! - **Tension limits**: Maximum force before failure
//!
//! # Quick Start
//!
//! ```
//! use sim_tendon::{FixedTendon, TendonCoefficient, CableProperties, TendonActuator};
//! use sim_types::JointId;
//!
//! // Create a differential tendon coupling two joints
//! let tendon = FixedTendon::new("differential")
//!     .with_coefficient(JointId::new(0), 1.0)   // +1 coupling
//!     .with_coefficient(JointId::new(1), -1.0)  // -1 coupling
//!     .with_rest_length(0.5)
//!     .with_cable(CableProperties::steel_cable(0.002)); // 2mm steel cable
//!
//! // Compute tendon force from joint positions
//! let joint_positions = &[0.1, 0.2];
//! let joint_velocities = &[0.0, 0.0];
//! let force = tendon.compute_force(joint_positions, joint_velocities);
//! ```
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops for reinforcement learning
//! - Hardware control systems
//! - Analysis and optimization tools
//! - Integration with other physics engines

#![doc(html_root_url = "https://docs.rs/sim-tendon/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,
    clippy::module_name_repetitions,
    clippy::doc_markdown,
    clippy::must_use_candidate,
    clippy::missing_errors_doc,
    clippy::missing_panics_doc,
    clippy::similar_names,
    clippy::too_many_lines,
    clippy::suboptimal_flops,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::let_and_return,
    clippy::derivable_impls,
    clippy::imprecise_flops,
    clippy::while_float,
    clippy::or_fun_call
)]
#![cfg_attr(test, allow(clippy::float_cmp, clippy::let_underscore_must_use))]

pub mod cable;
pub mod error;
pub mod fixed;
pub mod path;
pub mod pulley;
pub mod spatial;
pub mod wrapping;

// Re-export main types at crate root
pub use cable::{CableProperties, CableState, TensionResult};
pub use error::TendonError;
pub use fixed::{FixedTendon, TendonCoefficient};
pub use path::{AttachmentPoint, TendonPath, TendonSegment};
pub use pulley::{Pulley, PulleyConfig, PulleySystem};
pub use spatial::{SpatialTendon, SpatialTendonConfig};
pub use wrapping::{CylinderWrap, SphereWrap, WrapResult, WrappingGeometry};

/// Trait for tendon actuators that can compute force from length/velocity.
///
/// This trait provides a common interface for different tendon types,
/// allowing them to be used interchangeably with the constraint system.
pub trait TendonActuator {
    /// Get the tendon's rest length (unstretched).
    fn rest_length(&self) -> f64;

    /// Compute the current tendon length.
    ///
    /// # Arguments
    ///
    /// * `joint_positions` - Current joint positions (radians or meters)
    ///
    /// # Returns
    ///
    /// Current tendon length in meters.
    fn compute_length(&self, joint_positions: &[f64]) -> f64;

    /// Compute the current tendon velocity.
    ///
    /// # Arguments
    ///
    /// * `joint_positions` - Current joint positions
    /// * `joint_velocities` - Current joint velocities
    ///
    /// # Returns
    ///
    /// Current tendon velocity in m/s (positive = lengthening).
    fn compute_velocity(&self, joint_positions: &[f64], joint_velocities: &[f64]) -> f64;

    /// Compute the tendon force given current state.
    ///
    /// # Arguments
    ///
    /// * `joint_positions` - Current joint positions
    /// * `joint_velocities` - Current joint velocities
    ///
    /// # Returns
    ///
    /// Tendon tension in Newtons (always non-negative for cables).
    fn compute_force(&self, joint_positions: &[f64], joint_velocities: &[f64]) -> f64;

    /// Compute the Jacobian (force transmission ratios) to joints.
    ///
    /// The Jacobian maps tendon force to joint torques/forces:
    /// τᵢ = Jᵢ × F_tendon
    ///
    /// # Returns
    ///
    /// Vector of Jacobian values (one per coupled joint).
    fn jacobian(&self, joint_positions: &[f64]) -> Vec<f64>;

    /// Get the number of joints this tendon couples.
    fn num_joints(&self) -> usize;
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_fixed_tendon_basic() {
        use sim_types::JointId;

        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)  // 5cm moment arm
            .with_rest_length(0.3);

        // At zero angle, length equals rest length
        let length = tendon.compute_length(&[0.0]);
        assert_relative_eq!(length, 0.3, epsilon = 1e-10);

        // At 1 radian, length increases by 0.05m
        let length = tendon.compute_length(&[1.0]);
        assert_relative_eq!(length, 0.35, epsilon = 1e-10);
    }

    #[test]
    fn test_cable_properties() {
        let cable = CableProperties::default();
        assert!(cable.stiffness > 0.0);
        assert!(cable.damping >= 0.0);
    }

    #[test]
    fn test_tendon_jacobian() {
        use sim_types::JointId;

        let tendon = FixedTendon::new("test")
            .with_coefficient(JointId::new(0), 0.05)
            .with_coefficient(JointId::new(1), -0.03);

        let jacobian = tendon.jacobian(&[0.0, 0.0]);
        assert_eq!(jacobian.len(), 2);
        assert_relative_eq!(jacobian[0], 0.05, epsilon = 1e-10);
        assert_relative_eq!(jacobian[1], -0.03, epsilon = 1e-10);
    }
}
