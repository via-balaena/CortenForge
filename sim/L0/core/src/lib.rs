//! Core physics simulation engine.
//!
//! This crate provides the MuJoCo-aligned Model/Data architecture for physics-based
//! simulations, including collision detection (GJK/EPA, BVH, height fields, SDF),
//! contact resolution (PGS solver), and forward/inverse dynamics. It follows
//! Todorov's design where:
//!
//! - [`Model`] is static (immutable after loading)
//! - [`Data`] is dynamic (qpos/qvel are the source of truth)
//! - Body poses are computed via forward kinematics
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                         Model                               │
//! │  Static: kinematic tree, joint definitions, geometries      │
//! └─────────────────────────┬───────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │                          Data                               │
//! │  Dynamic: qpos, qvel → FK → xpos, xquat                     │
//! │  One step: forward() then integrate()                       │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Layer 0 Crate
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless training loops
//! - Hardware control code
//! - Analysis tools
//! - Other engines
//!
//! # Quick Start
//!
//! ```ignore
//! use sim_core::{Model, Data};
//! use sim_mjcf::load_model;
//!
//! // Load a model from MJCF
//! let model = load_model(MJCF_XML).expect("Failed to load model");
//!
//! // Create simulation data
//! let mut data = model.make_data();
//!
//! // Step the simulation
//! for _ in 0..1000 {
//!     data.step(&model)?;
//! }
//!
//! // Access body poses (computed from qpos via FK)
//! for i in 0..model.nbody {
//!     println!("Body {} position: {:?}", i, data.xpos[i]);
//! }
//! ```

#![doc(html_root_url = "https://docs.rs/sim-core/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]
#![allow(
    clippy::missing_const_for_fn,       // Many methods can't be const due to nalgebra
    clippy::suboptimal_flops,           // mul_add style changes aren't always clearer
    clippy::neg_cmp_op_on_partial_ord,  // !(x >= 0.0) is intentional for NaN rejection
    clippy::option_if_let_else,         // if-let is often more readable than map_or_else
    clippy::too_many_lines,             // Physics functions naturally have many steps
    clippy::doc_markdown,               // Not all technical terms need backticks
)]

// Collision shape primitives (canonical source)
pub mod collision_shape;

// Core simulation algorithms
pub mod gjk_epa;
pub mod heightfield;
pub mod mesh;
pub mod mid_phase;
pub mod mujoco_pipeline;
pub mod raycast;
pub mod sdf;

// Simulation transition derivatives (FD and analytical)
pub mod derivatives;

// Contact geometry types (moved from sim-contact)
pub mod contact;

// Batched simulation (N independent environments sharing one Model)
pub mod batch;

pub use batch::BatchSim;
pub use collision_shape::{Aabb, Axis, CollisionShape};

pub use contact::{ContactForce, ContactManifold, ContactPoint};
pub use heightfield::{HeightFieldContact, HeightFieldData};
pub use mesh::{
    MeshContact, Triangle, TriangleMeshData, closest_point_on_triangle, mesh_box_contact,
    mesh_capsule_contact, mesh_mesh_contact, mesh_mesh_deepest_contact, mesh_sphere_contact,
    triangle_box_contact, triangle_capsule_contact, triangle_sphere_contact,
};
pub use mid_phase::{Bvh, BvhPrimitive, bvh_from_triangle_mesh};
pub use raycast::{RaycastHit, raycast_shape};
pub use sdf::{
    SdfCollisionData, SdfContact, sdf_box_contact, sdf_capsule_contact, sdf_point_contact,
    sdf_sphere_contact,
};

// MuJoCo-style physics pipeline types (primary API)
pub use mujoco_pipeline::{
    // Actuator types
    ActuatorDynamics,
    ActuatorTransmission,
    BiasType,
    // Constraint types and state
    ConstraintState,
    ConstraintType,
    // Contact representation
    Contact,
    ContactPair,
    // MuJoCo-aligned Model/Data architecture (core API)
    Data,
    // Equality constraint types
    EqualityType,
    GainType,
    // Geometry types
    GeomType,
    // Integration method selection
    Integrator,
    // Keyframe types
    Keyframe,
    // Joint types
    MjJointType,
    // Sensor types
    MjObjectType,
    MjSensorDataType,
    MjSensorType,
    Model,
    // Error handling
    ResetError,
    // Constraint solver algorithm
    SolverStat,
    SolverType,
    // Spatial algebra types
    SpatialVector,
    StepError,
    // Tendon types
    TendonType,
    // Warmstart key type for efc_lambda contact cache
    WarmstartKey,
    // Wrap object types
    WrapType,
    // Utility functions for position/velocity differentiation
    mj_differentiate_pos,
    mj_integrate_pos_explicit,
    warmstart_key,
};

pub use derivatives::{
    DerivativeConfig, TransitionMatrices, fd_convergence_check, max_relative_error,
    mjd_quat_integrate, mjd_smooth_vel, mjd_transition, mjd_transition_fd, mjd_transition_hybrid,
    validate_analytical_vs_fd,
};

// Re-export key types from sim-types for convenience
pub use sim_types::{
    Action, ActionType, BodyId, ExternalForce, Gravity, JointCommand, JointCommandType, JointId,
    JointLimits, JointState, JointType, MassProperties, Observation, ObservationType, Pose,
    RigidBodyState, SimError, SimulationConfig, SolverConfig, Twist,
};

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::unreadable_literal,
    clippy::uninlined_format_args,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_precision_loss,
    clippy::cast_lossless,
    clippy::manual_range_contains,
    clippy::map_unwrap_or
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_model_data_basic() {
        // Test basic Model/Data API using n-link pendulum
        let model = Model::n_link_pendulum(1, 1.0, 0.1);
        let mut data = model.make_data();

        // Initial position should be zero (equilibrium)
        assert_relative_eq!(data.qpos[0], 0.0, epsilon = 1e-6);

        // Start at 45 degrees (not equilibrium) so it will swing
        data.qpos[0] = std::f64::consts::FRAC_PI_4;
        data.forward(&model).expect("forward failed");

        // Step for 0.5 seconds
        for _ in 0..500 {
            data.step(&model).expect("step failed");
        }

        // Pendulum should have swung past equilibrium (negative angle now)
        // Since we started at +45°, it should have swung down and past 0
        assert!(data.qpos[0].abs() > 1e-3, "Pendulum should swing");
    }

    #[test]
    fn test_model_data_energy() {
        // Test energy conservation in zero gravity using n-link pendulum
        // Create a pendulum but step without gravity (check kinetic energy conservation)
        let mut model = Model::n_link_pendulum(1, 1.0, 0.1);
        model.gravity = nalgebra::Vector3::zeros();
        let mut data = model.make_data();

        // Give it initial velocity
        data.qvel[0] = 1.0;
        data.forward(&model).expect("forward failed");

        let initial_ke = data.energy_kinetic;

        // Step for a while
        for _ in 0..1000 {
            data.step(&model).expect("step failed");
        }

        let final_ke = data.energy_kinetic;

        // Energy should be conserved (within 1%)
        let drift = (final_ke - initial_ke).abs() / initial_ke;
        assert!(drift < 0.01, "Energy drift too large: {}%", drift * 100.0);
    }
}
