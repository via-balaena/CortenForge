//! Unified physics simulation API for CortenForge.
//!
//! This crate re-exports the complete physics simulation stack:
//!
//! - [`sim_types`] - Core data types (bodies, joints, poses, actions)
//! - [`sim_core`] - Simulation engine (world, stepper, integrators)
//! - [`sim_constraint`] - Joint constraints (revolute, prismatic, fixed, etc.)
//! - [`sim_contact`] - Contact dynamics (collision response, friction)
//! - [`sim_urdf`] - URDF robot description parser
//!
//! # Layer 0
//!
//! This is a Layer 0 crate with **zero Bevy dependencies**. It can be used in:
//!
//! - Headless RL training environments
//! - Hardware robot control systems
//! - Analysis and planning tools
//! - Integration with other game engines
//!
//! # Quick Start
//!
//! ```
//! use sim_physics::prelude::*;
//!
//! // Create a world with default config (Earth gravity, 240Hz)
//! let mut world = World::default();
//!
//! // Add a falling sphere
//! let sphere_id = world.add_body(
//!     RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 10.0))),
//!     MassProperties::sphere(1.0, 0.5),
//! );
//!
//! // Create a stepper and simulate
//! let mut stepper = Stepper::new();
//! stepper.run_for(&mut world, 1.0).unwrap();
//!
//! // Check final state
//! let body = world.body(sphere_id).unwrap();
//! println!("Final height: {:.2} m", body.state.pose.position.z);
//! ```
//!
//! # Loading URDF Robots
//!
//! ```
//! use sim_physics::prelude::*;
//!
//! let urdf = r#"
//!     <robot name="simple_arm">
//!         <link name="base">
//!             <inertial><mass value="1.0"/><inertia ixx="0.1" iyy="0.1" izz="0.1"/></inertial>
//!         </link>
//!         <link name="arm">
//!             <inertial><mass value="0.5"/><inertia ixx="0.05" iyy="0.05" izz="0.05"/></inertial>
//!         </link>
//!         <joint name="shoulder" type="revolute">
//!             <parent link="base"/><child link="arm"/>
//!             <axis xyz="0 1 0"/>
//!             <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
//!         </joint>
//!     </robot>
//! "#;
//!
//! let robot = load_urdf_str(urdf).unwrap();
//! let mut world = World::default();
//! let spawned = robot.spawn_at_origin(&mut world).unwrap();
//!
//! println!("Loaded robot with {} bodies", world.body_count());
//! ```
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                      sim-physics (this crate)                   │
//! │                     Unified API / re-exports                    │
//! └─────────────────────────────────────────────────────────────────┘
//!                                  │
//!          ┌───────────────────────┼───────────────────────┐
//!          │                       │                       │
//!          ▼                       ▼                       ▼
//! ┌─────────────────┐   ┌─────────────────┐   ┌─────────────────┐
//! │    sim-urdf     │   │  sim-constraint │   │   sim-contact   │
//! │  URDF parsing   │   │ Joint dynamics  │   │ Contact forces  │
//! └────────┬────────┘   └────────┬────────┘   └────────┬────────┘
//!          │                     │                     │
//!          └──────────────┬──────┴─────────────────────┘
//!                         ▼
//!               ┌─────────────────┐
//!               │    sim-core     │
//!               │ World, Stepper  │
//!               └────────┬────────┘
//!                        │
//!                        ▼
//!               ┌─────────────────┐
//!               │   sim-types     │
//!               │  Data structs   │
//!               └─────────────────┘
//! ```

#![doc(html_root_url = "https://docs.rs/sim-physics/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]

// Re-export sub-crates
pub use sim_constraint;
pub use sim_contact;
pub use sim_core;
pub use sim_types;
pub use sim_urdf;

// Re-export nalgebra for convenience
pub use nalgebra;

/// Prelude module for convenient imports.
///
/// Import everything you need with a single line:
///
/// ```
/// use sim_physics::prelude::*;
/// ```
pub mod prelude {
    // ========================================================================
    // Core types from sim-types
    // ========================================================================

    // Bodies and motion
    pub use sim_types::{BodyId, MassProperties, Pose, RigidBodyState, Twist};

    // Joints
    pub use sim_types::{JointId, JointLimits, JointState, JointType};

    // Actions and observations
    pub use sim_types::{
        Action, ActionType, ExternalForce, JointCommand, JointCommandType, Observation,
        ObservationType,
    };

    // Configuration
    pub use sim_types::{Gravity, IntegrationMethod, SimulationConfig, SolverConfig};

    // Errors
    pub use sim_types::SimError;

    // ========================================================================
    // Simulation engine from sim-core
    // ========================================================================

    pub use sim_core::{
        Body, CollisionShape, Joint, SimulationBuilder, StepResult, Stepper, StepperConfig, World,
    };

    // Integrators
    pub use sim_core::integrators::{
        ExplicitEuler, Integrator, RungeKutta4, SemiImplicitEuler, VelocityVerlet,
        integrate_with_method,
    };

    // ========================================================================
    // Joint constraints from sim-constraint
    // ========================================================================

    pub use sim_constraint::{
        // Solver
        ConstraintSolver,
        ConstraintSolverConfig,
        FixedJoint,
        // Motor
        JointMotor,
        // Limits
        LimitState,
        LimitStiffness,
        MotorMode,
        PrismaticJoint,
        // Joint types
        RevoluteJoint,
        SphericalJoint,
        UniversalJoint,
    };

    // ========================================================================
    // Contact dynamics from sim-contact
    // ========================================================================

    pub use sim_contact::{
        ContactForce,
        // Contact model
        ContactModel,
        ContactParams,
        ContactPoint,
        // Domain randomization
        DomainRandomization,
        // Friction
        FrictionModel,
        // Materials
        SurfaceMaterial,
    };

    // ========================================================================
    // URDF loading from sim-urdf
    // ========================================================================

    pub use sim_urdf::{
        LoadedRobot,
        SpawnedRobot,
        // Errors
        UrdfError,
        UrdfGeometry,
        UrdfInertial,
        UrdfJoint,
        UrdfJointType,
        UrdfLink,
        UrdfLoader,
        UrdfOrigin,
        // IR types (for inspection/modification)
        UrdfRobot,
        ValidationResult,
        // Loader
        load_urdf_file,
        load_urdf_str,
        // Parser (for advanced use)
        parse_urdf_str,
        // Validation
        validate,
    };

    // ========================================================================
    // Math types from nalgebra
    // ========================================================================

    pub use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::prelude::*;

    #[test]
    fn test_prelude_imports() {
        // Verify core types are accessible
        let _pose = Pose::identity();
        let _mass = MassProperties::sphere(1.0, 0.5);
        let _config = SimulationConfig::default();
    }

    #[test]
    fn test_basic_simulation() {
        let mut world = World::default();

        let body_id = world.add_body(
            RigidBodyState::at_rest(Pose::from_position(Point3::new(0.0, 0.0, 5.0))),
            MassProperties::sphere(1.0, 0.5),
        );

        let mut stepper = Stepper::new();
        stepper
            .run_for(&mut world, 0.1)
            .expect("simulation should run");

        let body = world.body(body_id).expect("body should exist");
        // Body should have fallen (z < 5.0)
        assert!(body.state.pose.position.z < 5.0);
    }

    #[test]
    fn test_urdf_loading() {
        let urdf = r#"
            <robot name="test">
                <link name="base">
                    <inertial>
                        <mass value="1.0"/>
                        <inertia ixx="0.1" iyy="0.1" izz="0.1"/>
                    </inertial>
                </link>
            </robot>
        "#;

        let robot = load_urdf_str(urdf).expect("should parse");
        assert_eq!(robot.name, "test");

        let mut world = World::default();
        let spawned = robot.spawn_at_origin(&mut world).expect("should spawn");

        assert_eq!(world.body_count(), 1);
        assert!(spawned.link_id("base").is_some());
    }

    #[test]
    fn test_joint_types_accessible() {
        // Verify constraint types are accessible
        let _limits = sim_constraint::JointLimits::new(-1.0, 1.0);
        let _motor = JointMotor::velocity(1.0, 10.0);
    }

    #[test]
    fn test_contact_types_accessible() {
        // Verify contact types are accessible
        let _params = ContactParams::default();
        let _model = ContactModel::new(ContactParams::default());
    }
}
