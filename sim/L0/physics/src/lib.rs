//! Unified physics simulation API for CortenForge.
//!
//! This crate re-exports the complete physics simulation stack:
//!
//! - [`sim_types`] - Core data types (bodies, joints, poses, actions)
//! - [`sim_core`] - Simulation engine (world, stepper, integrators)
//! - [`sim_constraint`] - Joint constraints (revolute, prismatic, fixed, etc.)
//! - [`sim_contact`] - Contact dynamics (collision response, friction)
//! - [`sim_urdf`] - URDF robot description parser
//! - [`sim_mjcf`] - MJCF (`MuJoCo` XML Format) model loader
//! - [`sim_muscle`] - Hill-type muscle actuators for biomechanical simulation
//! - [`sim_tendon`] - Tendon and cable simulation for cable-driven robots
//! - [`sim_sensor`] - Sensor simulation (IMU, force/torque, touch, etc.)
//! - [`sim_deformable`] - Soft body and deformable simulation (behind `deformable` feature)
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
//! # Loading MJCF Models
//!
//! ```
//! use sim_physics::prelude::*;
//!
//! let mjcf = r#"
//!     <mujoco model="pendulum">
//!         <worldbody>
//!             <body name="base" pos="0 0 1">
//!                 <geom type="sphere" size="0.05" mass="1.0"/>
//!                 <body name="arm" pos="0 0 -0.5">
//!                     <joint name="hinge" type="hinge" axis="0 1 0"/>
//!                     <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.4" mass="0.1"/>
//!                 </body>
//!             </body>
//!         </worldbody>
//!     </mujoco>
//! "#;
//!
//! let model = load_mjcf_str(mjcf).unwrap();
//! let mut world = World::default();
//! let spawned = model.spawn_at_origin(&mut world).unwrap();
//!
//! println!("Loaded MJCF model with {} bodies", world.body_count());
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
//!     ┌────────────────────────────┼────────────────────────────┐
//!     │           │                │                │           │
//!     ▼           ▼                ▼                ▼           ▼
//! ┌────────┐ ┌────────┐    ┌─────────────┐   ┌─────────┐ ┌─────────┐
//! │sim-urdf│ │sim-mjcf│    │sim-constraint│   │sim-contact│ │  ...  │
//! │  URDF  │ │  MJCF  │    │Joint dynamics│   │ Contact │ │        │
//! └───┬────┘ └───┬────┘    └──────┬──────┘   └────┬────┘ └────────┘
//!     │          │                │               │
//!     └──────────┴────────┬───────┴───────────────┘
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
#[cfg(feature = "deformable")]
pub use sim_deformable;
pub use sim_mjcf;
pub use sim_muscle;
pub use sim_sensor;
pub use sim_tendon;
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
    // MuJoCo-style articulated body system from sim-core
    // ========================================================================

    pub use sim_core::{
        // Core articulated body system
        ArticulatedBody,
        ArticulatedJoint,
        ArticulatedJointType,
        ArticulatedSystem,
        // Index types
        BodyIndex,
        // Demo systems
        BouncingBall,
        // PGS constraint solver
        Constraint as PGSConstraint,
        ConstraintType as PGSConstraintType,
        DoublePendulum,
        JointIndex,
        NLinkPendulum,
        PGSConfig,
        PGSResult,
        PGSSolver,
        SimplePendulum,
        // Spatial algebra types
        SpatialMatrix,
        SpatialVector,
        // World integration
        SpawnedArticulation,
        SpherePile,
        SphericalPendulum,
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
        ValidationResult as UrdfValidationResult,
        // Loader
        load_urdf_file,
        load_urdf_str,
        // Parser (for advanced use)
        parse_urdf_str,
        // Validation
        validate as validate_urdf,
    };

    // ========================================================================
    // MJCF loading from sim-mjcf
    // ========================================================================

    pub use sim_mjcf::{
        // Configuration
        ExtendedSolverConfig,
        // Loaded model types
        GeomInfo,
        LoadedActuator,
        LoadedFixedTendon,
        LoadedModel,
        LoadedMuscle,
        LoadedSpatialTendon,
        LoadedTendon,
        // IR types (for inspection/modification)
        MjcfActuator,
        MjcfActuatorType,
        MjcfBody,
        // Errors
        MjcfError,
        MjcfGeom,
        MjcfGeomType,
        MjcfJoint,
        MjcfJointType as MjcfJointKind,
        MjcfLoader,
        MjcfModel,
        MjcfOption,
        SiteInfo,
        SpawnedModel,
        // Validation
        ValidationResult as MjcfValidationResult,
        // Loader - main entry points
        load_mjcf_file,
        load_mjcf_str,
        // Parser (for advanced use)
        parse_mjcf_str,
        validate as validate_mjcf,
    };

    // ========================================================================
    // Muscle simulation from sim-muscle
    // ========================================================================

    pub use sim_muscle::{
        // Activation dynamics
        ActivationDynamics,
        ActivationState,
        // Force curves
        ActiveForceLengthCurve,
        BiarticularlMuscleConfig,
        ConstantMomentArm,
        // State and diagnostics
        FiberState,
        ForceVelocityCurve,
        // Main muscle types
        HillMuscle,
        HillMuscleConfig,
        MomentArmModel,
        MuscleActuator,
        MuscleDiagnostics,
        MuscleForceCurves,
        MuscleForceResult,
        MuscleGroup,
        // Kinematics
        MusclePath,
        PassiveForceLengthCurve,
        PolynomialMomentArm,
        SplineMomentArm,
        ViaPoint,
    };

    // ========================================================================
    // Tendon simulation from sim-tendon
    // ========================================================================

    pub use sim_tendon::{
        // Path and attachments
        AttachmentPoint,
        // Cable properties
        CableProperties,
        CableState,
        // Wrapping geometry
        CylinderWrap,
        // Main tendon types
        FixedTendon,
        // Pulley systems
        Pulley,
        PulleyConfig,
        PulleySystem,
        SpatialTendon,
        SpatialTendonConfig,
        SphereWrap,
        TendonActuator,
        TendonCoefficient,
        // Errors
        TendonError,
        TendonPath,
        TendonSegment,
        TensionResult,
        WrapResult,
        WrappingGeometry,
    };

    // ========================================================================
    // Sensor simulation from sim-sensor
    // ========================================================================

    pub use sim_sensor::{
        ForceTorqueReading,
        ForceTorqueSensor,
        ForceTorqueSensorConfig,
        // Sensor types
        Imu,
        ImuConfig,
        ImuReading,
        Magnetometer,
        MagnetometerConfig,
        MagnetometerReading,
        Rangefinder,
        RangefinderConfig,
        RangefinderReading,
        RayCaster,
        RayHit,
        // Common types
        SensorData,
        // Errors
        SensorError,
        SensorId,
        SensorReading,
        SensorType,
        TouchReading,
        TouchSensor,
        TouchSensorConfig,
    };

    // ========================================================================
    // Deformable simulation from sim-deformable (optional feature)
    // ========================================================================

    #[cfg(feature = "deformable")]
    pub use sim_deformable::{
        BendingConstraint,
        // Skinning
        Bone,
        BoneWeight,
        // 1D: Ropes/cables
        CapsuleChain,
        CapsuleChainConfig,
        // 2D: Cloth
        Cloth,
        ClothConfig,
        // Constraints
        Constraint as DeformableConstraint,
        ConstraintType as DeformableConstraintType,
        // Deformable body trait
        DeformableBody,
        // Errors
        DeformableError,
        // Types
        DeformableId,
        // Mesh types
        DeformableMesh,
        DistanceConstraint as DeformableDistanceConstraint,
        Edge as DeformableEdge,
        FlexEdgeConstraint,
        FlexEdgeType,
        // Materials
        Material,
        MaterialPreset,
        Skeleton,
        SkinnedMesh,
        SkinnedMeshBuilder,
        SkinningMethod,
        SkinningResult,
        // 3D: Soft bodies
        SoftBody,
        SoftBodyConfig,
        // Solver
        SolverConfig as DeformableSolverConfig,
        Tetrahedron,
        Triangle as DeformableTriangle,
        Vertex,
        VertexFlags,
        VertexWeights,
        VolumeConstraint,
        XpbdSolver,
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

    #[test]
    fn test_mjcf_loading() {
        let mjcf = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(mjcf).expect("should parse");
        assert_eq!(model.name, "test");

        let mut world = World::default();
        let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

        assert_eq!(world.body_count(), 1);
        assert!(spawned.body_id("base").is_some());
    }

    #[test]
    fn test_mjcf_with_joints() {
        let mjcf = r#"
            <mujoco model="pendulum">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <geom type="sphere" size="0.05" mass="1.0"/>
                        <body name="arm" pos="0 0 -0.3">
                            <joint name="hinge" type="hinge" axis="0 1 0" limited="true" range="-3.14 3.14"/>
                            <geom type="capsule" size="0.02" fromto="0 0 0 0 0 -0.2" mass="0.1"/>
                        </body>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(mjcf).expect("should parse");
        assert_eq!(model.name, "pendulum");
        assert_eq!(model.bodies.len(), 2);
        assert_eq!(model.joints.len(), 1);

        let mut world = World::default();
        let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

        assert_eq!(world.body_count(), 2);
        assert_eq!(world.joint_count(), 1);
        assert!(spawned.joint_id("hinge").is_some());
    }

    #[test]
    fn test_mjcf_world_simulation_pipeline() {
        // Integration test: MJCF → World → simulate
        // Note: In MJCF, bodies without joints are considered "welded" to their parent.
        // A body needs a "free" joint to move independently under gravity.
        // Mass and inertia are computed from the geom (sphere with mass=1.0, radius=0.1).
        let mjcf = r#"
            <mujoco model="falling_ball">
                <option gravity="0 0 -9.81"/>
                <worldbody>
                    <body name="ball" pos="0 0 5">
                        <joint type="free"/>
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_mjcf_str(mjcf).expect("should parse");
        assert_eq!(model.bodies.len(), 1, "should have 1 body");
        assert_eq!(model.joints.len(), 1, "should have 1 joint (free)");

        let mut world = World::default();
        let spawned = model.spawn_at_origin(&mut world).expect("should spawn");

        let ball_id = spawned.body_id("ball").expect("ball should exist");
        let initial_z = world.body(ball_id).expect("body").state.pose.position.z;

        // Simulate for a short time
        let mut stepper = Stepper::new();
        stepper.run_for(&mut world, 0.1).expect("should simulate");

        let final_z = world.body(ball_id).expect("body").state.pose.position.z;

        // Ball should have fallen by at least some amount under gravity
        // After 0.1s under -9.81 m/s² gravity: Δz ≈ 0.5 * 9.81 * 0.1² = 0.049m
        assert!(
            final_z < initial_z - 0.01,
            "Ball should fall: initial_z={initial_z}, final_z={final_z}"
        );
    }
}
