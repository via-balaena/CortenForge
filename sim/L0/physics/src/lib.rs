//! Unified physics simulation API for CortenForge.
//!
//! This crate re-exports the complete physics simulation stack:
//!
//! - [`sim_types`] - Core data types (bodies, joints, poses, actions)
//! - [`sim_core`] - Simulation engine (Model/Data architecture, integrators)
//! - [`sim_constraint`] - Joint constraints (revolute, prismatic, fixed, etc.)
//! - [`sim_urdf`] - URDF robot description parser
//! - [`sim_mjcf`] - MJCF (`MuJoCo` XML Format) model loader
//! - [`sim_muscle`] - Hill-type muscle actuators for biomechanical simulation
//! - [`sim_tendon`] - Tendon and cable simulation for cable-driven robots
//! - [`sim_sensor`] - Sensor simulation (IMU, force/torque, touch, etc.)
//! - `sim_deformable` - Soft body and deformable simulation (behind `deformable` feature, planned)
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
//! # Quick Start with Model/Data API
//!
//! ```
//! use sim_physics::prelude::*;
//! use sim_mjcf::load_model;
//!
//! // Load a model from MJCF
//! let mjcf = r#"
//!     <mujoco model="ball">
//!         <worldbody>
//!             <body name="ball" pos="0 0 5">
//!                 <joint type="free"/>
//!                 <geom type="sphere" size="0.1" mass="1.0"/>
//!             </body>
//!         </worldbody>
//!     </mujoco>
//! "#;
//!
//! let model = load_model(mjcf).expect("should load");
//! let mut data = model.make_data();
//!
//! // Step the simulation
//! for _ in 0..100 {
//!     data.step(&model).expect("step failed");
//! }
//!
//! // Access body poses (computed from qpos via FK)
//! println!("Body 0 position: {:?}", data.xpos[0]);
//! ```
//!
//! # Loading URDF Robots
//!
//! URDF robots are converted to MJCF and loaded via the Model/Data API:
//!
//! ```
//! use sim_urdf::load_urdf_model;
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
//! let model = load_urdf_model(urdf).expect("should load");
//! let mut data = model.make_data();
//! data.step(&model).expect("step failed");
//!
//! println!("Loaded robot with {} bodies, {} joints", model.nbody, model.njnt);
//! ```
//!
//! # Architecture
//!
//! The physics engine follows `MuJoCo`'s Model/Data architecture:
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

#![doc(html_root_url = "https://docs.rs/sim-physics/0.7.0")]
#![deny(clippy::unwrap_used, clippy::expect_used)]
#![warn(missing_docs)]

// Re-export sub-crates
pub use sim_constraint;
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
    // MuJoCo-style Model/Data architecture from sim-core
    // ========================================================================

    // Primary API
    pub use sim_core::{Data, Model};

    // Collision shapes
    pub use sim_core::{Aabb, Axis, CollisionShape};

    // Additional core types
    pub use sim_core::{Contact, GeomType, Integrator as MjIntegrator, MjJointType, StepError};

    // Integrators
    pub use sim_core::integrators::{
        ExplicitEuler, Integrator, RungeKutta4, SemiImplicitEuler, VelocityVerlet,
        integrate_with_method,
    };

    // ========================================================================
    // Joint constraints from sim-constraint
    // ========================================================================

    pub use sim_constraint::{
        // Body/joint force types (for constraint solving)
        BodyState,
        CylindricalJoint,
        FixedJoint,
        FreeJoint,
        JointForce,
        // Motor
        JointMotor,
        // Limits
        LimitState,
        LimitStiffness,
        MotorMode,
        PlanarJoint,
        PrismaticJoint,
        // Joint types
        RevoluteJoint,
        SphericalJoint,
        UniversalJoint,
    };

    // ========================================================================
    // Contact types from sim-core
    // ========================================================================

    pub use sim_core::{ContactForce, ContactManifold, ContactPoint};

    // ========================================================================
    // URDF loading from sim-urdf
    // ========================================================================

    pub use sim_urdf::{
        // Errors
        UrdfError,
        UrdfGeometry,
        UrdfInertial,
        UrdfJoint,
        UrdfJointType,
        UrdfLink,
        UrdfOrigin,
        // IR types (for inspection/modification)
        UrdfRobot,
        ValidationResult as UrdfValidationResult,
        // Primary loader (returns Model)
        load_urdf_model,
        // Parser (for advanced use)
        parse_urdf_str,
        // URDF to MJCF conversion
        robot_to_mjcf,
        urdf_to_mjcf,
        // Validation
        validate as validate_urdf,
    };

    // ========================================================================
    // MJCF loading from sim-mjcf
    // ========================================================================

    pub use sim_mjcf::{
        // Configuration
        ExtendedSolverConfig,
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
        MjcfModel,
        MjcfOption,
        // Model conversion errors
        ModelConversionError,
        // Validation
        ValidationResult as MjcfValidationResult,
        // Primary loader (returns Model)
        load_model,
        model_from_mjcf,
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
    fn test_model_data_simulation() {
        // Test Model/Data API with n-link pendulum
        let model = Model::n_link_pendulum(1, 1.0, 0.1);
        let mut data = model.make_data();

        // Set initial angle and step
        data.qpos[0] = std::f64::consts::FRAC_PI_4;
        data.forward(&model).expect("forward failed");

        for _ in 0..100 {
            data.step(&model).expect("step failed");
        }

        // Pendulum should have swung
        assert!(data.qpos[0].abs() > 0.0);
    }

    #[test]
    fn test_mjcf_model_data() {
        let mjcf = r#"
            <mujoco model="test">
                <worldbody>
                    <body name="base" pos="0 0 1">
                        <geom type="sphere" size="0.1" mass="1.0"/>
                    </body>
                </worldbody>
            </mujoco>
        "#;

        let model = load_model(mjcf).expect("should parse");
        assert_eq!(model.name, "test");

        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");
        data.step(&model).expect("step failed");
    }

    #[test]
    fn test_urdf_model_data() {
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

        let model = load_urdf_model(urdf).expect("should parse");
        assert_eq!(model.name, "test");

        let mut data = model.make_data();
        data.forward(&model).expect("forward failed");
        data.step(&model).expect("step failed");
    }

    #[test]
    fn test_joint_types_accessible() {
        // Verify constraint types are accessible
        let _limits = sim_constraint::JointLimits::new(-1.0, 1.0);
        let _motor = JointMotor::velocity(1.0, 10.0);
    }

    #[test]
    fn test_collision_shape_accessible() {
        // Verify CollisionShape is accessible through prelude
        let sphere = CollisionShape::sphere(1.0);
        assert!(sphere.is_convex());
    }
}
