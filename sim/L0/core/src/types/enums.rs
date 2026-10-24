//! Enums and error types for the MuJoCo-aligned physics pipeline.
//!
//! This module defines the type-level vocabulary shared across all pipeline
//! stages: joint types, geometry types, solver types, constraint types,
//! sensor types, integrator selection, sleep policy, and error types.

use nalgebra::Vector3;

/// Element type for name↔index lookup via [`Model::name2id`](super::Model::name2id) / [`Model::id2name`](super::Model::id2name).
///
/// Each variant corresponds to a named element category in the model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ElementType {
    /// Body elements (indexed by body_id).
    Body,
    /// Joint elements (indexed by jnt_id).
    Joint,
    /// Geom elements (indexed by geom_id).
    Geom,
    /// Site elements (indexed by site_id).
    Site,
    /// Tendon elements (indexed by tendon_id).
    Tendon,
    /// Actuator elements (indexed by actuator_id).
    Actuator,
    /// Sensor elements (indexed by sensor_id).
    Sensor,
    /// Mesh assets (indexed by mesh_id).
    Mesh,
    /// Height field assets (indexed by hfield_id).
    Hfield,
    /// Equality constraints (indexed by eq_id).
    Equality,
    /// Keyframe elements (indexed by keyframe_id).
    Keyframe,
}

/// Joint type following `MuJoCo` conventions.
///
/// Named `MjJointType` to distinguish from `sim_types::JointType`.
/// `MuJoCo` uses different names (Hinge vs Revolute, Slide vs Prismatic).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjJointType {
    /// Hinge joint (1 DOF): rotation about a single axis.
    /// qpos: 1 scalar (angle in radians)
    /// qvel: 1 scalar (angular velocity)
    #[default]
    Hinge,
    /// Slide joint (1 DOF): translation along a single axis.
    /// qpos: 1 scalar (displacement)
    /// qvel: 1 scalar (linear velocity)
    Slide,
    /// Ball joint (3 DOF): free rotation (spherical).
    /// qpos: 4 scalars (unit quaternion w, x, y, z)
    /// qvel: 3 scalars (angular velocity)
    Ball,
    /// Free joint (6 DOF): floating body with no constraints.
    /// qpos: 7 scalars (position x,y,z + quaternion w,x,y,z)
    /// qvel: 6 scalars (linear velocity + angular velocity)
    Free,
}

impl MjJointType {
    /// Number of position coordinates (nq contribution).
    #[must_use]
    pub const fn nq(self) -> usize {
        match self {
            Self::Hinge | Self::Slide => 1,
            Self::Ball => 4, // quaternion
            Self::Free => 7, // pos + quat
        }
    }

    /// Number of velocity coordinates / DOFs (nv contribution).
    #[must_use]
    pub const fn nv(self) -> usize {
        match self {
            Self::Hinge | Self::Slide => 1,
            Self::Ball => 3, // angular velocity
            Self::Free => 6, // linear + angular velocity
        }
    }

    /// Whether this joint type uses quaternion representation.
    #[must_use]
    pub const fn uses_quaternion(self) -> bool {
        matches!(self, Self::Ball | Self::Free)
    }

    /// Whether this joint type supports springs (linear displacement from equilibrium).
    /// Ball/Free joints use quaternions and don't have a simple spring formulation.
    #[must_use]
    pub const fn supports_spring(self) -> bool {
        matches!(self, Self::Hinge | Self::Slide)
    }
}

/// Geometry type for collision detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GeomType {
    /// Plane (infinite, typically used for ground).
    Plane,
    /// Sphere defined by radius.
    #[default]
    Sphere,
    /// Capsule (cylinder with hemispherical caps).
    Capsule,
    /// Cylinder.
    Cylinder,
    /// Box (rectangular cuboid).
    Box,
    /// Ellipsoid.
    Ellipsoid,
    /// Convex mesh (requires mesh data).
    Mesh,
    /// Height field terrain.
    Hfield,
    /// Signed distance field (CortenForge extension — programmatic construction only).
    Sdf,
}

impl GeomType {
    /// MuJoCo's `mjtGeom` value: a contact lists the geom of the lower one
    /// first (`mj_collideGeoms`, `engine_collision_driver.c:1475-1480`).
    #[must_use]
    pub(crate) const fn mjtgeom(self) -> u8 {
        match self {
            Self::Plane => 0,
            Self::Hfield => 1,
            Self::Sphere => 2,
            Self::Capsule => 3,
            Self::Ellipsoid => 4,
            Self::Cylinder => 5,
            Self::Box => 6,
            Self::Mesh => 7,
            Self::Sdf => 8,
        }
    }

    /// Compute the bounding sphere radius for a geometry from its type and size.
    ///
    /// This is the canonical implementation used by both:
    /// - Model compilation (pre-computing `geom_rbound`)
    /// - `Shape::bounding_radius()` for runtime shapes
    ///
    /// # Arguments
    /// * `size` - Type-specific size parameters from `geom_size`:
    ///   - Sphere: `[radius, _, _]`
    ///   - Box: `[half_x, half_y, half_z]`
    ///   - Capsule: `[radius, half_length, _]`
    ///   - Cylinder: `[radius, half_length, _]`
    ///   - Ellipsoid: `[radius_x, radius_y, radius_z]`
    ///   - Plane: ignored (returns infinity)
    ///   - Mesh: `[scale_x, scale_y, scale_z]` (conservative estimate)
    // `imprecise_flops`: the cylinder's naive `sqrt(r² + h²)` (not `hypot`)
    // to match MuJoCo's `GetRBound` bit for bit.
    #[allow(clippy::imprecise_flops)]
    #[must_use]
    pub fn bounding_radius(self, size: Vector3<f64>) -> f64 {
        match self {
            Self::Sphere => size.x,
            Self::Box => size.norm(), // Distance from center to corner
            Self::Capsule => size.x + size.y, // radius + half_length
            // sqrt(r² + h²), as MuJoCo's `GetRBound` (`user_objects.cc:3435`).
            Self::Cylinder => (size.x * size.x + size.y * size.y).sqrt(),
            Self::Ellipsoid => size.x.max(size.y).max(size.z), // Max semi-axis
            Self::Plane => f64::INFINITY,                      // Planes are infinite
            Self::Mesh => {
                // Conservative estimate from scale factors.
                // Full implementation would use mesh AABB at load time.
                let scale = size.x.max(size.y).max(size.z);
                if scale > 0.0 { scale * 10.0 } else { 10.0 }
            }
            Self::Hfield => {
                // Conservative: horizontal half-diagonal from geom_size [x, y, z_top].
                // True bounding radius is overwritten by post-build pass.
                nalgebra::Vector2::new(size.x, size.y).norm()
            }
            Self::Sdf => {
                // Conservative: treat as axis-aligned box with half-extents from geom_size.
                // True bounding radius is overwritten by post-build pass using
                // SdfGrid::aabb().
                size.norm()
            }
        }
    }
}

/// Actuator transmission type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ActuatorTransmission {
    /// Direct joint actuation.
    #[default]
    Joint,
    /// Tendon actuation.
    Tendon,
    /// Site-based actuation.
    Site,
    /// Body (adhesion) actuation.
    Body,
    /// Slider-crank mechanism: crank site + slider site + rod.
    /// MuJoCo: `mjTRN_SLIDERCRANK`.
    SliderCrank,
    /// Joint transmission with force in parent frame.
    /// For hinge/slide joints, identical to `Joint`.
    /// MuJoCo: `mjTRN_JOINTINPARENT`.
    JointInParent,
}

/// Actuator dynamics type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ActuatorDynamics {
    /// No dynamics — input = ctrl (direct passthrough).
    #[default]
    None,
    /// First-order filter (Euler): act_dot = (ctrl - act) / tau.
    Filter,
    /// First-order filter (exact): act_dot = (ctrl - act) / tau,
    /// integrated as act += act_dot * tau * (1 - exp(-h/tau)).
    /// MuJoCo reference: `mjDYN_FILTEREXACT`.
    FilterExact,
    /// Integrator: act_dot = ctrl.
    Integrator,
    /// Muscle activation dynamics.
    Muscle,
    /// Hill-type muscle activation dynamics (CortenForge extension).
    /// Uses same `muscle_activation_dynamics()` as `Muscle` for activation,
    /// but pairs with `GainType::HillMuscle` / `BiasType::HillMuscle` for
    /// Hill-type force generation (Gaussian FL, Hill FV, pennation angle).
    HillMuscle,
    /// Millard2012-equilibrium muscle activation dynamics (CortenForge extension).
    /// Same `muscle_activation_dynamics()` as `Muscle`, paired with
    /// `GainType::MillardMuscle` / `BiasType::MillardMuscle` for the faithful
    /// OpenSim-Millard force model (validated vs OpenSim 4.6; see `forward::millard`).
    MillardMuscle,
    /// User-defined dynamics (via `cb_act_dyn` callback).
    /// MuJoCo reference: `mjDYN_USER`.
    User,
}

/// Actuator gain type — controls how `gain` is computed in Phase 2.
///
/// MuJoCo reference: `mjtGain` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum GainType {
    /// gain = gainprm\[0\] (constant).
    #[default]
    Fixed,
    /// gain = gainprm\[0\] + gainprm\[1\]*length + gainprm\[2\]*velocity.
    Affine,
    /// Muscle FLV gain (handled separately in the Muscle path).
    Muscle,
    /// Hill-type muscle active force (CortenForge extension).
    /// gain = −F0 × FL(L_norm) × FV(V_norm) × cos(α).
    HillMuscle,
    /// Millard2012 muscle active force (CortenForge extension).
    /// gain = −F0 × AFL(l̄) × FV(v̄) × cos(penn), the per-activation active force
    /// (paired with `BiasType::MillardMuscle`). See `forward::millard_active_gain`.
    MillardMuscle,
    /// User-defined gain (via `cb_act_gain` callback).
    /// MuJoCo reference: `mjGAIN_USER`.
    User,
}

/// Actuator bias type — controls how `bias` is computed in Phase 2.
///
/// MuJoCo reference: `mjtBias` enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum BiasType {
    /// bias = 0.
    #[default]
    None,
    /// bias = biasprm\[0\] + biasprm\[1\]*length + biasprm\[2\]*velocity.
    Affine,
    /// Muscle passive force (handled separately in the Muscle path).
    Muscle,
    /// Hill-type muscle passive force (CortenForge extension).
    /// bias = −F0 × FP(L_norm) × cos(α).
    HillMuscle,
    /// Millard2012 muscle passive + damping force (CortenForge extension).
    /// bias = −F0 × (PFL(l̄) + β·v̄) × cos(penn). See `forward::millard_passive_bias`.
    MillardMuscle,
    /// User-defined bias (via `cb_act_bias` callback).
    /// MuJoCo reference: `mjBIAS_USER`.
    User,
}

/// Interpolation method for actuator and sensor history buffers.
///
/// MuJoCo: `actuator_history[2*i + 1]` / `sensor_history[2*i + 1]` stores 0 (ZOH), 1 (linear), 2 (cubic).
/// MJCF keywords: `"zoh"`, `"linear"`, `"cubic"` (lowercase only).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum InterpolationType {
    /// Zero-order hold (default). MuJoCo int value: 0.
    #[default]
    Zoh = 0,
    /// Linear interpolation. MuJoCo int value: 1.
    Linear = 1,
    /// Cubic interpolation. MuJoCo int value: 2.
    Cubic = 2,
}

impl std::str::FromStr for InterpolationType {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "zoh" => Ok(Self::Zoh),
            "linear" => Ok(Self::Linear),
            "cubic" => Ok(Self::Cubic),
            _ => Err(format!(
                "invalid interp keyword '{s}': expected 'zoh', 'linear', or 'cubic'"
            )),
        }
    }
}

/// MuJoCo's integer code: 0 zero-order hold, 1 linear, 2 cubic. Another
/// value is refused and returned: MuJoCo's buffer read takes any value
/// other than 0 and 1 as cubic, so no mapping of it is the one a caller
/// meant.
impl TryFrom<i32> for InterpolationType {
    type Error = i32;

    fn try_from(v: i32) -> Result<Self, i32> {
        match v {
            0 => Ok(Self::Zoh),
            1 => Ok(Self::Linear),
            2 => Ok(Self::Cubic),
            other => Err(other),
        }
    }
}

/// Tendon wrap object type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum WrapType {
    /// Site point (tendon passes through).
    #[default]
    Site,
    /// Geom wrapping (tendon wraps around sphere/cylinder).
    Geom,
    /// Joint coupling (tendon length changes with joint angle).
    Joint,
    /// Pulley (changes tendon direction, may have divisor).
    Pulley,
}

/// Tendon type (pipeline-local enum, converted from MjcfTendonType in model builder).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TendonType {
    /// Fixed (linear coupling): L = Σ coef_i * q_i, constant Jacobian.
    #[default]
    Fixed,
    /// Spatial (3D path routing through sites): not yet implemented.
    Spatial,
}

/// Equality constraint type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum EqualityType {
    /// Connect: constrains two body points to coincide.
    /// Removes 3 DOF (translation only).
    #[default]
    Connect,
    /// Weld: constrains two body frames to be identical.
    /// Removes 6 DOF (translation + rotation).
    Weld,
    /// Joint: polynomial constraint between two joints.
    /// q2 = poly(q1) where poly = c0 + c1*q1 + c2*q1^2 + ...
    Joint,
    /// Tendon: polynomial constraint between two tendons.
    /// len2 = poly(len1).
    Tendon,
    /// Distance: constrains distance between two geom centers.
    /// |p1 - p2| = d (removes 1 DOF).
    /// `eq_obj1id`/`eq_obj2id` store geom IDs (not body IDs).
    Distance,
    /// Flex: holds each non-rigid edge of flex `eq_obj1id` at its rest
    /// length, one row per edge (MuJoCo's `mjEQ_FLEX`); `eq_obj2id` is
    /// `usize::MAX`.
    Flex,
}

/// `MuJoCo` sensor type.
///
/// Matches `MuJoCo`'s mjtSensor enum. Each sensor type reads different
/// quantities from the simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjSensorType {
    // ========== Common sensors ==========
    /// Touch sensor (contact force magnitude, 1D).
    #[default]
    Touch,
    /// Accelerometer (linear acceleration, 3D).
    Accelerometer,
    /// Velocity sensor (linear velocity, 3D).
    Velocimeter,
    /// Gyroscope (angular velocity, 3D).
    Gyro,
    /// Force sensor (3D force).
    Force,
    /// Torque sensor (3D torque).
    Torque,
    /// Magnetometer (magnetic field, 3D).
    Magnetometer,
    /// Rangefinder (distance to nearest surface, 1D).
    Rangefinder,

    // ========== Joint/tendon sensors ==========
    /// Joint position scalar (hinge/slide only, 1D).
    JointPos,
    /// Joint velocity scalar (hinge/slide only, 1D).
    JointVel,
    /// Ball joint orientation quaternion (4D). MuJoCo: mjSENS_BALLQUAT.
    BallQuat,
    /// Ball joint angular velocity (3D). MuJoCo: mjSENS_BALLANGVEL.
    BallAngVel,
    /// Tendon length (1D).
    TendonPos,
    /// Tendon velocity (1D).
    TendonVel,
    /// Actuator length (1D).
    ActuatorPos,
    /// Actuator velocity (1D).
    ActuatorVel,
    /// Actuator force (1D).
    ActuatorFrc,
    /// Joint limit force (scalar, 1D). MuJoCo: mjSENS_JOINTLIMITFRC.
    /// Returns the unsigned constraint force magnitude when the joint's position
    /// limit is active; 0 when within limits.
    JointLimitFrc,
    /// Tendon limit force (scalar, 1D). MuJoCo: mjSENS_TENDONLIMITFRC.
    /// Returns the unsigned constraint force magnitude when the tendon's length
    /// limit is active; 0 when within limits.
    TendonLimitFrc,

    // ========== Position/orientation sensors ==========
    /// Site/body frame position (3D).
    FramePos,
    /// Site/body frame orientation as quaternion (4D).
    FrameQuat,
    /// Site/body frame axis (3D).
    FrameXAxis,
    /// Site/body frame Y axis (3D).
    FrameYAxis,
    /// Site/body frame Z axis (3D).
    FrameZAxis,
    /// Site/body frame linear velocity (3D).
    FrameLinVel,
    /// Site/body frame angular velocity (3D).
    FrameAngVel,
    /// Site/body frame linear acceleration (3D).
    FrameLinAcc,
    /// Site/body frame angular acceleration (3D).
    FrameAngAcc,

    // ========== Global sensors ==========
    /// Subtree center of mass (3D).
    SubtreeCom,
    /// Subtree linear momentum (3D).
    SubtreeLinVel,
    /// Subtree angular momentum (3D).
    SubtreeAngMom,

    // ========== New sensors (Phase 6 Spec C) ==========
    /// Simulation clock (reads data.time, 1D). MuJoCo: mjSENS_CLOCK.
    Clock,
    /// Net actuator force at joint DOF (1D). MuJoCo: mjSENS_JOINTACTFRC.
    /// Reads `data.qfrc_actuator[model.jnt_dof_adr[objid]]`.
    JointActuatorFrc,
    /// Signed distance between two geoms or bodies (1D). MuJoCo: mjSENS_GEOMDIST.
    GeomDist,
    /// Surface normal at nearest point between geoms (3D). MuJoCo: mjSENS_GEOMNORMAL.
    GeomNormal,
    /// Nearest surface points between two geoms (6D). MuJoCo: mjSENS_GEOMFROMTO.
    GeomFromTo,

    // ========== User-defined ==========
    /// User-defined sensor (arbitrary dimension).
    User,

    // ========== Plugin (§66) ==========
    /// Plugin-controlled sensor (dimension set by plugin `nsensordata`).
    /// MuJoCo: `mjSENS_PLUGIN`.
    Plugin,
}

impl MjSensorType {
    /// Get the dimension (number of data elements) for this sensor type.
    #[must_use]
    pub const fn dim(self) -> usize {
        match self {
            Self::Touch
            | Self::JointPos
            | Self::JointVel
            | Self::TendonPos
            | Self::TendonVel
            | Self::ActuatorPos
            | Self::ActuatorVel
            | Self::ActuatorFrc
            | Self::JointLimitFrc
            | Self::TendonLimitFrc
            | Self::Rangefinder
            | Self::Clock
            | Self::JointActuatorFrc
            | Self::GeomDist => 1,

            Self::Accelerometer
            | Self::Velocimeter
            | Self::Gyro
            | Self::Force
            | Self::Torque
            | Self::Magnetometer
            | Self::BallAngVel
            | Self::FramePos
            | Self::FrameXAxis
            | Self::FrameYAxis
            | Self::FrameZAxis
            | Self::FrameLinVel
            | Self::FrameAngVel
            | Self::FrameLinAcc
            | Self::FrameAngAcc
            | Self::SubtreeCom
            | Self::SubtreeLinVel
            | Self::SubtreeAngMom
            | Self::GeomNormal => 3,

            Self::BallQuat | Self::FrameQuat => 4,

            Self::GeomFromTo => 6,

            Self::User | Self::Plugin => 0, // Variable, set explicitly
        }
    }

    /// Get the MuJoCo data kind for this sensor type.
    ///
    /// Determines how `apply_cutoff()` clamps sensor values. Matches MuJoCo's
    /// `user_objects.cc` datatype assignment switch.
    #[must_use]
    pub const fn data_kind(self) -> MjSensorDataKind {
        match self {
            // POSITIVE: clipped above by the cutoff, not below
            Self::Touch => MjSensorDataKind::Positive,

            // QUATERNION: unit quaternions
            Self::BallQuat | Self::FrameQuat => MjSensorDataKind::Quaternion,

            // AXIS: unit axis vectors
            Self::FrameXAxis | Self::FrameYAxis | Self::FrameZAxis | Self::GeomNormal => {
                MjSensorDataKind::Axis
            }

            // REAL: everything else (default in MuJoCo)
            _ => MjSensorDataKind::Real,
        }
    }
}

/// Pipeline stage for user sensor callbacks (DT-79).
///
/// Passed to `cb_sensor` to indicate which stage the callback is being
/// invoked from, so the callback can read the appropriate Data fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SensorStage {
    /// Position stage (after FK, before velocity).
    Pos,
    /// Velocity stage (after velocity FK, before acceleration).
    Vel,
    /// Acceleration stage (after constraint solve).
    Acc,
}

/// MuJoCo sensor data kind — determines postprocess cutoff behavior.
///
/// MuJoCo's `sensor_datatype` stores the data kind (`mjDATATYPE_REAL`, etc.),
/// which controls how `apply_cutoff()` clamps sensor values:
/// - `Real` → `clamp(-cutoff, cutoff)` (symmetric)
/// - `Positive` → `min(cutoff, value)` (preserves negative values)
/// - `Axis` / `Quaternion` → no clamping (normalized vectors/quaternions
///   should not be clamped to a magnitude range)
///
/// Note: CortenForge's `MjSensorDataType` stores the *pipeline stage*
/// (Position/Velocity/Acceleration), NOT this data kind. These are separate
/// concepts that MuJoCo happens to also call "datatype."
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MjSensorDataKind {
    /// Scalar real value — clamp to `[-cutoff, cutoff]`. MuJoCo: `mjDATATYPE_REAL` (0).
    Real,
    /// Non-negative value — clamp to `min(cutoff, value)`. MuJoCo: `mjDATATYPE_POSITIVE` (1).
    Positive,
    /// Unit axis vector — no cutoff clamping. MuJoCo: `mjDATATYPE_AXIS` (2).
    Axis,
    /// Unit quaternion — no cutoff clamping. MuJoCo: `mjDATATYPE_QUATERNION` (3).
    Quaternion,
}

/// Sensor data dependency stage.
///
/// Indicates when the sensor can be computed in the forward dynamics pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjSensorDataType {
    /// Computed in `mj_sensorPos` (after forward kinematics).
    #[default]
    Position,
    /// Computed in `mj_sensorVel` (after velocity FK).
    Velocity,
    /// Computed in `mj_sensorAcc` (after acceleration computation).
    Acceleration,
}

/// Object type for sensor attachment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum MjObjectType {
    /// No object (world-relative).
    #[default]
    None,
    /// Body — inertial/COM frame (reads `xipos`/`ximat`). MuJoCo `mjOBJ_BODY` (1).
    Body,
    /// XBody — joint frame origin (reads `xpos`/`xmat`). MuJoCo `mjOBJ_XBODY` (2).
    XBody,
    /// Joint.
    Joint,
    /// Geom.
    Geom,
    /// Site.
    Site,
    /// Actuator.
    Actuator,
    /// Tendon.
    Tendon,
    /// Plugin instance (§66).
    Plugin,
}

/// Per-tree sleep policy controlling automatic body deactivation (§16.0).
///
/// Resolved during model construction: `Auto` variants are computed from
/// tree properties (actuators, tendons, deformable bodies); user variants
/// come from the MJCF `<body sleep="...">` attribute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepPolicy {
    /// Compiler decides (initial state, resolved before use).
    Auto,
    /// Compiler determined: never sleep (has actuators, multi-tree tendons, etc.).
    AutoNever,
    /// Compiler determined: allowed to sleep.
    AutoAllowed,
    /// User policy: never sleep. XML: `sleep="never"`.
    Never,
    /// User policy: allowed to sleep. XML: `sleep="allowed"`.
    Allowed,
    /// User policy: start asleep. XML: `sleep="init"`.
    Init,
}

/// Whether a frame is its body's frame or the body's inertial frame (MuJoCo
/// `mjtSameFrame`).
///
/// Two frames are one to within 1e-6 in each component. The kinematics copy
/// such a frame rather than compute it, as MuJoCo's `mj_local2Global` does. A
/// body's own inertial frame is compared with the body's frame only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SameFrame {
    /// Neither: computed from the body's frame.
    #[default]
    None,
    /// The body's frame.
    Body,
    /// The body's inertial frame.
    Inertia,
    /// The body's orientation, at another position.
    BodyRot,
    /// The inertial frame's orientation, at another position.
    InertiaRot,
}

/// Per-body sleep state for efficient pipeline gating (§16.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepState {
    /// A body in no kinematic tree: the world and every body welded to it,
    /// except a body under a mocap body, which is `Awake`. Never sleeps.
    Static,
    /// Body is asleep. Position/velocity stages are skipped.
    Asleep,
    /// Body is awake. Full pipeline computation.
    Awake,
}

// ── Disable flags (mjtDisableBit, mjNDISABLE = 19) ──
// Each bit gates a pipeline subsystem. Bit set = subsystem disabled.
// Default: all bits clear (disableflags = 0), matching MuJoCo's mj_defaultOption().

/// Skip constraint assembly + collision detection.
pub const DISABLE_CONSTRAINT: u32 = 1 << 0;
/// Skip equality constraint rows.
pub const DISABLE_EQUALITY: u32 = 1 << 1;
/// Skip joint/tendon friction loss constraints.
pub const DISABLE_FRICTIONLOSS: u32 = 1 << 2;
/// Skip joint/tendon limit rows.
pub const DISABLE_LIMIT: u32 = 1 << 3;
/// Skip collision detection + contact rows.
pub const DISABLE_CONTACT: u32 = 1 << 4;
/// Skip passive spring forces.
pub const DISABLE_SPRING: u32 = 1 << 5;
/// Skip passive damping forces.
pub const DISABLE_DAMPER: u32 = 1 << 6;
/// Zero gravity in `mj_rne()`.
pub const DISABLE_GRAVITY: u32 = 1 << 7;
/// Skip clamping ctrl values to ctrlrange.
pub const DISABLE_CLAMPCTRL: u32 = 1 << 8;
/// Zero-initialize solver instead of warmstart.
pub const DISABLE_WARMSTART: u32 = 1 << 9;
/// Disable parent-child collision filtering.
pub const DISABLE_FILTERPARENT: u32 = 1 << 10;
/// Skip actuator force computation.
pub const DISABLE_ACTUATION: u32 = 1 << 11;
/// Skip `solref[0] >= 2*timestep` enforcement.
pub const DISABLE_REFSAFE: u32 = 1 << 12;
/// Skip all sensor evaluation.
pub const DISABLE_SENSOR: u32 = 1 << 13;
/// Skip the midphases: a pair of bodies collides each geom of one with each
/// of the other (MuJoCo's `mj_collideTree` skipped), and a mesh's triangle
/// hierarchy is not descended.
pub const DISABLE_MIDPHASE: u32 = 1 << 14;
/// Skip implicit damping in Euler integrator.
pub const DISABLE_EULERDAMP: u32 = 1 << 15;
/// Skip auto-reset on NaN/divergence.
pub const DISABLE_AUTORESET: u32 = 1 << 16;
/// Collide convex pairs with libccd instead of MuJoCo's native convex collision.
///
/// sim-core does not port libccd: [`Model::try_make_data`] refuses a model
/// with this flag (registry `D-NATIVECCD`).
///
/// [`Model::try_make_data`]: crate::Model::try_make_data
pub const DISABLE_NATIVECCD: u32 = 1 << 17;
/// Skip island discovery.
pub const DISABLE_ISLAND: u32 = 1 << 18;

// ── Enable flags (mjtEnableBit, mjNENABLE = 6) ──
// Each bit enables an optional subsystem. Bit set = subsystem enabled.
// Default: all bits clear (enableflags = 0).

/// Enable contact parameter override.
pub const ENABLE_OVERRIDE: u32 = 1 << 0;
/// Enable potential + kinetic energy computation.
pub const ENABLE_ENERGY: u32 = 1 << 1;
/// Enable forward/inverse comparison stats.
pub const ENABLE_FWDINV: u32 = 1 << 2;
/// Discrete-time inverse dynamics.
pub const ENABLE_INVDISCRETE: u32 = 1 << 3;
/// Multi-point CCD for flat surfaces.
pub const ENABLE_MULTICCD: u32 = 1 << 4;
/// Enable body sleeping/deactivation.
/// Set via MJCF `<option><flag sleep="enable"/>`.
pub const ENABLE_SLEEP: u32 = 1 << 5;

/// Minimum number of consecutive sub-threshold timesteps before a tree
/// can transition to sleep. Matches MuJoCo's `mjMINAWAKE = 10`.
pub const MIN_AWAKE: i32 = 10;

/// Constraint solver algorithm (matches MuJoCo's `mjSOL_*`).
///
/// All solver types operate on the same unified constraint rows assembled by
/// `assemble_unified_constraints()`. PGS works in dual (force) space; CG and
/// Newton share the primal `mj_sol_primal` infrastructure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SolverType {
    /// Projected Gauss-Seidel.
    /// Dual-space solver: operates on constraint forces via the regularized
    /// Delassus matrix AR = J·M⁻¹·J^T + diag(R). Handles all constraint types
    /// with per-type projection (bilateral, box, unilateral, friction cone).
    /// First-order (linear convergence): under-converged at the default 100
    /// iterations on stiff/weakly-coupled DOFs.
    PGS,
    /// Primal Polak-Ribiere conjugate gradient, MuJoCo's `mj_solCG`: the
    /// primal solver Newton shares (`mj_solPrimal`), preconditioned by M⁻¹,
    /// directions by Polak-Ribière. Its iterate is the acceleration.
    CG,
    /// Newton, MuJoCo's `mj_solNewton`: the primal solver CG shares
    /// (`mj_solPrimal`), preconditioned by the Hessian `M + J'·D·J` (plus the
    /// cones'). Its iterate is the acceleration; at its iteration cap it keeps
    /// the last, as MuJoCo's.
    ///
    /// Default solver, matching MuJoCo (Newton has been MuJoCo's default since
    /// 2.0). The GPU pipeline is hardwired to Newton, so this keeps a
    /// default-configured model consistent across the CPU and GPU paths.
    #[default]
    Newton,
}

/// The constraint Jacobian's layout, MuJoCo's `mjtJacobian` (`<option
/// jacobian>`): it decides whether the constraint pipeline takes its sparse
/// path ([`Model::is_sparse`](crate::Model::is_sparse)).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum JacobianType {
    /// Dense at every size.
    Dense,
    /// Sparse at every size.
    Sparse,
    /// Dense below 60 dofs, sparse from 60 (MuJoCo's default).
    #[default]
    Auto,
}

/// Constraint type annotation per row in the unified constraint system.
///
/// Each scalar row of the constraint Jacobian (`efc_J`) is tagged with one
/// of these types to determine its cost function and state machine behavior
/// in `PrimalUpdateConstraint` (§15.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstraintType {
    /// Equality constraint (connect, weld, joint, tendon, distance, flex).
    Equality,
    /// A dof's friction loss (Huber cost), MuJoCo's `mjCNSTR_FRICTION_DOF`;
    /// `efc_id` is the dof.
    FrictionDof,
    /// A tendon's friction loss (Huber cost), MuJoCo's
    /// `mjCNSTR_FRICTION_TENDON`; `efc_id` is the tendon.
    FrictionTendon,
    /// Joint limit constraint.
    LimitJoint,
    /// Tendon limit constraint.
    LimitTendon,
    /// Frictionless contact (condim=1 or μ≈0). Unilateral, scalar projection.
    ContactFrictionless,
    /// Pyramidal friction cone facet (§32). Each facet is an independent
    /// non-negative constraint with combined Jacobian `J_normal ± μ·J_friction`.
    /// A condim=3 contact produces 4 facets, condim=4→6, condim=6→10.
    ContactPyramidal,
    /// Contact with elliptic friction cone (condim ≥ 3 and cone == elliptic).
    ContactElliptic,
}

impl ConstraintType {
    /// A friction-loss row, a dof's or a tendon's.
    #[must_use]
    pub const fn is_friction_loss(self) -> bool {
        matches!(self, Self::FrictionDof | Self::FrictionTendon)
    }
}

/// Constraint state per scalar row, determined by `PrimalUpdateConstraint`.
///
/// Maps to MuJoCo's `mjCNSTRSTATE_*` values. The state determines which
/// branch of the cost function and Hessian contribution is active for each
/// constraint row during Newton iterations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ConstraintState {
    /// Active with quadratic cost: row contributes D_i * J_i^T * J_i to Hessian.
    #[default]
    Quadratic,
    /// Constraint satisfied (inactive): zero force, no Hessian contribution.
    Satisfied,
    /// Linear regime, negative side (friction loss below -R*floss).
    LinearNeg,
    /// Linear regime, positive side (friction loss above +R*floss).
    LinearPos,
    /// Elliptic friction cone active: coupled Hessian across contact rows.
    Cone,
}

/// One solver iteration's statistics, MuJoCo's `mjSolverStat`
/// (`mjdata.h`), as its `saveStats` records them (`engine_solver.c:38-65`).
///
/// Newton and CG fill every field; PGS and noslip `improvement`, `nactive`
/// and `nchange`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SolverStat {
    /// Cost decrease over the iteration, scaled: `scale · (old − new)`.
    pub improvement: f64,
    /// Norm of the gradient after the iteration, scaled: `scale · ‖grad‖`.
    pub gradient: f64,
    /// The line search's slope at the step it took, `|∂cost/∂α| · scale /
    /// ‖search‖`; 1 when it computed none.
    pub lineslope: f64,
    /// Constraint rows not satisfied (Quadratic, Linear or Cone).
    pub nactive: usize,
    /// Constraint rows whose state changed this iteration.
    pub nchange: usize,
    /// Line-search cost evaluations this iteration.
    pub neval: usize,
    /// Cholesky factor updates this iteration (Newton): one per row that
    /// entered or left the quadratic state, one per row of each cone, or the
    /// row count when the factor was made anew.
    pub nupdate: usize,
}

/// Integration method.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[non_exhaustive]
pub enum Integrator {
    /// Semi-implicit Euler (`MuJoCo` default).
    #[default]
    Euler,
    /// 4th order Runge-Kutta.
    RungeKutta4,
    /// Implicit Euler for diagonal per-DOF spring/damper forces.
    ImplicitSpringDamper,
    /// Full implicit integration with asymmetric D and LU factorization.
    /// Includes Coriolis velocity derivatives.
    Implicit,
    /// Implicit integration without the Coriolis velocity derivatives.
    ImplicitFast,
}

/// Errors that stop a simulation step.
///
/// A bad state is not an error: a NaN, ±inf or |x| > 1e10 in `qpos`, `qvel`
/// or `qacc` resets the `Data` (unless `DISABLE_AUTORESET` is set) and the
/// step returns `Ok`; check [`Data::divergence_detected`]. A bad control
/// (after clamping to `ctrlrange`) makes every actuator's control input 0 for
/// that pass (an actuator with an activation still acts on it), leaves `ctrl`
/// as written and counts [`Warning::BadCtrl`], which `divergence_detected`
/// does not read; under `implicit` and `implicitfast` the velocity derivative
/// still reads it for an actuator without dynamics whose gain has a velocity
/// term. All as MuJoCo.
///
/// [`Data::divergence_detected`]: crate::Data::divergence_detected
/// [`Warning::BadCtrl`]: crate::Warning::BadCtrl
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// A Cholesky factorization failed: under the implicit spring-damper
    /// integrator the modified mass matrix (M + h*D + h²*K) is not positive
    /// definite; or Newton's sparse Hessian has a diagonal under mjMINVAL,
    /// where MuJoCo stops ("rank-deficient sparse Hessian",
    /// `engine_solver.c:1644-1646`).
    CholeskyFailed,
    /// Under the implicit integrator, a diagonal of `M − h·D`'s LU factor is
    /// below mjMINVAL in magnitude.
    LuSingular,
    /// `model.timestep` is not positive and finite: zero, negative, NaN or infinite.
    InvalidTimestep,
    /// A `Data` array does not have the length the model requires: the `Data` was made by
    /// another model, or the caller resized one of its arrays.
    DataShapeMismatch {
        /// The `Data` field (`"geom_xpos"`).
        field: &'static str,
        /// The length the model requires.
        expected: usize,
        /// The field's length.
        actual: usize,
    },
    /// Finite-difference derivatives refuse the model's integrator (RK4), as
    /// MuJoCo 3.5.0's `mjd_transitionFD` and `mjd_inverseFD` do ("RK4
    /// integrator is not supported", `engine_derivative_fd.c:544-546`,
    /// `:614-616`).
    UnsupportedIntegrator {
        /// The model's integrator.
        integrator: Integrator,
    },
    /// Finite-difference transition derivatives refuse a model with history
    /// buffers (actuator or sensor delays), as `mjd_transitionFD` does ("delays
    /// are not supported", `engine_derivative_fd.c:547-549`).
    UnsupportedHistory {
        /// The model's history buffer length.
        nhistory: usize,
    },
    /// Finite-difference inverse-dynamics derivatives refuse the noslip
    /// solver, as `mjd_inverseFD` does ("noslip solver is not supported",
    /// `engine_derivative_fd.c:618-620`).
    UnsupportedNoslip {
        /// The model's noslip iterations.
        iterations: usize,
    },
    /// Sleep is enabled and equality `eq`, a tendon equality, is active:
    /// MuJoCo 3.5.0 raises an error in the position stage then ("tendon
    /// equality does not yet support sleeping", `mj_wakeEquality`,
    /// `engine_sleep.c:398-400`), so the calls that run that stage refuse it:
    /// `step`, `step1`, `forward`, `forward_skip` from `MjStage::None`, the
    /// transition finite differences and the reset that puts `Init` trees
    /// to sleep.
    TendonEqualityWithSleep {
        /// The equality.
        eq: usize,
    },
    /// With sleep enabled, the inverse dynamics' position stage
    /// ([`Data::inverse`](crate::Data::inverse), from `MjStage::None`) met a
    /// contact between two geoms, not excluded, one of whose bodies sleeps:
    /// that stage wakes no tree a contact touches (MuJoCo's `mj_invPosition`
    /// runs no `mj_wakeCollision`), and MuJoCo 3.5.0's constraint counting
    /// stops there ("contact … involves sleeping geom …", `mj_nc`,
    /// `engine_core_constraint.c:1944-1956`). The forward pass wakes such a
    /// tree first, so a forward pass at the same state, then the inverse,
    /// runs.
    ContactWithSleepingGeom {
        /// The contact, an index into `Data::contacts`.
        contact: usize,
        /// Its sleeping geom.
        geom: usize,
    },
    /// `ENABLE_INVDISCRETE` under an integrator the discrete inverse
    /// dynamics do not support: RK4, which MuJoCo 3.5.0's `mj_discreteAcc`
    /// refuses ("discrete inverse dynamics is not supported by RK4
    /// integrator", `engine_inverse.c:85-88`), and ImplicitSpringDamper,
    /// which MuJoCo does not have. Refused by
    /// [`Data::inverse`](crate::Data::inverse) and
    /// [`mjd_inverse_fd`](crate::mjd_inverse_fd) before any work, and, with
    /// `ENABLE_FWDINV`, by the steps that compare the forward and the inverse
    /// dynamics (`step`, `step2`) when the comparison reaches the inverse
    /// (the forward pass made constraint rows), before the advance, as
    /// MuJoCo's `mj_step` stops there.
    UnsupportedInverseDiscrete {
        /// The integrator.
        integrator: Integrator,
    },
    /// A contact's normal, as its collision function wrote it, is shorter
    /// than 0.5: MuJoCo 3.5.0's `mju_makeFrame` stops there ("xaxis of
    /// contact frame undefined", `engine_util_spatial.c:511-513`, which
    /// `mj_setContact` calls for every contact), so the position stage
    /// refuses it after collision. A [`PhysicsShape`](crate::PhysicsShape)
    /// whose `gradient` is that short writes one. A normal that is not
    /// finite passes MuJoCo's test and is not refused.
    UndefinedContactFrame {
        /// The contact, an index into `Data::contacts`.
        contact: usize,
    },
    /// A geom contact between two static bodies (under a `<pair>`), whose
    /// rows reach no kinematic tree, at the dense size with islands: MuJoCo
    /// 3.5.0's `mj_island` stops there ("contact … is between two static
    /// bodies", `treeFirst`, `engine_island.c:225-228`). At the sparse size
    /// the contact makes no rows, and without islands its rows are solved.
    ConstraintWithoutTree {
        /// The row, an index into the `efc_*` arrays.
        row: usize,
    },
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CholeskyFailed => {
                write!(
                    f,
                    "Cholesky decomposition failed (implicit integration, or Newton's sparse Hessian)"
                )
            }
            Self::LuSingular => {
                write!(f, "LU decomposition failed in implicit integration")
            }
            Self::InvalidTimestep => write!(f, "timestep must be positive and finite"),
            Self::DataShapeMismatch {
                field,
                expected,
                actual,
            } => write!(
                f,
                "data.{field} has length {actual}, but the model needs {expected}: the Data was \
                 made by another model, or the array was resized"
            ),
            Self::UnsupportedIntegrator { integrator } => write!(
                f,
                "finite-difference derivatives: {integrator:?} integrator is not supported"
            ),
            Self::UnsupportedHistory { nhistory } => write!(
                f,
                "finite-difference derivatives: delays are not supported (nhistory {nhistory})"
            ),
            Self::UnsupportedNoslip { iterations } => write!(
                f,
                "inverse finite-difference derivatives: noslip solver is not supported \
                 ({iterations} iterations)"
            ),
            Self::TendonEqualityWithSleep { eq } => write!(
                f,
                "equality {eq}: tendon equality does not yet support sleeping"
            ),
            Self::ContactWithSleepingGeom { contact, geom } => {
                write!(f, "contact {contact} involves sleeping geom {geom}")
            }
            Self::UnsupportedInverseDiscrete { integrator } => write!(
                f,
                "discrete inverse dynamics is not supported by {integrator:?} integrator"
            ),
            Self::UndefinedContactFrame { contact } => write!(
                f,
                "contact {contact}: xaxis of contact frame undefined (normal shorter than 0.5)"
            ),
            Self::ConstraintWithoutTree { row } => {
                write!(f, "constraint row {row} reaches no kinematic tree")
            }
        }
    }
}

impl std::error::Error for StepError {}

/// Error returned by state reset operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ResetError {
    /// The keyframe index is out of range.
    InvalidKeyframeIndex {
        /// The requested index.
        index: usize,
        /// The number of keyframes in the model.
        nkeyframe: usize,
    },
    /// The model has history buffers and a timestep that is not positive, so
    /// their timestamps cannot be laid out (MuJoCo `_resetData`,
    /// `engine_io.c:1266-1270`).
    InvalidTimestep,
    /// Sensor `sensor` is a user or plugin sensor with a delay, whose sample
    /// cannot be computed when the state advances (MuJoCo `mjERROR`s there).
    DelayedUserSensor {
        /// The sensor.
        sensor: usize,
    },
    /// Of the `marked` trees whose policy is `Init`, `mj_sleep` put only
    /// `slept` to sleep (one is in an island with an awake tree); `tree`,
    /// rooted at `root_body`, is the first it did not (MuJoCo `_resetData`
    /// raises an error, `engine_io.c:1472-1493`).
    InitSleep {
        /// The trees whose policy is `Init`.
        marked: usize,
        /// The trees `mj_sleep` put to sleep.
        slept: usize,
        /// The first `Init` tree left awake.
        tree: usize,
        /// That tree's root body.
        root_body: usize,
    },
    /// The forward pass a reset runs before putting the `Init` trees to sleep
    /// failed: a tendon equality with sleep, on which MuJoCo's reset raises
    /// the same error, or the implicit spring-damper integrator's
    /// factorization.
    InitForward(StepError),
}

impl std::fmt::Display for ResetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidKeyframeIndex { index, nkeyframe } => {
                write!(
                    f,
                    "invalid keyframe index {index} (model has {nkeyframe} keyframes)"
                )
            }
            Self::InvalidTimestep => f.write_str(HISTORY_TIMESTEP),
            Self::DelayedUserSensor { sensor } => write!(f, "{}", delayed_user_sensor(*sensor)),
            Self::InitSleep {
                marked,
                slept,
                tree,
                root_body,
            } => f.write_str(&init_sleep(*marked, *slept, *tree, *root_body)),
            Self::InitForward(e) => write!(f, "{INIT_FORWARD}: {e}"),
        }
    }
}

/// The message of `InitSleep` in [`ResetError`] and [`MakeDataError`]:
/// MuJoCo's (`engine_io.c:1490-1492`), with ids.
fn init_sleep(marked: usize, slept: usize, tree: usize, root_body: usize) -> String {
    format!(
        "{marked} trees were marked as sleep='init' but only {slept} could be slept; body \
         {root_body} is the root of tree {tree}, the first that could not be slept"
    )
}

/// The message of `InitForward` in [`ResetError`] and [`MakeDataError`].
const INIT_FORWARD: &str = "the forward pass before the sleep='init' trees sleep failed";

/// The message of `InvalidTimestep` in [`ResetError`] and [`MakeDataError`].
const HISTORY_TIMESTEP: &str = "history buffers require a positive timestep";

/// The message of `DelayedUserSensor` in [`ResetError`] and [`MakeDataError`].
fn delayed_user_sensor(sensor: usize) -> String {
    format!("sensor {sensor} is a user or plugin sensor with a delay, which cannot be computed")
}

impl std::error::Error for ResetError {}

/// Why [`Model::try_make_data`](crate::Model::try_make_data) cannot make a `Data`.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum MakeDataError {
    /// The model's joints break a layout rule
    /// ([`Model::check_joint_layout`](crate::Model::check_joint_layout)).
    JointLayout(JointLayoutError),
    /// A limited range is not a valid range
    /// ([`Model::check_ranges`](crate::Model::check_ranges)).
    Range(RangeError),
    /// Plugin instance `instance`'s [`Plugin::init`](crate::plugin::Plugin::init)
    /// returned an error.
    PluginInit {
        /// The plugin instance.
        instance: usize,
        /// The plugin's message.
        message: String,
    },
    /// The model has history buffers and a timestep that is not positive
    /// (MuJoCo `_resetData`, `engine_io.c:1266-1270`).
    InvalidTimestep,
    /// Sensor `sensor` is a user or plugin sensor with a delay, whose sample
    /// cannot be computed when the state advances (MuJoCo `mjERROR`s there).
    DelayedUserSensor {
        /// The sensor.
        sensor: usize,
    },
    /// Of the `marked` trees whose policy is `Init`, `mj_sleep` put only
    /// `slept` to sleep; see [`ResetError::InitSleep`]. MuJoCo refuses such a
    /// model at compile, which makes a Data.
    InitSleep {
        /// The trees whose policy is `Init`.
        marked: usize,
        /// The trees `mj_sleep` put to sleep.
        slept: usize,
        /// The first `Init` tree left awake.
        tree: usize,
        /// That tree's root body.
        root_body: usize,
    },
    /// The forward pass before the `Init` trees sleep failed; see
    /// [`ResetError::InitForward`].
    InitForward(StepError),
    /// The model sets a flag for a part of MuJoCo sim-core does not port.
    Unsupported {
        /// The flag and what it would take.
        feature: &'static str,
    },
}

impl std::fmt::Display for MakeDataError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::JointLayout(e) => write!(f, "{e}"),
            Self::Range(e) => write!(f, "{e}"),
            Self::PluginInit { instance, message } => {
                write!(f, "plugin init failed for instance {instance}: {message}")
            }
            Self::InvalidTimestep => f.write_str(HISTORY_TIMESTEP),
            Self::DelayedUserSensor { sensor } => write!(f, "{}", delayed_user_sensor(*sensor)),
            Self::InitSleep {
                marked,
                slept,
                tree,
                root_body,
            } => f.write_str(&init_sleep(*marked, *slept, *tree, *root_body)),
            Self::InitForward(e) => write!(f, "{INIT_FORWARD}: {e}"),
            Self::Unsupported { feature } => write!(f, "unsupported: {feature}"),
        }
    }
}

impl std::error::Error for MakeDataError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::JointLayout(e) => Some(e),
            Self::Range(e) => Some(e),
            Self::InitForward(e) => Some(e),
            Self::PluginInit { .. }
            | Self::InvalidTimestep
            | Self::DelayedUserSensor { .. }
            | Self::InitSleep { .. }
            | Self::Unsupported { .. } => None,
        }
    }
}

impl From<JointLayoutError> for MakeDataError {
    fn from(e: JointLayoutError) -> Self {
        Self::JointLayout(e)
    }
}

impl From<RangeError> for MakeDataError {
    fn from(e: RangeError) -> Self {
        Self::Range(e)
    }
}

/// Why a history-buffer call refuses.
///
/// The calls are [`Data::read_ctrl`](crate::Data::read_ctrl),
/// [`Data::read_sensor`](crate::Data::read_sensor),
/// [`Data::init_ctrl_history`](crate::Data::init_ctrl_history) and
/// [`Data::init_sensor_history`](crate::Data::init_sensor_history). An index
/// out of range, a missing buffer and times that do not increase are where
/// MuJoCo 3.5.0's `mj_readCtrl`, `mj_readSensor`, `mj_initCtrlHistory` or
/// `mj_initSensorHistory` raises `mjERROR`; a slice of the wrong length
/// (MuJoCo's C takes no lengths; its Python binding checks them) and a
/// `Data` of another shape (registry row `D-DATA-SHAPE`) are this API's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum HistoryError {
    /// The actuator index is not below `nu`.
    InvalidActuator {
        /// The index passed.
        id: usize,
        /// The model's `nu`.
        nu: usize,
    },
    /// The sensor index is not below `nsensor`.
    InvalidSensor {
        /// The index passed.
        id: usize,
        /// The model's `nsensor`.
        nsensor: usize,
    },
    /// The actuator or sensor has no history buffer (`nsample <= 0`).
    NoBuffer,
    /// A slice has another length than the buffer or the sensor needs.
    WrongLength {
        /// The length needed.
        expected: usize,
        /// The length passed.
        actual: usize,
    },
    /// `times[index + 1]` is not above `times[index]` by at least `1e-15`.
    TimesNotIncreasing {
        /// The first index of the pair.
        index: usize,
    },
    /// An array the call reads or writes (`history`, `ctrl`, `sensordata`)
    /// does not have the length the model requires: the `Data` was made by
    /// another model, or the caller resized it.
    DataShapeMismatch {
        /// The `Data` field.
        field: &'static str,
        /// The length the model requires.
        expected: usize,
        /// The field's length.
        actual: usize,
    },
}

impl std::fmt::Display for HistoryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidActuator { id, nu } => write!(f, "invalid actuator id {id} (nu {nu})"),
            Self::InvalidSensor { id, nsensor } => {
                write!(f, "invalid sensor id {id} (nsensor {nsensor})")
            }
            Self::NoBuffer => f.write_str("no history buffer (nsample <= 0)"),
            Self::WrongLength { expected, actual } => {
                write!(f, "expected {expected} values, got {actual}")
            }
            Self::TimesNotIncreasing { index } => write!(
                f,
                "times must be strictly increasing, got times[{index}] >= times[{}]",
                index + 1
            ),
            Self::DataShapeMismatch {
                field,
                expected,
                actual,
            } => write!(
                f,
                "data.{field} has length {actual}, the model requires {expected}"
            ),
        }
    }
}

impl std::error::Error for HistoryError {}

/// Why [`Model::recompute_derived`](crate::Model::recompute_derived) refuses a model.
///
/// Its joint layout or one of its limited ranges, which
/// [`Model::try_make_data`](crate::Model::try_make_data) refuses too.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ModelError {
    /// [`Model::check_joint_layout`](crate::Model::check_joint_layout)'s error.
    JointLayout(JointLayoutError),
    /// [`Model::check_ranges`](crate::Model::check_ranges)'s error.
    Range(RangeError),
}

impl std::fmt::Display for ModelError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::JointLayout(e) => write!(f, "{e}"),
            Self::Range(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for ModelError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::JointLayout(e) => Some(e),
            Self::Range(e) => Some(e),
        }
    }
}

impl From<JointLayoutError> for ModelError {
    fn from(e: JointLayoutError) -> Self {
        Self::JointLayout(e)
    }
}

impl From<RangeError> for ModelError {
    fn from(e: RangeError) -> Self {
        Self::Range(e)
    }
}

/// A joint layout [`Model::check_joint_layout`](crate::Model::check_joint_layout)
/// refuses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum JointLayoutError {
    /// Body `body`'s joints have `ndof` degrees of freedom, more than 6
    /// (MuJoCo: "more than 6 dofs"). A free joint sharing its body is this.
    TooManyDofs {
        /// The body.
        body: usize,
        /// Its joints' degrees of freedom.
        ndof: usize,
    },
    /// Ball joint `joint` is not the last joint on body `body`.
    BallNotLast {
        /// The body.
        body: usize,
        /// The ball joint.
        joint: usize,
    },
}

impl std::fmt::Display for JointLayoutError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooManyDofs { body, ndof } => write!(
                f,
                "body {body} has {ndof} degrees of freedom, more than 6 \
                 (a free joint must be the body's only joint)"
            ),
            Self::BallNotLast { body, joint } => write!(
                f,
                "ball joint {joint} on body {body} must be the LAST joint on its body; \
                 a later same-body joint would over-rotate its motion subspace"
            ),
        }
    }
}

impl std::error::Error for JointLayoutError {}

/// A limited range [`Model::check_ranges`](crate::Model::check_ranges) refuses:
/// entry `index` of the model's `field`, e.g. `jnt_range` 3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub struct RangeError {
    /// The `Model` field, e.g. `"actuator_ctrlrange"`.
    pub field: &'static str,
    /// The entry.
    pub index: usize,
}

impl std::fmt::Display for RangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}[{}] is not a valid range (see Model::check_ranges)",
            self.field, self.index
        )
    }
}

impl std::error::Error for RangeError {}

/// Bending model for dim=2 flex bodies.
///
/// `Cotangent` (default) matches MuJoCo's cotangent Laplacian (Wardetzky/Garg).
/// `Bridson` preserves the dihedral angle spring model (CortenForge extension).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum FlexBendingType {
    /// Wardetzky/Garg cotangent Laplacian (MuJoCo-conformant, default).
    #[default]
    Cotangent,
    /// Bridson dihedral angle springs (nonlinear, large-deformation accurate).
    Bridson,
}

/// Self-collision algorithm selection for flex bodies.
///
/// Matches MuJoCo's `mjtFlexSelf` enum. Controls how a flex's active
/// elements are tested against each other.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum FlexSelfCollide {
    /// No self-collision.
    None = 0,
    /// Each pair of active elements in turn.
    Narrow = 1,
    /// The flex's hierarchy against itself.
    Bvh = 2,
    /// Sweep and prune of the active elements' boxes along the longest axis
    /// of the hierarchy's root box.
    Sap = 3,
    /// Automatic: BVH for dim=3 (solids), SAP otherwise (shells/cables).
    #[default]
    Auto = 4,
}
