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
    #[must_use]
    pub fn bounding_radius(self, size: Vector3<f64>) -> f64 {
        match self {
            Self::Sphere => size.x,
            Self::Box => size.norm(), // Distance from center to corner
            Self::Capsule => size.x + size.y, // radius + half_length
            Self::Cylinder => size.x.hypot(size.y), // sqrt(r² + h²)
            Self::Ellipsoid => size.x.max(size.y).max(size.z), // Max semi-axis
            Self::Plane => f64::INFINITY, // Planes are infinite
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
            // POSITIVE: non-negative sensors (preserve negative sentinels)
            Self::Touch | Self::Rangefinder => MjSensorDataKind::Positive,

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
/// - `Positive` → `min(cutoff, value)` (preserves negative sentinels like -1.0)
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
/// Skip BVH midphase → brute-force broadphase.
pub const DISABLE_MIDPHASE: u32 = 1 << 14;
/// Skip implicit damping in Euler integrator.
pub const DISABLE_EULERDAMP: u32 = 1 << 15;
/// Skip auto-reset on NaN/divergence.
pub const DISABLE_AUTORESET: u32 = 1 << 16;
/// Fall back to libccd for convex collision.
pub const DISABLE_NATIVECCD: u32 = 1 << 17;
/// Skip island discovery (the constraint solve is global either way).
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
    /// iterations on stiff/weakly-coupled DOFs. Retained as the universal
    /// fallback for the CG and Newton solvers.
    PGS,
    /// Primal Polak-Ribiere conjugate gradient (matches MuJoCo's `mj_solCG`).
    /// Shares `mj_sol_primal` infrastructure with Newton: same constraint
    /// evaluation, line search, and cost function. Uses M⁻¹ preconditioner
    /// instead of Newton's H⁻¹, and PR direction instead of Newton direction.
    CG,
    /// Newton solver with analytical second-order derivatives (§15 of spec).
    /// Primal solver operating on accelerations with H⁻¹ preconditioner.
    /// Converges in 2-3 iterations vs PGS's 20+. Falls back to PGS
    /// on Cholesky failure or non-convergence.
    ///
    /// Default solver, matching MuJoCo (Newton has been MuJoCo's default since
    /// 2.0). The GPU pipeline is hardwired to Newton, so this keeps a
    /// default-configured model consistent across the CPU and GPU paths.
    #[default]
    Newton,
}

/// Constraint type annotation per row in the unified constraint system.
///
/// Each scalar row of the constraint Jacobian (`efc_J`) is tagged with one
/// of these types to determine its cost function and state machine behavior
/// in `PrimalUpdateConstraint` (§15.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ConstraintType {
    /// Equality constraint (connect, weld, joint, distance).
    Equality,
    /// DOF or tendon friction loss (Huber cost).
    FrictionLoss,
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
    /// Flex edge-length constraint (soft equality, matches MuJoCo's mjEQ_FLEX).
    FlexEdge,
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

/// Per-iteration Newton solver statistics, matching MuJoCo's `mjSolverStat`.
///
/// Populated by `newton_solve()` during each outer iteration. The array
/// `data.solver_stat` has length `data.solver_niter` after convergence.
#[derive(Debug, Clone, Copy, Default)]
pub struct SolverStat {
    /// Scaled cost improvement: `scale * (old_cost - new_cost)`.
    pub improvement: f64,
    /// Scaled gradient norm: `scale * ||grad||`.
    pub gradient: f64,
    /// Directional derivative along search direction at step start:
    /// `grad^T · search / ||search||`. Negative means descent.
    pub lineslope: f64,
    /// Number of active constraints (state == Quadratic or Cone).
    pub nactive: usize,
    /// Number of constraint state transitions this iteration.
    pub nchange: usize,
    /// Number of line search evaluations this iteration.
    pub nline: usize,
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
    /// Includes Coriolis velocity derivatives for maximum accuracy.
    Implicit,
    /// Fast implicit integration with symmetric D and Cholesky factorization.
    /// Skips Coriolis velocity derivatives for performance.
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
/// still reads it for an actuator whose gain has a velocity term. All as
/// MuJoCo.
///
/// [`Data::divergence_detected`]: crate::Data::divergence_detected
/// [`Warning::BadCtrl`]: crate::Warning::BadCtrl
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// Cholesky decomposition failed in implicit integration.
    /// This indicates the modified mass matrix (M + h*D + h²*K) is not positive definite,
    /// likely due to negative stiffness/damping or numerical instability.
    CholeskyFailed,
    /// LU decomposition failed (zero pivot in M − h·D).
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
}

impl std::fmt::Display for StepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::CholeskyFailed => {
                write!(f, "Cholesky decomposition failed in implicit integration")
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
    /// the same error, or an implicit factorization.
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
            | Self::InitSleep { .. } => None,
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
/// Matches MuJoCo's `mjtFlexSelf` enum. Controls how non-adjacent element
/// pairs are tested for collision within a single flex body.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum FlexSelfCollide {
    /// No self-collision (non-adjacent elements not tested).
    None = 0,
    /// Brute-force: test all non-adjacent element pairs O(n²).
    Narrow = 1,
    /// BVH midphase: per-element AABB tree for candidate pair pruning.
    Bvh = 2,
    /// Sweep-and-prune midphase: sort element AABBs along axis of max variance.
    Sap = 3,
    /// Automatic: BVH for dim=3 (solids), SAP otherwise (shells/cables).
    #[default]
    Auto = 4,
}
