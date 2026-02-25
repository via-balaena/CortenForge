//! Enums and error types for the MuJoCo-aligned physics pipeline.
//!
//! This module defines the type-level vocabulary shared across all pipeline
//! stages: joint types, geometry types, solver types, constraint types,
//! sensor types, integrator selection, sleep policy, and error types.

use nalgebra::Vector3;

/// Element type for name↔index lookup via [`Model::name2id`] / [`Model::id2name`].
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
    /// - `CollisionShape::bounding_radius()` for runtime shapes
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
                // SdfCollisionData::aabb().
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
    /// User-defined bias (via `cb_act_bias` callback).
    /// MuJoCo reference: `mjBIAS_USER`.
    User,
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

    // ========== User-defined ==========
    /// User-defined sensor (arbitrary dimension).
    User,
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
            | Self::Rangefinder => 1,

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
            | Self::SubtreeAngMom => 3,

            Self::BallQuat | Self::FrameQuat => 4,

            Self::User => 0, // Variable, must be set explicitly
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
    /// Body.
    Body,
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
    /// Body 0 (world) or body with no DOFs. Always computed, never sleeps.
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
/// Skip island discovery → global solve.
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
    /// Projected Gauss-Seidel (default, matches MuJoCo).
    /// Dual-space solver: operates on constraint forces via the regularized
    /// Delassus matrix AR = J·M⁻¹·J^T + diag(R). Handles all constraint types
    /// with per-type projection (bilateral, box, unilateral, friction cone).
    #[default]
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

/// Errors that can occur during a simulation step.
///
/// Following Rust idioms, step() returns Result<(), StepError> instead of
/// silently correcting issues. Users must handle failures explicitly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StepError {
    /// Cholesky decomposition failed in implicit integration.
    /// This indicates the modified mass matrix (M + h*D + h²*K) is not positive definite,
    /// likely due to negative stiffness/damping or numerical instability.
    CholeskyFailed,
    /// LU decomposition failed (zero pivot in M − h·D).
    LuSingular,
    /// Timestep is zero or negative.
    InvalidTimestep,
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
            Self::InvalidTimestep => write!(f, "timestep is zero or negative"),
        }
    }
}

impl std::error::Error for StepError {}

/// Errors during Init-sleep validation (§16.24).
#[derive(Debug, Clone)]
pub enum SleepError {
    /// Init-sleep tree has no DOFs (cannot meaningfully sleep).
    InitSleepInvalidTree {
        /// The tree index that failed validation.
        tree: usize,
    },
    /// Statically-coupled tree group contains a mix of Init and non-Init
    /// trees. All trees connected by equality constraints or multi-tree
    /// tendons must have the same Init policy.
    InitSleepMixedIsland {
        /// The union-find representative tree for the mixed group.
        group_root: usize,
    },
}

impl std::fmt::Display for SleepError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InitSleepInvalidTree { tree } => {
                write!(f, "Init-sleep tree {tree} has no DOFs")
            }
            Self::InitSleepMixedIsland { group_root } => {
                write!(
                    f,
                    "mixed Init/non-Init trees in coupled group (root={group_root})"
                )
            }
        }
    }
}

impl std::error::Error for SleepError {}

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
        }
    }
}

impl std::error::Error for ResetError {}
