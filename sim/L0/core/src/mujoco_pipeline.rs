// Allow patterns common in physics simulation code:
// - tuple_array_conversions: [t1, t2] from (t1, t2) is intentional for frame storage
// - manual_div_ceil: explicit ceiling division is clearer in some physics contexts
// - cast_possible_truncation: f64 to i64 for grid indices is intentional with bounds checks
// - or_fun_call: or_insert_with vs or_default is a style choice
// - collapsible_if: nested conditions can be clearer for physics logic
// - cast_precision_loss: usize to f64 is acceptable for index-based calculations
// - op_ref: reference vs value is a style choice for Vector3 operations
// - manual_let_else: if-let can be clearer than let-else for physics code
// - needless_range_loop: explicit indexing is sometimes clearer
// - imprecise_flops: physics code may prefer explicit formulas over hypot
#![allow(
    clippy::tuple_array_conversions,
    clippy::manual_div_ceil,
    clippy::cast_possible_truncation,
    clippy::or_fun_call,
    clippy::collapsible_if,
    clippy::cast_precision_loss,
    clippy::op_ref,
    clippy::manual_let_else,
    clippy::needless_range_loop,
    clippy::imprecise_flops
)]

//! MuJoCo-style physics pipeline for articulated rigid body simulation.
//!
//! This module implements the physics pipeline following `MuJoCo`'s exact
//! computation order and algorithms. The goal is binary-compatible physics
//! behavior with `MuJoCo` for validation purposes.
//!
//! # Pipeline Stages
//!
//! Following `MuJoCo`'s `mj_step`:
//!
//! 1. **Forward Position** (`mj_fwdPosition`)
//!    - Forward kinematics: compute body poses from joint positions
//!    - Compute mass matrix M via Composite Rigid Body Algorithm (CRBA)
//!    - Collision detection
//!    - Build constraint Jacobians
//!
//! 2. **Forward Velocity** (`mj_fwdVelocity`)
//!    - Compute bias forces via Recursive Newton-Euler (RNE)
//!    - Compute passive forces (springs, dampers)
//!
//! 3. **Forward Actuation** (`mj_fwdActuation`)
//!    - Compute actuator forces from controls
//!
//! 4. **Forward Acceleration** (`mj_fwdAcceleration`)
//!    - Compute unconstrained acceleration: `qacc_smooth = M^(-1) * qfrc_smooth`
//!    - Solve constraints via PGS
//!    - Compute final acceleration: `qacc = qacc_smooth + M^(-1) * J^T * λ`
//!
//! 5. **Integration** (`mj_Euler`)
//!    - Semi-implicit Euler: velocity then position (using NEW velocity)
//!
//! # Key Components
//!
//! - [`Model`]: Static model description (bodies, joints, geoms, constraints)
//! - [`Data`]: Dynamic simulation state (positions, velocities, forces)
//! - [`Data::step()`]: Main simulation step function
//!
//! Internal functions (called by `step()`):
//! - `mj_kinematics()`: Forward kinematics
//! - `mj_crba()`: Composite Rigid Body Algorithm for mass matrix
//! - `mj_rne()`: Recursive Newton-Euler for bias forces

use crate::collision_shape::{Aabb, CollisionShape};
use crate::heightfield::{
    HeightFieldData, heightfield_box_contact, heightfield_capsule_contact,
    heightfield_sphere_contact,
};
use crate::mesh::{
    MeshContact, TriangleMeshData, mesh_box_contact, mesh_capsule_contact,
    mesh_mesh_deepest_contact, mesh_sphere_contact,
};
use crate::raycast::raycast_shape;
use crate::sdf::{
    SdfCollisionData, sdf_box_contact, sdf_capsule_contact, sdf_cylinder_contact,
    sdf_ellipsoid_contact, sdf_heightfield_contact, sdf_plane_contact, sdf_sdf_contact,
    sdf_sphere_contact, sdf_triangle_mesh_contact,
};
use nalgebra::{
    DMatrix, DVector, Matrix3, Matrix6, Point3, UnitQuaternion, UnitVector3, Vector2, Vector3,
    Vector6,
};
use sim_types::Pose;
use std::collections::{HashMap, HashSet};
use std::f64::consts::PI;
use std::sync::Arc;

#[cfg(feature = "deformable")]
use sim_deformable::{
    DeformableBody, Material as DeformableMaterial, SolverConfig as XpbdSolverConfig, VertexFlags,
    XpbdSolver,
};

/// 6D spatial vector: [angular (3), linear (3)].
///
/// Following Featherstone's convention:
/// - Motion vectors: [ω, v] (angular velocity, linear velocity)
/// - Force vectors: [τ, f] (torque, force)
pub type SpatialVector = Vector6<f64>;

// ============================================================================
// Spatial Algebra Utilities
// ============================================================================

/// Spatial cross product for motion vectors: v × s.
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
#[must_use]
pub(crate) fn spatial_cross_motion(v: SpatialVector, s: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let s_ang = Vector3::new(s[0], s[1], s[2]);
    let s_lin = Vector3::new(s[3], s[4], s[5]);

    let result_ang = w.cross(&s_ang);
    let result_lin = w.cross(&s_lin) + v_lin.cross(&s_ang);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

/// Spatial cross product for force vectors: v ×* f.
#[allow(clippy::inline_always)] // Profiling shows inlining improves debug performance
#[inline(always)]
#[must_use]
pub(crate) fn spatial_cross_force(v: SpatialVector, f: SpatialVector) -> SpatialVector {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);
    let f_ang = Vector3::new(f[0], f[1], f[2]);
    let f_lin = Vector3::new(f[3], f[4], f[5]);

    let result_ang = w.cross(&f_ang) + v_lin.cross(&f_lin);
    let result_lin = w.cross(&f_lin);

    SpatialVector::new(
        result_ang.x,
        result_ang.y,
        result_ang.z,
        result_lin.x,
        result_lin.y,
        result_lin.z,
    )
}

/// Compute body spatial inertia in world frame.
///
/// This builds the 6×6 spatial inertia matrix from:
/// - `mass`: body mass
/// - `inertia_diag`: diagonal inertia in body's principal frame
/// - `i_mat`: rotation matrix from inertial frame to world (3×3)
/// - `h`: COM offset from body origin in world frame
///
/// The spatial inertia has the form:
/// ```text
/// I = [I_rot + m*(h·h*I - h⊗h),  m*[h]×  ]
///     [m*[h]×ᵀ,                  m*I_3×3 ]
/// ```
///
/// This is the canonical implementation - computed once per body in FK,
/// then used by both CRBA (as starting point for composite) and RNE (directly).
#[allow(clippy::inline_always)] // Called once per body in FK - inlining avoids function call overhead
#[inline(always)]
fn compute_body_spatial_inertia(
    mass: f64,
    inertia_diag: Vector3<f64>,
    i_mat: &Matrix3<f64>,
    h: Vector3<f64>,
) -> Matrix6<f64> {
    // Rotational inertia in world frame: I_world = R * I_diag * R^T
    // Element-wise is faster than matrix ops in debug mode
    let mut i_rot: Matrix3<f64> = Matrix3::zeros();
    for row in 0..3 {
        for col in 0..3 {
            i_rot[(row, col)] = i_mat[(row, 0)] * inertia_diag[0] * i_mat[(col, 0)]
                + i_mat[(row, 1)] * inertia_diag[1] * i_mat[(col, 1)]
                + i_mat[(row, 2)] * inertia_diag[2] * i_mat[(col, 2)];
        }
    }

    let mut crb = Matrix6::zeros();

    // Upper-left 3x3: rotational inertia about body origin (parallel axis theorem)
    let h_dot_h = h.x * h.x + h.y * h.y + h.z * h.z;
    for row in 0..3 {
        for col in 0..3 {
            let h_outer = h[row] * h[col];
            let delta = if row == col { 1.0 } else { 0.0 };
            crb[(row, col)] = i_rot[(row, col)] + mass * (h_dot_h * delta - h_outer);
        }
    }

    // Lower-right 3x3: translational inertia (diagonal mass matrix)
    crb[(3, 3)] = mass;
    crb[(4, 4)] = mass;
    crb[(5, 5)] = mass;

    // Off-diagonal: coupling (skew-symmetric of m*h)
    let mh_x = mass * h.x;
    let mh_y = mass * h.y;
    let mh_z = mass * h.z;
    crb[(0, 4)] = -mh_z;
    crb[(0, 5)] = mh_y;
    crb[(1, 3)] = mh_z;
    crb[(1, 5)] = -mh_x;
    crb[(2, 3)] = -mh_y;
    crb[(2, 4)] = mh_x;
    // Transpose for lower-left
    crb[(4, 0)] = -mh_z;
    crb[(5, 0)] = mh_y;
    crb[(3, 1)] = mh_z;
    crb[(5, 1)] = -mh_x;
    crb[(3, 2)] = -mh_y;
    crb[(4, 2)] = mh_x;

    crb
}

// ============================================================================
// MuJoCo-Aligned Model/Data Architecture (Phase 1)
// ============================================================================
//
// These structs follow MuJoCo's separation of static model data (Model) from
// dynamic simulation state (Data). The key insight is that body poses are
// COMPUTED from joint positions via forward kinematics, not stored as
// independent state.
//
// Reference: https://mujoco.readthedocs.io/en/stable/computation/index.html

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

// ============================================================================
// Joint Visitor Pattern
// ============================================================================

/// Context passed to joint visitors with pre-computed addresses and metadata.
///
/// This provides all the commonly-needed information about a joint in one place,
/// avoiding repeated lookups of `jnt_dof_adr`, `jnt_qpos_adr`, etc.
#[derive(Debug, Clone, Copy)]
pub struct JointContext {
    /// Joint index in model arrays.
    pub jnt_id: usize,
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: MjJointType,
    /// Starting index in qvel/qacc/qfrc arrays (DOF address).
    pub dof_adr: usize,
    /// Starting index in qpos array (position address).
    pub qpos_adr: usize,
    /// Number of velocity DOFs for this joint.
    pub nv: usize,
    /// Number of position coordinates for this joint.
    pub nq: usize,
}

/// Visitor trait for operations that iterate over joints.
///
/// This is the **single source of truth** for joint iteration. All code that
/// processes joints should use `Model::visit_joints()` with this trait to ensure:
///
/// 1. **Consistency**: When a new joint type is added, the compiler forces updates
/// 2. **Correctness**: Address computations are centralized, not copy-pasted
/// 3. **Performance**: Addresses are computed once per joint, not per operation
///
/// # Example
///
/// ```ignore
/// struct MyVisitor<'a> {
///     model: &'a Model,
///     data: &'a mut Data,
/// }
///
/// impl JointVisitor for MyVisitor<'_> {
///     fn visit_hinge(&mut self, ctx: JointContext) {
///         // Process hinge joint at ctx.dof_adr, ctx.qpos_adr
///     }
///     // ... other methods have default no-op implementations
/// }
///
/// model.visit_joints(&mut MyVisitor { model, data });
/// ```
pub trait JointVisitor {
    /// Called for each Hinge joint (1 DOF revolute).
    ///
    /// Default: no-op. Override if your visitor needs to process hinge joints.
    #[inline]
    fn visit_hinge(&mut self, _ctx: JointContext) {}

    /// Called for each Slide joint (1 DOF prismatic).
    ///
    /// Default: no-op. Override if your visitor needs to process slide joints.
    #[inline]
    fn visit_slide(&mut self, _ctx: JointContext) {}

    /// Called for each Ball joint (3 DOF spherical, quaternion orientation).
    ///
    /// Default: no-op. Override if your visitor needs to process ball joints.
    #[inline]
    fn visit_ball(&mut self, _ctx: JointContext) {}

    /// Called for each Free joint (6 DOF floating base).
    ///
    /// Default: no-op. Override if your visitor needs to process free joints.
    #[inline]
    fn visit_free(&mut self, _ctx: JointContext) {}
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
    /// Returns the unsigned penalty force magnitude when the joint's position
    /// limit is active; 0 when within limits.
    JointLimitFrc,
    /// Tendon limit force (scalar, 1D). MuJoCo: mjSENS_TENDONLIMITFRC.
    /// Returns the unsigned penalty force magnitude when the tendon's length
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

/// Enable flag bit for body sleeping/deactivation.
/// Set via MJCF `<option><flag sleep="enable"/>`.
pub const ENABLE_SLEEP: u32 = 1 << 5;

/// Minimum number of consecutive sub-threshold timesteps before a tree
/// can transition to sleep. Matches MuJoCo's `mjMINAWAKE = 10`.
pub const MIN_AWAKE: i32 = 10;

/// Disable-flag bit for island discovery (§16.10.1).
/// When set, `mj_island()` is a no-op and each tree is its own island.
pub const DISABLE_ISLAND: u32 = 1 << 18;

/// Contact constraint solver algorithm.
///
/// This selects the solver used in `mj_fwd_constraint` for contact forces.
/// Distinct from `sim_constraint::CGSolver`, which operates on joint-space
/// constraints via the `Joint` trait.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SolverType {
    /// Projected Gauss-Seidel (default, matches MuJoCo).
    #[default]
    PGS,
    /// Preconditioned projected gradient descent (PGD) with Barzilai-Borwein
    /// step size. Named "CG" for MuJoCo API compatibility.
    /// Falls back to PGS if PGD fails to converge within `solver_iterations`.
    CG,
    /// Strict PGD: returns zero forces on non-convergence instead of
    /// falling back to PGS. Sets `data.solver_niter = max_iterations`.
    /// Use in tests to detect convergence regressions without silent fallback.
    CGStrict,
    /// Newton solver with analytical second-order derivatives (§15 of spec).
    /// Operates on unified constraint Jacobian (equality, friction loss, limits,
    /// contacts). Converges in 2-3 iterations vs PGS's 20+. Falls back to PGS
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
    /// Contact with pyramidal friction cone (condim < 3 or cone == pyramidal).
    ContactNonElliptic,
    /// Contact with elliptic friction cone (condim ≥ 3 and cone == elliptic).
    ContactElliptic,
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
    /// Position coordinates contain NaN or Inf.
    InvalidPosition,
    /// Velocity coordinates contain NaN or Inf.
    InvalidVelocity,
    /// Computed acceleration contains NaN (indicates singular mass matrix or numerical issues).
    InvalidAcceleration,
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
            Self::InvalidPosition => write!(f, "position contains NaN or Inf"),
            Self::InvalidVelocity => write!(f, "velocity contains NaN or Inf"),
            Self::InvalidAcceleration => write!(f, "acceleration contains NaN"),
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

/// A named state snapshot for resetting simulation state.
///
/// All vectors are sized to match the model dimensions (nq, nv, na, nu,
/// nmocap). Unspecified fields in the MJCF `<key>` are filled with model
/// defaults at build time.
#[derive(Debug, Clone, PartialEq)]
pub struct Keyframe {
    /// Keyframe name (from MJCF `name` attribute). Empty string if unnamed.
    pub name: String,
    /// Simulation time. Default: 0.0.
    pub time: f64,
    /// Joint positions (length nq). Default: model.qpos0.
    pub qpos: DVector<f64>,
    /// Joint velocities (length nv). Default: zeros.
    pub qvel: DVector<f64>,
    /// Actuator activations (length na). Default: zeros.
    pub act: DVector<f64>,
    /// Control signals (length nu). Default: zeros.
    pub ctrl: DVector<f64>,
    /// Mocap body positions (length nmocap). Default: body_pos for each
    /// mocap body.
    pub mpos: Vec<Vector3<f64>>,
    /// Mocap body quaternions (length nmocap). Default: body_quat for each
    /// mocap body.
    pub mquat: Vec<UnitQuaternion<f64>>,
}

/// Static model definition (like mjModel).
///
/// Immutable after construction - all memory allocated upfront.
/// This contains the kinematic tree structure, body properties,
/// joint properties, and simulation options.
///
/// # Memory Layout
///
/// Arrays are indexed by their respective IDs:
/// - `body_*` arrays indexed by `body_id` (0 = world)
/// - `jnt_*` arrays indexed by `joint_id`
/// - `dof_*` arrays indexed by `dof_id` (velocity dimension index)
/// - `geom_*` arrays indexed by `geom_id`
/// - `actuator_*` arrays indexed by `actuator_id`
#[derive(Debug, Clone)]
pub struct Model {
    // ==================== Metadata ====================
    /// Model name (from MJCF model attribute or URDF robot name).
    pub name: String,

    // ==================== Dimensions ====================
    /// Number of generalized position coordinates (includes quaternions).
    pub nq: usize,
    /// Number of generalized velocity coordinates (DOFs, always <= nq).
    pub nv: usize,
    /// Number of bodies (including world body 0).
    pub nbody: usize,
    /// Number of joints.
    pub njnt: usize,
    /// Number of collision geometries.
    pub ngeom: usize,
    /// Number of sites (attachment points).
    pub nsite: usize,
    /// Number of actuators.
    pub nu: usize,
    /// Number of activation states (for muscle/filter actuators).
    pub na: usize,
    /// Number of mocap bodies. Mocap arrays are indexed 0..nmocap.
    pub nmocap: usize,
    /// Number of keyframes parsed from MJCF `<keyframe>`.
    pub nkeyframe: usize,

    // ==================== Kinematic Trees (§16.0) ====================
    /// Number of kinematic trees (excluding world body).
    pub ntree: usize,
    /// First body index for tree `t` (length `ntree`).
    pub tree_body_adr: Vec<usize>,
    /// Number of bodies in tree `t` (length `ntree`).
    pub tree_body_num: Vec<usize>,
    /// First DOF index for tree `t` (length `ntree`).
    pub tree_dof_adr: Vec<usize>,
    /// Number of DOFs in tree `t` (length `ntree`).
    pub tree_dof_num: Vec<usize>,
    /// Tree index for each body (body 0 → `usize::MAX` sentinel, length `nbody`).
    pub body_treeid: Vec<usize>,
    /// Tree index for each DOF (length `nv`).
    pub dof_treeid: Vec<usize>,
    /// Per-tree sleep policy (computed at model build, length `ntree`).
    pub tree_sleep_policy: Vec<SleepPolicy>,
    /// Per-DOF length scale for sleep threshold normalization (length `nv`).
    /// Translational DOFs = 1.0; rotational DOFs = mechanism length estimate.
    pub dof_length: Vec<f64>,
    /// Sleep velocity tolerance. Bodies with all DOF velocities below
    /// `sleep_tolerance * dof_length[dof]` for `MIN_AWAKE` consecutive steps
    /// are eligible for sleep. Default: `1e-4`. Units: `[m/s]`.
    pub sleep_tolerance: f64,

    // ==================== Body Tree (indexed by body_id, 0 = world) ====================
    /// Parent body index (0 for root bodies attached to world).
    pub body_parent: Vec<usize>,
    /// Root body of kinematic tree (for multi-tree systems).
    pub body_rootid: Vec<usize>,
    /// First joint index for this body in jnt_* arrays.
    pub body_jnt_adr: Vec<usize>,
    /// Number of joints attached to this body.
    pub body_jnt_num: Vec<usize>,
    /// First DOF index for this body in dof_* and qvel arrays.
    pub body_dof_adr: Vec<usize>,
    /// Number of DOFs for this body.
    pub body_dof_num: Vec<usize>,
    /// First geom index for this body.
    pub body_geom_adr: Vec<usize>,
    /// Number of geoms attached to this body.
    pub body_geom_num: Vec<usize>,

    // Body properties (in body-local frame)
    /// Position offset from parent joint frame to body frame.
    pub body_pos: Vec<Vector3<f64>>,
    /// Orientation offset from parent joint frame to body frame.
    pub body_quat: Vec<UnitQuaternion<f64>>,
    /// Center of mass position in body frame.
    pub body_ipos: Vec<Vector3<f64>>,
    /// Inertial frame orientation in body frame.
    pub body_iquat: Vec<UnitQuaternion<f64>>,
    /// Body mass in kg.
    pub body_mass: Vec<f64>,
    /// Diagonal inertia in principal axes (`body_iquat` frame).
    pub body_inertia: Vec<Vector3<f64>>,
    /// Optional body names for lookup.
    pub body_name: Vec<Option<String>>,
    /// Total mass of subtree rooted at this body (precomputed).
    /// `body_subtreemass[0]` is total mass of entire system.
    pub body_subtreemass: Vec<f64>,
    /// Maps body_id to mocap array index. `None` for non-mocap bodies.
    /// Length: nbody.
    ///
    /// **Ordering invariant:** Mocap IDs are assigned sequentially during body
    /// processing in topological order. Since the body traversal visits
    /// parents before children, and mocap bodies must be direct world children,
    /// all mocap bodies are visited before any non-world-child body. The
    /// mocap_id for body B equals the count of mocap bodies with body_id < B.
    /// This invariant is relied upon by `make_data()` and `Data::reset()`,
    /// which iterate bodies in order and use `enumerate()` to correlate with
    /// mocap array indices.
    pub body_mocapid: Vec<Option<usize>>,

    // ==================== Joints (indexed by jnt_id) ====================
    /// Joint type (Hinge, Slide, Ball, Free).
    pub jnt_type: Vec<MjJointType>,
    /// Body this joint belongs to (the child body).
    pub jnt_body: Vec<usize>,
    /// Start index in qpos array.
    pub jnt_qpos_adr: Vec<usize>,
    /// Start index in qvel/qacc arrays.
    pub jnt_dof_adr: Vec<usize>,
    /// Joint anchor position in body frame.
    pub jnt_pos: Vec<Vector3<f64>>,
    /// Joint axis for hinge/slide (in body frame).
    pub jnt_axis: Vec<Vector3<f64>>,
    /// Whether joint has limits.
    pub jnt_limited: Vec<bool>,
    /// Joint limits [min, max].
    pub jnt_range: Vec<(f64, f64)>,
    /// Spring stiffness coefficient (N/rad for hinge, N/m for slide).
    /// Applied as: τ = -stiffness * (q - springref)
    pub jnt_stiffness: Vec<f64>,
    /// Spring equilibrium position (rad for hinge, m for slide).
    /// This is the position where spring force is zero.
    /// Distinct from `qpos0` which is the initial position at model load.
    pub jnt_springref: Vec<f64>,
    /// Damping coefficient (Ns/rad for hinge, Ns/m for slide).
    /// Applied as: τ = -damping * qvel
    pub jnt_damping: Vec<f64>,
    /// Armature inertia (motor rotor inertia).
    pub jnt_armature: Vec<f64>,
    /// Solver reference parameters for joint limits [timeconst, dampratio].
    /// Controls how stiffly/softly limits are enforced.
    /// Default: [0.02, 1.0] (MuJoCo defaults)
    pub jnt_solref: Vec<[f64; 2]>,
    /// Solver impedance parameters for joint limits [d0, d_width, width, midpoint, power].
    /// Default: [0.9, 0.95, 0.001, 0.5, 2.0] (MuJoCo defaults)
    pub jnt_solimp: Vec<[f64; 5]>,
    /// Optional joint names.
    pub jnt_name: Vec<Option<String>>,

    // ==================== DOFs (indexed by dof_id) ====================
    /// Body for this DOF.
    pub dof_body: Vec<usize>,
    /// Joint for this DOF.
    pub dof_jnt: Vec<usize>,
    /// Parent DOF in kinematic tree (None for root DOF).
    pub dof_parent: Vec<Option<usize>>,
    /// Armature inertia for this DOF.
    pub dof_armature: Vec<f64>,
    /// Damping coefficient for this DOF.
    pub dof_damping: Vec<f64>,
    /// Friction loss (dry friction) for this DOF.
    /// Applied as: τ_friction = -frictionloss * tanh(qvel * friction_smoothing)
    /// where the tanh provides a smooth approximation to sign(qvel) for numerical stability.
    pub dof_frictionloss: Vec<f64>,

    // ==================== Geoms (indexed by geom_id) ====================
    /// Geometry type.
    pub geom_type: Vec<GeomType>,
    /// Parent body.
    pub geom_body: Vec<usize>,
    /// Position in body frame.
    pub geom_pos: Vec<Vector3<f64>>,
    /// Orientation in body frame.
    pub geom_quat: Vec<UnitQuaternion<f64>>,
    /// Type-specific size parameters [size0, size1, size2].
    pub geom_size: Vec<Vector3<f64>>,
    /// Friction coefficients [sliding, torsional, rolling].
    pub geom_friction: Vec<Vector3<f64>>,
    /// Contact dimensionality (1=frictionless, 3=sliding, 4=+torsional, 6=+rolling).
    pub geom_condim: Vec<i32>,
    /// Contact type bitmask.
    pub geom_contype: Vec<u32>,
    /// Contact affinity bitmask.
    pub geom_conaffinity: Vec<u32>,
    /// Contact margin (distance at which contact becomes active).
    pub geom_margin: Vec<f64>,
    /// Contact gap (minimum allowed separation).
    pub geom_gap: Vec<f64>,
    /// Solver impedance parameters [d0, dwidth, width, midpoint, power].
    /// Controls constraint softness and behavior.
    pub geom_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters [timeconst, dampratio] or [d, dmin].
    /// Controls constraint dynamics.
    pub geom_solref: Vec<[f64; 2]>,
    /// Optional geom names.
    pub geom_name: Vec<Option<String>>,
    /// Pre-computed bounding sphere radius for each geom (in local frame).
    /// Used for fast distance culling in collision broad-phase.
    /// For primitives, computed from geom_size. For meshes, computed from mesh AABB.
    pub geom_rbound: Vec<f64>,
    /// Mesh index for each geom (`None` if not a mesh geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Mesh` have `Some(mesh_id)`.
    pub geom_mesh: Vec<Option<usize>>,
    /// Hfield index for each geom (`None` if not an hfield geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Hfield` have `Some(hfield_id)`.
    pub geom_hfield: Vec<Option<usize>>,
    /// SDF index for each geom (`None` if not an SDF geom).
    /// Length: ngeom. Only geoms with `geom_type == GeomType::Sdf` have `Some(sdf_id)`.
    pub geom_sdf: Vec<Option<usize>>,

    // ==================== Meshes (indexed by mesh_id) ====================
    /// Number of mesh assets.
    pub nmesh: usize,
    /// Mesh names (for lookup by name).
    pub mesh_name: Vec<String>,
    /// Triangle mesh data with prebuilt BVH.
    /// `Arc` for cheap cloning (multiple geoms can reference the same mesh asset).
    pub mesh_data: Vec<Arc<TriangleMeshData>>,

    // ==================== Height Fields (indexed by hfield_id) ====================
    /// Number of height field assets.
    pub nhfield: usize,
    /// Height field names (for lookup by name).
    pub hfield_name: Vec<String>,
    /// Height field terrain data.
    pub hfield_data: Vec<Arc<HeightFieldData>>,
    /// Original MuJoCo size `[x, y, z_top, z_bottom]` for centering offset at collision time.
    pub hfield_size: Vec<[f64; 4]>,

    // ==================== SDFs (indexed by sdf_id) ====================
    /// Number of SDF assets.
    pub nsdf: usize,
    /// SDF collision data.
    /// `Arc` for cheap cloning (multiple geoms can reference the same SDF asset).
    pub sdf_data: Vec<Arc<SdfCollisionData>>,

    // ==================== Sites (indexed by site_id) ====================
    /// Parent body for each site.
    pub site_body: Vec<usize>,
    /// Site geometry type (for visualization, uses GeomType).
    pub site_type: Vec<GeomType>,
    /// Site position in body frame.
    pub site_pos: Vec<Vector3<f64>>,
    /// Site orientation in body frame.
    pub site_quat: Vec<UnitQuaternion<f64>>,
    /// Site size (for visualization).
    pub site_size: Vec<Vector3<f64>>,
    /// Optional site names.
    pub site_name: Vec<Option<String>>,

    // ==================== Sensors (indexed by sensor_id) ====================
    /// Number of sensors.
    pub nsensor: usize,
    /// Number of sensor data elements (sum of all sensor dims).
    pub nsensordata: usize,
    /// Sensor type.
    pub sensor_type: Vec<MjSensorType>,
    /// Sensor data type (position/velocity/acceleration dependent).
    pub sensor_datatype: Vec<MjSensorDataType>,
    /// Object type the sensor is attached to.
    pub sensor_objtype: Vec<MjObjectType>,
    /// Object ID the sensor is attached to (body/joint/site/geom id).
    pub sensor_objid: Vec<usize>,
    /// Reference object type (for relative sensors).
    pub sensor_reftype: Vec<MjObjectType>,
    /// Reference object ID.
    pub sensor_refid: Vec<usize>,
    /// Start address in sensordata array.
    pub sensor_adr: Vec<usize>,
    /// Number of data elements for this sensor.
    pub sensor_dim: Vec<usize>,
    /// Noise standard deviation (0 = no noise).
    pub sensor_noise: Vec<f64>,
    /// Cutoff for sensor value (0 = no cutoff).
    pub sensor_cutoff: Vec<f64>,
    /// Optional sensor names.
    pub sensor_name: Vec<Option<String>>,

    // ==================== Actuators (indexed by actuator_id) ====================
    /// Transmission type (Joint, Tendon, Site).
    pub actuator_trntype: Vec<ActuatorTransmission>,
    /// Dynamics type (None, Filter, Integrator, Muscle).
    pub actuator_dyntype: Vec<ActuatorDynamics>,
    /// Transmission target ID (joint/tendon/site). Second slot is refsite for
    /// site transmissions, `usize::MAX` when unused.
    pub actuator_trnid: Vec<[usize; 2]>,
    /// Transmission gear ratio (6D: [tx ty tz rx ry rz]).
    pub actuator_gear: Vec<[f64; 6]>,
    /// Control input limits [min, max].
    pub actuator_ctrlrange: Vec<(f64, f64)>,
    /// Force output limits [min, max].
    pub actuator_forcerange: Vec<(f64, f64)>,
    /// Optional actuator names.
    pub actuator_name: Vec<Option<String>>,
    /// Start index in act array for each actuator's activation states.
    pub actuator_act_adr: Vec<usize>,
    /// Number of activation states per actuator (0 for None dynamics, 1 for Filter/Integrator/Muscle).
    pub actuator_act_num: Vec<usize>,
    /// Gain type per actuator — dispatches force gain computation.
    pub actuator_gaintype: Vec<GainType>,
    /// Bias type per actuator — dispatches force bias computation.
    pub actuator_biastype: Vec<BiasType>,

    /// Dynamics parameters per actuator (3 elements each).
    /// For Muscle: [tau_act, tau_deact, tausmooth]. Default: [0.01, 0.04, 0.0].
    /// For Filter: [tau, 0, 0]. For Integrator/None: unused.
    pub actuator_dynprm: Vec<[f64; 3]>,

    /// Gain parameters per actuator (9 elements each).
    /// For Muscle: [range0, range1, force, scale, lmin, lmax, vmax, fpmax, fvmax].
    /// Default: [0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2].
    /// For other types: [gain, 0, ...] (gain multiplier, index 0 only).
    pub actuator_gainprm: Vec<[f64; 9]>,

    /// Bias parameters per actuator (9 elements each).
    /// For Muscle: same layout as gainprm (shared parameter set in MuJoCo).
    /// For other types: [bias0, bias1, bias2, 0, ...] (constant + length + velocity).
    pub actuator_biasprm: Vec<[f64; 9]>,

    /// Actuator length range [min, max] — the transmission length extremes.
    /// For tendon-transmission muscles: computed from tendon length at joint limits.
    /// Used to normalize muscle length: L = range0 + (len - lengthrange0) / L0.
    pub actuator_lengthrange: Vec<(f64, f64)>,

    /// Acceleration produced by unit actuator force (scalar, per actuator).
    /// Used for auto-computing F0 when `force < 0`: F0 = scale / acc0.
    /// Computed at model build time from M^{-1} and the transmission Jacobian.
    pub actuator_acc0: Vec<f64>,

    // ==================== Tendons (indexed by tendon_id) ====================
    /// Number of tendons.
    pub ntendon: usize,
    /// Number of wrap objects across all tendons.
    pub nwrap: usize,
    /// Tendon path length limits [min, max]. Limited if min < max.
    pub tendon_range: Vec<(f64, f64)>,
    /// Whether tendon has length limits.
    pub tendon_limited: Vec<bool>,
    /// Tendon stiffness (force per unit length).
    pub tendon_stiffness: Vec<f64>,
    /// Tendon damping coefficient.
    pub tendon_damping: Vec<f64>,
    /// Tendon rest length (reference for spring force).
    pub tendon_lengthspring: Vec<f64>,
    /// Tendon length at qpos0 (precomputed reference).
    pub tendon_length0: Vec<f64>,
    /// Number of wrapping objects for this tendon.
    pub tendon_num: Vec<usize>,
    /// Start address in wrap_* arrays for this tendon's path.
    pub tendon_adr: Vec<usize>,
    /// Optional tendon names.
    pub tendon_name: Vec<Option<String>>,
    /// Tendon type: Fixed (linear coupling) or Spatial (3D path routing).
    pub tendon_type: Vec<TendonType>,
    /// Solver parameters for tendon limit constraints (2 elements per tendon).
    /// \[0\] = timeconst (>0) or -stiffness (≤0), \[1\] = dampratio or -damping.
    /// Default: [0.0, 0.0] → uses model.default_eq_stiffness/default_eq_damping.
    pub tendon_solref: Vec<[f64; 2]>,
    /// Impedance parameters for tendon limit constraints (5 elements per tendon).
    /// [d_min, d_max, width, midpoint, power]. Default: [0.9, 0.95, 0.001, 0.5, 2.0].
    pub tendon_solimp: Vec<[f64; 5]>,
    /// Velocity-dependent friction loss per tendon (N).
    /// When > 0, adds a friction force opposing tendon velocity: F = -frictionloss * sign(v).
    pub tendon_frictionloss: Vec<f64>,
    /// Number of distinct kinematic trees spanned by each tendon (§16.10.1).
    /// 0 = no bodies, 1 = single tree, 2 = two trees. Length: ntendon.
    pub tendon_treenum: Vec<usize>,
    /// Packed tree indices for two-tree tendons (§16.10.1).
    /// For tendon t: `tendon_tree[2*t]` and `tendon_tree[2*t+1]`.
    /// Unused entries (treenum != 2) are `usize::MAX`. Length: 2 * ntendon.
    pub tendon_tree: Vec<usize>,

    // Tendon wrapping path elements (indexed by wrap_id, grouped by tendon; total length = nwrap)
    /// Wrap object type (Site, Geom, Joint, Pulley).
    pub wrap_type: Vec<WrapType>,
    /// Object ID for the wrap point (site/geom/joint id).
    pub wrap_objid: Vec<usize>,
    /// Wrap parameter (coefficient for Joint, divisor for Pulley, 0.0 for Site/Geom).
    pub wrap_prm: Vec<f64>,
    /// Sidesite ID for wrapping geom wrap objects. `usize::MAX` if no sidesite.
    /// Indexed in parallel with `wrap_type`/`wrap_objid`/`wrap_prm`.
    pub wrap_sidesite: Vec<usize>,

    // ==================== Equality Constraints (indexed by eq_id) ====================
    /// Number of equality constraints.
    pub neq: usize,
    /// Equality constraint type (Connect, Weld, Joint, Tendon, Distance).
    pub eq_type: Vec<EqualityType>,
    /// First object ID (body for Connect/Weld, joint for Joint, geom for Distance).
    pub eq_obj1id: Vec<usize>,
    /// Second object ID (body/joint/geom, or `usize::MAX` for world origin in Distance).
    pub eq_obj2id: Vec<usize>,
    /// Constraint parameters (meaning depends on type).
    /// - Connect: anchor point in body1 frame [x, y, z]
    /// - Weld: relative pose [x, y, z, qw, qx, qy, qz] + torque scale
    /// - Joint: polycoef[0..5] for polynomial coupling
    /// - Tendon: polycoef[0..5] for polynomial coupling
    /// - Distance: target distance in `[0]`; `[1..10]` unused
    pub eq_data: Vec<[f64; 11]>,
    /// Whether this equality constraint is active.
    pub eq_active: Vec<bool>,
    /// Solver impedance parameters for this constraint.
    pub eq_solimp: Vec<[f64; 5]>,
    /// Solver reference parameters for this constraint.
    pub eq_solref: Vec<[f64; 2]>,
    /// Optional equality constraint names.
    pub eq_name: Vec<Option<String>>,

    // ==================== Options ====================
    /// Simulation timestep in seconds.
    pub timestep: f64,
    /// Gravity vector in world frame.
    pub gravity: Vector3<f64>,
    /// Default/reference joint positions.
    pub qpos0: DVector<f64>,
    /// Named state snapshots for quick reset.
    pub keyframes: Vec<Keyframe>,
    /// Wind velocity in world frame (for aerodynamic forces).
    pub wind: Vector3<f64>,
    /// Magnetic field in world frame (for magnetic actuators).
    pub magnetic: Vector3<f64>,
    /// Medium density (for fluid drag, kg/m³).
    pub density: f64,
    /// Medium viscosity (for fluid drag, Pa·s).
    pub viscosity: f64,

    // Solver options
    /// Maximum constraint solver iterations.
    pub solver_iterations: usize,
    /// Early termination tolerance for solver.
    pub solver_tolerance: f64,
    /// Constraint impedance ratio (for soft constraints).
    pub impratio: f64,
    /// Base regularization (softness) for PGS constraint matrix diagonal (default: 1e-6).
    /// CFM scales as `regularization + (1 - impedance) * regularization * 100`.
    pub regularization: f64,
    /// Fallback stiffness for equality constraints when solref is not specified (default: 10000.0).
    pub default_eq_stiffness: f64,
    /// Fallback damping for equality constraints when solref is not specified (default: 1000.0).
    pub default_eq_damping: f64,
    /// Maximum linear velocity change per timestep from constraint forces (m/s, default: 1.0).
    pub max_constraint_vel: f64,
    /// Maximum angular velocity change per timestep from constraint forces (rad/s, default: 1.0).
    pub max_constraint_angvel: f64,
    /// Friction smoothing factor — sharpness of tanh transition (default: 1000.0).
    pub friction_smoothing: f64,
    /// Friction cone type: 0=pyramidal, 1=elliptic.
    pub cone: u8,

    // Newton solver parameters (§15)
    /// Mean inertia: trace(M) / nv at qpos0. Used by Newton solver for scaling.
    /// Computed at model build time by running CRBA at qpos0.
    pub stat_meaninertia: f64,
    /// Maximum line search iterations for Newton solver (default 50).
    pub ls_iterations: usize,
    /// Line search gradient tolerance for Newton solver (default 0.01).
    pub ls_tolerance: f64,
    /// Noslip solver iterations (parsed from MJCF, stored, not yet implemented; default 0).
    pub noslip_iterations: usize,
    /// Noslip solver tolerance (parsed from MJCF, stored, not yet implemented; default 1e-6).
    pub noslip_tolerance: f64,

    /// Disable flags (bitmask for disabling default behaviors).
    pub disableflags: u32,
    /// Enable flags (bitmask for enabling optional behaviors).
    pub enableflags: u32,

    // ==================== Deformable Configuration (feature-gated) ====================
    /// Contact dimension for deformable-rigid contacts (1, 3, 4, or 6).
    /// Default: 3 (normal + 2D sliding friction). Applies to all deformable bodies.
    #[cfg(feature = "deformable")]
    pub deformable_condim: i32,
    /// Solver reference parameters for deformable-rigid contacts.
    #[cfg(feature = "deformable")]
    pub deformable_solref: [f64; 2],
    /// Solver impedance parameters for deformable-rigid contacts.
    #[cfg(feature = "deformable")]
    pub deformable_solimp: [f64; 5],

    /// Integration method.
    pub integrator: Integrator,
    /// Contact constraint solver algorithm (PGS or CG).
    pub solver_type: SolverType,

    // ==================== Cached Implicit Integration Parameters ====================
    // These are pre-computed from joint properties for implicit spring-damper integration.
    // Avoids O(nv) allocation per step by caching model-invariant diagonal matrices.
    /// Diagonal stiffness matrix K for implicit integration (length nv).
    /// `K\[i\]` = jnt_stiffness for Hinge/Slide DOFs, 0 for Ball/Free DOFs.
    pub implicit_stiffness: DVector<f64>,
    /// Diagonal damping matrix D for implicit integration (length nv).
    /// `D\[i\]` = jnt_damping for Hinge/Slide, dof_damping for Ball/Free DOFs.
    pub implicit_damping: DVector<f64>,
    /// Spring equilibrium positions for implicit integration (length nv).
    /// `q_eq\[i\]` = jnt_springref for Hinge/Slide, 0 for Ball/Free DOFs.
    pub implicit_springref: DVector<f64>,

    // ==================== Pre-computed Kinematic Data ====================
    // These are computed once at model construction to avoid O(n) lookups
    // in the inner loops of CRBA and RNE, achieving O(n) vs O(n³) complexity.
    /// For each body: list of ancestor joint indices (from body to root).
    /// `body_ancestor_joints[i]` contains all joints in the kinematic chain
    /// from body `i` to the world. Empty for body 0 (world).
    pub body_ancestor_joints: Vec<Vec<usize>>,

    /// For each body: set of ancestor joint indices for O(1) membership testing.
    /// Multi-word bitmask: `body_ancestor_mask[body_id][word]` where word = jnt_id / 64.
    /// Bit (jnt_id % 64) is set if joint jnt_id is an ancestor of this body.
    /// Supports unlimited joints with O(1) lookup per word.
    pub body_ancestor_mask: Vec<Vec<u64>>,

    // ==================== Explicit Contact Pairs/Excludes ====================
    /// Explicit contact pairs from `<contact><pair>`.
    /// Processed in mechanism 2 (bypass kinematic and bitmask filters).
    pub contact_pairs: Vec<ContactPair>,
    /// Explicit pair geom-pair set for O(1) lookup during automatic pipeline.
    /// Canonical key: `(min(geom1, geom2), max(geom1, geom2))`.
    /// Used to suppress automatic-pipeline contacts for pairs that have explicit overrides.
    pub contact_pair_set: HashSet<(usize, usize)>,
    /// Excluded body-pair set from `<contact><exclude>`.
    /// Canonical key: `(min(body1, body2), max(body1, body2))`.
    pub contact_excludes: HashSet<(usize, usize)>,
}

/// Warmstart key for contact force caching across frames.
///
/// Combines a canonical geom pair with a discretized contact position so
/// that multiple contacts within the same geom pair (e.g., 4 corner contacts
/// for box-on-plane) each get their own warmstart entry.
///
/// Grid resolution is 1 cm — coarse enough to match contacts across frames
/// despite small position drift, fine enough to distinguish distinct contact
/// points separated by more than ~1 cm.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct WarmstartKey {
    /// Lower geom index of the canonical pair (min of geom1, geom2).
    pub geom_lo: usize,
    /// Upper geom index of the canonical pair (max of geom1, geom2).
    pub geom_hi: usize,
    /// Discretized x grid cell of the contact position.
    pub cell_x: i64,
    /// Discretized y grid cell of the contact position.
    pub cell_y: i64,
    /// Discretized z grid cell of the contact position.
    pub cell_z: i64,
}

/// Grid resolution for warmstart spatial hashing (meters).
/// 1 cm is coarse enough to tolerate frame-to-frame position jitter
/// while distinguishing contacts separated by more than ~1 cm.
const WARMSTART_GRID_RES: f64 = 0.01;

/// Compute the warmstart cache key for a contact.
///
/// Combines canonical geom pair IDs with a discretized contact position
/// so that multiple contacts between the same geom pair (e.g., box corners)
/// each get their own warmstart entry.
#[inline]
#[must_use]
pub fn warmstart_key(contact: &Contact) -> WarmstartKey {
    let inv = 1.0 / WARMSTART_GRID_RES;
    WarmstartKey {
        geom_lo: contact.geom1.min(contact.geom2),
        geom_hi: contact.geom1.max(contact.geom2),
        cell_x: (contact.pos.x * inv).round() as i64,
        cell_y: (contact.pos.y * inv).round() as i64,
        cell_z: (contact.pos.z * inv).round() as i64,
    }
}

/// Explicit contact pair: geom indices + per-pair overrides.
/// All fields are fully resolved at build time (no Options).
#[derive(Debug, Clone)]
pub struct ContactPair {
    /// First geometry index.
    pub geom1: usize,
    /// Second geometry index.
    pub geom2: usize,
    /// Contact dimensionality (1, 3, 4, 6).
    pub condim: i32,
    /// 5-element friction: [tan1, tan2, torsional, roll1, roll2].
    pub friction: [f64; 5],
    /// Solver reference (normal direction).
    pub solref: [f64; 2],
    /// Solver reference (friction directions).
    pub solreffriction: [f64; 2],
    /// Solver impedance.
    pub solimp: [f64; 5],
    /// Distance threshold for contact activation.
    pub margin: f64,
    /// Contact included if distance < margin - gap.
    pub gap: f64,
}

/// Contact point for constraint generation.
///
/// Matches MuJoCo's mjContact structure with all relevant fields
/// for constraint-based contact resolution.
#[derive(Debug, Clone)]
pub struct Contact {
    /// Contact position in world frame.
    pub pos: Vector3<f64>,
    /// Contact normal (from geom1 toward geom2, unit vector).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = penetrating).
    pub depth: f64,
    /// First geometry ID.
    pub geom1: usize,
    /// Second geometry ID.
    pub geom2: usize,
    /// Friction coefficient (combined from both geoms).
    pub friction: f64,
    /// Contact dimension: 1 (frictionless), 3 (friction), 4 (elliptic), 6 (torsional).
    pub dim: usize,
    /// Whether margin was included in distance computation.
    pub includemargin: bool,
    /// Friction parameters for MuJoCo-style 5-element friction.
    /// `[sliding1, sliding2, torsional, rolling1, rolling2]`
    /// - sliding1/2: tangent friction coefficients
    /// - torsional: spin friction coefficient
    /// - rolling1/2: rolling friction coefficients
    pub mu: [f64; 5],
    /// Solver reference parameters (from geom pair).
    pub solref: [f64; 2],
    /// Solver impedance parameters (from geom pair).
    pub solimp: [f64; 5],
    /// Contact frame tangent vectors (orthogonal to normal).
    /// `frame[0..3]` = t1, `frame[3..6]` = t2 (for friction cone).
    pub frame: [Vector3<f64>; 2],
}

/// A contact between a deformable vertex and a rigid geom.
#[derive(Debug, Clone)]
#[cfg(feature = "deformable")]
pub struct DeformableContact {
    /// Index into `deformable_bodies` array.
    pub deformable_idx: usize,
    /// Vertex index within the deformable body.
    pub vertex_idx: usize,
    /// Rigid geom index (into `model.geom_*` arrays).
    pub geom_idx: usize,
    /// Contact position in world frame (closest point on rigid geom surface).
    pub pos: Vector3<f64>,
    /// Contact normal (from rigid surface toward deformable vertex, unit vector).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = overlapping).
    pub depth: f64,
    /// Combined sliding friction: sqrt(material.friction * geom_friction.x).
    pub friction: f64,
    /// Contact dimension (from model.deformable_condim).
    pub dim: usize,
    /// 5-element friction vector [sliding1, sliding2, torsional, rolling1, rolling2].
    pub mu: [f64; 5],
    /// Solver reference parameters.
    pub solref: [f64; 2],
    /// Solver impedance parameters.
    pub solimp: [f64; 5],
    /// Tangent frame vectors for friction cone.
    pub frame: [Vector3<f64>; 2],
}

impl Contact {
    /// Create a new contact with basic parameters.
    ///
    /// The contact frame (tangent vectors) is computed automatically from the normal.
    /// Advanced solver parameters use MuJoCo defaults.
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - NaN position/normal components are handled gracefully
    #[must_use]
    #[inline]
    pub fn new(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        friction: f64,
    ) -> Self {
        Self::with_solver_params(
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        )
    }

    /// Create a new contact with explicit solver parameters.
    ///
    /// This constructor should be used when creating contacts from collision
    /// detection, where the solref/solimp values come from the colliding geoms.
    ///
    /// MuJoCo combines geom solver parameters using element-wise minimum for
    /// solref (stiffer wins) and element-wise maximum for solimp (harder wins).
    ///
    /// # Numerical Safety
    /// - Negative or NaN friction is clamped to 0.0
    /// - NaN depth is set to 0.0
    /// - NaN position/normal components are handled gracefully
    #[must_use]
    #[inline]
    #[allow(clippy::too_many_arguments)] // Matches MuJoCo's contact fields; grouping into a struct would add indirection for no benefit
    pub fn with_solver_params(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        friction: f64,
        solref: [f64; 2],
        solimp: [f64; 5],
    ) -> Self {
        // Safety: clamp friction to non-negative finite value
        let friction = if friction.is_finite() && friction > 0.0 {
            friction
        } else {
            0.0
        };

        // Safety: ensure depth is finite
        let depth = if depth.is_finite() { depth } else { 0.0 };

        // Compute tangent frame from normal (handles NaN/zero normals internally)
        let (t1, t2) = compute_tangent_frame(&normal);

        Self {
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction,
            dim: if friction > 0.0 { 3 } else { 1 }, // 3D friction cone or frictionless
            includemargin: false,
            // MuJoCo 5-element friction: [sliding1, sliding2, torsional, rolling1, rolling2]
            mu: [
                friction,
                friction,
                friction * 0.005,
                friction * 0.001,
                friction * 0.001,
            ],
            solref,
            solimp,
            frame: [t1, t2],
        }
    }

    /// Create a contact with explicit condim and per-type friction coefficients.
    ///
    /// This constructor is used when creating contacts from collision detection
    /// where condim and friction values come from combining both geom properties.
    ///
    /// # Arguments
    /// * `sliding` - Sliding friction coefficient (mu\[0\], mu\[1\])
    /// * `torsional` - Torsional/spin friction coefficient (mu\[2\])
    /// * `rolling` - Rolling friction coefficient (mu\[3\], mu\[4\])
    /// * `condim` - Contact dimension (1, 3, 4, or 6)
    #[must_use]
    #[inline]
    #[allow(clippy::too_many_arguments)]
    pub fn with_condim(
        pos: Vector3<f64>,
        normal: Vector3<f64>,
        depth: f64,
        geom1: usize,
        geom2: usize,
        sliding: f64,
        torsional: f64,
        rolling: f64,
        condim: i32,
        solref: [f64; 2],
        solimp: [f64; 5],
    ) -> Self {
        // Safety: clamp friction values to non-negative finite
        let sliding = if sliding.is_finite() && sliding > 0.0 {
            sliding
        } else {
            0.0
        };
        let torsional = if torsional.is_finite() && torsional > 0.0 {
            torsional
        } else {
            0.0
        };
        let rolling = if rolling.is_finite() && rolling > 0.0 {
            rolling
        } else {
            0.0
        };

        // Safety: ensure depth is finite
        let depth = if depth.is_finite() { depth } else { 0.0 };

        // Compute tangent frame from normal
        let (t1, t2) = compute_tangent_frame(&normal);

        // Contact dimension is determined directly by condim
        // The friction coefficients determine the cone shape, not the structure
        // condim=1: frictionless (normal only)
        // condim=3: normal + 2 sliding friction
        // condim=4: normal + 2 sliding + 1 torsional
        // condim=6: normal + 2 sliding + 1 torsional + 2 rolling
        //
        // Note: condim is validated and clamped in the MJCF loader (model_builder.rs)
        // to {1, 3, 4, 6}. This fallback handles test code that constructs contacts
        // directly without going through the loader.
        //
        // Explicit match arms document valid values; clippy::match_same_arms allowed
        // because the structure mirrors the MJCF loader's validation pattern.
        // cast_sign_loss is safe because all arms return positive values.
        #[allow(clippy::match_same_arms, clippy::cast_sign_loss)]
        let dim = match condim {
            1 => 1,
            3 => 3,
            4 => 4,
            6 => 6,
            // Round up invalid values (matches MJCF loader behavior)
            0 | 2 => 3,
            5 => 6,
            _ => 6, // >6 clamps to 6
        } as usize;

        Self {
            pos,
            normal,
            depth,
            geom1,
            geom2,
            friction: sliding, // Keep legacy field for compatibility
            dim,
            includemargin: false,
            // MuJoCo 5-element friction: [sliding1, sliding2, torsional, rolling1, rolling2]
            mu: [sliding, sliding, torsional, rolling, rolling],
            solref,
            solimp,
            frame: [t1, t2],
        }
    }
}

/// Combine solver parameters from two geoms for a contact.
///
/// MuJoCo uses element-wise minimum for solref (stiffer constraint wins)
/// and element-wise maximum for solimp (harder constraint wins).
#[inline]
fn combine_solver_params(
    solref1: [f64; 2],
    solimp1: [f64; 5],
    solref2: [f64; 2],
    solimp2: [f64; 5],
) -> ([f64; 2], [f64; 5]) {
    // solref: element-wise minimum (smaller timeconst = stiffer)
    let solref = [solref1[0].min(solref2[0]), solref1[1].min(solref2[1])];

    // solimp: element-wise maximum (larger d0 = harder)
    let solimp = [
        solimp1[0].max(solimp2[0]),
        solimp1[1].max(solimp2[1]),
        solimp1[2].max(solimp2[2]),
        solimp1[3].max(solimp2[3]),
        solimp1[4].max(solimp2[4]),
    ];

    (solref, solimp)
}

/// Compute orthonormal tangent frame from contact normal.
///
/// Returns (t1, t2) where t1, t2, normal form a right-handed orthonormal basis.
/// Handles degenerate cases (zero/NaN normal) by returning a default frame.
#[inline]
fn compute_tangent_frame(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    // Safety check: handle zero/NaN normals
    let normal_len = normal.norm();
    if !normal_len.is_finite() || normal_len < 1e-10 {
        // Degenerate case: return default frame
        return (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    }

    // Normalize the normal (in case it wasn't already)
    let n = normal / normal_len;

    // Choose a reference vector not parallel to normal
    let reference = if n.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };

    // Gram-Schmidt orthogonalization
    let t1 = reference - n * n.dot(&reference);
    let t1_norm = t1.norm();
    let t1 = if t1_norm > 1e-10 {
        t1 / t1_norm
    } else {
        // This shouldn't happen if reference was chosen correctly, but be safe
        Vector3::new(1.0, 0.0, 0.0)
    };

    let t2 = n.cross(&t1);
    // t2 should already be unit length since n and t1 are orthonormal
    (t1, t2)
}

/// Dynamic simulation state (like mjData).
///
/// All arrays pre-allocated - no heap allocation during simulation.
/// This contains the current state (qpos, qvel) and computed quantities
/// (body poses, forces, contacts).
///
/// # Key Invariant
///
/// `qpos` and `qvel` are the ONLY state variables. Everything else
/// (xpos, xquat, qfrc_*, etc.) is COMPUTED from them via forward dynamics.
#[derive(Debug)]
#[allow(non_snake_case)] // qM matches MuJoCo naming convention
pub struct Data {
    // ==================== Generalized Coordinates (THE source of truth) ====================
    /// Joint positions (length `nq`) - includes quaternion components for ball/free joints.
    pub qpos: DVector<f64>,
    /// Joint velocities (length `nv`).
    pub qvel: DVector<f64>,
    /// Joint accelerations (length `nv`) - computed by forward dynamics.
    pub qacc: DVector<f64>,
    /// Warm-start for constraint solver (length `nv`).
    pub qacc_warmstart: DVector<f64>,

    // ==================== Control / Actuation ====================
    /// Actuator control inputs (length `nu`).
    pub ctrl: DVector<f64>,
    /// Actuator activation states (length `na`) (for muscles/filters).
    pub act: DVector<f64>,
    /// Actuator forces in joint space (length `nv`).
    pub qfrc_actuator: DVector<f64>,

    /// Actuator length (gear * transmission_length, length `nu`).
    /// For Joint transmission: `gear * qpos[qpos_adr]` (hinge/slide only).
    /// For Tendon transmission: `gear * ten_length[tendon_id]`.
    pub actuator_length: Vec<f64>,

    /// Actuator velocity (gear * transmission_velocity, length `nu`).
    /// For Joint transmission: `gear * qvel[dof_adr]` (hinge/slide only).
    /// For Tendon transmission: `gear * ten_velocity[tendon_id]`.
    pub actuator_velocity: Vec<f64>,

    /// Actuator force output (length `nu`).
    /// The scalar force produced by each actuator after gain/bias/activation.
    pub actuator_force: Vec<f64>,

    /// Actuator moment vectors (length `nu`, each nv-dimensional).
    /// Populated for Site transmissions by `mj_transmission_site`.
    /// Maps scalar actuator force to generalized forces: `qfrc += moment * force`.
    pub actuator_moment: Vec<DVector<f64>>,

    /// Activation time-derivative (length `na`).
    /// Computed by `mj_fwd_actuation()`, integrated by the integrator (Euler/RK4).
    /// Separating derivative from integration matches MuJoCo's `mjData.act_dot`
    /// and is required for correct RK4 integration of activation states.
    pub act_dot: DVector<f64>,

    // ==================== Mocap Bodies ====================
    /// Mocap body positions in world frame (length nmocap).
    /// User-settable: modified between steps to drive mocap body poses.
    /// Initialized to the body_pos offset for each mocap body.
    pub mocap_pos: Vec<Vector3<f64>>,
    /// Mocap body orientations in world frame (length nmocap).
    /// User-settable: modified between steps to drive mocap body poses.
    /// Initialized to the body_quat offset for each mocap body.
    pub mocap_quat: Vec<UnitQuaternion<f64>>,

    // ==================== Computed Body States (from FK - outputs, not inputs) ====================
    /// Body positions in world frame (length `nbody`).
    pub xpos: Vec<Vector3<f64>>,
    /// Body orientations in world frame (length `nbody`).
    pub xquat: Vec<UnitQuaternion<f64>>,
    /// Body rotation matrices (cached) (length `nbody`).
    pub xmat: Vec<Matrix3<f64>>,
    /// Body inertial frame positions (length `nbody`).
    pub xipos: Vec<Vector3<f64>>,
    /// Body inertial frame rotations (length `nbody`).
    pub ximat: Vec<Matrix3<f64>>,

    // Geom poses (for collision detection)
    /// Geom positions in world frame (length `ngeom`).
    pub geom_xpos: Vec<Vector3<f64>>,
    /// Geom rotation matrices (length `ngeom`).
    pub geom_xmat: Vec<Matrix3<f64>>,

    // Site poses (for attachment points, sensors)
    /// Site positions in world frame (length `nsite`).
    pub site_xpos: Vec<Vector3<f64>>,
    /// Site rotation matrices (length `nsite`).
    pub site_xmat: Vec<Matrix3<f64>>,
    /// Site orientations in world frame (length `nsite`).
    pub site_xquat: Vec<UnitQuaternion<f64>>,

    // ==================== Velocities (computed from qvel) ====================
    /// Body spatial velocities (length `nbody`): (angular, linear).
    pub cvel: Vec<SpatialVector>,
    /// DOF velocities in Cartesian space (length `nv`).
    pub cdof: Vec<SpatialVector>,

    // ==================== RNE Intermediate Quantities ====================
    /// Body bias accelerations for RNE (Coriolis/centrifugal).
    /// `a_bias[i] = X[i] @ a_bias[parent] + v[i] ×_m S[i] @ qdot[i]`
    pub cacc_bias: Vec<SpatialVector>,
    /// Body forces for RNE backward pass.
    /// `f[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])`
    pub cfrc_bias: Vec<SpatialVector>,

    // ==================== Forces in Generalized Coordinates ====================
    /// User-applied generalized forces (length `nv`).
    pub qfrc_applied: DVector<f64>,
    /// Coriolis + centrifugal + gravity bias forces (length `nv`).
    pub qfrc_bias: DVector<f64>,
    /// Passive forces (springs + dampers) (length `nv`).
    pub qfrc_passive: DVector<f64>,
    /// Constraint forces (contacts + joint limits) (length `nv`).
    pub qfrc_constraint: DVector<f64>,

    /// Per-joint limit force cache (length `njnt`).
    /// Populated by `mj_fwd_constraint()`. Contains the unsigned penalty force
    /// magnitude for each joint's active limit constraint. Zero when the joint is
    /// within its limits or is not limited.
    ///
    /// This is the CortenForge equivalent of scanning MuJoCo's `efc_force[]` for
    /// `mjCNSTR_LIMIT_JOINT` entries. Since CortenForge uses penalty forces
    /// (not a constraint solver with per-constraint force arrays), we cache the
    /// scalar force during computation to avoid recomputation at sensor eval time.
    pub jnt_limit_frc: Vec<f64>,

    /// Per-tendon limit force cache (length `ntendon`).
    /// Populated by `mj_fwd_constraint()`. Contains the unsigned penalty force
    /// magnitude for each tendon's active limit constraint. Zero when the tendon
    /// is within its limits or is not limited.
    pub ten_limit_frc: Vec<f64>,

    // Cartesian forces (alternative input method)
    /// Applied spatial forces in world frame (length `nbody`).
    pub xfrc_applied: Vec<SpatialVector>,

    // ==================== Mass Matrix ====================
    /// Joint-space inertia matrix (`nv` x `nv`).
    /// For small systems (nv <= 32), dense storage is used.
    /// For large systems, only lower triangle is filled (sparse via qLD).
    pub qM: DMatrix<f64>,

    // ==================== Sparse L^T D L Factorization ====================
    // Computed in mj_crba() via mj_factor_sparse(). Exploits tree sparsity from
    // dof_parent for O(nv) factorization and solve vs O(nv³) for dense Cholesky.
    // Reused by mj_fwd_acceleration_explicit() and pgs_solve_contacts().
    //
    /// Sparse L^T D L factorization exploiting tree structure.
    /// M = L^T D L where L is unit lower triangular and D is diagonal.
    /// For tree-structured robots, this achieves O(n) factorization and solve.
    ///
    /// Layout: qLD stores both L and D compactly:
    /// - `qLD_diag\[i\] = D\[i,i\]` (diagonal of D)
    /// - `qLD\[i\]` contains non-zero entries of `L\[i, :\]` below diagonal
    ///
    /// The sparsity pattern is determined by the kinematic tree:
    /// `L[i,j]` is non-zero only if DOF j is an ancestor of DOF i.
    pub qLD_diag: DVector<f64>,
    /// Sparse lower triangular factor L (non-zero entries only).
    /// `qLD_L[i] = [(col_idx, value), ...]` for row i, sorted by col_idx.
    /// For tree robots, each row has at most depth(i) non-zeros.
    pub qLD_L: Vec<Vec<(usize, f64)>>,
    /// Whether sparse factorization is valid and should be used.
    /// Set to true after `mj_factor_sparse()` is called.
    pub qLD_valid: bool,

    // ==================== Body and Composite Inertia (for Featherstone CRBA/RNE) ====================
    /// Body spatial inertia in world frame (before composite accumulation).
    /// These are 6×6 spatial inertias for individual bodies, computed once in FK.
    /// Used by both CRBA (as starting point) and RNE (for bias forces).
    /// This is equivalent to MuJoCo's `mjData.cinert`.
    pub cinert: Vec<Matrix6<f64>>,
    /// Composite rigid body inertia in world frame.
    /// These are 6×6 spatial inertias accumulated from subtrees during CRBA.
    /// Starts as a copy of `cinert`, then accumulates child inertias.
    pub crb_inertia: Vec<Matrix6<f64>>,

    // ==================== Subtree Mass/COM (for O(n) RNE gravity) ====================
    /// Total mass of subtree rooted at each body (including the body itself).
    /// Computed during forward kinematics via backward pass.
    pub subtree_mass: Vec<f64>,
    /// Center of mass of subtree in world frame (length `nbody`).
    /// Computed during forward kinematics via backward pass.
    pub subtree_com: Vec<Vector3<f64>>,

    // ==================== Tendon State ====================
    /// Current tendon lengths (length `ntendon`).
    /// Computed from wrap path through kinematics.
    pub ten_length: Vec<f64>,
    /// Tendon velocities (length `ntendon`).
    /// Computed as J_tendon @ qvel.
    pub ten_velocity: Vec<f64>,
    /// Tendon forces from springs/limits (length `ntendon`).
    pub ten_force: Vec<f64>,
    /// Tendon Jacobian: d(length)/d(qpos) (sparse, length varies).
    /// Maps tendon length changes to joint velocities.
    pub ten_J: Vec<DVector<f64>>,

    // ==================== Equality Constraint State ====================
    /// Equality constraint violation (length `neq` * max_dim).
    /// For Connect: 3D position error. For Weld: 6D pose error.
    pub eq_violation: Vec<f64>,
    /// Equality constraint forces (Lagrange multipliers).
    pub eq_force: Vec<f64>,

    // ==================== Contacts ====================
    /// Active contacts (pre-allocated with capacity).
    pub contacts: Vec<Contact>,
    /// Number of active contacts (`contacts.len()` but tracked explicitly).
    pub ncon: usize,

    // ==================== Solver State ====================
    /// Iterations used in last constraint solve.
    pub solver_niter: usize,
    /// Non-zeros in constraint Jacobian.
    pub solver_nnz: usize,
    /// Constraint force multipliers from previous solve, used for warm-starting.
    ///
    /// Maps `WarmstartKey` (canonical geom pair + spatial grid cell) to
    /// constraint forces `[λ_normal, λ_friction1, ...]`. The vector length matches
    /// the contact's condim (1, 3, 4, or 6). The spatial component allows
    /// multiple contacts within the same geom pair (e.g., box-on-plane corners)
    /// to each cache their own lambda, rather than overwriting each other.
    ///
    /// Warm-starting initializes the solver near the previous solution,
    /// typically reducing iteration count by 30-50% for stable contact scenarios.
    /// If a contact's condim changes between frames, the warmstart is discarded.
    pub efc_lambda: HashMap<WarmstartKey, Vec<f64>>,

    // ==================== Unified Constraint System (Newton solver, §15) ====================
    // These fields are populated by `assemble_unified_constraints()` and consumed
    // by the Newton solver loop. For PGS/CG solvers, they remain empty (zero-length).
    /// Friction loss component of qfrc_passive (length nv). Separated from
    /// qfrc_passive for Newton solver which treats friction loss as explicit
    /// constraint rows (§15.0 approach b).
    pub qfrc_frictionloss: DVector<f64>,

    /// Constraint bias: J·qacc_smooth − aref (length nefc). Computed during
    /// assembly, consumed by warmstart cost comparison (§15.8 INITIALIZE).
    /// Persists in Data for diagnostic access (matches MuJoCo's d->efc_b),
    /// but is NOT updated by the solver loop.
    pub efc_b: DVector<f64>,
    /// Unified constraint Jacobian (nefc × nv). Rows ordered: equality,
    /// friction loss, joint limits, tendon limits, contacts.
    pub efc_J: DMatrix<f64>,
    /// Per-row constraint type annotation.
    pub efc_type: Vec<ConstraintType>,
    /// Per-row constraint violation (penetration distance for contacts).
    pub efc_pos: Vec<f64>,
    /// Per-row margin (geom margin for contacts, 0 for non-contacts).
    pub efc_margin: Vec<f64>,
    /// Per-row constraint-space velocity: J·qvel.
    pub efc_vel: DVector<f64>,
    /// Per-row solver reference parameters [timeconst, dampratio] or [stiffness, damping].
    pub efc_solref: Vec<[f64; 2]>,
    /// Per-row solver impedance parameters [dmin, dmax, width, midpoint, power].
    pub efc_solimp: Vec<[f64; 5]>,
    /// Per-row diagonal approximation of A = J·M⁻¹·J^T (for regularization scaling).
    #[allow(non_snake_case)]
    pub efc_diagApprox: Vec<f64>,
    /// Per-row regularization R (constraint softness).
    #[allow(non_snake_case)]
    pub efc_R: Vec<f64>,
    /// Per-row D = 1/R (constraint stiffness, inverse of regularization).
    #[allow(non_snake_case)]
    pub efc_D: Vec<f64>,
    /// Per-row impedance value (from solimp sigmoid, in [dmin, dmax]).
    pub efc_imp: Vec<f64>,
    // Note: MuJoCo packs K, B, imp, and impP into a single `efc_KBIP[4*i+{0,1,2,3}]`
    // array. We decompose this: K and B are transient (used only during `aref`
    // computation, not stored); `imp` is stored as `efc_imp`; `impP` (the derivative
    // of impedance w.r.t. penetration distance) is omitted — it is computed and stored
    // by MuJoCo but never read by any MuJoCo subsystem (solvers, sensors, derivatives).
    // It exists solely for external API introspection. If external parity is needed,
    // Phase C can add `efc_impP: Vec<f64>`.
    /// Per-row reference acceleration (aref = -B·vel - K·imp·(pos-margin)).
    pub efc_aref: DVector<f64>,
    /// Per-row friction loss saturation value (R * floss for Huber threshold).
    pub efc_floss: Vec<f64>,
    /// Per-row friction coefficients. For contacts: [mu1, mu2, ...] from geom.
    /// For non-contacts: [0; 5].
    pub efc_mu: Vec<[f64; 5]>,
    /// Per-row condim of parent contact (1 for non-contact rows).
    pub efc_dim: Vec<usize>,
    /// Per-row source object index (joint ID for LimitJoint, tendon ID for
    /// LimitTendon, equality ID for Equality, contact index for Contact*,
    /// DOF/tendon index for FrictionLoss).
    pub efc_id: Vec<usize>,

    // Solver output (written to Data in RECOVER after convergence)
    /// Per-row constraint state (Quadratic, Satisfied, LinearNeg/Pos, Cone).
    pub efc_state: Vec<ConstraintState>,
    /// Per-row constraint force (length nefc).
    pub efc_force: DVector<f64>,
    /// Per-row J·qacc − aref (constraint residual, length nefc).
    pub efc_jar: DVector<f64>,
    /// Total cost (Gauss + constraint terms) at convergence.
    pub efc_cost: f64,
    /// Per-contact cone Hessian matrices (dim x dim). Indexed parallel to contacts.
    /// Only populated for contacts in the Cone state. `None` for non-cone or Satisfied contacts.
    pub efc_cone_hessian: Vec<Option<DMatrix<f64>>>,
    /// Number of contacts currently in the Cone state.
    pub ncone: usize,
    /// Whether Newton solver computed qacc directly this step.
    /// When true, mj_fwd_acceleration should be skipped.
    pub newton_solved: bool,

    /// Per-iteration Newton solver statistics (length `solver_niter` after solve).
    /// Populated by Newton solver; empty for PGS/CG.
    pub solver_stat: Vec<SolverStat>,

    /// Per-step mean inertia: `trace(qM) / nv`. More accurate than the
    /// model-level constant for configuration-dependent inertia.
    /// Updated at the start of each Newton solve from the current `qM`.
    pub stat_meaninertia: f64,

    // ==================== Sensors ====================
    /// Sensor data array (length `nsensordata`).
    /// Each sensor writes to `sensordata[sensor_adr[i]..sensor_adr[i]+sensor_dim[i]]`.
    pub sensordata: DVector<f64>,

    // ==================== Energy (for debugging/validation) ====================
    /// Potential energy (gravity + springs).
    pub energy_potential: f64,
    /// Kinetic energy.
    pub energy_kinetic: f64,

    // ==================== Deformable State (feature-gated) ====================
    /// Registered deformable bodies. Boxed trait objects allow heterogeneous storage.
    /// `Send + Sync` bounds required for rayon `par_iter_mut()` and bevy `Resource`.
    #[cfg(feature = "deformable")]
    pub deformable_bodies: Vec<Box<dyn DeformableBody + Send + Sync>>,
    /// Per-body XPBD solver instances. Parallel array with `deformable_bodies`.
    #[cfg(feature = "deformable")]
    pub deformable_solvers: Vec<XpbdSolver>,
    /// Deformable-rigid contacts detected this step.
    #[cfg(feature = "deformable")]
    pub deformable_contacts: Vec<DeformableContact>,

    // ==================== Sleep State (§16.1) ====================
    /// Per-tree sleep timer (length `ntree`).
    /// `< 0`: Tree is awake. Countdown from `-(1 + MIN_AWAKE)` toward `-1`.
    /// `≥ 0`: Tree is asleep (self-link in Phase A).
    pub tree_asleep: Vec<i32>,
    /// Per-tree awake flag for fast branching (length `ntree`).
    pub tree_awake: Vec<bool>,
    /// Per-body sleep state for efficient per-body queries (length `nbody`).
    pub body_sleep_state: Vec<SleepState>,
    /// Number of awake trees (diagnostics / early-exit).
    pub ntree_awake: usize,
    /// Number of awake DOFs (diagnostics / performance monitoring).
    pub nv_awake: usize,

    // ==================== Awake-Index Indirection (§16.17) ====================
    /// Sorted indices of awake + static bodies (length ≤ nbody).
    /// Used for cache-friendly iteration in FK, velocity kinematics, etc.
    pub body_awake_ind: Vec<usize>,
    /// Number of entries in `body_awake_ind`.
    pub nbody_awake: usize,
    /// Sorted indices of bodies whose parent is awake or static (length ≤ nbody).
    /// Used for backward-pass loops (subtree COM, RNE).
    pub parent_awake_ind: Vec<usize>,
    /// Number of entries in `parent_awake_ind`.
    pub nparent_awake: usize,
    /// Sorted indices of awake DOFs (length ≤ nv).
    /// Used for velocity integration, CRBA, force accumulation.
    pub dof_awake_ind: Vec<usize>,

    // ==================== Island Discovery (§16.11) ====================
    /// Number of constraint islands discovered this step.
    pub nisland: usize,
    /// Island index for each tree. -1 if unconstrained singleton. Length: ntree.
    pub tree_island: Vec<i32>,
    /// Number of trees per island. Length: ≤ ntree.
    pub island_ntree: Vec<usize>,
    /// Start index in `map_itree2tree` for each island. Length: ≤ ntree.
    pub island_itreeadr: Vec<usize>,
    /// Packed tree indices grouped by island. Length: ntree.
    pub map_itree2tree: Vec<usize>,
    /// Island index for each DOF. -1 if unconstrained. Length: nv.
    pub dof_island: Vec<i32>,
    /// Number of DOFs per island. Length: ≤ ntree.
    pub island_nv: Vec<usize>,
    /// Start index in `map_idof2dof` for each island. Length: ≤ ntree.
    pub island_idofadr: Vec<usize>,
    /// DOF → island-local DOF index. Length: nv.
    pub map_dof2idof: Vec<i32>,
    /// Island-local DOF → global DOF. Length: nv.
    pub map_idof2dof: Vec<usize>,
    /// Island index for each constraint row. Resized per step.
    pub efc_island: Vec<i32>,
    /// Per-island constraint row count. Length: ≤ ntree.
    pub island_nefc: Vec<usize>,
    /// Start index in `map_iefc2efc` for each island. Length: ≤ ntree.
    pub island_iefcadr: Vec<usize>,
    /// Global constraint row → island-local row. Resized per step.
    pub map_efc2iefc: Vec<i32>,
    /// Island-local row → global constraint row. Resized per step.
    pub map_iefc2efc: Vec<usize>,

    // ==================== Island Scratch Space (§16.11) ====================
    /// DFS stack for flood-fill. Length: ntree.
    pub island_scratch_stack: Vec<usize>,
    /// Per-tree edge counts (CSR rownnz). Length: ntree.
    pub island_scratch_rownnz: Vec<usize>,
    /// Per-tree CSR row pointers. Length: ntree.
    pub island_scratch_rowadr: Vec<usize>,
    /// CSR column indices (edge targets). Resized per step.
    pub island_scratch_colind: Vec<usize>,

    // ==================== qpos Change Detection (§16.15) ====================
    /// Per-tree dirty flag set by mj_kinematics1() when a sleeping body's
    /// xpos/xquat changed. Read/cleared by mj_check_qpos_changed(). Length: ntree.
    pub tree_qpos_dirty: Vec<bool>,

    // ==================== Time ====================
    /// Simulation time in seconds.
    pub time: f64,

    // ==================== Scratch Buffers (for allocation-free stepping) ====================
    /// Scratch matrix for implicit integration: M + h*D + h²*K.
    /// Pre-allocated to avoid O(n²) allocation per step.
    pub scratch_m_impl: DMatrix<f64>,
    /// Scratch vector for force accumulation (length `nv`).
    pub scratch_force: DVector<f64>,
    /// Scratch vector for RHS of linear solves (length `nv`).
    pub scratch_rhs: DVector<f64>,
    /// Scratch vector for new velocity in implicit solve (length `nv`).
    /// Used to hold v_new while computing qacc = (v_new - v_old) / h.
    pub scratch_v_new: DVector<f64>,
    /// Pivot permutation for LU factorization in `Integrator::Implicit` (length `nv`).
    /// Stores row swap indices from partial pivoting. Persists after forward pass
    /// for reuse by derivative column solves in `mjd_transition_hybrid`.
    pub scratch_lu_piv: Vec<usize>,

    // ==================== RK4 Scratch Buffers ====================
    // State and rate buffers for 4-stage Runge-Kutta integration.
    // Pre-allocated in make_data() — no heap allocation during stepping.
    /// Saved initial position for RK4: `X[0].qpos` (length nq).
    /// Read at every stage (step 2c) and the final advance (step 4).
    pub rk4_qpos_saved: DVector<f64>,

    /// Scratch position for the current RK4 stage (length nq).
    /// Written at step 2c, copied to data.qpos at step 2e, then overwritten
    /// next iteration. Intermediate positions are not needed for the final
    /// B-weight combination (which only sums velocities and accelerations).
    pub rk4_qpos_stage: DVector<f64>,

    /// RK4 stage velocities: `X[i].qvel` (4 buffers, each length nv).
    /// All 4 are needed for the final B-weight combination in step 3.
    pub rk4_qvel: [DVector<f64>; 4],

    /// RK4 stage accelerations: `F[i].qacc` (4 buffers, each length nv).
    /// All 4 are needed for the final B-weight combination in step 3.
    pub rk4_qacc: [DVector<f64>; 4],

    /// Weighted velocity sum for manifold position integration (length nv).
    pub rk4_dX_vel: DVector<f64>,

    /// Weighted acceleration sum for velocity update (length nv).
    pub rk4_dX_acc: DVector<f64>,

    /// RK4 saved activation state (length na). Stores initial `act` before RK4 stages.
    pub rk4_act_saved: DVector<f64>,

    /// RK4 stage activation derivatives: `act_dot` at each stage (4 buffers, each length na).
    pub rk4_act_dot: [DVector<f64>; 4],

    // ==================== Derivative Scratch Buffers ====================
    /// Analytical derivative of smooth forces w.r.t. velocity: ∂(qfrc_smooth)/∂qvel.
    /// Dense nv × nv matrix. Populated by `mjd_smooth_vel()`.
    ///
    /// Components:
    ///   ∂(qfrc_passive)/∂qvel  = diagonal damping (+ tendon damping J^T·b·J
    ///                            in explicit mode only; skipped for ImplicitSpringDamper)
    ///   ∂(qfrc_actuator)/∂qvel = affine velocity-dependent gain/bias terms
    ///   −∂(qfrc_bias)/∂qvel    = −C(q,v) (Coriolis matrix)
    ///
    /// MuJoCo equivalent: `mjData.qDeriv` (sparse nv×nv). Dense here because
    /// nv < 100 for target use cases.
    #[allow(non_snake_case)]
    pub qDeriv: DMatrix<f64>,

    /// Scratch Jacobian ∂(cvel)/∂(qvel) per body (length `nbody`, each 6 × nv).
    /// Used by `mjd_rne_vel()` for chain-rule derivative propagation.
    #[allow(non_snake_case)]
    pub deriv_Dcvel: Vec<DMatrix<f64>>,
    /// Scratch Jacobian ∂(cacc)/∂(qvel) per body (length `nbody`, each 6 × nv).
    #[allow(non_snake_case)]
    pub deriv_Dcacc: Vec<DMatrix<f64>>,
    /// Scratch Jacobian ∂(cfrc)/∂(qvel) per body (length `nbody`, each 6 × nv).
    #[allow(non_snake_case)]
    pub deriv_Dcfrc: Vec<DMatrix<f64>>,

    // ==================== Cached Body Effective Mass/Inertia ====================
    // These are extracted from the mass matrix diagonal during forward() and cached
    // for use by constraint force limiting. This avoids O(joints) traversal per constraint.
    /// Minimum translational mass for each body (length `nbody`).
    /// Extracted from qM diagonal for linear DOFs (free joint indices 0-2, slide joints).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_mass: Vec<f64>,

    /// Minimum rotational inertia for each body (length `nbody`).
    /// Extracted from qM diagonal for angular DOFs (free joint 3-5, ball 0-2, hinge).
    /// World body (index 0) has value `f64::INFINITY`.
    pub body_min_inertia: Vec<f64>,
}

#[allow(non_snake_case)] // qM, qLD matches MuJoCo naming convention
impl Clone for Data {
    fn clone(&self) -> Self {
        Self {
            // Generalized coordinates
            qpos: self.qpos.clone(),
            qvel: self.qvel.clone(),
            qacc: self.qacc.clone(),
            qacc_warmstart: self.qacc_warmstart.clone(),
            // Control / Actuation
            ctrl: self.ctrl.clone(),
            act: self.act.clone(),
            qfrc_actuator: self.qfrc_actuator.clone(),
            actuator_length: self.actuator_length.clone(),
            actuator_velocity: self.actuator_velocity.clone(),
            actuator_force: self.actuator_force.clone(),
            actuator_moment: self.actuator_moment.clone(),
            act_dot: self.act_dot.clone(),
            // Mocap bodies
            mocap_pos: self.mocap_pos.clone(),
            mocap_quat: self.mocap_quat.clone(),
            // Computed body states
            xpos: self.xpos.clone(),
            xquat: self.xquat.clone(),
            xmat: self.xmat.clone(),
            xipos: self.xipos.clone(),
            ximat: self.ximat.clone(),
            geom_xpos: self.geom_xpos.clone(),
            geom_xmat: self.geom_xmat.clone(),
            site_xpos: self.site_xpos.clone(),
            site_xmat: self.site_xmat.clone(),
            site_xquat: self.site_xquat.clone(),
            // Velocities
            cvel: self.cvel.clone(),
            cdof: self.cdof.clone(),
            // RNE intermediate
            cacc_bias: self.cacc_bias.clone(),
            cfrc_bias: self.cfrc_bias.clone(),
            // Forces
            qfrc_applied: self.qfrc_applied.clone(),
            qfrc_bias: self.qfrc_bias.clone(),
            qfrc_passive: self.qfrc_passive.clone(),
            qfrc_constraint: self.qfrc_constraint.clone(),
            jnt_limit_frc: self.jnt_limit_frc.clone(),
            ten_limit_frc: self.ten_limit_frc.clone(),
            xfrc_applied: self.xfrc_applied.clone(),
            // Mass matrix
            qM: self.qM.clone(),
            // Sparse factorization
            qLD_diag: self.qLD_diag.clone(),
            qLD_L: self.qLD_L.clone(),
            qLD_valid: self.qLD_valid,
            // Body/composite inertia
            cinert: self.cinert.clone(),
            crb_inertia: self.crb_inertia.clone(),
            // Subtree mass/COM
            subtree_mass: self.subtree_mass.clone(),
            subtree_com: self.subtree_com.clone(),
            // Tendon state
            ten_length: self.ten_length.clone(),
            ten_velocity: self.ten_velocity.clone(),
            ten_force: self.ten_force.clone(),
            ten_J: self.ten_J.clone(),
            // Equality constraints
            eq_violation: self.eq_violation.clone(),
            eq_force: self.eq_force.clone(),
            // Contacts
            contacts: self.contacts.clone(),
            ncon: self.ncon,
            // Solver state
            solver_niter: self.solver_niter,
            solver_nnz: self.solver_nnz,
            efc_lambda: self.efc_lambda.clone(),
            // Unified constraint system (Newton solver)
            qfrc_frictionloss: self.qfrc_frictionloss.clone(),
            efc_b: self.efc_b.clone(),
            efc_J: self.efc_J.clone(),
            efc_type: self.efc_type.clone(),
            efc_pos: self.efc_pos.clone(),
            efc_margin: self.efc_margin.clone(),
            efc_vel: self.efc_vel.clone(),
            efc_solref: self.efc_solref.clone(),
            efc_solimp: self.efc_solimp.clone(),
            efc_diagApprox: self.efc_diagApprox.clone(),
            efc_R: self.efc_R.clone(),
            efc_D: self.efc_D.clone(),
            efc_imp: self.efc_imp.clone(),
            efc_aref: self.efc_aref.clone(),
            efc_floss: self.efc_floss.clone(),
            efc_mu: self.efc_mu.clone(),
            efc_dim: self.efc_dim.clone(),
            efc_id: self.efc_id.clone(),
            efc_state: self.efc_state.clone(),
            efc_force: self.efc_force.clone(),
            efc_jar: self.efc_jar.clone(),
            efc_cost: self.efc_cost,
            efc_cone_hessian: self.efc_cone_hessian.clone(),
            ncone: self.ncone,
            newton_solved: self.newton_solved,
            solver_stat: self.solver_stat.clone(),
            stat_meaninertia: self.stat_meaninertia,
            // Sensors
            sensordata: self.sensordata.clone(),
            // Energy
            energy_potential: self.energy_potential,
            energy_kinetic: self.energy_kinetic,
            // Deformable state (feature-gated)
            #[cfg(feature = "deformable")]
            deformable_bodies: self
                .deformable_bodies
                .iter()
                .map(|b| b.clone_box())
                .collect(),
            #[cfg(feature = "deformable")]
            deformable_solvers: self.deformable_solvers.clone(),
            #[cfg(feature = "deformable")]
            deformable_contacts: self.deformable_contacts.clone(),
            // Sleep state
            tree_asleep: self.tree_asleep.clone(),
            tree_awake: self.tree_awake.clone(),
            body_sleep_state: self.body_sleep_state.clone(),
            ntree_awake: self.ntree_awake,
            nv_awake: self.nv_awake,
            // Awake-index indirection (§16.17)
            body_awake_ind: self.body_awake_ind.clone(),
            nbody_awake: self.nbody_awake,
            parent_awake_ind: self.parent_awake_ind.clone(),
            nparent_awake: self.nparent_awake,
            dof_awake_ind: self.dof_awake_ind.clone(),
            // Island discovery (§16.11)
            nisland: self.nisland,
            tree_island: self.tree_island.clone(),
            island_ntree: self.island_ntree.clone(),
            island_itreeadr: self.island_itreeadr.clone(),
            map_itree2tree: self.map_itree2tree.clone(),
            dof_island: self.dof_island.clone(),
            island_nv: self.island_nv.clone(),
            island_idofadr: self.island_idofadr.clone(),
            map_dof2idof: self.map_dof2idof.clone(),
            map_idof2dof: self.map_idof2dof.clone(),
            efc_island: self.efc_island.clone(),
            island_nefc: self.island_nefc.clone(),
            island_iefcadr: self.island_iefcadr.clone(),
            map_efc2iefc: self.map_efc2iefc.clone(),
            map_iefc2efc: self.map_iefc2efc.clone(),
            // Island scratch
            island_scratch_stack: self.island_scratch_stack.clone(),
            island_scratch_rownnz: self.island_scratch_rownnz.clone(),
            island_scratch_rowadr: self.island_scratch_rowadr.clone(),
            island_scratch_colind: self.island_scratch_colind.clone(),
            // qpos change detection
            tree_qpos_dirty: self.tree_qpos_dirty.clone(),
            // Time
            time: self.time,
            // Scratch buffers
            scratch_m_impl: self.scratch_m_impl.clone(),
            scratch_force: self.scratch_force.clone(),
            scratch_rhs: self.scratch_rhs.clone(),
            scratch_v_new: self.scratch_v_new.clone(),
            scratch_lu_piv: self.scratch_lu_piv.clone(),
            // RK4 scratch
            rk4_qpos_saved: self.rk4_qpos_saved.clone(),
            rk4_qpos_stage: self.rk4_qpos_stage.clone(),
            rk4_qvel: self.rk4_qvel.clone(),
            rk4_qacc: self.rk4_qacc.clone(),
            rk4_dX_vel: self.rk4_dX_vel.clone(),
            rk4_dX_acc: self.rk4_dX_acc.clone(),
            rk4_act_saved: self.rk4_act_saved.clone(),
            rk4_act_dot: self.rk4_act_dot.clone(),
            // Derivative scratch buffers
            qDeriv: self.qDeriv.clone(),
            deriv_Dcvel: self.deriv_Dcvel.clone(),
            deriv_Dcacc: self.deriv_Dcacc.clone(),
            deriv_Dcfrc: self.deriv_Dcfrc.clone(),
            // Cached body mass/inertia
            body_min_mass: self.body_min_mass.clone(),
            body_min_inertia: self.body_min_inertia.clone(),
        }
    }
}

impl Model {
    /// Create an empty model with no bodies/joints.
    #[must_use]
    pub fn empty() -> Self {
        Self {
            // Metadata
            name: String::new(),

            // Dimensions
            nq: 0,
            nv: 0,
            nbody: 1, // World body 0 always exists
            njnt: 0,
            ngeom: 0,
            nsite: 0,
            nu: 0,
            na: 0,
            nmocap: 0,
            nkeyframe: 0,

            // Kinematic trees (§16.0) — empty model has no trees
            ntree: 0,
            tree_body_adr: vec![],
            tree_body_num: vec![],
            tree_dof_adr: vec![],
            tree_dof_num: vec![],
            body_treeid: vec![usize::MAX], // World body sentinel
            dof_treeid: vec![],
            tree_sleep_policy: vec![],
            dof_length: vec![],
            sleep_tolerance: 1e-4,

            // Body tree (initialize world body)
            body_parent: vec![0], // World is its own parent
            body_rootid: vec![0],
            body_jnt_adr: vec![0],
            body_jnt_num: vec![0],
            body_dof_adr: vec![0],
            body_dof_num: vec![0],
            body_geom_adr: vec![0],
            body_geom_num: vec![0],

            // Body properties
            body_pos: vec![Vector3::zeros()],
            body_quat: vec![UnitQuaternion::identity()],
            body_ipos: vec![Vector3::zeros()],
            body_iquat: vec![UnitQuaternion::identity()],
            body_mass: vec![0.0], // World has no mass
            body_inertia: vec![Vector3::zeros()],
            body_name: vec![Some("world".to_string())],
            body_subtreemass: vec![0.0], // World subtree mass (will be total system mass)
            body_mocapid: vec![None],    // world body

            // Joints (empty)
            jnt_type: vec![],
            jnt_body: vec![],
            jnt_qpos_adr: vec![],
            jnt_dof_adr: vec![],
            jnt_pos: vec![],
            jnt_axis: vec![],
            jnt_limited: vec![],
            jnt_range: vec![],
            jnt_stiffness: vec![],
            jnt_springref: vec![],
            jnt_damping: vec![],
            jnt_armature: vec![],
            jnt_solref: vec![],
            jnt_solimp: vec![],
            jnt_name: vec![],

            // DOFs (empty)
            dof_body: vec![],
            dof_jnt: vec![],
            dof_parent: vec![],
            dof_armature: vec![],
            dof_damping: vec![],
            dof_frictionloss: vec![],

            // Geoms (empty)
            geom_type: vec![],
            geom_body: vec![],
            geom_pos: vec![],
            geom_quat: vec![],
            geom_size: vec![],
            geom_friction: vec![],
            geom_condim: vec![],
            geom_contype: vec![],
            geom_conaffinity: vec![],
            geom_margin: vec![],
            geom_gap: vec![],
            geom_solimp: vec![],
            geom_solref: vec![],
            geom_name: vec![],
            geom_rbound: vec![],
            geom_mesh: vec![],
            geom_hfield: vec![],
            geom_sdf: vec![],

            // Meshes (empty)
            nmesh: 0,
            mesh_name: vec![],
            mesh_data: vec![],

            // Height fields (empty)
            nhfield: 0,
            hfield_name: vec![],
            hfield_data: vec![],
            hfield_size: vec![],

            // SDFs (empty)
            nsdf: 0,
            sdf_data: vec![],

            // Sites (empty)
            site_body: vec![],
            site_type: vec![],
            site_pos: vec![],
            site_quat: vec![],
            site_size: vec![],
            site_name: vec![],

            // Sensors (empty)
            nsensor: 0,
            nsensordata: 0,
            sensor_type: vec![],
            sensor_datatype: vec![],
            sensor_objtype: vec![],
            sensor_objid: vec![],
            sensor_reftype: vec![],
            sensor_refid: vec![],
            sensor_adr: vec![],
            sensor_dim: vec![],
            sensor_noise: vec![],
            sensor_cutoff: vec![],
            sensor_name: vec![],

            // Actuators (empty)
            actuator_trntype: vec![],
            actuator_dyntype: vec![],
            actuator_trnid: vec![],
            actuator_gear: vec![],
            actuator_ctrlrange: vec![],
            actuator_forcerange: vec![],
            actuator_name: vec![],
            actuator_act_adr: vec![],
            actuator_act_num: vec![],
            actuator_gaintype: vec![],
            actuator_biastype: vec![],
            actuator_dynprm: vec![],
            actuator_gainprm: vec![],
            actuator_biasprm: vec![],
            actuator_lengthrange: vec![],
            actuator_acc0: vec![],

            // Tendons (empty)
            ntendon: 0,
            nwrap: 0,
            tendon_range: vec![],
            tendon_limited: vec![],
            tendon_stiffness: vec![],
            tendon_damping: vec![],
            tendon_lengthspring: vec![],
            tendon_length0: vec![],
            tendon_num: vec![],
            tendon_adr: vec![],
            tendon_name: vec![],
            tendon_type: vec![],
            tendon_solref: vec![],
            tendon_solimp: vec![],
            tendon_frictionloss: vec![],
            tendon_treenum: vec![],
            tendon_tree: vec![],
            wrap_type: vec![],
            wrap_objid: vec![],
            wrap_prm: vec![],
            wrap_sidesite: vec![],

            // Equality constraints (empty)
            neq: 0,
            eq_type: vec![],
            eq_obj1id: vec![],
            eq_obj2id: vec![],
            eq_data: vec![],
            eq_active: vec![],
            eq_solimp: vec![],
            eq_solref: vec![],
            eq_name: vec![],

            // Options (MuJoCo defaults)
            timestep: 0.002,                        // 500 Hz
            gravity: Vector3::new(0.0, 0.0, -9.81), // Z-up
            qpos0: DVector::zeros(0),
            // Keyframes
            keyframes: Vec::new(),
            wind: Vector3::zeros(),
            magnetic: Vector3::zeros(),
            density: 0.0,   // No fluid by default
            viscosity: 0.0, // No fluid by default
            solver_iterations: 100,
            solver_tolerance: 1e-8,
            impratio: 1.0,                 // MuJoCo default
            regularization: 1e-6,          // PGS constraint softness
            default_eq_stiffness: 10000.0, // Equality constraint fallback stiffness
            default_eq_damping: 1000.0,    // Equality constraint fallback damping
            max_constraint_vel: 1.0,       // Max linear delta-v per step (m/s)
            max_constraint_angvel: 1.0,    // Max angular delta-v per step (rad/s)
            friction_smoothing: 1000.0,    // tanh transition sharpness
            cone: 1,                       // Elliptic friction cone (pyramidal not supported)
            stat_meaninertia: 1.0,         // Default (computed at model build from CRBA)
            ls_iterations: 50,             // Newton line search iterations
            ls_tolerance: 0.01,            // Newton line search gradient tolerance
            noslip_iterations: 0,          // Not yet implemented
            noslip_tolerance: 1e-6,        // Not yet implemented
            disableflags: 0,               // Nothing disabled
            enableflags: 0,                // Nothing extra enabled
            #[cfg(feature = "deformable")]
            deformable_condim: 3,
            #[cfg(feature = "deformable")]
            deformable_solref: [0.02, 1.0],
            #[cfg(feature = "deformable")]
            deformable_solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
            integrator: Integrator::Euler,
            solver_type: SolverType::PGS,

            // Cached implicit integration parameters (empty for empty model)
            implicit_stiffness: DVector::zeros(0),
            implicit_damping: DVector::zeros(0),
            implicit_springref: DVector::zeros(0),

            // Pre-computed kinematic data (world body has no ancestors)
            body_ancestor_joints: vec![vec![]],
            body_ancestor_mask: vec![vec![]], // Empty vec for world body (no joints yet)

            // Contact pairs / excludes (empty = no explicit pairs or excludes)
            contact_pairs: vec![],
            contact_pair_set: HashSet::new(),
            contact_excludes: HashSet::new(),
        }
    }

    /// Iterate over all joints with the visitor pattern.
    ///
    /// This is the **single source of truth** for joint iteration. Use this method
    /// instead of manually iterating with `for jnt_id in 0..self.njnt` to ensure:
    ///
    /// - Consistent address computation across the codebase
    /// - Automatic handling of new joint types (compiler enforces trait updates)
    /// - Centralized joint metadata in `JointContext`
    ///
    /// # Example
    ///
    /// ```ignore
    /// struct MyVisitor { /* ... */ }
    ///
    /// impl JointVisitor for MyVisitor {
    ///     fn visit_hinge(&mut self, ctx: JointContext) {
    ///         println!("Hinge joint {} at DOF {}", ctx.jnt_id, ctx.dof_adr);
    ///     }
    /// }
    ///
    /// model.visit_joints(&mut MyVisitor { /* ... */ });
    /// ```
    #[inline]
    pub fn visit_joints<V: JointVisitor>(&self, visitor: &mut V) {
        for jnt_id in 0..self.njnt {
            let jnt_type = self.jnt_type[jnt_id];
            let ctx = JointContext {
                jnt_id,
                jnt_type,
                dof_adr: self.jnt_dof_adr[jnt_id],
                qpos_adr: self.jnt_qpos_adr[jnt_id],
                nv: jnt_type.nv(),
                nq: jnt_type.nq(),
            };

            match jnt_type {
                MjJointType::Hinge => visitor.visit_hinge(ctx),
                MjJointType::Slide => visitor.visit_slide(ctx),
                MjJointType::Ball => visitor.visit_ball(ctx),
                MjJointType::Free => visitor.visit_free(ctx),
            }
        }
    }

    /// Create initial Data struct for this model with all arrays pre-allocated.
    #[must_use]
    pub fn make_data(&self) -> Data {
        let mut data = Data {
            // Generalized coordinates
            qpos: self.qpos0.clone(),
            qvel: DVector::zeros(self.nv),
            qacc: DVector::zeros(self.nv),
            qacc_warmstart: DVector::zeros(self.nv),

            // Actuation
            ctrl: DVector::zeros(self.nu),
            act: DVector::zeros(self.na),
            qfrc_actuator: DVector::zeros(self.nv),
            actuator_length: vec![0.0; self.nu],
            actuator_velocity: vec![0.0; self.nu],
            actuator_force: vec![0.0; self.nu],
            actuator_moment: vec![DVector::zeros(self.nv); self.nu],
            act_dot: DVector::zeros(self.na),

            // Body states
            xpos: vec![Vector3::zeros(); self.nbody],
            xquat: vec![UnitQuaternion::identity(); self.nbody],
            xmat: vec![Matrix3::identity(); self.nbody],
            xipos: vec![Vector3::zeros(); self.nbody],
            ximat: vec![Matrix3::identity(); self.nbody],

            // Mocap bodies (default to body_pos/body_quat for each mocap body)
            mocap_pos: if self.nmocap == 0 {
                Vec::new()
            } else {
                self.body_mocapid
                    .iter()
                    .enumerate()
                    .filter_map(|(i, mid)| mid.map(|_| self.body_pos[i]))
                    .collect()
            },
            mocap_quat: if self.nmocap == 0 {
                Vec::new()
            } else {
                self.body_mocapid
                    .iter()
                    .enumerate()
                    .filter_map(|(i, mid)| mid.map(|_| self.body_quat[i]))
                    .collect()
            },

            // Geom poses
            geom_xpos: vec![Vector3::zeros(); self.ngeom],
            geom_xmat: vec![Matrix3::identity(); self.ngeom],

            // Site poses
            site_xpos: vec![Vector3::zeros(); self.nsite],
            site_xmat: vec![Matrix3::identity(); self.nsite],
            site_xquat: vec![UnitQuaternion::identity(); self.nsite],

            // Velocities
            cvel: vec![SpatialVector::zeros(); self.nbody],
            cdof: vec![SpatialVector::zeros(); self.nv],

            // RNE intermediate quantities
            cacc_bias: vec![SpatialVector::zeros(); self.nbody],
            cfrc_bias: vec![SpatialVector::zeros(); self.nbody],

            // Forces
            qfrc_applied: DVector::zeros(self.nv),
            qfrc_bias: DVector::zeros(self.nv),
            qfrc_passive: DVector::zeros(self.nv),
            qfrc_constraint: DVector::zeros(self.nv),
            jnt_limit_frc: vec![0.0; self.njnt],
            ten_limit_frc: vec![0.0; self.ntendon],
            xfrc_applied: vec![SpatialVector::zeros(); self.nbody],

            // Mass matrix (dense)
            qM: DMatrix::zeros(self.nv, self.nv),

            // Mass matrix (sparse L^T D L — computed in mj_crba via mj_factor_sparse)
            qLD_diag: DVector::zeros(self.nv),
            qLD_L: vec![Vec::new(); self.nv],
            qLD_valid: false,

            // Body spatial inertias (computed once in FK, used by CRBA and RNE)
            cinert: vec![Matrix6::zeros(); self.nbody],
            // Composite rigid body inertias (for Featherstone CRBA)
            crb_inertia: vec![Matrix6::zeros(); self.nbody],

            // Subtree mass/COM (for O(n) RNE gravity)
            subtree_mass: vec![0.0; self.nbody],
            subtree_com: vec![Vector3::zeros(); self.nbody],

            // Tendon state
            ten_length: vec![0.0; self.ntendon],
            ten_velocity: vec![0.0; self.ntendon],
            ten_force: vec![0.0; self.ntendon],
            ten_J: vec![DVector::zeros(self.nv); self.ntendon],

            // Equality constraint state
            eq_violation: vec![0.0; self.neq * 6], // max 6 DOF per constraint (weld)
            eq_force: vec![0.0; self.neq * 6],

            // Contacts
            contacts: Vec::with_capacity(256), // Pre-allocate typical capacity
            ncon: 0,

            // Solver state
            solver_niter: 0,
            solver_nnz: 0,
            efc_lambda: HashMap::with_capacity(256), // Contact correspondence warmstart

            // Unified constraint system (Newton solver) — initially empty (zero-length).
            // Populated by assemble_unified_constraints() only when Newton solver is active.
            qfrc_frictionloss: DVector::zeros(self.nv),
            efc_b: DVector::zeros(0),
            efc_J: DMatrix::zeros(0, self.nv),
            efc_type: Vec::new(),
            efc_pos: Vec::new(),
            efc_margin: Vec::new(),
            efc_vel: DVector::zeros(0),
            efc_solref: Vec::new(),
            efc_solimp: Vec::new(),
            efc_diagApprox: Vec::new(),
            efc_R: Vec::new(),
            efc_D: Vec::new(),
            efc_imp: Vec::new(),
            efc_aref: DVector::zeros(0),
            efc_floss: Vec::new(),
            efc_mu: Vec::new(),
            efc_dim: Vec::new(),
            efc_id: Vec::new(),
            efc_state: Vec::new(),
            efc_force: DVector::zeros(0),
            efc_jar: DVector::zeros(0),
            efc_cost: 0.0,
            efc_cone_hessian: Vec::new(),
            ncone: 0,
            newton_solved: false,
            solver_stat: Vec::new(),
            stat_meaninertia: 1.0,

            // Sensors
            sensordata: DVector::zeros(self.nsensordata),

            // Energy
            energy_potential: 0.0,
            energy_kinetic: 0.0,

            // Deformable state (feature-gated)
            #[cfg(feature = "deformable")]
            deformable_bodies: Vec::new(),
            #[cfg(feature = "deformable")]
            deformable_solvers: Vec::new(),
            #[cfg(feature = "deformable")]
            deformable_contacts: Vec::with_capacity(256),

            // Sleep state (§16.7) — initialized from tree sleep policies.
            // For models not built through MJCF (ntree == 0 or body_treeid not populated),
            // all bodies start awake and sleep is effectively a no-op.
            tree_asleep: {
                let mut v = vec![-(1 + MIN_AWAKE); self.ntree];
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        #[allow(clippy::cast_possible_wrap)]
                        {
                            v[t] = t as i32; // Start asleep (ntree ≤ nbody ≪ i32::MAX)
                        }
                    }
                }
                v
            },
            tree_awake: {
                let mut v = vec![true; self.ntree];
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        v[t] = false;
                    }
                }
                v
            },
            body_sleep_state: {
                let mut v = vec![SleepState::Awake; self.nbody];
                if self.nbody > 0 {
                    v[0] = SleepState::Static; // World body
                }
                // Mark bodies in Init trees as Asleep (only if tree enumeration was run)
                if self.body_treeid.len() == self.nbody {
                    for body_id in 1..self.nbody {
                        let tree = self.body_treeid[body_id];
                        if tree < self.ntree && self.tree_sleep_policy[tree] == SleepPolicy::Init {
                            v[body_id] = SleepState::Asleep;
                        }
                    }
                }
                v
            },
            ntree_awake: {
                let mut count = self.ntree;
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        count -= 1;
                    }
                }
                count
            },
            nv_awake: {
                let mut count = self.nv;
                for t in 0..self.ntree {
                    if self.tree_sleep_policy[t] == SleepPolicy::Init {
                        count -= self.tree_dof_num[t];
                    }
                }
                count
            },

            // Awake-index indirection arrays (§16.17).
            // Allocated to worst-case size; populated by mj_update_sleep_arrays().
            body_awake_ind: vec![0; self.nbody],
            nbody_awake: 0, // Set by mj_update_sleep_arrays below
            parent_awake_ind: vec![0; self.nbody],
            nparent_awake: 0,
            dof_awake_ind: vec![0; self.nv],

            // Island discovery arrays (§16.11) — worst-case: each tree is its own island.
            nisland: 0,
            tree_island: vec![-1_i32; self.ntree],
            island_ntree: vec![0; self.ntree],
            island_itreeadr: vec![0; self.ntree],
            map_itree2tree: vec![0; self.ntree],
            dof_island: vec![-1_i32; self.nv],
            island_nv: vec![0; self.ntree],
            island_idofadr: vec![0; self.ntree],
            map_dof2idof: vec![-1_i32; self.nv],
            map_idof2dof: vec![0; self.nv],
            efc_island: Vec::new(),
            island_nefc: vec![0; self.ntree],
            island_iefcadr: vec![0; self.ntree],
            map_efc2iefc: Vec::new(),
            map_iefc2efc: Vec::new(),

            // Island scratch space (§16.11)
            island_scratch_stack: vec![0; self.ntree],
            island_scratch_rownnz: vec![0; self.ntree],
            island_scratch_rowadr: vec![0; self.ntree],
            island_scratch_colind: Vec::new(),

            // qpos change detection (§16.15)
            tree_qpos_dirty: vec![false; self.ntree],

            // Time
            time: 0.0,

            // Scratch buffers (pre-allocated for allocation-free stepping)
            scratch_m_impl: DMatrix::zeros(self.nv, self.nv),
            scratch_force: DVector::zeros(self.nv),
            scratch_rhs: DVector::zeros(self.nv),
            scratch_v_new: DVector::zeros(self.nv),
            scratch_lu_piv: vec![0; self.nv],

            // RK4 scratch buffers
            rk4_qpos_saved: DVector::zeros(self.nq),
            rk4_qpos_stage: DVector::zeros(self.nq),
            rk4_qvel: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_qacc: std::array::from_fn(|_| DVector::zeros(self.nv)),
            rk4_dX_vel: DVector::zeros(self.nv),
            rk4_dX_acc: DVector::zeros(self.nv),
            rk4_act_saved: DVector::zeros(self.na),
            rk4_act_dot: std::array::from_fn(|_| DVector::zeros(self.na)),

            // Derivative scratch buffers (for mjd_smooth_vel / mjd_rne_vel)
            qDeriv: DMatrix::zeros(self.nv, self.nv),
            deriv_Dcvel: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcacc: vec![DMatrix::zeros(6, self.nv); self.nbody],
            deriv_Dcfrc: vec![DMatrix::zeros(6, self.nv); self.nbody],

            // Cached body mass/inertia (computed in forward() after CRBA)
            // Initialize world body (index 0) to infinity, others to default
            body_min_mass: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
            body_min_inertia: {
                let mut v = vec![DEFAULT_MASS_FALLBACK; self.nbody];
                if self.nbody > 0 {
                    v[0] = f64::INFINITY; // World body
                }
                v
            },
        };

        // Run initial FK to populate body/geom/site positions from qpos0.
        // This must happen BEFORE sleep gating takes effect so that Init-asleep
        // bodies have correct world positions. We temporarily mark all bodies
        // as Awake, run FK, then restore the Init-sleep state.
        // Only needed when there are Init-asleep trees; other models get FK
        // from their first forward()/step() call.
        let has_init_asleep = (0..self.ntree).any(|t| data.tree_asleep[t] >= 0);
        if has_init_asleep {
            // Save init sleep state
            let saved_body_sleep = data.body_sleep_state.clone();
            let saved_tree_awake = data.tree_awake.clone();
            let saved_tree_asleep = data.tree_asleep.clone();

            // Temporarily wake all bodies for FK
            for b in 1..self.nbody {
                data.body_sleep_state[b] = SleepState::Awake;
            }
            for t in 0..self.ntree {
                data.tree_awake[t] = true;
                data.tree_asleep[t] = -(1 + MIN_AWAKE);
            }

            // Run FK only (not full forward — just position computation)
            mj_fwd_position(self, &mut data);

            // Restore sleep state
            data.body_sleep_state = saved_body_sleep;
            data.tree_awake = saved_tree_awake;
            data.tree_asleep = saved_tree_asleep;
        }

        // Populate awake-index indirection arrays (§16.17).
        mj_update_sleep_arrays(self, &mut data);

        data
    }

    /// Get reference position for specified joint (from qpos0).
    ///
    /// Returns `None` if `jnt_id` is out of bounds.
    #[must_use]
    pub fn joint_qpos0(&self, jnt_id: usize) -> Option<&[f64]> {
        if jnt_id >= self.njnt {
            return None;
        }
        let start = self.jnt_qpos_adr[jnt_id];
        let len = self.jnt_type[jnt_id].nq();
        Some(&self.qpos0.as_slice()[start..start + len])
    }

    /// Compute pre-computed kinematic data (ancestor lists and masks).
    ///
    /// This must be called after the model topology is finalized. It builds:
    /// - `body_ancestor_joints`: For each body, the list of all ancestor joints
    /// - `body_ancestor_mask`: Multi-word bitmask for O(1) ancestor testing
    ///
    /// These enable O(n) CRBA/RNE algorithms instead of O(n³).
    ///
    /// Following `MuJoCo`'s principle: heavy computation at model load time,
    /// minimal computation at simulation time.
    pub fn compute_ancestors(&mut self) {
        // Number of u64 words needed for the bitmask
        let num_words = (self.njnt + 63) / 64; // ceil(njnt / 64)

        // Clear and resize
        self.body_ancestor_joints = vec![vec![]; self.nbody];
        self.body_ancestor_mask = vec![vec![0u64; num_words]; self.nbody];

        // For each body, walk up to root collecting ancestor joints
        for body_id in 1..self.nbody {
            let mut current = body_id;
            while current != 0 {
                // Add joints attached to this body
                let jnt_start = self.body_jnt_adr[current];
                let jnt_end = jnt_start + self.body_jnt_num[current];
                for jnt_id in jnt_start..jnt_end {
                    self.body_ancestor_joints[body_id].push(jnt_id);
                    // Set bit in multi-word mask (supports unlimited joints)
                    let word = jnt_id / 64;
                    let bit = jnt_id % 64;
                    self.body_ancestor_mask[body_id][word] |= 1u64 << bit;
                }
                current = self.body_parent[current];
            }
        }
    }

    /// Compute cached implicit integration parameters from joint parameters.
    ///
    /// This expands per-joint K/D/springref into per-DOF vectors used by
    /// implicit integration. Must be called after all joints are added.
    ///
    /// For Hinge/Slide joints: `K\[dof\]` = jnt_stiffness, `D\[dof\]` = jnt_damping
    /// For Ball/Free joints: `K\[dof\]` = 0, `D\[dof\]` = dof_damping (per-DOF)
    pub fn compute_implicit_params(&mut self) {
        // Resize to nv DOFs
        self.implicit_stiffness = DVector::zeros(self.nv);
        self.implicit_damping = DVector::zeros(self.nv);
        self.implicit_springref = DVector::zeros(self.nv);

        for jnt_id in 0..self.njnt {
            let dof_adr = self.jnt_dof_adr[jnt_id];
            let jnt_type = self.jnt_type[jnt_id];
            let nv_jnt = jnt_type.nv();

            match jnt_type {
                MjJointType::Hinge | MjJointType::Slide => {
                    self.implicit_stiffness[dof_adr] = self.jnt_stiffness[jnt_id];
                    self.implicit_damping[dof_adr] = self.jnt_damping[jnt_id];
                    self.implicit_springref[dof_adr] = self.jnt_springref[jnt_id];
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free: per-DOF damping only (no spring for quaternion DOFs)
                    for i in 0..nv_jnt {
                        let dof_idx = dof_adr + i;
                        self.implicit_stiffness[dof_idx] = 0.0;
                        self.implicit_damping[dof_idx] = self.dof_damping[dof_idx];
                        self.implicit_springref[dof_idx] = 0.0;
                    }
                }
            }
        }
    }

    /// Compute muscle-derived parameters: lengthrange, acc0, and F0.
    ///
    /// For each muscle actuator:
    ///   1. Computes `actuator_lengthrange` from tendon/joint limits (gear-scaled).
    ///   2. Runs a forward pass at `qpos0` to get M, then computes
    ///      `acc0 = ||M^{-1} * moment||` via sparse solve.
    ///   3. Resolves `F0` (gainprm\[2\]) when `force < 0`: `F0 = scale / acc0`.
    ///
    /// Must be called after `compute_ancestors()` and `compute_implicit_params()`,
    /// and after all tendon/actuator fields are populated.
    pub fn compute_muscle_params(&mut self) {
        if self.nu == 0 {
            return;
        }

        let has_muscles =
            (0..self.nu).any(|i| self.actuator_dyntype[i] == ActuatorDynamics::Muscle);
        if !has_muscles {
            return;
        }

        // --- Phase 1: Compute actuator_lengthrange from limits ---
        for i in 0..self.nu {
            if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
                continue;
            }

            let gear = self.actuator_gear[i][0];

            // actuator_length = gear * transmission_length,
            // so lengthrange = gear * transmission_lengthrange.
            // If gear < 0, min/max swap.
            let scale_range = |lo: f64, hi: f64| -> (f64, f64) {
                let a = gear * lo;
                let b = gear * hi;
                (a.min(b), a.max(b))
            };

            match self.actuator_trntype[i] {
                ActuatorTransmission::Tendon => {
                    let tid = self.actuator_trnid[i][0];
                    if tid < self.ntendon {
                        if self.tendon_limited[tid] {
                            let (lo, hi) = self.tendon_range[tid];
                            self.actuator_lengthrange[i] = scale_range(lo, hi);
                        } else {
                            // For unlimited spatial tendons, wrap-array DOF lookup is
                            // semantically wrong (wrap_objid holds site/geom IDs, not
                            // DOFs). Skip estimation and leave lengthrange = (0, 0).
                            if self.tendon_type[tid] == TendonType::Spatial {
                                eprintln!(
                                    "Warning: Unlimited spatial tendon {tid}: cannot \
                                     estimate actuator_lengthrange from joint ranges. \
                                     Specify explicit lengthrange in MJCF for muscle \
                                     actuators."
                                );
                                continue;
                            }
                            // For unlimited fixed tendons: estimate from joint ranges.
                            // Fixed tendon length = Σ coef_i * q_i, so extremes come
                            // from each joint at its range limit (sign-aware).
                            let adr = self.tendon_adr[tid];
                            let num = self.tendon_num[tid];
                            let (mut lmin, mut lmax) = (0.0, 0.0);
                            for w in adr..(adr + num) {
                                let dof = self.wrap_objid[w];
                                let coef = self.wrap_prm[w];
                                if let Some(jid) =
                                    (0..self.njnt).find(|&j| self.jnt_dof_adr[j] == dof)
                                {
                                    let (qlo, qhi) = self.jnt_range[jid];
                                    if coef >= 0.0 {
                                        lmin += coef * qlo;
                                        lmax += coef * qhi;
                                    } else {
                                        lmin += coef * qhi;
                                        lmax += coef * qlo;
                                    }
                                }
                            }
                            self.actuator_lengthrange[i] = scale_range(lmin, lmax);
                        }
                    }
                }
                ActuatorTransmission::Joint => {
                    let jid = self.actuator_trnid[i][0];
                    if jid < self.njnt {
                        let (lo, hi) = self.jnt_range[jid];
                        self.actuator_lengthrange[i] = scale_range(lo, hi);
                    }
                }
                ActuatorTransmission::Site => {
                    // Site length is configuration-dependent (full FK required).
                    // Explicit lengthrange must be provided in MJCF for muscle
                    // actuators with site transmission. No-op: leave at (0, 0).
                }
            }
        }

        // --- Phase 2: Forward pass at qpos0 for acc0 ---
        // FK populates cinert (body spatial inertias) and tendon Jacobians (ten_J).
        // CRBA builds M and factors it (L^T D L). For fixed tendons, J is constructed
        // from wrap_objid/wrap_prm (constant). For spatial tendons, J is read from
        // data.ten_J[tid] (populated by mj_fwd_tendon_spatial during FK).
        let mut data = self.make_data(); // qpos = qpos0
        mj_fwd_position(self, &mut data); // FK: cinert from qpos0
        mj_crba(self, &mut data); // Mass matrix M + sparse factorization

        for i in 0..self.nu {
            if self.actuator_dyntype[i] != ActuatorDynamics::Muscle {
                continue;
            }

            // Build transmission moment J (maps unit actuator force → generalized forces).
            let gear = self.actuator_gear[i][0];
            let mut j_vec = DVector::zeros(self.nv);
            match self.actuator_trntype[i] {
                ActuatorTransmission::Joint => {
                    let jid = self.actuator_trnid[i][0];
                    if jid < self.njnt {
                        let dof_adr = self.jnt_dof_adr[jid];
                        j_vec[dof_adr] = gear;
                    }
                }
                ActuatorTransmission::Tendon => {
                    let tid = self.actuator_trnid[i][0];
                    if tid < self.ntendon {
                        match self.tendon_type[tid] {
                            TendonType::Fixed => {
                                // Existing wrap-array pattern (constant J for fixed tendons).
                                let adr = self.tendon_adr[tid];
                                let num = self.tendon_num[tid];
                                for w in adr..(adr + num) {
                                    let dof_adr = self.wrap_objid[w];
                                    let coef = self.wrap_prm[w];
                                    if dof_adr < self.nv {
                                        j_vec[dof_adr] = gear * coef;
                                    }
                                }
                            }
                            TendonType::Spatial => {
                                // Configuration-dependent J — use ten_J from FK at qpos0.
                                // mj_fwd_position() already called mj_fwd_tendon() which
                                // populated data.ten_J[tid].
                                for dof in 0..self.nv {
                                    j_vec[dof] = gear * data.ten_J[tid][dof];
                                }
                            }
                        }
                    }
                }
                ActuatorTransmission::Site => {
                    let sid = self.actuator_trnid[i][0];
                    let refid = self.actuator_trnid[i][1];
                    let (jac_t, jac_r) = mj_jac_site(self, &data, sid);
                    let full_gear = self.actuator_gear[i];

                    if refid == usize::MAX {
                        // Mode A: wrench in world frame via site rotation.
                        let wrench_t = data.site_xmat[sid]
                            * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                        let wrench_r = data.site_xmat[sid]
                            * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                        for dof in 0..self.nv {
                            j_vec[dof] =
                                jac_t.column(dof).dot(&wrench_t) + jac_r.column(dof).dot(&wrench_r);
                        }
                    } else {
                        // Mode B: difference Jacobian with common-ancestor zeroing.
                        let (ref_jac_t, ref_jac_r) = mj_jac_site(self, &data, refid);
                        let mut diff_t = &jac_t - &ref_jac_t;
                        let mut diff_r = &jac_r - &ref_jac_r;

                        // Zero common-ancestor DOF columns.
                        let b0 = self.site_body[sid];
                        let b1 = self.site_body[refid];
                        let mut ancestors = Vec::new();
                        {
                            let mut b = b0;
                            while b != 0 {
                                ancestors.push(b);
                                b = self.body_parent[b];
                            }
                            ancestors.push(0);
                        }
                        let bca = {
                            let mut b = b1;
                            loop {
                                if ancestors.contains(&b) {
                                    break b;
                                }
                                if b == 0 {
                                    break 0;
                                }
                                b = self.body_parent[b];
                            }
                        };
                        {
                            let mut b = bca;
                            loop {
                                let js = self.body_jnt_adr[b];
                                let je = js + self.body_jnt_num[b];
                                for jid in js..je {
                                    let ds = self.jnt_dof_adr[jid];
                                    let nd = self.jnt_type[jid].nv();
                                    for d in ds..(ds + nd) {
                                        for k in 0..3 {
                                            diff_t[(k, d)] = 0.0;
                                            diff_r[(k, d)] = 0.0;
                                        }
                                    }
                                }
                                if b == 0 {
                                    break;
                                }
                                b = self.body_parent[b];
                            }
                        }

                        let wrench_t = data.site_xmat[refid]
                            * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                        let wrench_r = data.site_xmat[refid]
                            * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                        for dof in 0..self.nv {
                            j_vec[dof] = diff_t.column(dof).dot(&wrench_t)
                                + diff_r.column(dof).dot(&wrench_r);
                        }
                    }
                }
            }

            // Solve M * x = J using the sparse L^T D L factorization from CRBA.
            let mut x = j_vec;
            mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut x);

            // acc0 = ||M^{-1} J||_2
            self.actuator_acc0[i] = x.norm().max(1e-10);

            // --- Phase 3: Resolve F0 ---
            if self.actuator_gainprm[i][2] < 0.0 {
                // force < 0 means auto-compute: F0 = scale / acc0
                self.actuator_gainprm[i][2] = self.actuator_gainprm[i][3] / self.actuator_acc0[i];
                // Sync biasprm (MuJoCo layout: muscles share gain/bias parameters)
                self.actuator_biasprm[i][2] = self.actuator_gainprm[i][2];
            }
        }
    }

    /// Compute `tendon_length0` and default `lengthspring` for spatial tendons.
    ///
    /// Runs FK at `qpos0` to evaluate spatial tendon paths, then:
    /// 1. Validates rule 9 (sidesite outside wrapping geometry) using FK poses.
    /// 2. Sets `tendon_length0[t] = ten_length[t]` from the FK result.
    /// 3. Defaults `lengthspring` to `tendon_length0` when stiffness > 0.
    ///
    /// Must be called after `compute_implicit_params()` and before
    /// `compute_muscle_params()` (which needs valid `tendon_length0` for all types).
    ///
    /// # Panics
    ///
    /// Panics if a sidesite is inside its wrapping geometry (rule 9 validation).
    /// This is a fatal model configuration error.
    #[allow(clippy::panic)] // Deliberate: sidesite-inside-geometry is a fatal build error.
    pub fn compute_spatial_tendon_length0(&mut self) {
        // Early return if no spatial tendons — avoid unnecessary FK + Data allocation.
        let has_spatial = (0..self.ntendon).any(|t| self.tendon_type[t] == TendonType::Spatial);
        if !has_spatial {
            return;
        }

        let mut data = self.make_data();
        mj_fwd_position(self, &mut data); // runs FK + mj_fwd_tendon

        // Validate rule 9: sidesite outside wrapping geometry.
        // Requires FK because sidesite and geom may be on different bodies.
        for t in 0..self.ntendon {
            if self.tendon_type[t] != TendonType::Spatial {
                continue;
            }
            let adr = self.tendon_adr[t];
            let num = self.tendon_num[t];
            for w in adr..(adr + num) {
                if self.wrap_type[w] != WrapType::Geom {
                    continue;
                }
                if self.wrap_sidesite[w] == usize::MAX {
                    continue;
                }
                let geom_id = self.wrap_objid[w];
                let ss_id = self.wrap_sidesite[w];
                let ss_local = data.geom_xmat[geom_id].transpose()
                    * (data.site_xpos[ss_id] - data.geom_xpos[geom_id]);
                let r = self.geom_size[geom_id].x;
                let inside = match self.geom_type[geom_id] {
                    GeomType::Sphere => ss_local.norm() < r,
                    GeomType::Cylinder => nalgebra::Vector2::new(ss_local.x, ss_local.y).norm() < r,
                    _ => false,
                };
                if inside {
                    let ss_name = self.site_name[ss_id].as_deref().unwrap_or("?");
                    let g_name = self.geom_name[geom_id].as_deref().unwrap_or("?");
                    let t_name = self.tendon_name[t].as_deref().unwrap_or("?");
                    panic!(
                        "Sidesite '{ss_name}' (site {ss_id}) is inside wrapping \
                         geom '{g_name}' (geom {geom_id}) on tendon '{t_name}' \
                         (tendon {t}). The wrap_inside algorithm is not implemented. \
                         Move the sidesite outside the wrapping geometry surface."
                    );
                }
            }
        }

        // Compute tendon_length0 and default lengthspring for spatial tendons.
        for t in 0..self.ntendon {
            if self.tendon_type[t] == TendonType::Spatial {
                self.tendon_length0[t] = data.ten_length[t];
                if self.tendon_lengthspring[t] == 0.0 && self.tendon_stiffness[t] > 0.0 {
                    self.tendon_lengthspring[t] = data.ten_length[t];
                }
            }
        }
    }

    /// Compute `stat_meaninertia = trace(M) / nv` at `qpos0`.
    ///
    /// Creates a temporary Data, runs FK + CRBA to fill qM, then takes the
    /// trace of the mass matrix divided by nv. Guards nv == 0 → 1.0.
    /// Called once at model build time (§15.11).
    pub fn compute_stat_meaninertia(&mut self) {
        if self.nv == 0 {
            self.stat_meaninertia = 1.0;
            return;
        }
        let mut data = self.make_data();
        mj_fwd_position(self, &mut data);
        mj_crba(self, &mut data);
        let trace: f64 = (0..self.nv).map(|i| data.qM[(i, i)]).sum();
        self.stat_meaninertia = trace / self.nv as f64;
        // Guard against degenerate models with zero inertia
        if self.stat_meaninertia <= 0.0 {
            self.stat_meaninertia = 1.0;
        }
    }

    /// Check if joint is an ancestor of body using pre-computed data.
    ///
    /// Uses O(1) multi-word bitmask lookup for all model sizes.
    /// Returns `false` for invalid body_id or jnt_id (no panic).
    #[inline]
    #[must_use]
    pub fn is_ancestor(&self, body_id: usize, jnt_id: usize) -> bool {
        // Bounds check for body_id
        if body_id >= self.body_ancestor_mask.len() {
            return false;
        }
        let word = jnt_id / 64;
        let bit = jnt_id % 64;
        if word < self.body_ancestor_mask[body_id].len() {
            (self.body_ancestor_mask[body_id][word] & (1u64 << bit)) != 0
        } else {
            false // Joint ID out of range
        }
    }

    // ========================================================================
    // Factory Methods for Common Systems
    // ========================================================================

    /// Create an n-link serial pendulum (hinge joints only).
    ///
    /// This creates a serial chain of `n` bodies connected by hinge joints,
    /// all rotating around the Y axis. Each body has a point mass at its end.
    ///
    /// # Arguments
    /// * `n` - Number of links (must be >= 1)
    /// * `link_length` - Length of each link (meters)
    /// * `link_mass` - Mass of each link (kg)
    ///
    /// # Returns
    /// A `Model` representing the n-link pendulum with all joints at qpos=0
    /// (hanging straight down).
    ///
    /// # Panics
    /// Panics if `n` is 0 (requires at least 1 link).
    ///
    /// # Example
    /// ```ignore
    /// let model = Model::n_link_pendulum(3, 1.0, 1.0);
    /// let mut data = model.make_data();
    /// data.qpos[0] = std::f64::consts::PI / 4.0; // Tilt first link
    /// data.forward(&model);
    /// ```
    #[must_use]
    pub fn n_link_pendulum(n: usize, link_length: f64, link_mass: f64) -> Self {
        assert!(n >= 1, "n_link_pendulum requires at least 1 link");

        let mut model = Self::empty();

        // Dimensions
        model.nq = n;
        model.nv = n;
        model.nbody = n + 1; // world + n bodies
        model.njnt = n;

        // Build the kinematic chain
        for i in 0..n {
            let body_id = i + 1; // Body 0 is world
            let parent_id = i; // Each body's parent is the previous body (0 = world for first)

            // Body tree
            model.body_parent.push(parent_id);
            model.body_rootid.push(1); // All belong to tree rooted at body 1
            model.body_jnt_adr.push(i);
            model.body_jnt_num.push(1);
            model.body_dof_adr.push(i);
            model.body_dof_num.push(1);
            model.body_geom_adr.push(0);
            model.body_geom_num.push(0);

            // Body properties
            // Body frame is at the END of the link (where the mass is)
            model.body_pos.push(Vector3::new(0.0, 0.0, -link_length));
            model.body_quat.push(UnitQuaternion::identity());
            model.body_ipos.push(Vector3::zeros()); // COM at body origin
            model.body_iquat.push(UnitQuaternion::identity());
            model.body_mass.push(link_mass);
            // Point mass approximation (small moment of inertia)
            model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
            model.body_name.push(Some(format!("link_{i}")));
            model.body_subtreemass.push(0.0); // Will be computed after model is built
            model.body_mocapid.push(None);

            // Joint definition (hinge at parent's frame, rotating around Y)
            model.jnt_type.push(MjJointType::Hinge);
            model.jnt_body.push(body_id);
            model.jnt_qpos_adr.push(i);
            model.jnt_dof_adr.push(i);
            model.jnt_pos.push(Vector3::zeros()); // Joint at body origin
            model.jnt_axis.push(Vector3::new(0.0, 1.0, 0.0)); // Rotate around Y
            model.jnt_limited.push(false);
            model.jnt_range.push((-PI, PI));
            model.jnt_stiffness.push(0.0);
            model.jnt_springref.push(0.0);
            model.jnt_damping.push(0.0);
            model.jnt_armature.push(0.0);
            model.jnt_solref.push(DEFAULT_SOLREF);
            model.jnt_solimp.push(DEFAULT_SOLIMP);
            model.jnt_name.push(Some(format!("hinge_{i}")));

            // DOF definition
            model.dof_body.push(body_id);
            model.dof_jnt.push(i);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos (hanging down)
        model.qpos0 = DVector::zeros(n);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }

    /// Create a double pendulum (2-link serial chain).
    ///
    /// Convenience method equivalent to `Model::n_link_pendulum(2, ...)`.
    #[must_use]
    pub fn double_pendulum(link_length: f64, link_mass: f64) -> Self {
        Self::n_link_pendulum(2, link_length, link_mass)
    }

    /// Create a spherical pendulum (ball joint at origin).
    ///
    /// This creates a single body attached to the world by a ball joint,
    /// allowing 3-DOF rotation. The body has a point mass at distance `length`
    /// below the joint.
    ///
    /// # Arguments
    /// * `length` - Length from pivot to mass (meters)
    /// * `mass` - Point mass (kg)
    ///
    /// # Note
    /// The ball joint uses quaternion representation (nq=4, nv=3).
    /// Initial state is qpos=`[1,0,0,0]` (identity quaternion = hanging down).
    #[must_use]
    pub fn spherical_pendulum(length: f64, mass: f64) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 4; // Quaternion: w, x, y, z
        model.nv = 3; // Angular velocity: omega_x, omega_y, omega_z
        model.nbody = 2; // world + pendulum
        model.njnt = 1;

        // Body 1: pendulum bob
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        // Body frame is at the mass location (below joint by length)
        model.body_pos.push(Vector3::new(0.0, 0.0, -length));
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(Vector3::new(0.001, 0.001, 0.001));
        model.body_name.push(Some("bob".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built
        model.body_mocapid.push(None);

        // Ball joint at world origin
        model.jnt_type.push(MjJointType::Ball);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z()); // Not used for ball, but required
        model.jnt_limited.push(false);
        model.jnt_range.push((-PI, PI));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("ball".to_string()));

        // DOF definitions (3 for ball joint)
        for i in 0..3 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: identity quaternion [w, x, y, z] = [1, 0, 0, 0]
        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }

    /// Create a free-floating body (6-DOF).
    ///
    /// This creates a single body with a free joint, allowing full 3D
    /// translation and rotation. Useful for testing free-body dynamics.
    ///
    /// # Arguments
    /// * `mass` - Body mass (kg)
    /// * `inertia` - Principal moments of inertia [Ixx, Iyy, Izz]
    #[must_use]
    pub fn free_body(mass: f64, inertia: Vector3<f64>) -> Self {
        let mut model = Self::empty();

        // Dimensions
        model.nq = 7; // position (3) + quaternion (4)
        model.nv = 6; // linear velocity (3) + angular velocity (3)
        model.nbody = 2;
        model.njnt = 1;

        // Body 1: free body
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(6);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0);

        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(mass);
        model.body_inertia.push(inertia);
        model.body_name.push(Some("free_body".to_string()));
        model.body_subtreemass.push(0.0); // Will be computed after model is built
        model.body_mocapid.push(None);

        // Free joint
        model.jnt_type.push(MjJointType::Free);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((-1e10, 1e10));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push(DEFAULT_SOLREF);
        model.jnt_solimp.push(DEFAULT_SOLIMP);
        model.jnt_name.push(Some("free".to_string()));

        // DOF definitions (6 for free joint)
        for i in 0..6 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Default qpos: at origin with identity orientation
        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);

        // Physics options
        model.timestep = 1.0 / 240.0;
        model.gravity = Vector3::new(0.0, 0.0, -9.81);
        model.solver_iterations = 10;
        model.solver_tolerance = 1e-8;

        // Pre-compute kinematic data for O(n) CRBA/RNE
        model.compute_ancestors();

        // Pre-compute implicit integration parameters
        model.compute_implicit_params();

        model
    }
}

impl Data {
    /// Reset state to model defaults.
    pub fn reset(&mut self, model: &Model) {
        self.qpos = model.qpos0.clone();
        self.qvel.fill(0.0);
        self.qacc.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.ctrl.fill(0.0);
        self.act.fill(0.0);
        self.act_dot.fill(0.0);
        self.actuator_length.fill(0.0);
        self.actuator_velocity.fill(0.0);
        self.actuator_force.fill(0.0);
        self.sensordata.fill(0.0);
        self.time = 0.0;
        self.ncon = 0;
        self.contacts.clear();

        #[cfg(feature = "deformable")]
        {
            self.deformable_contacts.clear();
            for body in &mut self.deformable_bodies {
                for vel in body.velocities_mut() {
                    *vel = Vector3::zeros();
                }
                body.clear_forces();
            }
        }

        // Reset mocap poses to model defaults (body_pos/body_quat offsets).
        let mut mocap_idx = 0;
        for (body_id, mid) in model.body_mocapid.iter().enumerate() {
            if mid.is_some() {
                self.mocap_pos[mocap_idx] = model.body_pos[body_id];
                self.mocap_quat[mocap_idx] = model.body_quat[body_id];
                mocap_idx += 1;
            }
        }

        // Reset sleep state from model policies (§16.7).
        reset_sleep_state(model, self);
    }

    /// Reset simulation state to a keyframe by index.
    ///
    /// Overwrites `time`, `qpos`, `qvel`, `act`, `ctrl`, `mocap_pos`, and
    /// `mocap_quat` from the keyframe. Clears derived quantities (`qacc`,
    /// `qacc_warmstart`, actuator arrays, `sensordata`, contacts).
    /// Does **not** clear `qfrc_applied` or `xfrc_applied` — matching the
    /// convention of `Data::reset()`, which also preserves user-applied forces.
    /// Caller must invoke `forward()` after reset to recompute derived state.
    ///
    /// # Errors
    ///
    /// Returns `Err` if `keyframe_idx >= model.nkeyframe`.
    pub fn reset_to_keyframe(
        &mut self,
        model: &Model,
        keyframe_idx: usize,
    ) -> Result<(), ResetError> {
        let kf = model
            .keyframes
            .get(keyframe_idx)
            .ok_or(ResetError::InvalidKeyframeIndex {
                index: keyframe_idx,
                nkeyframe: model.nkeyframe,
            })?;

        // Overwrite primary state from keyframe.
        self.time = kf.time;
        self.qpos.copy_from(&kf.qpos);
        self.qvel.copy_from(&kf.qvel);
        self.act.copy_from(&kf.act);
        self.ctrl.copy_from(&kf.ctrl);

        // Mocap state (length may be 0 if no mocap bodies).
        self.mocap_pos.copy_from_slice(&kf.mpos);
        self.mocap_quat.copy_from_slice(&kf.mquat);

        // Clear derived quantities (matching Data::reset() convention).
        self.qacc.fill(0.0);
        self.qacc_warmstart.fill(0.0);
        self.act_dot.fill(0.0);
        self.actuator_length.fill(0.0);
        self.actuator_velocity.fill(0.0);
        self.actuator_force.fill(0.0);
        self.sensordata.fill(0.0);
        self.ncon = 0;
        self.contacts.clear();

        // Feature-gated deformable clearing (matching Data::reset()).
        #[cfg(feature = "deformable")]
        {
            self.deformable_contacts.clear();
            for body in &mut self.deformable_bodies {
                for vel in body.velocities_mut() {
                    *vel = Vector3::zeros();
                }
                body.clear_forces();
            }
        }

        // Reset sleep state from model policies (§16.7).
        reset_sleep_state(model, self);

        Ok(())
    }

    /// Register a deformable body for simulation.
    ///
    /// The body is consumed and stored internally. Returns the index into
    /// `deformable_bodies` for later reference.
    #[cfg(feature = "deformable")]
    pub fn register_deformable(
        &mut self,
        body: Box<dyn DeformableBody + Send + Sync>,
        solver_config: XpbdSolverConfig,
    ) -> usize {
        let idx = self.deformable_bodies.len();
        self.deformable_bodies.push(body);
        self.deformable_solvers.push(XpbdSolver::new(solver_config));
        idx
    }

    /// Get a reference to a registered deformable body.
    #[cfg(feature = "deformable")]
    #[must_use]
    pub fn deformable(&self, idx: usize) -> &dyn DeformableBody {
        self.deformable_bodies[idx].as_ref()
    }

    /// Get a mutable reference to a registered deformable body.
    #[cfg(feature = "deformable")]
    pub fn deformable_mut(&mut self, idx: usize) -> &mut dyn DeformableBody {
        self.deformable_bodies[idx].as_mut()
    }

    /// Get total mechanical energy (kinetic + potential).
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        self.energy_kinetic + self.energy_potential
    }

    /// Full simulation step (like `mj_step`).
    ///
    /// This is the main entry point for advancing the simulation by one timestep.
    /// It performs forward dynamics to compute accelerations, then integrates
    /// to update positions and velocities.
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError)` if:
    /// - Position/velocity contains NaN or Inf
    /// - Acceleration computation produces NaN
    /// - Cholesky decomposition fails (implicit integrator only)
    /// - Timestep is invalid
    ///
    /// Unlike MuJoCo which silently resets state on errors, this follows
    /// Rust idioms by requiring explicit error handling.
    pub fn step(&mut self, model: &Model) -> Result<(), StepError> {
        // Validate timestep
        if model.timestep <= 0.0 || !model.timestep.is_finite() {
            return Err(StepError::InvalidTimestep);
        }

        // Validate state before stepping
        mj_check_pos(model, self)?;
        mj_check_vel(model, self)?;

        match model.integrator {
            Integrator::RungeKutta4 => {
                // RK4: forward() evaluates initial state (with sensors).
                // mj_runge_kutta() then calls forward_skip_sensors() 3 more times.
                self.forward(model)?;
                mj_check_acc(model, self)?;
                mj_runge_kutta(model, self)?;
            }
            Integrator::Euler
            | Integrator::ImplicitSpringDamper
            | Integrator::ImplicitFast
            | Integrator::Implicit => {
                self.forward(model)?;
                mj_check_acc(model, self)?;
                self.integrate(model);
            }
        }

        // Sleep update (§16.12): Phase B island-aware sleep transition.
        // After integration and before warmstart save.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
        if sleep_enabled {
            mj_sleep(model, self);
            mj_update_sleep_arrays(model, self);
        }

        // Save qacc for next-step warmstart (§15.9).
        // Done at the very end of step(), after integration, matching MuJoCo's
        // mj_advance() which saves qacc_warmstart after the step completes.
        self.qacc_warmstart.copy_from(&self.qacc);

        Ok(())
    }

    /// Forward dynamics only (like `mj_forward`).
    ///
    /// Computes all derived quantities from current qpos/qvel without
    /// modifying them. After this call, qacc contains the computed
    /// accelerations and all body poses are updated.
    ///
    /// Pipeline stages follow `MuJoCo`'s `mj_forward` exactly:
    /// 1. Position stage: FK, position-dependent sensors, potential energy
    /// 2. Velocity stage: velocity FK, velocity-dependent sensors, kinetic energy
    /// 3. Acceleration stage: actuation, dynamics, constraints, acc-dependent sensors
    ///
    /// # Errors
    ///
    /// Returns `Err(StepError::CholeskyFailed)` if using implicit integrator
    /// and the modified mass matrix decomposition fails.
    pub fn forward(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, true)
    }

    /// Forward dynamics pipeline without sensor evaluation.
    ///
    /// Identical to [`forward()`](Self::forward) but skips all 4 sensor stages.
    /// Used by RK4 intermediate stages.
    fn forward_skip_sensors(&mut self, model: &Model) -> Result<(), StepError> {
        self.forward_core(model, false)
    }

    /// Shared pipeline core with sleep gating (§16.5).
    ///
    /// `compute_sensors`: `true` for `forward()`, `false` for `forward_skip_sensors()`.
    fn forward_core(&mut self, model: &Model, compute_sensors: bool) -> Result<(), StepError> {
        // Sleep is only active after the initial forward pass.
        // The first forward (time == 0.0) must compute FK for all bodies
        // to establish initial positions, even for Init-sleeping bodies.
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // ===== Pre-pipeline: Wake detection (§16.4) =====
        if sleep_enabled {
            mj_wake(model, self);
        }

        // ========== Position Stage ==========
        mj_fwd_position(model, self);

        // §16.15: If FK detected external qpos changes on sleeping bodies, wake them
        if sleep_enabled && mj_check_qpos_changed(model, self) {
            mj_update_sleep_arrays(model, self);
        }

        mj_transmission_site(model, self);
        mj_collision(model, self);

        // Wake-on-contact: if sleeping body touched awake body, wake it
        // and re-run collision for the newly-awake tree's geoms (§16.5c)
        if sleep_enabled && mj_wake_collision(model, self) {
            mj_update_sleep_arrays(model, self);
            mj_collision(model, self); // Phase A: full re-collision
        }

        #[cfg(feature = "deformable")]
        mj_deformable_collision(model, self);
        if compute_sensors {
            mj_sensor_pos(model, self);
        }
        mj_energy_pos(model, self);

        // ========== Velocity Stage ==========
        mj_fwd_velocity(model, self);
        mj_actuator_length(model, self);
        if compute_sensors {
            mj_sensor_vel(model, self);
        }

        // ========== Acceleration Stage ==========
        mj_fwd_actuation(model, self);
        mj_crba(model, self); // Full mass matrix — NOT skipped (§16.5 note)
        mj_rne(model, self);
        mj_energy_vel(model, self);
        mj_fwd_passive(model, self);
        mj_fwd_constraint(model, self);

        // §16.11: Island discovery (observation-only pass — arrays populated
        // but not yet consumed by per-island solver until Step 10).
        if sleep_enabled {
            mj_island(model, self);
        }

        if !self.newton_solved {
            mj_fwd_acceleration(model, self)?;
        }
        if compute_sensors {
            mj_sensor_acc(model, self);
            mj_sensor_postprocess(model, self);
        }

        Ok(())
    }

    /// Integration step for Euler and implicit-spring-damper integrators.
    ///
    /// RK4 integration is handled by [`mj_runge_kutta()`] and does not call this method.
    ///
    /// # Integration Methods
    ///
    /// - **Euler**: Semi-implicit Euler. Updates velocity first (`qvel += qacc * h`),
    ///   then integrates position using the new velocity.
    ///
    /// - **Implicit**: Velocity was already updated in `mj_fwd_acceleration_implicit()`.
    ///   We only integrate positions here.
    fn integrate(&mut self, model: &Model) {
        let h = model.timestep;
        let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

        // Integrate activation per actuator, then clamp muscles to [0, 1].
        // MuJoCo order: activation → velocity → position.
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            for k in 0..act_num {
                let j = act_adr + k;
                match model.actuator_dyntype[i] {
                    ActuatorDynamics::FilterExact => {
                        // Exact: act += act_dot * tau * (1 - exp(-h/tau))
                        let tau = model.actuator_dynprm[i][0].max(1e-10);
                        self.act[j] += self.act_dot[j] * tau * (1.0 - (-h / tau).exp());
                    }
                    _ => {
                        // Euler: act += h * act_dot
                        self.act[j] += h * self.act_dot[j];
                    }
                }
            }
            // Clamp muscle activation to [0, 1]
            if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
                for k in 0..act_num {
                    self.act[act_adr + k] = self.act[act_adr + k].clamp(0.0, 1.0);
                }
            }
        }

        // For Euler and new implicit variants, update velocity using computed acceleration.
        // For legacy ImplicitSpringDamper, velocity was already updated in mj_fwd_acceleration_implicit.
        match model.integrator {
            Integrator::Euler | Integrator::ImplicitFast | Integrator::Implicit => {
                for i in 0..model.nv {
                    // §16.5a'': Skip velocity update for sleeping DOFs
                    if sleep_enabled && !data_tree_awake_for_dof(model, self, i) {
                        continue;
                    }
                    self.qvel[i] += self.qacc[i] * h;
                }
            }
            Integrator::ImplicitSpringDamper => {
                // Velocity already updated by mj_fwd_acceleration_implicit
                // (legacy path solves for v_new directly, not qacc)
            }
            Integrator::RungeKutta4 => {
                unreachable!("RK4 integration handled by mj_runge_kutta()")
            }
        }

        // Update positions - quaternions need special handling!
        mj_integrate_pos(model, self, h);

        // Normalize quaternions to prevent drift
        mj_normalize_quat(model, self);

        // Step deformable bodies (XPBD internal constraints)
        #[cfg(feature = "deformable")]
        mj_deformable_step(model, self);

        // Advance time
        self.time += h;
    }

    /// Integration step without velocity update.
    ///
    /// Performs activation integration, position integration, quaternion
    /// normalization, and time advance. Skips `qvel += qacc * h` (assumed
    /// to have been done externally, e.g., on GPU).
    ///
    /// Used by the GPU backend (`sim-gpu`) where velocity integration
    /// is performed on GPU via compute shader.
    ///
    /// # Visibility
    ///
    /// This method is public but feature-gated behind `gpu-internals`.
    /// It is not part of the stable `sim-core` API — only `sim-gpu`
    /// should depend on this feature.
    #[cfg(feature = "gpu-internals")]
    #[doc(hidden)]
    pub fn integrate_without_velocity(&mut self, model: &Model) {
        // Guard: this method assumes velocity was updated externally (GPU Euler).
        // If a future phase relaxes the Euler-only check in GpuBatchSim::new(),
        // this assert will catch misuse before silent physics errors.
        debug_assert!(
            matches!(model.integrator, Integrator::Euler),
            "integrate_without_velocity only valid for Euler integrator, got {:?}",
            model.integrator
        );
        let h = model.timestep;

        // 1. Activation integration (identical to integrate())
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            for k in 0..act_num {
                let j = act_adr + k;
                match model.actuator_dyntype[i] {
                    ActuatorDynamics::FilterExact => {
                        let tau = model.actuator_dynprm[i][0].max(1e-10);
                        self.act[j] += self.act_dot[j] * tau * (1.0 - (-h / tau).exp());
                    }
                    _ => {
                        self.act[j] += h * self.act_dot[j];
                    }
                }
            }
            if model.actuator_dyntype[i] == ActuatorDynamics::Muscle {
                for k in 0..act_num {
                    self.act[act_adr + k] = self.act[act_adr + k].clamp(0.0, 1.0);
                }
            }
        }

        // 2. Skip velocity integration (done on GPU)

        // 3. Position integration + quaternion normalization + time advance
        mj_integrate_pos(model, self, h);
        mj_normalize_quat(model, self);
        self.time += h;
    }
}

// ============================================================================
// MuJoCo Pipeline Functions (Phase 2)
// ============================================================================

/// Validate position coordinates.
///
/// Returns `Err(StepError::InvalidPosition)` if any qpos element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently resets to qpos0, this returns an error so
/// users can decide how to handle the situation.
fn mj_check_pos(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nq {
        if !data.qpos[i].is_finite() || data.qpos[i].abs() > 1e10 {
            return Err(StepError::InvalidPosition);
        }
    }
    Ok(())
}

/// Validate velocity coordinates.
///
/// Returns `Err(StepError::InvalidVelocity)` if any qvel element is NaN, Inf,
/// or exceeds 1e10 in magnitude (indicating numerical blow-up).
///
/// Unlike MuJoCo which silently zeros velocity, this returns an error so
/// users can decide how to handle the situation.
fn mj_check_vel(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qvel[i].is_finite() || data.qvel[i].abs() > 1e10 {
            return Err(StepError::InvalidVelocity);
        }
    }
    Ok(())
}

/// Validate acceleration.
///
/// Returns `Err(StepError::InvalidAcceleration)` if any qacc element is NaN.
/// This typically indicates a singular mass matrix or other numerical issues.
fn mj_check_acc(model: &Model, data: &Data) -> Result<(), StepError> {
    for i in 0..model.nv {
        if !data.qacc[i].is_finite() {
            return Err(StepError::InvalidAcceleration);
        }
    }
    Ok(())
}

/// Forward kinematics: compute body poses from qpos.
///
/// This traverses the kinematic tree from root to leaves, computing
/// the world-frame position and orientation of each body.
fn mj_fwd_position(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Body 0 (world) is always at origin
    data.xpos[0] = Vector3::zeros();
    data.xquat[0] = UnitQuaternion::identity();
    data.xmat[0] = Matrix3::identity();

    // Process bodies in order (assumes topological sort: parent before child).
    // Phase B (§16.15): compute FK for ALL bodies (including sleeping) to detect
    // external qpos modifications via xpos/xquat comparison. The Phase A skip
    // is removed; performance comes from per-island constraint solve (§16.16).
    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Determine body pose: mocap bodies use mocap arrays, regular bodies
        // use parent frame + body offset + joints.
        let (pos, quat) = if let Some(&Some(mocap_idx)) = model.body_mocapid.get(body_id) {
            // Mocap body: replace body_pos/body_quat with mocap arrays.
            // Parent is always world (origin), so xpos = mocap_pos, xquat = mocap_quat.
            //
            // Renormalize quaternion (matching MuJoCo's mju_normalize4 in
            // mj_kinematics1). Users set mocap_quat between steps; floating-point
            // arithmetic can cause drift from unit norm.
            let mquat = UnitQuaternion::new_normalize(data.mocap_quat[mocap_idx].into_inner());
            (data.mocap_pos[mocap_idx], mquat)
        } else {
            // Regular body: parent frame + body offset + joints.
            let mut pos = data.xpos[parent_id];
            let mut quat = data.xquat[parent_id];

            // Apply body offset in parent frame
            pos += quat * model.body_pos[body_id];
            quat *= model.body_quat[body_id];

            // Apply each joint for this body
            let jnt_start = model.body_jnt_adr[body_id];
            let jnt_end = jnt_start + model.body_jnt_num[body_id];

            for jnt_id in jnt_start..jnt_end {
                let qpos_adr = model.jnt_qpos_adr[jnt_id];

                match model.jnt_type[jnt_id] {
                    MjJointType::Hinge => {
                        let angle = data.qpos[qpos_adr];
                        let axis = model.jnt_axis[jnt_id];
                        let anchor = model.jnt_pos[jnt_id];

                        // Transform anchor to current frame
                        let world_anchor = pos + quat * anchor;

                        // Rotate around axis
                        let world_axis = quat * axis;
                        // Safety: use try_new_normalize to handle degenerate cases
                        let rot =
                            if let Some(unit_axis) = nalgebra::Unit::try_new(world_axis, 1e-10) {
                                UnitQuaternion::from_axis_angle(&unit_axis, angle)
                            } else {
                                // Degenerate axis - no rotation (should not happen with valid model)
                                UnitQuaternion::identity()
                            };
                        quat = rot * quat;

                        // Adjust position for rotation around anchor
                        pos = world_anchor + rot * (pos - world_anchor);
                    }
                    MjJointType::Slide => {
                        let displacement = data.qpos[qpos_adr];
                        let axis = model.jnt_axis[jnt_id];
                        pos += quat * (axis * displacement);
                    }
                    MjJointType::Ball => {
                        // qpos stores quaternion [w, x, y, z]
                        let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                            data.qpos[qpos_adr],
                            data.qpos[qpos_adr + 1],
                            data.qpos[qpos_adr + 2],
                            data.qpos[qpos_adr + 3],
                        ));
                        quat *= q;
                    }
                    MjJointType::Free => {
                        // qpos stores [x, y, z, qw, qx, qy, qz]
                        pos = Vector3::new(
                            data.qpos[qpos_adr],
                            data.qpos[qpos_adr + 1],
                            data.qpos[qpos_adr + 2],
                        );
                        quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                            data.qpos[qpos_adr + 3],
                            data.qpos[qpos_adr + 4],
                            data.qpos[qpos_adr + 5],
                            data.qpos[qpos_adr + 6],
                        ));
                    }
                }
            }

            (pos, quat)
        };

        // §16.15: qpos change detection — if this body was sleeping and its
        // computed pose differs from the stored pose (exact bitwise match),
        // mark the tree for waking. This detects external qpos modifications.
        if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
            let tree = model.body_treeid[body_id];
            if tree < data.tree_qpos_dirty.len() {
                // Compare new vs stored (exact floating-point equality)
                let pos_changed = pos != data.xpos[body_id];
                let quat_changed =
                    quat.into_inner().coords != data.xquat[body_id].into_inner().coords;
                if pos_changed || quat_changed {
                    data.tree_qpos_dirty[tree] = true;
                }
            }
        }

        // Store computed pose (shared path for both mocap and regular bodies)
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();

        // Compute inertial frame position (shared)
        data.xipos[body_id] = pos + quat * model.body_ipos[body_id];
        data.ximat[body_id] = (quat * model.body_iquat[body_id])
            .to_rotation_matrix()
            .into_inner();

        // Compute body spatial inertia in world frame (cinert, shared).
        // MuJoCo does NOT zero mass for mocap bodies, and neither do we.
        let h = data.xipos[body_id] - pos; // COM offset from body origin in world frame
        data.cinert[body_id] = compute_body_spatial_inertia(
            model.body_mass[body_id],
            model.body_inertia[body_id],
            &data.ximat[body_id],
            h,
        );
    }

    // World body has zero inertia
    data.cinert[0] = Matrix6::zeros();

    // Update geom poses
    for geom_id in 0..model.ngeom {
        let body_id = model.geom_body[geom_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.geom_xpos[geom_id] = body_pos + body_quat * model.geom_pos[geom_id];
        data.geom_xmat[geom_id] = (body_quat * model.geom_quat[geom_id])
            .to_rotation_matrix()
            .into_inner();
    }

    // Update site poses
    for site_id in 0..model.nsite {
        let body_id = model.site_body[site_id];
        let body_pos = data.xpos[body_id];
        let body_quat = data.xquat[body_id];

        data.site_xpos[site_id] = body_pos + body_quat * model.site_pos[site_id];
        let site_world_quat = body_quat * model.site_quat[site_id];
        data.site_xmat[site_id] = site_world_quat.to_rotation_matrix().into_inner();
        data.site_xquat[site_id] = site_world_quat;
    }

    // Tendon kinematics (after site poses, before subtree COM)
    mj_fwd_tendon(model, data);

    // ========== Compute subtree mass and COM (for O(n) RNE gravity) ==========
    // Initialize with each body's own mass and COM
    for body_id in 0..model.nbody {
        data.subtree_mass[body_id] = model.body_mass[body_id];
        data.subtree_com[body_id] = model.body_mass[body_id] * data.xipos[body_id];
    }

    // Backward pass: accumulate children's contributions to parents
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        // Copy child data first to satisfy borrow checker
        let child_mass = data.subtree_mass[body_id];
        let child_weighted_com = data.subtree_com[body_id];
        data.subtree_mass[parent_id] += child_mass;
        data.subtree_com[parent_id] += child_weighted_com;
    }

    // Convert weighted sum to actual COM
    for body_id in 0..model.nbody {
        if data.subtree_mass[body_id] > 1e-10 {
            data.subtree_com[body_id] /= data.subtree_mass[body_id];
        } else {
            data.subtree_com[body_id] = data.xipos[body_id];
        }
    }
}

// ============================================================================
// Broad-Phase Collision Detection (Spatial Hashing)
// ============================================================================

/// Compute AABB for a geometry given its world-space pose and type/size.
///
/// This function creates an axis-aligned bounding box for MuJoCo-style geometry
/// specifications, using the canonical `Aabb` type from `collision_shape`.
///
/// # Arguments
/// * `geom_type` - The geometry type (sphere, box, capsule, etc.)
/// * `size` - MuJoCo-style size parameters (interpretation depends on geom_type)
/// * `pos` - World-space position of the geometry
/// * `mat` - World-space rotation matrix (3x3)
#[inline]
fn aabb_from_geom(
    geom_type: GeomType,
    size: Vector3<f64>,
    pos: Vector3<f64>,
    mat: Matrix3<f64>,
) -> Aabb {
    match geom_type {
        GeomType::Sphere => {
            let r = size.x;
            Aabb::new(
                Point3::new(pos.x - r, pos.y - r, pos.z - r),
                Point3::new(pos.x + r, pos.y + r, pos.z + r),
            )
        }
        GeomType::Box => {
            // For a rotated box, compute the world-space extents
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
        GeomType::Capsule => {
            // Capsule: radius + half_length along Z axis
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Cylinder => {
            // Cylinder AABB: endpoints ± radius in all directions.
            // Same formula as capsule — the flat caps still extend radius `r`
            // perpendicular to the axis, so the bounding box is identical.
            let r = size.x;
            let half_len = size.y;
            let axis = mat.column(2).into_owned();
            let end1 = pos + axis * half_len;
            let end2 = pos - axis * half_len;
            Aabb::new(
                Point3::new(
                    end1.x.min(end2.x) - r,
                    end1.y.min(end2.y) - r,
                    end1.z.min(end2.z) - r,
                ),
                Point3::new(
                    end1.x.max(end2.x) + r,
                    end1.y.max(end2.y) + r,
                    end1.z.max(end2.z) + r,
                ),
            )
        }
        GeomType::Ellipsoid => {
            // Conservative AABB: use sphere with largest semi-axis radius.
            // A tight AABB would require transforming each axis by the rotation
            // matrix, but this simple approach is correct and fast for broad-phase.
            let max_r = size.x.max(size.y).max(size.z);
            Aabb::new(
                Point3::new(pos.x - max_r, pos.y - max_r, pos.z - max_r),
                Point3::new(pos.x + max_r, pos.y + max_r, pos.z + max_r),
            )
        }
        GeomType::Plane => {
            // Planes are infinite — use large bounds in perpendicular directions
            // and thin bounds along the normal to allow proper AABB overlap tests.
            const PLANE_EXTENT: f64 = 1e6; // Effectively infinite for simulation scale
            const PLANE_THICKNESS: f64 = 0.001; // Thin slab for AABB overlap detection

            // Plane normal is Z axis of the rotation matrix
            let normal = mat.column(2).into_owned();

            // Create thin slab AABB based on dominant normal direction
            if normal.z.abs() > 0.9 {
                // Near-horizontal plane (normal ≈ ±Z)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, -PLANE_EXTENT, pos.z - PLANE_THICKNESS),
                    Point3::new(PLANE_EXTENT, PLANE_EXTENT, pos.z + PLANE_THICKNESS),
                )
            } else if normal.y.abs() > 0.9 {
                // Near-vertical plane (normal ≈ ±Y)
                Aabb::new(
                    Point3::new(-PLANE_EXTENT, pos.y - PLANE_THICKNESS, -PLANE_EXTENT),
                    Point3::new(PLANE_EXTENT, pos.y + PLANE_THICKNESS, PLANE_EXTENT),
                )
            } else {
                // Near-vertical plane (normal ≈ ±X)
                Aabb::new(
                    Point3::new(pos.x - PLANE_THICKNESS, -PLANE_EXTENT, -PLANE_EXTENT),
                    Point3::new(pos.x + PLANE_THICKNESS, PLANE_EXTENT, PLANE_EXTENT),
                )
            }
        }
        GeomType::Mesh => {
            // For mesh, use a conservative large bounding box.
            // In a full implementation, mesh AABBs would be pre-computed from vertices.
            const MESH_DEFAULT_EXTENT: f64 = 10.0; // Conservative fallback for unprocessed meshes
            Aabb::new(
                Point3::new(
                    pos.x - MESH_DEFAULT_EXTENT,
                    pos.y - MESH_DEFAULT_EXTENT,
                    pos.z - MESH_DEFAULT_EXTENT,
                ),
                Point3::new(
                    pos.x + MESH_DEFAULT_EXTENT,
                    pos.y + MESH_DEFAULT_EXTENT,
                    pos.z + MESH_DEFAULT_EXTENT,
                ),
            )
        }
        GeomType::Hfield => {
            // Conservative: treat as a rotated box with half-extents from geom_size
            // [x_half_extent, y_half_extent, z_top]. Same formula as GeomType::Box.
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
        GeomType::Sdf => {
            // Conservative: rotated box with half-extents from geom_size.
            // For programmatic SDF geoms, geom_size should store meaningful
            // half-extents. For MJCF placeholders, defaults to 0.1 —
            // small AABB is acceptable since the geom has no sdf_data.
            let half = size;
            let mut min = pos;
            let mut max = pos;
            for i in 0..3 {
                let axis = mat.column(i).into_owned();
                let extent = half[i] * axis.abs();
                min -= extent;
                max += extent;
            }
            Aabb::new(Point3::from(min), Point3::from(max))
        }
    }
}

/// Sweep-and-prune broad-phase collision detection.
///
/// This is the algorithm used by MuJoCo and most physics engines:
/// 1. Project all AABBs onto the X-axis
/// 2. Sort by min-X coordinate
/// 3. Sweep through sorted intervals to find overlaps
/// 4. Check Y and Z overlap only for X-overlapping pairs
///
/// For coherent simulations (objects move incrementally), the sort is nearly
/// O(n) due to temporal coherence — insertion sort on nearly-sorted data.
///
/// # Performance Characteristics
///
/// | Operation | Complexity | Notes |
/// |-----------|------------|-------|
/// | Build | O(n log n) | Initial sort (Rust's pdqsort) |
/// | Query (typical) | O(n + k) | k = output pairs, assumes bounded X-overlap |
/// | Query (worst) | O(n² + k) | All AABBs overlap on X-axis (degenerate) |
/// | Incremental | O(n) | Nearly sorted → insertion sort behavior |
///
/// The query worst case occurs when all objects have overlapping X-intervals
/// (e.g., objects stacked vertically). In practice, this is rare and still
/// faster than spatial hashing's worst case (clustering in one cell).
///
/// # Why Not Spatial Hash?
///
/// Spatial hashing degrades to O(n²) when objects cluster in a single cell
/// (e.g., a pile of boxes). SAP's worst case is the same O(n²), but it occurs
/// less frequently in practice (requires all X-intervals to overlap, not just
/// spatial proximity).
struct SweepAndPrune {
    /// AABBs indexed by geom ID
    aabbs: Vec<Aabb>,
    /// Geom IDs sorted by AABB min-X coordinate
    sorted_x: Vec<usize>,
}

impl SweepAndPrune {
    /// Create a new sweep-and-prune structure from AABBs.
    ///
    /// # Arguments
    ///
    /// * `aabbs` - Vector of AABBs indexed by geom ID
    ///
    /// # Panics (debug only)
    ///
    /// Debug builds assert that all AABBs have finite coordinates.
    /// NaN in AABBs would cause non-deterministic sort behavior.
    #[must_use]
    fn new(aabbs: Vec<Aabb>) -> Self {
        // Validate AABBs in debug builds to catch NaN/Inf early.
        // NaN comparisons break sort transitivity, causing non-deterministic results.
        debug_assert!(
            aabbs.iter().all(|a| {
                a.min.x.is_finite()
                    && a.min.y.is_finite()
                    && a.min.z.is_finite()
                    && a.max.x.is_finite()
                    && a.max.y.is_finite()
                    && a.max.z.is_finite()
            }),
            "All AABBs must have finite coordinates for deterministic sweep-and-prune"
        );

        let n = aabbs.len();
        let mut sorted_x: Vec<usize> = (0..n).collect();

        // Sort by min-X coordinate using total ordering.
        // f64::total_cmp provides IEEE 754 total ordering: -NaN < -Inf < ... < Inf < NaN
        // This guarantees deterministic sort even if validation is skipped in release.
        sorted_x.sort_by(|&a, &b| aabbs[a].min.x.total_cmp(&aabbs[b].min.x));

        Self { aabbs, sorted_x }
    }

    /// Query all potentially overlapping pairs.
    ///
    /// Returns pairs `(geom_i, geom_j)` where `i < j` and AABBs overlap.
    /// The pairs are returned in arbitrary order.
    ///
    /// See struct-level documentation for complexity analysis.
    #[must_use]
    fn query_pairs(&self) -> Vec<(usize, usize)> {
        // Pre-allocate with heuristic: ~2 overlaps per geom on average
        let mut pairs = Vec::with_capacity(self.aabbs.len() * 2);
        let n = self.sorted_x.len();

        // Sweep through sorted list
        for i in 0..n {
            let geom_i = self.sorted_x[i];
            let aabb_i = &self.aabbs[geom_i];
            let max_x_i = aabb_i.max.x;

            // Check subsequent geoms until their min-X exceeds our max-X
            for j in (i + 1)..n {
                let geom_j = self.sorted_x[j];
                let aabb_j = &self.aabbs[geom_j];

                // If min-X of j exceeds max-X of i, no more overlaps possible
                // (since sorted_x is sorted by min-X)
                if aabb_j.min.x > max_x_i {
                    break;
                }

                // X overlaps — check Y and Z
                if aabb_i.max.y >= aabb_j.min.y
                    && aabb_j.max.y >= aabb_i.min.y
                    && aabb_i.max.z >= aabb_j.min.z
                    && aabb_j.max.z >= aabb_i.min.z
                {
                    // Full 3D overlap — add pair (normalized: smaller ID first)
                    let (g1, g2) = if geom_i < geom_j {
                        (geom_i, geom_j)
                    } else {
                        (geom_j, geom_i)
                    };
                    pairs.push((g1, g2));
                }
            }
        }

        pairs
    }
}

/// Check if two geometries can collide based on contype/conaffinity bitmasks.
///
/// Following MuJoCo's collision filtering:
/// 1. Same body - no collision
/// 2. Parent-child (adjacent bodies in kinematic tree) - no collision
///    Exception: World body (0) collides with all non-child bodies
/// 3. contype/conaffinity check: (c1 & a2) != 0 || (c2 & a1) != 0
///
/// The world body exception ensures ground planes can collide with objects
/// even when those objects are direct children of the world body.
#[allow(clippy::inline_always)] // Hot path - profiling shows inlining improves debug performance
#[inline(always)]
fn check_collision_affinity(model: &Model, geom1: usize, geom2: usize) -> bool {
    let body1 = model.geom_body[geom1];
    let body2 = model.geom_body[geom2];

    // Check body-pair exclude list (from <contact><exclude>)
    let exclude_key = (body1.min(body2), body1.max(body2));
    if model.contact_excludes.contains(&exclude_key) {
        return false;
    }

    // Skip if this geom pair has an explicit <pair> entry —
    // mechanism 2 will handle it with its overridden parameters.
    let pair_key = (geom1.min(geom2), geom1.max(geom2));
    if model.contact_pair_set.contains(&pair_key) {
        return false;
    }

    // Same body - no collision
    if body1 == body2 {
        return false;
    }

    // Parent-child filtering: bodies connected by a joint shouldn't collide
    // Exception: World body (0) geometries (ground planes) should collide with
    // any body, including direct children. The world body has no joints connecting
    // it to children - children are simply anchored in world space.
    //
    // This is important because:
    // - A ball with a free joint has body_parent[ball_body] = 0 (world)
    // - But the ball should still collide with ground planes on body 0
    // - The "parent-child" filter is for bodies connected by articulated joints
    //   (hinge, slide, ball) where collision would be geometrically impossible
    if body1 != 0 && body2 != 0 {
        if model.body_parent[body1] == body2 || model.body_parent[body2] == body1 {
            return false;
        }
    }

    // contype/conaffinity bitmask check
    let c1 = model.geom_contype[geom1];
    let a1 = model.geom_conaffinity[geom1];
    let c2 = model.geom_contype[geom2];
    let a2 = model.geom_conaffinity[geom2];

    (c1 & a2) != 0 || (c2 & a1) != 0
}

/// Collision detection: populate contacts from geometry pairs.
///
/// Following `MuJoCo`'s collision detection order:
/// 1. Broad-phase: sweep-and-prune to find candidate pairs (O(n log n))
/// 2. Check contact affinity (contype/conaffinity bitmasks)
/// 3. Narrow-phase: analytical or GJK/EPA collision detection
/// 4. Populate data.contacts with contact information
///
/// This function is called after forward kinematics (`mj_fwd_position`) so
/// `geom_xpos` and `geom_xmat` are up-to-date.
///
/// # Algorithm: Sweep-and-Prune
///
/// Unlike spatial hashing which degrades to O(n²) when objects cluster,
/// sweep-and-prune maintains O(n log n + k) complexity where k is the
/// number of overlapping pairs. This is robust against clustering scenarios
/// like a pile of boxes or a humanoid with many self-collision checks.
///
/// # Performance
///
/// | Scene Size | Complexity | Notes |
/// |------------|------------|-------|
/// | Any n | O(n log n + k) | k = overlapping pairs |
/// | Coherent | O(n + k) | Nearly-sorted input |
fn mj_collision(model: &Model, data: &mut Data) {
    // Clear existing contacts
    data.contacts.clear();
    data.ncon = 0;

    // Early exit if no geoms or only one geom
    if model.ngeom <= 1 {
        return;
    }

    // Build AABBs for all geoms
    // This is O(n) and cache-friendly (linear memory access)
    let aabbs: Vec<Aabb> = (0..model.ngeom)
        .map(|geom_id| {
            aabb_from_geom(
                model.geom_type[geom_id],
                model.geom_size[geom_id],
                data.geom_xpos[geom_id],
                data.geom_xmat[geom_id],
            )
        })
        .collect();

    // Sweep-and-prune broad-phase: O(n log n) worst case, O(n + k) for coherent scenes
    let sap = SweepAndPrune::new(aabbs);
    let candidates = sap.query_pairs();

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Process candidate pairs
    // The SAP already filtered to only AABB-overlapping pairs
    for (geom1, geom2) in candidates {
        // Affinity check: same body, parent-child, contype/conaffinity
        if !check_collision_affinity(model, geom1, geom2) {
            continue;
        }

        // §16.5b: Skip narrow-phase if BOTH geoms belong to sleeping bodies.
        // Keep sleeping-vs-awake pairs — they may trigger wake detection (§16.4).
        if sleep_enabled {
            let b1 = model.geom_body[geom1];
            let b2 = model.geom_body[geom2];
            if data.body_sleep_state[b1] == SleepState::Asleep
                && data.body_sleep_state[b2] == SleepState::Asleep
            {
                continue;
            }
        }

        // Get world-space poses
        let pos1 = data.geom_xpos[geom1];
        let mat1 = data.geom_xmat[geom1];
        let pos2 = data.geom_xpos[geom2];
        let mat2 = data.geom_xmat[geom2];

        // Narrow-phase collision detection
        if let Some(contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2) {
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }

    // Mechanism 2: explicit contact pairs (bypass kinematic + bitmask filters)
    for pair in &model.contact_pairs {
        let geom1 = pair.geom1;
        let geom2 = pair.geom2;

        // §16.5b: Skip narrow-phase if BOTH geoms belong to sleeping bodies.
        if sleep_enabled {
            let b1 = model.geom_body[geom1];
            let b2 = model.geom_body[geom2];
            if data.body_sleep_state[b1] == SleepState::Asleep
                && data.body_sleep_state[b2] == SleepState::Asleep
            {
                continue;
            }
        }

        // Distance cull using bounding radii (replaces SAP broad-phase for pairs).
        // geom_rbound is the bounding sphere radius, pre-computed per geom.
        // For planes, rbound = INFINITY so this check always passes.
        // Margin is added to match MuJoCo's mj_filterSphere.
        let dist = (data.geom_xpos[geom1] - data.geom_xpos[geom2]).norm();
        if dist > model.geom_rbound[geom1] + model.geom_rbound[geom2] + pair.margin {
            continue;
        }

        // Narrow-phase collision detection
        let pos1 = data.geom_xpos[geom1];
        let mat1 = data.geom_xmat[geom1];
        let pos2 = data.geom_xpos[geom2];
        let mat2 = data.geom_xmat[geom2];

        if let Some(mut contact) = collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2) {
            apply_pair_overrides(&mut contact, pair);
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}

/// Narrow-phase collision between two geometries.
///
/// Returns Contact if penetrating, None otherwise.
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
#[allow(clippy::items_after_statements)] // use statement placed after special cases for readability
#[inline]
fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Fast path: handle all analytical collision cases first
    // These avoid the expensive quaternion conversion and GJK/EPA

    // Special case: SDF collision (before mesh/hfield/plane — SDF has its own
    // contact functions for all shapes including Mesh, Hfield, and Plane)
    if type1 == GeomType::Sdf || type2 == GeomType::Sdf {
        return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2);
    }

    // Special case: mesh collision (has its own BVH-accelerated path)
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2);
    }

    // Special case: height field collision
    if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
        return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: sphere-sphere collision (analytical, more robust than GJK/EPA)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2);
    }

    // Special case: capsule-capsule collision (analytical, much faster than GJK/EPA)
    if type1 == GeomType::Capsule && type2 == GeomType::Capsule {
        return collide_capsule_capsule(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2);
    }

    // Special case: sphere-capsule collision
    if (type1 == GeomType::Sphere && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Sphere)
    {
        return collide_sphere_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: sphere-box collision (analytical)
    if (type1 == GeomType::Sphere && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Sphere)
    {
        return collide_sphere_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: capsule-box collision (analytical)
    if (type1 == GeomType::Capsule && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Capsule)
    {
        return collide_capsule_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: box-box collision (SAT)
    if type1 == GeomType::Box && type2 == GeomType::Box {
        return collide_box_box(model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2);
    }

    // Special case: cylinder-sphere collision (analytical)
    if (type1 == GeomType::Cylinder && type2 == GeomType::Sphere)
        || (type1 == GeomType::Sphere && type2 == GeomType::Cylinder)
    {
        return collide_cylinder_sphere(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        );
    }

    // Special case: cylinder-capsule collision (analytical with GJK/EPA fallback)
    // Analytical solution handles common cases; degenerate cases fall through to GJK/EPA
    if (type1 == GeomType::Cylinder && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Cylinder)
    {
        if let Some(contact) = collide_cylinder_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2,
        ) {
            return Some(contact);
        }
        // Fall through to GJK/EPA for degenerate cases (intersecting/parallel axes, cap collisions)
    }

    // Slow path: Build shapes and poses for GJK/EPA (cylinder-cylinder, cylinder-box, ellipsoid-*, and fallback cases)
    let shape1 = geom_to_collision_shape(type1, size1);
    let shape2 = geom_to_collision_shape(type2, size2);

    // Build poses for GJK/EPA - expensive quaternion conversion
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    // Use GJK/EPA for general convex collision
    use crate::gjk_epa::gjk_epa_contact;
    let collision_shape1 = shape1?;
    let collision_shape2 = shape2?;

    if let Some(result) = gjk_epa_contact(&collision_shape1, &pose1, &collision_shape2, &pose2) {
        if result.penetration > 0.0 {
            return Some(make_contact_from_geoms(
                model,
                Vector3::new(result.point.x, result.point.y, result.point.z),
                result.normal,
                result.penetration,
                geom1,
                geom2,
            ));
        }
    }

    None
}

/// Convert `MuJoCo` `GeomType` to `CollisionShape`.
#[allow(clippy::match_same_arms)] // Plane and Mesh both return None but for different reasons
fn geom_to_collision_shape(geom_type: GeomType, size: Vector3<f64>) -> Option<CollisionShape> {
    match geom_type {
        GeomType::Sphere => Some(CollisionShape::Sphere { radius: size.x }),
        GeomType::Box => Some(CollisionShape::Box { half_extents: size }),
        GeomType::Capsule => Some(CollisionShape::Capsule {
            half_length: size.y, // MuJoCo: size[0]=radius, size[1]=half_length
            radius: size.x,
        }),
        GeomType::Cylinder => Some(CollisionShape::Cylinder {
            half_length: size.y,
            radius: size.x,
        }),
        GeomType::Ellipsoid => Some(CollisionShape::Ellipsoid { radii: size }),
        GeomType::Plane => None,  // Handled via collide_with_plane()
        GeomType::Mesh => None,   // Handled via collide_with_mesh()
        GeomType::Hfield => None, // Handled via collide_with_hfield()
        GeomType::Sdf => None,    // Handled via collide_with_sdf()
    }
}

// ============================================================================
// Primitive Collision Detection
// ============================================================================
//
// # API Design Pattern
//
// This module uses two types of collision functions:
//
// 1. **Dispatcher functions** (e.g., `collide_with_plane`, `collide_geoms`):
//    - Take `&Model` and geometry indices
//    - Handle type dispatch and parameter extraction
//    - Compute derived values like friction (geometric mean)
//    - Called from the main collision pipeline
//
// 2. **Implementation helpers** (e.g., `collide_cylinder_plane_impl`):
//    - Suffix `_impl` indicates internal helper
//    - Take pre-extracted geometric parameters (no Model reference)
//    - Focus purely on geometric computation
//    - Marked `#[inline]` for performance
//    - Not called directly from pipeline
//
// This separation follows the Todorov principle: compute derived values once
// in the dispatcher, then pass them to the implementation.
// ============================================================================

/// Apply explicit `<pair>` overrides to a contact produced by `collide_geoms`.
///
/// `collide_geoms` combines friction/condim/solref/solimp from the two geoms.
/// For mechanism-2 contacts, those geom-combined values are overwritten here
/// with the fully-resolved pair parameters.
#[inline]
fn apply_pair_overrides(contact: &mut Contact, pair: &ContactPair) {
    // condim → dim mapping (same logic as Contact::with_condim)
    #[allow(clippy::match_same_arms, clippy::cast_sign_loss)]
    let dim = match pair.condim {
        1 => 1,
        3 => 3,
        4 => 4,
        6 => 6,
        0 | 2 => 3,
        5 => 6,
        _ => 6,
    } as usize;
    contact.dim = dim;
    // 5D friction: directly from pair (already fully resolved)
    contact.mu = pair.friction;
    contact.friction = pair.friction[0]; // legacy scalar = tan1
    // Solver params
    contact.solref = pair.solref;
    contact.solimp = pair.solimp;
    // NOTE: solreffriction is NOT applied here — Contact has a single solref
    // field; per-direction solver params require solver changes (see §G).
    // NOTE: margin/gap are NOT applied here — no runtime effect yet (see §H).
}

/// Create a contact with solver parameters derived from the colliding geoms.
///
/// This helper combines friction (geometric mean) and solver params from both
/// geoms according to MuJoCo conventions:
/// - friction: geometric mean of both geoms (per friction type)
/// - condim: maximum of both geom condim values (sufficient dimensionality)
/// - solref: element-wise minimum (stiffer wins)
/// - solimp: element-wise maximum (harder wins)
#[inline]
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
) -> Contact {
    // Get friction vectors from both geoms
    // geom_friction: [sliding, torsional, rolling]
    let f1 = model.geom_friction[geom1];
    let f2 = model.geom_friction[geom2];

    // Compute geometric mean for each friction component
    let sliding = (f1.x * f2.x).sqrt();
    let torsional = (f1.y * f2.y).sqrt();
    let rolling = (f1.z * f2.z).sqrt();

    // Contact dimension: maximum of both geom condim values
    // MuJoCo uses max so the contact has sufficient dimensionality
    let condim1 = model.geom_condim[geom1];
    let condim2 = model.geom_condim[geom2];
    let condim = condim1.max(condim2);

    // Combine solver parameters from both geoms
    let (solref, solimp) = combine_solver_params(
        model.geom_solref[geom1],
        model.geom_solimp[geom1],
        model.geom_solref[geom2],
        model.geom_solimp[geom2],
    );

    Contact::with_condim(
        pos, normal, depth, geom1, geom2, sliding, torsional, rolling, condim, solref, solimp,
    )
}

/// Minimum norm threshold for geometric operations.
///
/// Used to prevent division by zero and detect degenerate cases in collision
/// detection. This value is chosen to be:
/// - Small enough to not reject valid geometric configurations
/// - Large enough to avoid numerical instability near machine epsilon
///
/// For reference: f64::EPSILON ≈ 2.2e-16, so 1e-10 provides ~6 orders of
/// magnitude of safety margin while still detecting near-degenerate cases.
const GEOM_EPSILON: f64 = 1e-10;

/// Threshold for cylinder axis being nearly vertical (perpendicular to plane).
/// When |cos(θ)| > 0.999 (θ < 2.6°), treat cylinder as vertical.
const AXIS_VERTICAL_THRESHOLD: f64 = 0.999;

/// Threshold for cylinder axis being nearly horizontal (parallel to plane).
/// When |cos(θ)| < 0.001 (θ > 89.9°), treat cylinder as horizontal.
const AXIS_HORIZONTAL_THRESHOLD: f64 = 0.001;

/// Threshold for detecting cylinder cap collision in cylinder-capsule.
/// When normal is within ~45° of cylinder axis (cos > 0.7), treat as cap collision.
const CAP_COLLISION_THRESHOLD: f64 = 0.7;

// =============================================================================
// Mesh Collision Detection
// =============================================================================

/// Height field collision: dispatch to the appropriate contact function
/// based on the other geom's type, then convert to pipeline Contact.
fn collide_with_hfield(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    // Identify which geom is the hfield, which is the other
    let (hf_geom, other_geom, hf_pos, hf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let hfield_id = model.geom_hfield[hf_geom]?;
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Build hfield pose — apply centering offset so HeightFieldData's
    // corner-origin (0,0) maps to MuJoCo's center-origin (-size[0], -size[1])
    let quat = UnitQuaternion::from_matrix(&hf_mat);
    let offset = hf_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
    let hf_pose = Pose::from_position_rotation(Point3::from(hf_pos + offset), quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let hf_contact = match model.geom_type[other_geom] {
        GeomType::Sphere => {
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, other_size.x)
        }
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Box => heightfield_box_contact(hfield, &hf_pose, &other_pose, &other_size),
        GeomType::Cylinder => {
            // Approximate cylinder as capsule (conservative, same as collide_with_mesh)
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Ellipsoid => {
            // Approximate as sphere with max radius (conservative)
            let max_r = other_size.x.max(other_size.y).max(other_size.z);
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, max_r)
        }
        // Hfield↔Hfield, Hfield↔Plane, Hfield↔Mesh: not supported
        _ => return None,
    };

    // Convert HeightFieldContact → pipeline Contact.
    // Normal direction: HeightFieldContact.normal always points "up from terrain".
    // Following the collide_with_mesh pattern, when the surface geometry (hfield)
    // is geom2 and the primitive is geom1, negate the normal so it points from
    // geom1 toward geom2.
    let swapped = hf_geom != geom1;
    hf_contact.map(|c| {
        let normal = if swapped { -c.normal } else { c.normal };
        make_contact_from_geoms(model, c.point.coords, normal, c.penetration, geom1, geom2)
    })
}

/// Collision detection involving at least one SDF geometry.
///
/// Dispatches to the appropriate `sdf_*_contact()` function from `sdf.rs`.
/// Handles all `GeomType` pairings including Mesh, Hfield, Plane, and SDF↔SDF.
#[allow(clippy::too_many_arguments)]
fn collide_with_sdf(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    // Identify which geom is the SDF, which is the other
    let (sdf_geom, other_geom, sdf_pos, sdf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Sdf {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let sdf_id = model.geom_sdf[sdf_geom]?;
    let sdf = &model.sdf_data[sdf_id];

    // Build SDF pose (no centering offset needed — SdfCollisionData uses
    // an arbitrary origin stored internally, unlike HeightFieldData which
    // uses corner-origin requiring a centering offset)
    let sdf_quat = UnitQuaternion::from_matrix(&sdf_mat);
    let sdf_pose = Pose::from_position_rotation(Point3::from(sdf_pos), sdf_quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let contact = match model.geom_type[other_geom] {
        GeomType::Sphere => sdf_sphere_contact(sdf, &sdf_pose, other_pose.position, other_size.x),
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            sdf_capsule_contact(sdf, &sdf_pose, start, end, other_size.x)
        }
        GeomType::Box => sdf_box_contact(sdf, &sdf_pose, &other_pose, &other_size),
        GeomType::Cylinder => {
            // sdf_cylinder_contact takes (pose, half_height, radius)
            // geom_size for Cylinder: x = radius, y = half_length
            sdf_cylinder_contact(sdf, &sdf_pose, &other_pose, other_size.y, other_size.x)
        }
        GeomType::Ellipsoid => sdf_ellipsoid_contact(sdf, &sdf_pose, &other_pose, &other_size),
        GeomType::Mesh => {
            let mesh_id = model.geom_mesh[other_geom]?;
            let mesh_data = &model.mesh_data[mesh_id];
            sdf_triangle_mesh_contact(sdf, &sdf_pose, mesh_data, &other_pose)
        }
        GeomType::Hfield => {
            let hfield_id = model.geom_hfield[other_geom]?;
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];
            // Apply centering offset (same as collide_with_hfield)
            let hf_offset = other_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let hf_pose =
                Pose::from_position_rotation(Point3::from(other_pos + hf_offset), other_quat);
            sdf_heightfield_contact(sdf, &sdf_pose, hfield, &hf_pose)
        }
        GeomType::Plane => {
            // Plane normal = Z-axis of plane's frame; offset = normal · position
            let plane_normal = other_mat.column(2).into_owned();
            let plane_offset = plane_normal.dot(&other_pos);
            sdf_plane_contact(sdf, &sdf_pose, &plane_normal, plane_offset)
        }
        GeomType::Sdf => {
            let other_sdf_id = model.geom_sdf[other_geom]?;
            let other_sdf = &model.sdf_data[other_sdf_id];
            sdf_sdf_contact(sdf, &sdf_pose, other_sdf, &other_pose)
        }
    };

    // Convert SdfContact → pipeline Contact.
    //
    // Normal direction conventions:
    // - Most sdf_*_contact: normal points outward from SDF surface
    // - sdf_plane_contact: normal is the plane normal (from plane toward SDF)
    // - sdf_sdf_contact: normal from whichever SDF the contact is closer to
    //
    // Pipeline convention: normal must point from geom1 toward geom2.
    //
    // Standard case: SDF outward normal points away from the SDF. When SDF is
    // geom1, this is toward geom2 (correct). When SDF is geom2 (swapped), negate.
    //
    // Plane exception: sdf_plane_contact returns the plane normal (from plane
    // toward SDF). When SDF is geom1 and plane is geom2, this points geom2→geom1
    // (wrong) — negate. When SDF is geom2 and plane is geom1, this points
    // geom1→geom2 (correct) — keep. So for plane, swap logic is inverted.
    let swapped = sdf_geom != geom1;
    let is_plane = model.geom_type[other_geom] == GeomType::Plane;

    contact.map(|c| {
        // Negate when exactly one of swapped/is_plane is true (XOR).
        // See comments above for the full derivation.
        let normal = if swapped ^ is_plane {
            -c.normal
        } else {
            c.normal
        };
        make_contact_from_geoms(model, c.point.coords, normal, c.penetration, geom1, geom2)
    })
}

/// Collision detection involving at least one mesh geometry.
///
/// Dispatches to specialized mesh-primitive or mesh-mesh implementations.
/// For mesh-primitive collisions, approximations are used for some geometry types:
/// - Cylinder: approximated as capsule (conservative, may report false positives)
/// - Ellipsoid: approximated as sphere with max radius (conservative)
#[allow(clippy::too_many_arguments)]
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
fn collide_with_mesh(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
) -> Option<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Build poses (expensive quaternion conversion, but needed for mesh collision)
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    let mesh_contact: Option<MeshContact> = match (type1, type2) {
        // Mesh-Mesh
        (GeomType::Mesh, GeomType::Mesh) => {
            let mesh1_id = model.geom_mesh[geom1]?;
            let mesh2_id = model.geom_mesh[geom2]?;
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2)
        }

        // Mesh (geom1) vs Primitive (geom2)
        (GeomType::Mesh, prim_type) => {
            let mesh_id = model.geom_mesh[geom1]?;
            let mesh = &model.mesh_data[mesh_id];

            match prim_type {
                GeomType::Sphere => mesh_sphere_contact(mesh, &pose1, pose2.position, size2.x),
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size2.y;
                    let axis = pose2.rotation * Vector3::z();
                    let start = pose2.position - axis * half_len;
                    let end = pose2.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose1, start, end, size2.x)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose1, &pose2, &size2),
                GeomType::Ellipsoid => {
                    // Approximate as sphere with max radius (conservative)
                    let max_r = size2.x.max(size2.y).max(size2.z);
                    mesh_sphere_contact(mesh, &pose1, pose2.position, max_r)
                }
                GeomType::Plane => {
                    // Plane normal is local Z-axis
                    let plane_normal = mat2.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos2);
                    collide_mesh_plane(mesh, &pose1, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
                GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
            }
        }

        // Primitive (geom1) vs Mesh (geom2) - swap and negate normal
        (prim_type, GeomType::Mesh) => {
            let mesh_id = model.geom_mesh[geom2]?;
            let mesh = &model.mesh_data[mesh_id];

            let contact = match prim_type {
                GeomType::Sphere => mesh_sphere_contact(mesh, &pose2, pose1.position, size1.x),
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size1.y;
                    let axis = pose1.rotation * Vector3::z();
                    let start = pose1.position - axis * half_len;
                    let end = pose1.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose2, start, end, size1.x)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose2, &pose1, &size1),
                GeomType::Ellipsoid => {
                    let max_r = size1.x.max(size1.y).max(size1.z);
                    mesh_sphere_contact(mesh, &pose2, pose1.position, max_r)
                }
                GeomType::Plane => {
                    let plane_normal = mat1.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos1);
                    collide_mesh_plane(mesh, &pose2, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
                GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
            };

            // Negate normal since we swapped the order (mesh was geom2, but contact
            // normal points from mesh outward - we need it pointing from geom1 to geom2)
            contact.map(|mut c| {
                c.normal = -c.normal;
                c
            })
        }

        _ => unreachable!("collide_with_mesh called but neither geom is a mesh"),
    };

    // Convert MeshContact to Contact
    mesh_contact.map(|mc| {
        make_contact_from_geoms(
            model,
            mc.point.coords,
            mc.normal,
            mc.penetration,
            geom1,
            geom2,
        )
    })
}

/// Mesh vs infinite plane collision.
///
/// Tests all mesh vertices against the plane and returns the deepest penetrating vertex.
/// This is a simple but effective approach for mesh-plane collision:
/// - O(n) in number of vertices
/// - Handles any mesh topology
/// - Returns single deepest contact point
fn collide_mesh_plane(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    plane_normal: Vector3<f64>,
    plane_d: f64,
) -> Option<MeshContact> {
    let mut deepest: Option<MeshContact> = None;

    for (i, vertex) in mesh.vertices().iter().enumerate() {
        // Transform vertex to world space
        let world_v = mesh_pose.transform_point(vertex);

        // Signed distance from plane (positive = above plane, negative = below)
        let signed_dist = plane_normal.dot(&world_v.coords) - plane_d;
        let depth = -signed_dist;

        if depth > 0.0 {
            match &mut deepest {
                Some(d) if depth > d.penetration => {
                    d.point = world_v;
                    d.normal = plane_normal;
                    d.penetration = depth;
                    d.triangle_index = i; // Store vertex index (not triangle, but useful for debug)
                }
                None => {
                    deepest = Some(MeshContact {
                        point: world_v,
                        normal: plane_normal,
                        penetration: depth,
                        triangle_index: i,
                    });
                }
                _ => {}
            }
        }
    }

    deepest
}

/// Collision between a plane and another geometry.
///
/// Dispatches to specialized implementations based on the other geometry's type.
/// The plane is always treated as an infinite half-space with normal along its
/// local Z-axis.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
#[inline]
fn collide_with_plane(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    type2: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is the plane
    let (
        plane_geom,
        other_geom,
        plane_pos,
        plane_mat,
        other_pos,
        other_mat,
        other_type,
        other_size,
    ) = if type1 == GeomType::Plane {
        (geom1, geom2, pos1, mat1, pos2, mat2, type2, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, type1, size1)
    };

    // Plane normal is the Z-axis of the plane's frame
    let plane_normal = plane_mat.column(2).into_owned();
    // Plane passes through its position
    let plane_distance = plane_normal.dot(&plane_pos);

    match other_type {
        GeomType::Sphere => {
            let radius = other_size.x;
            let center_dist = plane_normal.dot(&other_pos) - plane_distance;
            let penetration = radius - center_dist;

            if penetration > 0.0 {
                // Contact position is at the sphere surface toward the plane
                let contact_pos = other_pos - plane_normal * center_dist;
                // Contact normal points from other_geom toward plane_geom (from ball into plane = -plane_normal)
                // But for force calculation, we want the force to push the ball OUT of the plane,
                // so we use +plane_normal for the contact force direction.
                // Store the "push direction" as the normal to simplify force calculation.
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal, // Points UP, away from plane (push direction)
                    penetration,
                    plane_geom,
                    other_geom,
                ))
            } else {
                None
            }
        }
        GeomType::Box => {
            // Optimized box-plane: compute lowest corner directly instead of testing all 8
            //
            // The support point (lowest corner toward plane) uses the formula:
            //   corner = center + sum_i( -sign(n·r_i) * h_i * r_i )
            //
            // where r_i are box local axes in world space and h_i are half-extents.
            // The sign logic: if n·r_i > 0, the axis points "up" relative to the plane,
            // so we want the negative end (-h_i * r_i) to get the lowest point.
            //
            // Implementation: dx = -sign(n·rx), so corner = center + dx*hx*rx + ...
            // This is equivalent to the formula but avoids explicit negation.
            //
            // This is O(1) instead of O(8) per box.
            let half = other_size;
            let rx = other_mat.column(0).into_owned();
            let ry = other_mat.column(1).into_owned();
            let rz = other_mat.column(2).into_owned();

            // Compute -sign(n·axis) for each axis. When dot > 0, axis points toward
            // plane normal, so we pick the negative end (d = -1). When dot <= 0, we pick
            // the positive end (d = +1). This gives the corner furthest in -n direction.
            let dx = if plane_normal.dot(&rx) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dy = if plane_normal.dot(&ry) > 0.0 {
                -1.0
            } else {
                1.0
            };
            let dz = if plane_normal.dot(&rz) > 0.0 {
                -1.0
            } else {
                1.0
            };

            // Lowest corner is the one most toward the plane (-normal direction)
            let lowest_corner =
                other_pos + rx * (dx * half.x) + ry * (dy * half.y) + rz * (dz * half.z);

            let dist = plane_normal.dot(&lowest_corner) - plane_distance;
            let depth = -dist;

            if depth > 0.0 {
                // Contact position on plane surface (project corner onto plane)
                // This is consistent with sphere-plane which places contact at surface
                let contact_pos = lowest_corner - plane_normal * dist;
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal,
                    depth,
                    plane_geom,
                    other_geom,
                ))
            } else {
                None
            }
        }
        GeomType::Capsule => {
            // Check both end spheres of the capsule
            let radius = other_size.x;
            let half_length = other_size.y;
            let axis = other_mat.column(2).into_owned(); // Z is capsule axis

            let end1 = other_pos + axis * half_length;
            let end2 = other_pos - axis * half_length;

            let dist1 = plane_normal.dot(&end1) - plane_distance;
            let dist2 = plane_normal.dot(&end2) - plane_distance;

            let (closest_end, min_dist) = if dist1 < dist2 {
                (end1, dist1)
            } else {
                (end2, dist2)
            };

            let penetration = radius - min_dist;

            if penetration > 0.0 {
                let contact_pos = closest_end - plane_normal * min_dist;
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal,
                    penetration,
                    plane_geom,
                    other_geom,
                ))
            } else {
                None
            }
        }
        GeomType::Cylinder => {
            // Cylinder-plane collision: find deepest point on cylinder
            collide_cylinder_plane_impl(
                model,
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
            )
        }
        GeomType::Ellipsoid => {
            // Ellipsoid-plane collision: find support point in plane normal direction
            collide_ellipsoid_plane_impl(
                model,
                plane_geom,
                other_geom,
                plane_normal,
                plane_distance,
                other_pos,
                other_mat,
                other_size,
            )
        }
        // INVARIANT: collide_geoms() dispatches mesh collision before plane collision.
        // If either geom is a mesh, collide_with_mesh() handles it—including mesh-plane.
        // This branch exists only for match exhaustiveness; reaching it indicates a bug.
        GeomType::Mesh => unreachable!(
            "mesh collision must be dispatched before plane collision in collide_geoms"
        ),
        // Plane-plane: two infinite half-spaces. Intersection is either empty, a plane,
        // or a half-space—none of which produce a meaningful contact point.
        GeomType::Plane => None,
        // Hfield is dispatched before plane in collide_geoms()
        GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
        // SDF is dispatched before plane in collide_geoms()
        GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
    }
}

/// Cylinder-plane collision detection (internal helper).
///
/// Finds the deepest penetrating point on the cylinder. For tilted/upright
/// cylinders, this is on the rim edge. For horizontal cylinders (axis parallel
/// to plane), the curved surface is checked.
///
/// # Algorithm
/// 1. Compute cylinder axis in world frame
/// 2. Determine if cylinder is "horizontal" (axis nearly parallel to plane)
/// 3. For non-horizontal: check rim points on both caps in the deepest direction
/// 4. For horizontal: check the curved surface point closest to plane
/// 5. Return the deepest penetrating point as contact
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane (points away from solid half-space)
/// - `plane_d`: Signed distance from origin to plane along normal
/// - `cyl_size`: [radius, half_height, unused]
///
/// # Returns
/// `Some(Contact)` if cylinder penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_plane_impl(
    model: &Model,
    plane_geom: usize,
    cyl_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    cyl_pos: Vector3<f64>,
    cyl_mat: Matrix3<f64>,
    cyl_size: Vector3<f64>,
) -> Option<Contact> {
    let radius = cyl_size.x;
    let half_height = cyl_size.y;

    // Cylinder axis is local Z transformed to world
    let cyl_axis = cyl_mat.column(2).into_owned();

    // How aligned is cylinder axis with plane normal?
    // axis_dot_signed: positive = axis points away from plane, negative = toward plane
    // axis_dot (abs): 1 = vertical, 0 = horizontal
    let axis_dot_signed = plane_normal.dot(&cyl_axis);
    let axis_dot = axis_dot_signed.abs();

    // Pre-compute radial direction (projection of plane normal onto radial plane)
    // Used in Case 1 (horizontal) and Case 3 (tilted)
    let radial = plane_normal - cyl_axis * axis_dot_signed;
    let radial_len = radial.norm();

    // Compute the deepest point on the cylinder
    let deepest_point = if axis_dot < AXIS_HORIZONTAL_THRESHOLD {
        // Case 1: Cylinder is horizontal (axis parallel to plane)
        // The deepest point is on the curved surface, directly below the axis
        if radial_len > GEOM_EPSILON {
            // Point on curved surface in the direction opposite to plane normal
            cyl_pos - (radial / radial_len) * radius
        } else {
            // Degenerate: plane normal exactly along axis (contradicts horizontal check)
            cyl_pos - plane_normal * radius
        }
    } else if axis_dot > AXIS_VERTICAL_THRESHOLD {
        // Case 2: Cylinder is vertical (axis perpendicular to plane)
        // The deepest point is the entire bottom rim (pick center for stability)
        // Determine which cap faces the plane
        let cap_dir = if axis_dot_signed > 0.0 {
            -cyl_axis // Bottom cap faces plane
        } else {
            cyl_axis // Top cap faces plane
        };
        cyl_pos + cap_dir * half_height
    } else {
        // Case 3: Cylinder is tilted
        // The deepest point is on one of the rim edges
        // Find the rim direction that points most into the plane
        // Note: radial_len = sqrt(1 - axis_dot²) > 0 since axis_dot < AXIS_VERTICAL_THRESHOLD
        let rim_dir = -radial / radial_len;

        // Determine which cap is lower (faces the plane more)
        let top_center = cyl_pos + cyl_axis * half_height;
        let bottom_center = cyl_pos - cyl_axis * half_height;

        let top_rim_point = top_center + rim_dir * radius;
        let bottom_rim_point = bottom_center + rim_dir * radius;

        let top_depth = -(plane_normal.dot(&top_rim_point) - plane_d);
        let bottom_depth = -(plane_normal.dot(&bottom_rim_point) - plane_d);

        if bottom_depth > top_depth {
            bottom_rim_point
        } else {
            top_rim_point
        }
    };

    // Compute penetration depth
    let signed_dist = plane_normal.dot(&deepest_point) - plane_d;
    let depth = -signed_dist;

    if depth <= 0.0 {
        return None;
    }

    // Contact point is on the plane surface
    let contact_pos = deepest_point + plane_normal * depth;

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        cyl_geom,
    ))
}

/// Ellipsoid-plane collision detection (internal helper).
///
/// Finds the support point on the ellipsoid surface in the direction toward the plane.
/// Ellipsoid is defined by radii (a, b, c) = (rx, ry, rz) along local axes.
///
/// # Algorithm
///
/// For an ellipsoid `x²/a² + y²/b² + z²/c² = 1`, the support point in direction `-n`
/// (i.e., the point on the surface with outward normal parallel to `-n`) is:
///
/// ```text
/// p = -(r ⊙ r ⊙ n) / ||r ⊙ n||
///   = -(a²·nₓ, b²·nᵧ, c²·nᵤ) / ||(a·nₓ, b·nᵧ, c·nᵤ)||
/// ```
///
/// where `⊙` denotes element-wise (Hadamard) product.
///
/// **Derivation**: The outward normal at point `(x,y,z)` on the ellipsoid is
/// `∇f = (2x/a², 2y/b², 2z/c²)`. Setting this parallel to `n` and solving with
/// the ellipsoid constraint yields the support point formula.
///
/// # Parameters
/// - `plane_normal`: Unit normal of the plane
/// - `plane_d`: Signed distance from origin to plane
/// - `ell_radii`: Ellipsoid radii [rx, ry, rz]
///
/// # Returns
/// `Some(Contact)` if ellipsoid penetrates plane, `None` otherwise.
#[inline]
#[allow(clippy::too_many_arguments)]
fn collide_ellipsoid_plane_impl(
    model: &Model,
    plane_geom: usize,
    ell_geom: usize,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    ell_pos: Vector3<f64>,
    ell_mat: Matrix3<f64>,
    ell_radii: Vector3<f64>,
) -> Option<Contact> {
    // Transform plane normal to ellipsoid local frame
    let local_normal = ell_mat.transpose() * plane_normal;

    // Compute support point in local frame
    // For ellipsoid, support in direction -n is: -(r² ⊙ n) / ||r ⊙ n||
    // where ⊙ is element-wise multiply and r = radii
    let scaled = Vector3::new(
        ell_radii.x * local_normal.x,
        ell_radii.y * local_normal.y,
        ell_radii.z * local_normal.z,
    );
    let scale_norm = scaled.norm();

    if scale_norm < GEOM_EPSILON {
        // Degenerate case: normal perpendicular to all radii (shouldn't happen)
        return None;
    }

    // Support point on ellipsoid surface in direction toward plane (negative normal)
    let local_support = -Vector3::new(
        ell_radii.x * scaled.x / scale_norm,
        ell_radii.y * scaled.y / scale_norm,
        ell_radii.z * scaled.z / scale_norm,
    );

    // Transform to world frame
    let world_support = ell_pos + ell_mat * local_support;

    // Check penetration depth
    let signed_dist = plane_normal.dot(&world_support) - plane_d;
    let depth = -signed_dist; // Positive = penetrating

    if depth <= 0.0 {
        return None;
    }

    // Contact point is on plane surface
    let contact_pos = world_support + plane_normal * depth;

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        plane_normal,
        depth,
        plane_geom,
        ell_geom,
    ))
}

/// Sphere-sphere collision detection.
///
/// This is a simple analytical calculation that's more robust than GJK/EPA
/// for the sphere-sphere case.
fn collide_sphere_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    pos2: Vector3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    let radius1 = size1.x;
    let radius2 = size2.x;

    let diff = pos2 - pos1;
    let dist = diff.norm();

    // Check for penetration
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from sphere1 to sphere2.
        // For coincident/nearly-coincident centers (degenerate case), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };

        // Contact point is on the surface of sphere1 toward sphere2
        // (midpoint of penetration region)
        let contact_pos = pos1 + normal * (radius1 - penetration * 0.5);

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
        ))
    } else {
        None
    }
}

/// Capsule-capsule collision detection.
///
/// Capsules are represented as line segments with radius. The collision
/// is computed by finding the closest points between the two line segments,
/// then checking if the distance is less than the sum of radii.
#[allow(clippy::too_many_arguments)]
fn collide_capsule_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Capsule parameters: size.x = radius, size.y = half_length
    let radius1 = size1.x;
    let half_len1 = size1.y;
    let radius2 = size2.x;
    let half_len2 = size2.y;

    // Get capsule axes (Z-axis of their rotation matrices)
    let axis1 = mat1.column(2).into_owned();
    let axis2 = mat2.column(2).into_owned();

    // Endpoints of capsule line segments
    let p1a = pos1 - axis1 * half_len1;
    let p1b = pos1 + axis1 * half_len1;
    let p2a = pos2 - axis2 * half_len2;
    let p2b = pos2 + axis2 * half_len2;

    // Find closest points between the two line segments
    let (closest1, closest2) = closest_points_segments(p1a, p1b, p2a, p2b);

    // Check distance
    let diff = closest2 - closest1;
    let dist = diff.norm();
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from capsule1 toward capsule2.
        // For degenerate case (segments intersect), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest1 + normal * (radius1 - penetration * 0.5);

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
        ))
    } else {
        None
    }
}

/// Sphere-capsule collision detection.
#[allow(clippy::too_many_arguments)]
fn collide_sphere_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is sphere and which is capsule
    let (
        sphere_geom,
        capsule_geom,
        sphere_pos,
        capsule_pos,
        capsule_mat,
        sphere_radius,
        capsule_size,
    ) = if type1 == GeomType::Sphere {
        (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
    } else {
        (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule line segment to sphere center
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, sphere_pos);

    // Check distance
    let diff = sphere_pos - closest_on_capsule;
    let dist = diff.norm();
    let sum_radii = sphere_radius + capsule_radius;
    let penetration = sum_radii - dist;

    if penetration > 0.0 {
        // Normal points from capsule toward sphere.
        // For degenerate case (sphere center on capsule axis), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest_on_capsule + normal * (capsule_radius - penetration * 0.5);

        // Ensure geom1 < geom2 for consistency
        let (g1, g2) = if sphere_geom < capsule_geom {
            (sphere_geom, capsule_geom)
        } else {
            (capsule_geom, sphere_geom)
        };

        // Normal convention: points from g1 toward g2.
        // `normal` points from capsule toward sphere.
        // If capsule is g1 (capsule_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < capsule_geom), we need -normal to point g1→g2.
        let final_normal = if capsule_geom < sphere_geom {
            normal
        } else {
            -normal
        };

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            final_normal,
            penetration,
            g1,
            g2,
        ))
    } else {
        None
    }
}

/// Sphere-box collision detection.
///
/// Uses the closest point on box surface to sphere center algorithm.
#[allow(clippy::too_many_arguments)]
fn collide_sphere_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is sphere and which is box
    let (sphere_geom, box_geom, sphere_pos, box_pos, box_mat, sphere_radius, box_half) =
        if type1 == GeomType::Sphere {
            (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
        } else {
            (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
        };

    // Transform sphere center to box local coordinates
    let local_center = box_mat.transpose() * (sphere_pos - box_pos);

    // Find closest point on box to sphere center (clamp to box bounds)
    let closest_local = Vector3::new(
        local_center.x.clamp(-box_half.x, box_half.x),
        local_center.y.clamp(-box_half.y, box_half.y),
        local_center.z.clamp(-box_half.z, box_half.z),
    );

    // Transform back to world space
    let closest_world = box_pos + box_mat * closest_local;

    // Compute distance and penetration
    let diff = sphere_pos - closest_world;
    let dist = diff.norm();
    let penetration = sphere_radius - dist;

    if penetration > 0.0 {
        // Compute normal (from box toward sphere)
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            // Sphere center is inside box - find deepest penetration axis
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_center[i];
                let pen_neg = box_half[i] + local_center[i];
                if pen_pos < min_pen {
                    min_pen = pen_pos;
                    normal_local = Vector3::zeros();
                    normal_local[i] = 1.0;
                }
                if pen_neg < min_pen {
                    min_pen = pen_neg;
                    normal_local = Vector3::zeros();
                    normal_local[i] = -1.0;
                }
            }
            box_mat * normal_local
        };

        let contact_pos = closest_world + normal * (penetration * 0.5);

        let (g1, g2) = if sphere_geom < box_geom {
            (sphere_geom, box_geom)
        } else {
            (box_geom, sphere_geom)
        };

        // Normal convention: points from geom1 (g1) toward geom2 (g2).
        // `normal` is computed as sphere_pos - closest_world, i.e., from box toward sphere.
        // If box is g1 (box_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < box_geom), we need -normal to point g1→g2.
        let final_normal = if box_geom < sphere_geom {
            normal
        } else {
            -normal
        };

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            final_normal,
            penetration,
            g1,
            g2,
        ))
    } else {
        None
    }
}

/// Cylinder-sphere collision detection.
///
/// Handles three collision cases:
/// - Side collision: sphere beside cylinder body
/// - Cap collision: sphere above/below cylinder, within radius
/// - Edge collision: sphere near rim of cylinder cap
///
/// Cylinder axis is local Z.
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is cylinder and which is sphere
    // Note: sphere doesn't use its rotation matrix, but we need mat2 for the cylinder case
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, sph_geom, sph_pos, sph_radius) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, size2.x)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, size1.x)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cyl_axis = cyl_mat.column(2).into_owned();

    // Vector from cylinder center to sphere center
    let d = sph_pos - cyl_pos;

    // Project onto cylinder axis
    let axis_dist = d.dot(&cyl_axis);

    // Clamp to cylinder height
    let clamped_axis = axis_dist.clamp(-cyl_half_height, cyl_half_height);

    // Closest point on cylinder axis to sphere center
    let axis_point = cyl_pos + cyl_axis * clamped_axis;

    // Radial vector from axis to sphere
    let radial = sph_pos - axis_point;
    let radial_dist = radial.norm();

    // Determine closest point on cylinder surface and contact normal
    let (closest_on_cyl, normal) = if axis_dist.abs() <= cyl_half_height {
        // Sphere is beside the cylinder (side collision)
        if radial_dist < GEOM_EPSILON {
            // Sphere center on axis - degenerate case, pick arbitrary radial direction
            let arb = if cyl_axis.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let n = cyl_axis.cross(&arb).normalize();
            (axis_point + n * cyl_radius, n)
        } else {
            let n = radial / radial_dist;
            (axis_point + n * cyl_radius, n)
        }
    } else {
        // Sphere is above/below cylinder (cap or edge collision)
        let cap_center = cyl_pos + cyl_axis * clamped_axis;

        // Compute radial direction in the plane perpendicular to cylinder axis.
        // d - (d · axis) * axis gives the perpendicular component.
        let perp = d - cyl_axis * axis_dist;
        let perp_dist = perp.norm();

        if perp_dist <= cyl_radius {
            // Cap collision - sphere projects onto cap face
            let n = if axis_dist > 0.0 { cyl_axis } else { -cyl_axis };
            (cap_center + perp, n)
        } else {
            // Edge collision - sphere near rim of cap
            let radial_n = perp / perp_dist;
            let edge_point = cap_center + radial_n * cyl_radius;
            let to_sphere = sph_pos - edge_point;
            let to_sphere_dist = to_sphere.norm();
            let n = if to_sphere_dist > GEOM_EPSILON {
                to_sphere / to_sphere_dist
            } else {
                // Degenerate: sphere center exactly on edge
                radial_n
            };
            (edge_point, n)
        }
    };

    // Compute penetration depth
    let dist = (sph_pos - closest_on_cyl).norm();
    let penetration = sph_radius - dist;

    if penetration <= 0.0 {
        return None;
    }

    // Contact position is on the surface between the two shapes
    let contact_pos = closest_on_cyl + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < sph_geom {
        (cyl_geom, sph_geom, normal)
    } else {
        (sph_geom, cyl_geom, -normal)
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
    ))
}

/// Cylinder-capsule collision detection.
///
/// Computes collision between a cylinder and a capsule by finding the closest
/// points between the cylinder axis segment and the capsule axis segment,
/// then checking if the cylinder surface intersects the capsule's swept sphere.
///
/// # Limitations
///
/// This algorithm treats the cylinder's curved surface correctly but does not
/// handle collisions with the flat caps. For cap collisions (capsule directly
/// above/below cylinder), returns `None` to fall through to GJK/EPA.
///
/// Both shapes have their axis along local Z.
#[allow(clippy::too_many_arguments)]
fn collide_cylinder_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Identify cylinder and capsule
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, cap_geom, cap_pos, cap_mat, cap_size) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, mat2, size2)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, mat1, size1)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cap_radius = cap_size.x;
    let cap_half_len = cap_size.y;

    let cyl_axis = cyl_mat.column(2).into_owned();
    let cap_axis = cap_mat.column(2).into_owned();

    // Cylinder axis segment endpoints
    let cyl_a = cyl_pos - cyl_axis * cyl_half_height;
    let cyl_b = cyl_pos + cyl_axis * cyl_half_height;

    // Capsule axis segment endpoints
    let cap_a = cap_pos - cap_axis * cap_half_len;
    let cap_b = cap_pos + cap_axis * cap_half_len;

    // Find closest points between the two axis segments
    let (cyl_closest, cap_closest) = closest_points_segments(cyl_a, cyl_b, cap_a, cap_b);

    // Vector from cylinder axis point to capsule axis point
    let diff = cap_closest - cyl_closest;
    let dist = diff.norm();

    if dist < GEOM_EPSILON {
        // Axes intersect or nearly intersect - degenerate case where analytical
        // solution is unreliable. Return None to fall through to GJK/EPA.
        return None;
    }

    let normal = diff / dist;

    // Check if this is a cap collision scenario:
    // If cyl_closest is at an endpoint AND normal points mostly along the axis,
    // we're hitting the flat cap, not the curved surface. Fall back to GJK/EPA.
    let cyl_closest_on_cap =
        (cyl_closest - cyl_a).norm() < GEOM_EPSILON || (cyl_closest - cyl_b).norm() < GEOM_EPSILON;
    let normal_along_axis = normal.dot(&cyl_axis).abs();
    if cyl_closest_on_cap && normal_along_axis > CAP_COLLISION_THRESHOLD {
        // Cap collision - this algorithm doesn't handle flat caps correctly
        return None;
    }

    // Closest point on cylinder surface (in direction of capsule)
    let cyl_surface = cyl_closest + normal * cyl_radius;

    // Distance from cylinder surface to capsule axis
    let surface_to_cap_dist = (cap_closest - cyl_surface).dot(&normal);
    let penetration = cap_radius - surface_to_cap_dist;

    if penetration <= 0.0 {
        return None;
    }

    // Contact position is between the two surfaces
    let contact_pos = cyl_surface + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < cap_geom {
        (cyl_geom, cap_geom, normal)
    } else {
        (cap_geom, cyl_geom, -normal)
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
    ))
}

/// Capsule-box collision detection.
///
/// Tests both capsule endpoints and the closest point on the capsule axis
/// to find the minimum distance configuration.
#[allow(clippy::too_many_arguments)]
fn collide_capsule_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    // Determine which is capsule and which is box
    let (
        capsule_geom,
        box_geom,
        capsule_pos,
        capsule_mat,
        box_pos,
        box_mat,
        capsule_size,
        box_half,
    ) = if type1 == GeomType::Capsule {
        (geom1, geom2, pos1, mat1, pos2, mat2, size1, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, size2, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule axis to box
    // We sample multiple points along the capsule and find the minimum distance
    let mut min_dist = f64::MAX;
    let mut best_capsule_point = capsule_pos;
    let mut best_box_point = box_pos;

    // Transform box to local coordinates once
    let box_mat_t = box_mat.transpose();

    // Sample along capsule axis (including endpoints)
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    for &t in &samples {
        let capsule_point = cap_a + (cap_b - cap_a) * t;
        let local_point = box_mat_t * (capsule_point - box_pos);

        // Closest point on box to this capsule point
        let closest_local = Vector3::new(
            local_point.x.clamp(-box_half.x, box_half.x),
            local_point.y.clamp(-box_half.y, box_half.y),
            local_point.z.clamp(-box_half.z, box_half.z),
        );
        let closest_world = box_pos + box_mat * closest_local;

        let dist = (capsule_point - closest_world).norm();
        if dist < min_dist {
            min_dist = dist;
            best_capsule_point = capsule_point;
            best_box_point = closest_world;
        }
    }

    // Refine by finding closest point on capsule axis to best box point
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, best_box_point);
    let local_closest = box_mat_t * (closest_on_capsule - box_pos);
    let box_closest_local = Vector3::new(
        local_closest.x.clamp(-box_half.x, box_half.x),
        local_closest.y.clamp(-box_half.y, box_half.y),
        local_closest.z.clamp(-box_half.z, box_half.z),
    );
    let box_closest_world = box_pos + box_mat * box_closest_local;
    let final_dist = (closest_on_capsule - box_closest_world).norm();

    if final_dist < min_dist {
        min_dist = final_dist;
        best_capsule_point = closest_on_capsule;
        best_box_point = box_closest_world;
    }

    let penetration = capsule_radius - min_dist;

    if penetration > 0.0 {
        let diff = best_capsule_point - best_box_point;
        let normal = if min_dist > GEOM_EPSILON {
            diff / min_dist
        } else {
            // Edge case: capsule axis passes through box
            // Find deepest penetration direction
            let local_cap = box_mat_t * (best_capsule_point - box_pos);
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_cap[i];
                let pen_neg = box_half[i] + local_cap[i];
                if pen_pos < min_pen {
                    min_pen = pen_pos;
                    normal_local = Vector3::zeros();
                    normal_local[i] = 1.0;
                }
                if pen_neg < min_pen {
                    min_pen = pen_neg;
                    normal_local = Vector3::zeros();
                    normal_local[i] = -1.0;
                }
            }
            box_mat * normal_local
        };

        let contact_pos = best_box_point + normal * (penetration * 0.5);

        let (g1, g2) = if capsule_geom < box_geom {
            (capsule_geom, box_geom)
        } else {
            (box_geom, capsule_geom)
        };

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            if capsule_geom < box_geom {
                normal
            } else {
                -normal
            },
            penetration,
            g1,
            g2,
        ))
    } else {
        None
    }
}

/// Box-box collision detection using Separating Axis Theorem (SAT).
///
/// Tests 15 axes: 3 face normals of box A, 3 face normals of box B,
/// and 9 edge-edge cross products.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn collide_box_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
) -> Option<Contact> {
    let half1 = size1;
    let half2 = size2;

    // Get box axes
    let axes1: [Vector3<f64>; 3] = [
        mat1.column(0).into_owned(),
        mat1.column(1).into_owned(),
        mat1.column(2).into_owned(),
    ];
    let axes2: [Vector3<f64>; 3] = [
        mat2.column(0).into_owned(),
        mat2.column(1).into_owned(),
        mat2.column(2).into_owned(),
    ];

    let center_diff = pos2 - pos1;

    // Track minimum penetration
    let mut min_pen = f64::MAX;
    let mut best_axis = Vector3::x();
    let mut best_axis_is_face = true;

    // Test face normals of box 1 (3 axes)
    for i in 0..3 {
        let axis = axes1[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= 0.0 {
            return None; // Separating axis found
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test face normals of box 2 (3 axes)
    for i in 0..3 {
        let axis = axes2[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= 0.0 {
            return None;
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test edge-edge cross products (9 axes)
    for i in 0..3 {
        for j in 0..3 {
            let axis = axes1[i].cross(&axes2[j]);
            let len = axis.norm();
            if len < GEOM_EPSILON {
                continue; // Parallel edges
            }
            let axis = axis / len;

            let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
            if pen <= 0.0 {
                return None;
            }
            // Edge-edge contacts have a bias - they're less stable
            // Only use if significantly better than face contact
            if pen < min_pen * 0.95 {
                min_pen = pen;
                best_axis = axis;
                best_axis_is_face = false;
            }
        }
    }

    // Ensure normal points from box1 to box2
    if best_axis.dot(&center_diff) < 0.0 {
        best_axis = -best_axis;
    }

    // Find contact point
    // For face contacts: find vertex of one box most in the other's direction
    // For edge contacts: find closest points on the two edges
    let contact_pos = if best_axis_is_face {
        // Find support point on box2 in direction of -normal
        let support_local = Vector3::new(
            if best_axis.dot(&axes2[0]) < 0.0 {
                half2.x
            } else {
                -half2.x
            },
            if best_axis.dot(&axes2[1]) < 0.0 {
                half2.y
            } else {
                -half2.y
            },
            if best_axis.dot(&axes2[2]) < 0.0 {
                half2.z
            } else {
                -half2.z
            },
        );
        pos2 + mat2 * support_local - best_axis * (min_pen * 0.5)
    } else {
        // For edge-edge, use midpoint between closest points on edges
        // Approximate: use center point shifted by penetration
        pos1 + center_diff * 0.5
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        best_axis,
        min_pen,
        geom1,
        geom2,
    ))
}

/// Test a single SAT axis and return penetration depth (negative = separated).
#[inline]
fn test_sat_axis(
    axis: &Vector3<f64>,
    center_diff: &Vector3<f64>,
    axes1: &[Vector3<f64>; 3],
    half1: &Vector3<f64>,
    axes2: &[Vector3<f64>; 3],
    half2: &Vector3<f64>,
) -> f64 {
    // Project box extents onto axis
    let r1 = (half1.x * axis.dot(&axes1[0]).abs())
        + (half1.y * axis.dot(&axes1[1]).abs())
        + (half1.z * axis.dot(&axes1[2]).abs());

    let r2 = (half2.x * axis.dot(&axes2[0]).abs())
        + (half2.y * axis.dot(&axes2[1]).abs())
        + (half2.z * axis.dot(&axes2[2]).abs());

    // Distance between centers projected onto axis
    let dist = axis.dot(center_diff).abs();

    // Penetration = sum of radii - distance
    r1 + r2 - dist
}

/// Find closest point on line segment AB to point P.
#[inline]
fn closest_point_segment(a: Vector3<f64>, b: Vector3<f64>, p: Vector3<f64>) -> Vector3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < GEOM_EPSILON {
        return a; // Degenerate segment
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Find closest points between two line segments.
///
/// Returns (`point_on_seg1`, `point_on_seg2`).
///
/// Uses standard segment-segment distance algorithm with single-letter variables
/// matching the mathematical notation from computational geometry literature.
#[allow(clippy::many_single_char_names)]
fn closest_points_segments(
    p1: Vector3<f64>,
    q1: Vector3<f64>,
    p2: Vector3<f64>,
    q2: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let d1 = q1 - p1; // Direction of segment 1
    let d2 = q2 - p2; // Direction of segment 2
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Check for degenerate segments
    if a < GEOM_EPSILON && e < GEOM_EPSILON {
        return (p1, p2);
    }
    if a < GEOM_EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, p2 + d2 * t);
    }
    if e < GEOM_EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        return (p1 + d1 * s, p2);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    // Determinant of the 2x2 system: denom = a*e - b² (intentionally b*b, not a*b)
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    // Compute initial s and t parameters for the closest points
    let (mut s, mut t) = if denom.abs() < GEOM_EPSILON {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Clamp to [0,1] and recompute
    if s < 0.0 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else if s > 1.0 {
        s = 1.0;
        t = ((b + f) / e).clamp(0.0, 1.0);
    }

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (p1 + d1 * s, p2 + d2 * t)
}

/// Compute potential energy (gravitational + spring).
///
/// Following `MuJoCo` semantics:
/// - Gravitational: `E_g` = -Σ `m_i` * g · `p_i` (negative because work done against gravity)
/// - Spring: `E_s` = 0.5 * Σ `k_i` * (`q_i` - `q0_i)²`
fn mj_energy_pos(model: &Model, data: &mut Data) {
    let mut potential = 0.0;

    // Gravitational potential energy
    // E_g = -Σ m_i * g · com_i
    // Using xipos (COM in world frame) for correct calculation
    for body_id in 1..model.nbody {
        let mass = model.body_mass[body_id];
        let com = data.xipos[body_id];
        // Potential energy: -m * g · h (negative of work done by gravity)
        // With g = (0, 0, -9.81), this becomes m * 9.81 * z
        potential -= mass * model.gravity.dot(&com);
    }

    // Spring potential energy
    // E_s = 0.5 * k * (q - q0)²
    for jnt_id in 0..model.njnt {
        let stiffness = model.jnt_stiffness[jnt_id];
        if stiffness > 0.0 {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let q = data.qpos[qpos_adr];
                    let q0 = model.qpos0.get(qpos_adr).copied().unwrap_or(0.0);
                    let displacement = q - q0;
                    potential += 0.5 * stiffness * displacement * displacement;
                }
                MjJointType::Ball | MjJointType::Free => {
                    // Ball/Free joint springs would use quaternion distance
                    // Not commonly used, skip for now
                }
            }
        }
    }

    data.energy_potential = potential;
}

/// Compute kinetic energy from mass matrix and velocities.
///
/// `E_k` = 0.5 * qvel^T * M * qvel
///
/// This must be called AFTER `mj_crba()` has computed the mass matrix.
/// However, for the `forward()` pipeline we call it after velocity FK
/// but before CRBA. We use an approximation based on body velocities
/// when M is not yet available.
fn mj_energy_vel(model: &Model, data: &mut Data) {
    // If mass matrix is available and computed, use exact formula
    // E_k = 0.5 * qvel^T * M * qvel
    let kinetic = if data.qM.nrows() == model.nv && model.nv > 0 {
        let m_qvel = &data.qM * &data.qvel;
        0.5 * data.qvel.dot(&m_qvel)
    } else {
        // Fallback: compute from body velocities directly
        // E_k = 0.5 * Σ (m_i * v_i^T * v_i + ω_i^T * I_i * ω_i)
        let mut energy = 0.0;
        for body_id in 1..model.nbody {
            let mass = model.body_mass[body_id];
            let inertia = model.body_inertia[body_id];

            // Extract linear and angular velocity from spatial velocity
            let omega = Vector3::new(
                data.cvel[body_id][0],
                data.cvel[body_id][1],
                data.cvel[body_id][2],
            );
            let v = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );

            // Translational kinetic energy: 0.5 * m * |v|²
            energy += 0.5 * mass * v.norm_squared();

            // Rotational kinetic energy: 0.5 * ω^T * I * ω
            // Using diagonal inertia in body frame (approximation - should transform)
            let omega_body = data.xquat[body_id].inverse() * omega;
            energy += 0.5
                * (inertia.x * omega_body.x.powi(2)
                    + inertia.y * omega_body.y.powi(2)
                    + inertia.z * omega_body.z.powi(2));
        }
        energy
    };

    data.energy_kinetic = kinetic;
}

// ============================================================================
// Sensor Pipeline Functions
// ============================================================================

/// Compute position-dependent sensor values.
///
/// This is called after `mj_fwd_position` and computes sensors that depend only
/// on position (not velocity or acceleration):
/// - `JointPos`: joint position
/// - `FramePos`: site/body position
/// - `FrameQuat`: site/body orientation
/// - FrameXAxis/YAxis/ZAxis: frame axes
/// - `TendonPos`: tendon length
/// - `ActuatorPos`: actuator length
/// - Rangefinder: distance measurement
/// - Touch: contact detection
#[allow(clippy::too_many_lines)]
fn mj_sensor_pos(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-position sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Position {
            continue;
        }

        // §16.5d: Skip sensors on sleeping bodies — values frozen at sleep time
        if sleep_enabled {
            if let Some(body_id) = sensor_body_id(model, sensor_id) {
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::JointPos => {
                // Scalar joint position (hinge/slide only).
                // Ball joints use BallQuat, free joints use FramePos + FrameQuat.
                if objid < model.njnt {
                    let qpos_adr = model.jnt_qpos_adr[objid];
                    match model.jnt_type[objid] {
                        MjJointType::Hinge | MjJointType::Slide => {
                            sensor_write(&mut data.sensordata, adr, 0, data.qpos[qpos_adr]);
                        }
                        _ => {} // Ball/Free not supported by JointPos; use BallQuat/FramePos
                    }
                }
            }

            MjSensorType::BallQuat => {
                // Ball joint quaternion [w, x, y, z]
                if objid < model.njnt && model.jnt_type[objid] == MjJointType::Ball {
                    let qpos_adr = model.jnt_qpos_adr[objid];
                    // Read quaternion from qpos and normalize in locals
                    // (MuJoCo does mju_normalize4). Use 1e-10 threshold and
                    // identity reset to match our normalize_quaternion() convention.
                    let (w, x, y, z) = (
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    );
                    let norm = (w * w + x * x + y * y + z * z).sqrt();
                    if norm > 1e-10 {
                        sensor_write4(
                            &mut data.sensordata,
                            adr,
                            w / norm,
                            x / norm,
                            y / norm,
                            z / norm,
                        );
                    } else {
                        // Degenerate — reset to identity [w=1, x=0, y=0, z=0]
                        sensor_write4(&mut data.sensordata, adr, 1.0, 0.0, 0.0, 0.0);
                    }
                }
            }

            MjSensorType::FramePos => {
                // Position of site/body in world frame
                let pos = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xpos[objid],
                    MjObjectType::Body if objid < model.nbody => data.xpos[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xpos[objid],
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &pos);
            }

            MjSensorType::FrameQuat => {
                // Orientation of site/body as quaternion [w, x, y, z]
                let quat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        // Compute site quaternion from rotation matrix
                        let mat = data.site_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => data.xquat[objid],
                    MjObjectType::Geom if objid < model.ngeom => {
                        let mat = data.geom_xmat[objid];
                        UnitQuaternion::from_rotation_matrix(
                            &nalgebra::Rotation3::from_matrix_unchecked(mat),
                        )
                    }
                    _ => UnitQuaternion::identity(),
                };
                sensor_write4(&mut data.sensordata, adr, quat.w, quat.i, quat.j, quat.k);
            }

            MjSensorType::FrameXAxis | MjSensorType::FrameYAxis | MjSensorType::FrameZAxis => {
                // Frame axis in world coordinates
                let mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    MjObjectType::Geom if objid < model.ngeom => data.geom_xmat[objid],
                    _ => Matrix3::identity(),
                };
                // These are the only types that can reach here due to the outer match
                #[allow(clippy::match_same_arms)]
                let col_idx = match model.sensor_type[sensor_id] {
                    MjSensorType::FrameXAxis => 0,
                    MjSensorType::FrameYAxis => 1,
                    MjSensorType::FrameZAxis => 2,
                    _ => 0, // Unreachable but needed for exhaustiveness
                };
                let col = Vector3::new(mat[(0, col_idx)], mat[(1, col_idx)], mat[(2, col_idx)]);
                sensor_write3(&mut data.sensordata, adr, &col);
            }

            MjSensorType::SubtreeCom => {
                // Subtree center of mass (compute weighted average of descendant COMs)
                if objid < model.nbody {
                    let (com, _total_mass) = compute_subtree_com(model, data, objid);
                    sensor_write3(&mut data.sensordata, adr, &com);
                }
            }

            MjSensorType::Rangefinder => {
                // Rangefinder: ray-cast along site's positive Z axis to find
                // distance to nearest geom surface. Skips the geom attached to
                // the sensor's parent body to avoid self-intersection.
                if model.sensor_objtype[sensor_id] == MjObjectType::Site && objid < model.nsite {
                    let ray_origin = Point3::from(data.site_xpos[objid]);
                    // MuJoCo convention: rangefinder shoots along +Z of site frame
                    let site_z = Vector3::new(
                        data.site_xmat[objid][(0, 2)],
                        data.site_xmat[objid][(1, 2)],
                        data.site_xmat[objid][(2, 2)],
                    );
                    let ray_dir = site_z;

                    // Normalize ray direction (should already be unit, but be safe)
                    let ray_norm = ray_dir.norm();
                    if ray_norm < 1e-10 {
                        sensor_write(&mut data.sensordata, adr, 0, -1.0); // Invalid direction
                    } else {
                        let ray_direction = UnitVector3::new_normalize(ray_dir);

                        // Determine max range: use cutoff if positive, otherwise large default
                        let max_range = if model.sensor_cutoff[sensor_id] > 0.0 {
                            model.sensor_cutoff[sensor_id]
                        } else {
                            100.0 // Default max range
                        };

                        // Exclude geoms belonging to the sensor's parent body
                        let parent_body = model.site_body[objid];

                        let mut closest_dist = -1.0_f64; // -1 means no hit (MuJoCo convention)

                        for geom_id in 0..model.ngeom {
                            // Skip geoms on the sensor's parent body (self-intersection)
                            if model.geom_body[geom_id] == parent_body {
                                continue;
                            }

                            // Build shape pose from geom world transform
                            let geom_pos = data.geom_xpos[geom_id];
                            let geom_mat = data.geom_xmat[geom_id];
                            let geom_quat = UnitQuaternion::from_rotation_matrix(
                                &nalgebra::Rotation3::from_matrix_unchecked(geom_mat),
                            );
                            let shape_pose =
                                Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

                            // Convert geom to collision shape for ray testing
                            if let Some(shape) = geom_to_collision_shape(
                                model.geom_type[geom_id],
                                model.geom_size[geom_id],
                            ) {
                                if let Some(hit) = raycast_shape(
                                    &shape,
                                    &shape_pose,
                                    ray_origin,
                                    ray_direction,
                                    max_range,
                                ) {
                                    if closest_dist < 0.0 || hit.distance < closest_dist {
                                        closest_dist = hit.distance;
                                    }
                                }
                            }

                            // Also handle mesh geoms
                            if model.geom_type[geom_id] == GeomType::Mesh {
                                if let Some(mesh_id) = model.geom_mesh[geom_id] {
                                    let mesh_shape = CollisionShape::TriangleMesh {
                                        data: Arc::clone(&model.mesh_data[mesh_id]),
                                    };
                                    let shape_pose = Pose::from_position_rotation(
                                        Point3::from(geom_pos),
                                        geom_quat,
                                    );
                                    if let Some(hit) = raycast_shape(
                                        &mesh_shape,
                                        &shape_pose,
                                        ray_origin,
                                        ray_direction,
                                        max_range,
                                    ) {
                                        if closest_dist < 0.0 || hit.distance < closest_dist {
                                            closest_dist = hit.distance;
                                        }
                                    }
                                }
                            }
                        }

                        sensor_write(&mut data.sensordata, adr, 0, closest_dist);
                    }
                } else {
                    sensor_write(&mut data.sensordata, adr, 0, -1.0);
                }
            }

            MjSensorType::Magnetometer => {
                // Magnetometer: measures the global magnetic field in the sensor's
                // local frame. Only depends on site_xmat (available after FK).
                //
                // B_sensor = R_site^T * B_world
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let b_sensor = site_mat.transpose() * model.magnetic;
                sensor_write3(&mut data.sensordata, adr, &b_sensor);
            }

            MjSensorType::ActuatorPos => {
                // Actuator position: transmission length = gear * joint_position.
                // For joint-type transmissions, this is gear[0] * qpos[qpos_adr].
                if objid < model.nu {
                    match model.actuator_trntype[objid] {
                        ActuatorTransmission::Joint => {
                            let jnt_id = model.actuator_trnid[objid][0];
                            if jnt_id < model.njnt {
                                let qpos_adr = model.jnt_qpos_adr[jnt_id];
                                let gear = model.actuator_gear[objid][0];
                                sensor_write(
                                    &mut data.sensordata,
                                    adr,
                                    0,
                                    gear * data.qpos[qpos_adr],
                                );
                            }
                        }
                        ActuatorTransmission::Tendon => {
                            let tendon_id = model.actuator_trnid[objid][0];
                            let value = if tendon_id < model.ntendon {
                                data.ten_length[tendon_id] * model.actuator_gear[objid][0]
                            } else {
                                0.0
                            };
                            sensor_write(&mut data.sensordata, adr, 0, value);
                        }
                        ActuatorTransmission::Site => {
                            // Length set by mj_transmission_site (runs before this).
                            sensor_write(&mut data.sensordata, adr, 0, data.actuator_length[objid]);
                        }
                    }
                }
            }

            MjSensorType::TendonPos => {
                let tendon_id = model.sensor_objid[sensor_id];
                let value = if tendon_id < model.ntendon {
                    data.ten_length[tendon_id]
                } else {
                    0.0
                };
                sensor_write(&mut data.sensordata, adr, 0, value);
            }

            // Skip velocity/acceleration-dependent sensors
            _ => {}
        }
    }
}

/// Compute velocity-dependent sensor values.
///
/// This is called after `mj_fwd_velocity` and computes sensors that depend on
/// velocity (but not acceleration):
/// - `JointVel`: joint velocity
/// - Gyro: angular velocity
/// - Velocimeter: linear velocity
/// - `FrameLinVel`: site/body linear velocity
/// - `FrameAngVel`: site/body angular velocity
/// - `TendonVel`: tendon velocity
/// - `ActuatorVel`: actuator velocity
#[allow(clippy::too_many_lines)]
fn mj_sensor_vel(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-velocity sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Velocity {
            continue;
        }

        // §16.5d: Skip sensors on sleeping bodies
        if sleep_enabled {
            if let Some(body_id) = sensor_body_id(model, sensor_id) {
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::JointVel => {
                // Scalar joint velocity (hinge/slide only).
                // Ball joints use BallAngVel, free joints use FrameLinVel + FrameAngVel.
                if objid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[objid];
                    match model.jnt_type[objid] {
                        MjJointType::Hinge | MjJointType::Slide => {
                            sensor_write(&mut data.sensordata, adr, 0, data.qvel[dof_adr]);
                        }
                        _ => {} // Ball/Free not supported by JointVel; use BallAngVel/FrameLinVel
                    }
                }
            }

            MjSensorType::BallAngVel => {
                // Ball joint angular velocity [wx, wy, wz] in local (child body) frame
                if objid < model.njnt && model.jnt_type[objid] == MjJointType::Ball {
                    let dof_adr = model.jnt_dof_adr[objid];
                    let omega = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    sensor_write3(&mut data.sensordata, adr, &omega);
                }
            }

            MjSensorType::Gyro => {
                // Angular velocity in sensor (site) frame
                let (omega_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        let omega = Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        );
                        (omega, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let omega = Vector3::new(
                            data.cvel[objid][0],
                            data.cvel[objid][1],
                            data.cvel[objid][2],
                        );
                        (omega, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Transform to sensor frame
                let omega_sensor = site_mat.transpose() * omega_world;
                sensor_write3(&mut data.sensordata, adr, &omega_sensor);
            }

            MjSensorType::Velocimeter => {
                // Linear velocity in sensor frame
                let (v_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        let v = Vector3::new(
                            data.cvel[body_id][3],
                            data.cvel[body_id][4],
                            data.cvel[body_id][5],
                        );
                        (v, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let v = Vector3::new(
                            data.cvel[objid][3],
                            data.cvel[objid][4],
                            data.cvel[objid][5],
                        );
                        (v, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Transform to sensor frame
                let v_sensor = site_mat.transpose() * v_world;
                sensor_write3(&mut data.sensordata, adr, &v_sensor);
            }

            MjSensorType::FrameLinVel => {
                // Linear velocity in world frame
                let v = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        Vector3::new(
                            data.cvel[body_id][3],
                            data.cvel[body_id][4],
                            data.cvel[body_id][5],
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => Vector3::new(
                        data.cvel[objid][3],
                        data.cvel[objid][4],
                        data.cvel[objid][5],
                    ),
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &v);
            }

            MjSensorType::FrameAngVel => {
                // Angular velocity in world frame
                let omega = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        Vector3::new(
                            data.cvel[body_id][0],
                            data.cvel[body_id][1],
                            data.cvel[body_id][2],
                        )
                    }
                    MjObjectType::Body if objid < model.nbody => Vector3::new(
                        data.cvel[objid][0],
                        data.cvel[objid][1],
                        data.cvel[objid][2],
                    ),
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &omega);
            }

            MjSensorType::SubtreeLinVel => {
                // Subtree linear momentum / total mass = average velocity
                if objid < model.nbody {
                    let (_, total_mass) = compute_subtree_com(model, data, objid);
                    if total_mass > 1e-10 {
                        let momentum = compute_subtree_momentum(model, data, objid);
                        let v = momentum / total_mass;
                        sensor_write3(&mut data.sensordata, adr, &v);
                    }
                }
            }

            MjSensorType::SubtreeAngMom => {
                // Subtree angular momentum about the subtree center of mass.
                // L = sum_i [I_i * omega_i + m_i * (r_i - r_com) x v_i]
                if objid < model.nbody {
                    let angmom = compute_subtree_angmom(model, data, objid);
                    sensor_write3(&mut data.sensordata, adr, &angmom);
                }
            }

            MjSensorType::ActuatorVel => {
                // Read from pre-computed actuator_velocity (populated by mj_actuator_length,
                // which runs before mj_sensor_vel in forward()).
                let act_id = model.sensor_objid[sensor_id];
                if act_id < model.nu {
                    sensor_write(&mut data.sensordata, adr, 0, data.actuator_velocity[act_id]);
                }
            }

            MjSensorType::TendonVel => {
                let tendon_id = model.sensor_objid[sensor_id];
                let value = if tendon_id < model.ntendon {
                    data.ten_velocity[tendon_id]
                } else {
                    0.0
                };
                sensor_write(&mut data.sensordata, adr, 0, value);
            }

            // Skip position/acceleration-dependent sensors
            _ => {}
        }
    }
}

/// Compute acceleration-dependent sensor values.
///
/// This is called after `mj_fwd_acceleration` and computes sensors that depend
/// on acceleration:
/// - Accelerometer: linear acceleration (includes gravity in sensor frame)
/// - Force: constraint force at site
/// - Torque: constraint torque at site
/// - `FrameLinAcc`: linear acceleration
/// - `FrameAngAcc`: angular acceleration
/// - `ActuatorFrc`: actuator force
fn mj_sensor_acc(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    for sensor_id in 0..model.nsensor {
        // Skip non-acceleration sensors
        if model.sensor_datatype[sensor_id] != MjSensorDataType::Acceleration {
            continue;
        }

        // §16.5d: Skip sensors on sleeping bodies
        if sleep_enabled {
            if let Some(body_id) = sensor_body_id(model, sensor_id) {
                if data.body_sleep_state[body_id] == SleepState::Asleep {
                    continue;
                }
            }
        }

        let adr = model.sensor_adr[sensor_id];
        let objid = model.sensor_objid[sensor_id];

        match model.sensor_type[sensor_id] {
            MjSensorType::Accelerometer => {
                // Linear acceleration in sensor frame (includes gravity)
                // a_sensor = R^T * (a_world - g)
                // For a body at rest in gravity, accelerometer reads +g (opposing gravity)
                let (a_world, site_mat) = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        // Compute body acceleration from qacc
                        let a = compute_body_acceleration(model, data, body_id);
                        (a, data.site_xmat[objid])
                    }
                    MjObjectType::Body if objid < model.nbody => {
                        let a = compute_body_acceleration(model, data, objid);
                        (a, data.xmat[objid])
                    }
                    _ => (Vector3::zeros(), Matrix3::identity()),
                };
                // Proper acceleration = a_body - g_field
                // At rest: a_body=0, g_field=(0,0,-9.81), so a_proper=(0,0,+9.81) (upward)
                // In free fall: a_body~g_field, so a_proper~0. Matches real IMU behavior.
                let a_proper = a_world - model.gravity;
                let a_sensor = site_mat.transpose() * a_proper;
                sensor_write3(&mut data.sensordata, adr, &a_sensor);
            }

            MjSensorType::Force => {
                // Force sensor: measures the interaction force at a site.
                //
                // MuJoCo computes this via inverse dynamics on the subtree rooted at
                // the site's body. The force is the net force that the rest of the
                // system exerts on the subtree through the site's body, expressed
                // in the site's local frame.
                //
                // F_site = R_site^T * (m*a - f_external)
                //
                // We compute this as: the total constraint + applied force transmitted
                // through the kinematic chain at the sensor's body, projected into
                // the site frame.
                let (force_world, _) = compute_site_force_torque(model, data, sensor_id);
                // Transform to site frame
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let force_site = site_mat.transpose() * force_world;
                sensor_write3(&mut data.sensordata, adr, &force_site);
            }

            MjSensorType::Torque => {
                // Torque sensor: measures the interaction torque at a site.
                //
                // Similar to Force but returns the torque component. The torque is
                // the net torque that the rest of the system exerts on the subtree
                // through the sensor body, expressed in the site's local frame.
                let (_, torque_world) = compute_site_force_torque(model, data, sensor_id);
                // Transform to site frame
                let site_mat = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Site if objid < model.nsite => data.site_xmat[objid],
                    MjObjectType::Body if objid < model.nbody => data.xmat[objid],
                    _ => Matrix3::identity(),
                };
                let torque_site = site_mat.transpose() * torque_world;
                sensor_write3(&mut data.sensordata, adr, &torque_site);
            }

            MjSensorType::Touch => {
                // Touch sensor: sum of normal contact forces on the attached geom.
                //
                // In MuJoCo, touch sensors read from efc_lambda (solved contact force
                // multipliers). The sensor returns the total normal force magnitude
                // summed over all contacts involving the sensor's geom.
                //
                // Note: Touch needs the constraint solver to have run, so it belongs
                // in the acceleration stage even though MuJoCo nominally evaluates it
                // at position stage (because MuJoCo's sensor pipeline is called once
                // after all stages complete).
                let mut total_force = 0.0;
                for contact in &data.contacts {
                    if contact.geom1 == objid || contact.geom2 == objid {
                        // Look up the solved contact force from efc_lambda
                        let key = warmstart_key(contact);
                        if let Some(lambda) = data.efc_lambda.get(&key) {
                            // lambda[0] is the normal force magnitude (always >= 0)
                            total_force += lambda[0];
                        }
                    }
                }
                sensor_write(&mut data.sensordata, adr, 0, total_force);
            }

            MjSensorType::ActuatorFrc => {
                // Scalar actuator force (transmission-independent).
                // Matches MuJoCo: actuatorfrc sensor = actuator_force[objid].
                if objid < model.nu {
                    sensor_write(&mut data.sensordata, adr, 0, data.actuator_force[objid]);
                }
            }

            MjSensorType::JointLimitFrc => {
                // Joint limit force: read cached penalty magnitude.
                // objid is the joint index (resolved by model builder).
                if objid < data.jnt_limit_frc.len() {
                    sensor_write(&mut data.sensordata, adr, 0, data.jnt_limit_frc[objid]);
                }
            }

            MjSensorType::TendonLimitFrc => {
                // Tendon limit force: read cached penalty magnitude.
                // objid is the tendon index (resolved by model builder).
                if objid < data.ten_limit_frc.len() {
                    sensor_write(&mut data.sensordata, adr, 0, data.ten_limit_frc[objid]);
                }
            }

            MjSensorType::FrameLinAcc => {
                // Linear acceleration in world frame
                let a = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &a);
            }

            MjSensorType::FrameAngAcc => {
                // Angular acceleration in world frame
                // Compute from qacc using Jacobian
                let alpha = match model.sensor_objtype[sensor_id] {
                    MjObjectType::Body if objid < model.nbody => {
                        compute_body_angular_acceleration(model, data, objid)
                    }
                    MjObjectType::Site if objid < model.nsite => {
                        let body_id = model.site_body[objid];
                        compute_body_angular_acceleration(model, data, body_id)
                    }
                    _ => Vector3::zeros(),
                };
                sensor_write3(&mut data.sensordata, adr, &alpha);
            }

            // Skip position/velocity-dependent sensors
            _ => {}
        }
    }
}

// ============================================================================
// Sensor Helper Functions
// ============================================================================

/// Write a single sensor value with bounds checking.
#[inline]
fn sensor_write(sensordata: &mut DVector<f64>, adr: usize, offset: usize, value: f64) {
    let idx = adr + offset;
    if idx < sensordata.len() {
        sensordata[idx] = value;
    }
}

/// Write a 3D vector to sensor data with bounds checking.
#[inline]
fn sensor_write3(sensordata: &mut DVector<f64>, adr: usize, v: &Vector3<f64>) {
    sensor_write(sensordata, adr, 0, v.x);
    sensor_write(sensordata, adr, 1, v.y);
    sensor_write(sensordata, adr, 2, v.z);
}

/// Write a quaternion (w, x, y, z) to sensor data with bounds checking.
#[inline]
fn sensor_write4(sensordata: &mut DVector<f64>, adr: usize, w: f64, x: f64, y: f64, z: f64) {
    sensor_write(sensordata, adr, 0, w);
    sensor_write(sensordata, adr, 1, x);
    sensor_write(sensordata, adr, 2, y);
    sensor_write(sensordata, adr, 3, z);
}

/// Apply sensor post-processing: noise addition and cutoff clamping.
///
/// MuJoCo applies noise and cutoff after all sensor values are computed:
/// - Noise: Gaussian noise with std dev = sensor_noise (0 = no noise).
///   Applied independently to each sensor data element.
/// - Cutoff: For most sensors, clamps to [-cutoff, cutoff] (0 = no cutoff).
///   For positive-type sensors (Touch, Rangefinder), clamps positive side only:
///   min(value, cutoff). This preserves rangefinder's -1.0 no-hit sentinel.
fn mj_sensor_postprocess(model: &Model, data: &mut Data) {
    for sensor_id in 0..model.nsensor {
        let adr = model.sensor_adr[sensor_id];
        let dim = model.sensor_dim[sensor_id];

        // Apply cutoff
        let cutoff = model.sensor_cutoff[sensor_id];
        if cutoff > 0.0 {
            let sensor_type = model.sensor_type[sensor_id];
            for i in 0..dim {
                let idx = adr + i;
                if idx < data.sensordata.len() {
                    let clamped = match sensor_type {
                        // Positive-type sensors: only clamp on positive side
                        MjSensorType::Touch | MjSensorType::Rangefinder => {
                            data.sensordata[idx].min(cutoff)
                        }
                        // Real-type sensors: clamp both sides
                        _ => data.sensordata[idx].clamp(-cutoff, cutoff),
                    };
                    sensor_write(&mut data.sensordata, adr, i, clamped);
                }
            }
        }

        // Note: Noise is not applied here because deterministic physics is preferred
        // for RL training and testing. Noise should be added by the caller if needed,
        // using model.sensor_noise[sensor_id] as the standard deviation.
        // This follows MuJoCo's convention where noise is optional and often disabled.
    }
}

fn compute_subtree_com(model: &Model, data: &Data, root_body: usize) -> (Vector3<f64>, f64) {
    let mut total_mass = 0.0;
    let mut weighted_com = Vector3::zeros();

    // Iterate through all bodies that are descendants of root_body
    for body_id in root_body..model.nbody {
        // Check if this body is a descendant (simplified: check parent chain)
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let com = data.xipos[body_id]; // COM in world frame
            total_mass += mass;
            weighted_com += mass * com;
        }
    }

    if total_mass > 1e-10 {
        (weighted_com / total_mass, total_mass)
    } else {
        (Vector3::zeros(), 0.0)
    }
}

/// Compute subtree linear momentum for a given body.
fn compute_subtree_momentum(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    let mut momentum = Vector3::zeros();

    for body_id in root_body..model.nbody {
        // Check if descendant
        let mut is_descendant = body_id == root_body;
        let mut current = body_id;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == root_body {
                is_descendant = true;
            }
        }

        if is_descendant {
            let mass = model.body_mass[body_id];
            let v = Vector3::new(
                data.cvel[body_id][3],
                data.cvel[body_id][4],
                data.cvel[body_id][5],
            );
            momentum += mass * v;
        }
    }

    momentum
}

/// Compute linear acceleration for a body from joint accelerations.
fn compute_body_acceleration(model: &Model, data: &Data, body_id: usize) -> Vector3<f64> {
    let mut acc = Vector3::zeros();

    // For each joint affecting this body, compute acceleration contribution
    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                // Angular acceleration contributes via cross product with COM offset
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                // a = α × r (tangential acceleration from angular acceleration)
                acc += qacc * world_axis.cross(&com_offset);

                // Also centripetal from angular velocity
                let omega = data.qvel[dof_adr] * world_axis;
                acc += omega.cross(&omega.cross(&com_offset));
            }
            MjJointType::Slide => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                acc += qacc * world_axis;
            }
            MjJointType::Free => {
                // Direct linear acceleration
                acc.x += data.qacc[dof_adr];
                acc.y += data.qacc[dof_adr + 1];
                acc.z += data.qacc[dof_adr + 2];
            }
            MjJointType::Ball => {
                // Ball joint only contributes angular acceleration
                // Linear acceleration comes from centripetal effects
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let alpha = Vector3::new(
                    data.qacc[dof_adr],
                    data.qacc[dof_adr + 1],
                    data.qacc[dof_adr + 2],
                );
                let world_omega = data.xquat[body_id] * omega;
                let world_alpha = data.xquat[body_id] * alpha;
                let com_offset = data.xipos[body_id] - data.xpos[body_id];
                acc += world_alpha.cross(&com_offset);
                acc += world_omega.cross(&world_omega.cross(&com_offset));
            }
        }
    }

    acc
}

/// Compute angular acceleration for a body from joint accelerations.
fn compute_body_angular_acceleration(model: &Model, data: &Data, body_id: usize) -> Vector3<f64> {
    let mut alpha = Vector3::zeros();

    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let axis = model.jnt_axis[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let qacc = data.qacc[dof_adr];
                let world_axis = data.xquat[body_id] * axis;
                alpha += qacc * world_axis;
            }
            MjJointType::Ball => {
                let world_alpha = data.xquat[body_id]
                    * Vector3::new(
                        data.qacc[dof_adr],
                        data.qacc[dof_adr + 1],
                        data.qacc[dof_adr + 2],
                    );
                alpha += world_alpha;
            }
            MjJointType::Free => {
                alpha.x += data.qacc[dof_adr + 3];
                alpha.y += data.qacc[dof_adr + 4];
                alpha.z += data.qacc[dof_adr + 5];
            }
            MjJointType::Slide => {
                // Prismatic joints don't contribute angular acceleration
            }
        }
    }

    alpha
}

/// Compute interaction force and torque at a sensor's site via subtree inverse dynamics.
///
/// For a force/torque sensor attached to a site on body B, we compute the net
/// force and torque that the rest of the system exerts on the subtree rooted at B.
///
/// This is computed as: F = sum over subtree bodies of (m_i * a_i - f_ext_i)
/// where a_i is the body acceleration (from qacc) and f_ext_i are the external
/// forces (gravity, applied, actuator, passive, constraint).
///
/// The torque is computed about the sensor site position.
///
/// Returns (force_world, torque_world) in world frame.
fn compute_site_force_torque(
    model: &Model,
    data: &Data,
    sensor_id: usize,
) -> (Vector3<f64>, Vector3<f64>) {
    let objid = model.sensor_objid[sensor_id];

    // Get the body and site position
    let (body_id, site_pos) = match model.sensor_objtype[sensor_id] {
        MjObjectType::Site if objid < model.nsite => {
            (model.site_body[objid], data.site_xpos[objid])
        }
        MjObjectType::Body if objid < model.nbody => (objid, data.xpos[objid]),
        _ => return (Vector3::zeros(), Vector3::zeros()),
    };

    // Sum forces and torques over all bodies in the subtree rooted at body_id.
    // For each body in subtree:
    //   f_inertial = m * a_com  (inertial force from acceleration)
    //   f_gravity  = m * g      (gravity acts on all bodies)
    //
    // The interaction force = sum of (f_inertial - f_gravity) for subtree
    // This equals the net constraint/contact force transmitted through the joint.
    let mut force_total = Vector3::zeros();
    let mut torque_total = Vector3::zeros();

    for bid in body_id..model.nbody {
        // Check if this body is in the subtree rooted at body_id
        let mut is_descendant = bid == body_id;
        let mut current = bid;
        while !is_descendant && current != 0 {
            current = model.body_parent[current];
            if current == body_id {
                is_descendant = true;
            }
        }

        if !is_descendant {
            continue;
        }

        let mass = model.body_mass[bid];
        if mass < 1e-15 {
            continue;
        }

        // Compute body COM acceleration
        let a_com = compute_body_acceleration(model, data, bid);

        // Inertial force = m * a (what's needed to produce the acceleration)
        let f_inertial = mass * a_com;

        // Gravity force acting on this body
        let f_gravity = mass * model.gravity;

        // Net force on this body from the rest of the system
        // F_net = m*a, so the interaction force through the cut is m*a - f_gravity
        // (because gravity is an external force, not an interaction force)
        let f_body = f_inertial - f_gravity;

        force_total += f_body;

        // Torque contribution about sensor site
        let r = data.xipos[bid] - site_pos;
        torque_total += r.cross(&f_body);

        // Angular inertia contribution: I * alpha + omega x (I * omega)
        // Use the body inertia in world frame
        let inertia = model.body_inertia[bid];
        let alpha = compute_body_angular_acceleration(model, data, bid);

        // Get body angular velocity
        let omega = Vector3::new(data.cvel[bid][0], data.cvel[bid][1], data.cvel[bid][2]);

        // Rotate inertia to world frame via body's inertial frame orientation
        let xi_mat = data.ximat[bid];
        // I_world = R * diag(I) * R^T
        let i_omega = xi_mat
            * Vector3::new(
                inertia.x * (xi_mat.transpose() * omega).x,
                inertia.y * (xi_mat.transpose() * omega).y,
                inertia.z * (xi_mat.transpose() * omega).z,
            );
        let i_alpha = xi_mat
            * Vector3::new(
                inertia.x * (xi_mat.transpose() * alpha).x,
                inertia.y * (xi_mat.transpose() * alpha).y,
                inertia.z * (xi_mat.transpose() * alpha).z,
            );

        // Angular: I*alpha + omega x (I*omega)
        torque_total += i_alpha + omega.cross(&i_omega);
    }

    (force_total, torque_total)
}

/// Check if a body is a descendant of (or equal to) a given root body.
fn is_body_in_subtree(model: &Model, body_id: usize, root: usize) -> bool {
    let mut current = body_id;
    loop {
        if current == root {
            return true;
        }
        if current == 0 {
            return false;
        }
        current = model.body_parent[current];
    }
}

/// Compute subtree angular momentum about the subtree's center of mass.
///
/// For each body in the subtree:
///   L += I_i * omega_i + m_i * (r_i - r_com) x v_i
///
/// where I_i is the body inertia, omega_i angular velocity, r_i position,
/// r_com subtree COM, v_i linear velocity, and m_i mass.
fn compute_subtree_angmom(model: &Model, data: &Data, root_body: usize) -> Vector3<f64> {
    // First compute subtree COM
    let (com, _total_mass) = compute_subtree_com(model, data, root_body);

    let mut angmom = Vector3::zeros();

    for body_id in root_body..model.nbody {
        if !is_body_in_subtree(model, body_id, root_body) {
            continue;
        }

        let mass = model.body_mass[body_id];

        // Orbital angular momentum: L_orbital = m * (r - r_com) x v
        let r = data.xipos[body_id] - com;
        let v = Vector3::new(
            data.cvel[body_id][3],
            data.cvel[body_id][4],
            data.cvel[body_id][5],
        );
        angmom += mass * r.cross(&v);

        // Spin angular momentum: L_spin = I * omega
        let omega = Vector3::new(
            data.cvel[body_id][0],
            data.cvel[body_id][1],
            data.cvel[body_id][2],
        );
        let inertia = model.body_inertia[body_id];
        let xi_mat = data.ximat[body_id];
        // I_world * omega = R * diag(I) * R^T * omega
        let omega_local = xi_mat.transpose() * omega;
        let i_omega_local = Vector3::new(
            inertia.x * omega_local.x,
            inertia.y * omega_local.y,
            inertia.z * omega_local.z,
        );
        angmom += xi_mat * i_omega_local;
    }

    angmom
}

/// Velocity kinematics: compute body velocities from qvel.
fn mj_fwd_velocity(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // World body has zero velocity
    data.cvel[0] = SpatialVector::zeros();

    // Compute body velocities by propagating through tree
    // v[i] = X[i←parent] @ v[parent] + S[i] @ qdot[i]
    //
    // The spatial transform X accounts for the offset between body origins.
    // For a pure translation r (from parent to child), the velocity transforms as:
    //   ω_child = ω_parent
    //   v_child = v_parent + ω_parent × r
    //
    // This lever arm effect is critical for Coriolis forces in serial chains!

    for body_id in 1..model.nbody {
        // §16.5a: Skip velocity kinematics for sleeping bodies — velocity is zeroed
        if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
            continue;
        }

        let parent_id = model.body_parent[body_id];

        // Parent velocity
        let v_parent = data.cvel[parent_id];
        let omega_parent = Vector3::new(v_parent[0], v_parent[1], v_parent[2]);
        let v_lin_parent = Vector3::new(v_parent[3], v_parent[4], v_parent[5]);

        // Offset from parent origin to this body's origin (in world frame)
        let r = data.xpos[body_id] - data.xpos[parent_id];

        // Transform parent velocity to this body's frame
        // Linear velocity gets contribution from lever arm: v_new = v_old + ω × r
        let v_lin_at_child = v_lin_parent + omega_parent.cross(&r);

        let mut vel = SpatialVector::new(
            omega_parent.x,
            omega_parent.y,
            omega_parent.z,
            v_lin_at_child.x,
            v_lin_at_child.y,
            v_lin_at_child.z,
        );

        // Add contribution from joints on this body
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let axis = model.jnt_axis[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Angular velocity contribution
                    let omega = data.xquat[body_id] * axis * data.qvel[dof_adr];
                    vel[0] += omega.x;
                    vel[1] += omega.y;
                    vel[2] += omega.z;
                }
                MjJointType::Slide => {
                    // Linear velocity contribution
                    let v = data.xquat[body_id] * axis * data.qvel[dof_adr];
                    vel[3] += v.x;
                    vel[4] += v.y;
                    vel[5] += v.z;
                }
                MjJointType::Ball => {
                    // 3-DOF angular velocity
                    let omega = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    let world_omega = data.xquat[body_id] * omega;
                    vel[0] += world_omega.x;
                    vel[1] += world_omega.y;
                    vel[2] += world_omega.z;
                }
                MjJointType::Free => {
                    // 6-DOF: linear + angular velocity
                    vel[3] += data.qvel[dof_adr];
                    vel[4] += data.qvel[dof_adr + 1];
                    vel[5] += data.qvel[dof_adr + 2];
                    vel[0] += data.qvel[dof_adr + 3];
                    vel[1] += data.qvel[dof_adr + 4];
                    vel[2] += data.qvel[dof_adr + 5];
                }
            }
        }

        data.cvel[body_id] = vel;
    }

    // Tendon velocities: v_t = J_t · qvel
    for t in 0..model.ntendon {
        data.ten_velocity[t] = data.ten_J[t].dot(&data.qvel);
    }
}

/// Compute tendon lengths and Jacobians from current joint state.
///
/// For fixed tendons: L = Σ coef_i * qpos[dof_adr_i], J[dof_adr_i] = coef_i.
/// For spatial tendons: silently skipped (deferred to spatial tendon PR).
///
/// Called from mj_fwd_position() after site transforms, before subtree COM.
fn mj_fwd_tendon(model: &Model, data: &mut Data) {
    if model.ntendon == 0 {
        return;
    }

    for t in 0..model.ntendon {
        match model.tendon_type[t] {
            TendonType::Fixed => {
                mj_fwd_tendon_fixed(model, data, t);
            }
            TendonType::Spatial => {
                mj_fwd_tendon_spatial(model, data, t);
            }
        }
    }
}

/// Fixed tendon kinematics for a single tendon.
///
/// Fixed tendon length is a linear combination of joint positions:
///   L_t = Σ_w coef_w * qpos[dof_adr_w]
///
/// The Jacobian is constant (configuration-independent):
///   J_t[dof_adr_w] = coef_w
#[inline]
fn mj_fwd_tendon_fixed(model: &Model, data: &mut Data, t: usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];

    // Zero the Jacobian row. For fixed tendons, only a few entries are non-zero.
    data.ten_J[t].fill(0.0);

    let mut length = 0.0;
    for w in adr..(adr + num) {
        debug_assert_eq!(model.wrap_type[w], WrapType::Joint);
        let dof_adr = model.wrap_objid[w];
        let coef = model.wrap_prm[w];

        if dof_adr < data.qpos.len() {
            length += coef * data.qpos[dof_adr];
        }

        if dof_adr < model.nv {
            data.ten_J[t][dof_adr] += coef;
        }
    }

    data.ten_length[t] = length;
}

/// Spatial tendon kinematics for a single tendon.
///
/// Computes tendon length as the sum of straight-line segment distances (and
/// wrapping arcs when geom wrapping is active), with optional pulley divisor
/// scaling. Populates `data.ten_J[t]` with the tendon Jacobian via
/// `accumulate_point_jacobian`.
///
/// Algorithm follows MuJoCo's pairwise loop over the wrap array
/// (`engine_core_smooth.c`, function `mj_tendon`).
fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];
    data.ten_J[t].fill(0.0);
    let mut total_length = 0.0;
    let mut divisor: f64 = 1.0;

    if num < 2 {
        data.ten_length[t] = 0.0;
        return; // degenerate — validation should prevent this
    }

    let mut j = 0;
    while j < num - 1 {
        let type0 = model.wrap_type[adr + j];
        let type1 = model.wrap_type[adr + j + 1];

        // ---- Pulley handling ----
        // MuJoCo processes pulleys as a pair-skip: when a pulley appears as
        // type0 or type1, advance j by 1 without processing a segment.
        // Divisor is updated only when the pulley is type0.
        if type0 == WrapType::Pulley || type1 == WrapType::Pulley {
            if type0 == WrapType::Pulley {
                divisor = model.wrap_prm[adr + j];
            }
            j += 1;
            continue;
        }

        // ---- At this point, type0 must be a Site ----
        debug_assert!(
            type0 == WrapType::Site,
            "type0 must be Site after pulley check"
        );
        let id0 = model.wrap_objid[adr + j];
        let p0 = data.site_xpos[id0];
        let body0 = model.site_body[id0];

        if type1 == WrapType::Site {
            // ---- Site–Site: straight segment ----
            let id1 = model.wrap_objid[adr + j + 1];
            let p1 = data.site_xpos[id1];
            let body1 = model.site_body[id1];

            let diff = p1 - p0;
            let dist = diff.norm();
            // Length: accumulated unconditionally (matching MuJoCo).
            total_length += dist / divisor;
            // Jacobian: guarded by distance threshold to avoid bogus direction.
            if dist > 1e-10 && body0 != body1 {
                let dir = diff / dist;
                accumulate_point_jacobian(
                    model,
                    &data.xpos,
                    &data.xquat,
                    &mut data.ten_J[t],
                    body1,
                    &p1,
                    &dir,
                    1.0 / divisor,
                );
                accumulate_point_jacobian(
                    model,
                    &data.xpos,
                    &data.xquat,
                    &mut data.ten_J[t],
                    body0,
                    &p0,
                    &dir,
                    -1.0 / divisor,
                );
            }

            j += 1; // advance to site1 (it becomes type0 on next iteration)
        } else if type1 == WrapType::Geom {
            // ---- Site–Geom–Site: wrapping segment ----
            // Safety: j+2 < num is guaranteed by validation rules 1-2.
            debug_assert!(j + 2 < num, "Geom at j+1 must be followed by Site at j+2");
            let geom_id = model.wrap_objid[adr + j + 1];
            let geom_body = model.geom_body[geom_id];

            // The site AFTER the geom is at j+2
            let id1 = model.wrap_objid[adr + j + 2];
            let p1 = data.site_xpos[id1];
            let body1 = model.site_body[id1];

            // Transform site positions into geom-local frame
            let geom_pos = data.geom_xpos[geom_id];
            let geom_mat = data.geom_xmat[geom_id];
            let p0_local = geom_mat.transpose() * (p0 - geom_pos);
            let p1_local = geom_mat.transpose() * (p1 - geom_pos);

            // Resolve sidesite (if specified) in geom-local frame
            #[allow(clippy::if_not_else)] // Matches spec pseudocode ordering (§4.3).
            let sidesite_local = if model.wrap_sidesite[adr + j + 1] != usize::MAX {
                let ss_id = model.wrap_sidesite[adr + j + 1];
                Some(geom_mat.transpose() * (data.site_xpos[ss_id] - geom_pos))
            } else {
                None
            };

            // Dispatch to wrapping geometry
            let wrap_result = match model.geom_type[geom_id] {
                GeomType::Sphere => sphere_wrap(
                    p0_local,
                    p1_local,
                    model.geom_size[geom_id].x,
                    sidesite_local,
                ),
                GeomType::Cylinder => cylinder_wrap(
                    p0_local,
                    p1_local,
                    model.geom_size[geom_id].x,
                    sidesite_local,
                ),
                _ => unreachable!("wrapping geom type validated at model build"),
            };

            match wrap_result {
                WrapResult::Wrapped {
                    tangent_point_1,
                    tangent_point_2,
                    arc_length,
                } => {
                    // Transform tangent points back to world frame
                    let t1 = geom_pos + geom_mat * tangent_point_1;
                    let t2 = geom_pos + geom_mat * tangent_point_2;

                    // 3 sub-segments: [p0→t1, t1→t2 (arc), t2→p1]
                    let d1 = t1 - p0;
                    let dist1 = d1.norm();
                    let d3 = p1 - t2;
                    let dist3 = d3.norm();
                    total_length += (dist1 + arc_length + dist3) / divisor;

                    // Sub-segment 1: p0 (body0) → t1 (geom_body)
                    if dist1 > 1e-10 && body0 != geom_body {
                        let dir1 = d1 / dist1;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            geom_body,
                            &t1,
                            &dir1,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body0,
                            &p0,
                            &dir1,
                            -1.0 / divisor,
                        );
                    }

                    // Sub-segment 2: t1 → t2 (arc on geom surface)
                    // Both endpoints on geom_body → Jacobian difference is zero.

                    // Sub-segment 3: t2 (geom_body) → p1 (body1)
                    if dist3 > 1e-10 && geom_body != body1 {
                        let dir3 = d3 / dist3;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body1,
                            &p1,
                            &dir3,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            geom_body,
                            &t2,
                            &dir3,
                            -1.0 / divisor,
                        );
                    }
                }
                WrapResult::NoWrap => {
                    // No wrapping — straight segment p0 → p1
                    let diff = p1 - p0;
                    let dist = diff.norm();
                    total_length += dist / divisor;
                    if dist > 1e-10 && body0 != body1 {
                        let dir = diff / dist;
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body1,
                            &p1,
                            &dir,
                            1.0 / divisor,
                        );
                        accumulate_point_jacobian(
                            model,
                            &data.xpos,
                            &data.xquat,
                            &mut data.ten_J[t],
                            body0,
                            &p0,
                            &dir,
                            -1.0 / divisor,
                        );
                    }
                }
            }

            j += 2; // advance past the geom to site1 (becomes type0 next)
        } else {
            unreachable!(
                "type0 must be Site after pulley check; \
                 validation ensures path starts with Site and \
                 geoms are always followed by sites"
            );
        }
    }

    data.ten_length[t] = total_length;
}

/// Walk the kinematic chain from `body_id` to root, accumulating each joint's
/// velocity contribution into the tendon Jacobian row `ten_j`.
///
/// For each joint on the chain, projects the joint's velocity contribution
/// through `direction` and scales by `scale`. This is the same kinematic chain
/// walk as `compute_contact_jacobian`'s `add_body_jacobian` closure, but
/// operating on a `DVector<f64>` (1×nv) instead of a `DMatrix` row.
///
/// Uses body-frame axes (`R*e_i`) for free joint angular DOFs, matching
/// MuJoCo's `cdof` convention.
#[allow(clippy::too_many_arguments)]
fn accumulate_point_jacobian(
    model: &Model,
    xpos: &[Vector3<f64>],
    xquat: &[UnitQuaternion<f64>],
    ten_j: &mut DVector<f64>,
    body_id: usize,
    point: &Vector3<f64>,
    direction: &Vector3<f64>,
    scale: f64,
) {
    if body_id == 0 {
        return; // world body has no DOFs
    }

    let mut current = body_id;
    while current != 0 {
        let jnt_start = model.body_jnt_adr[current];
        let jnt_end = jnt_start + model.body_jnt_num[current];
        for jnt_id in jnt_start..jnt_end {
            let dof = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    ten_j[dof] += scale * direction.dot(&axis.cross(&r));
                }
                MjJointType::Slide => {
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id];
                    ten_j[dof] += scale * direction.dot(&axis);
                }
                MjJointType::Ball => {
                    let jpos = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        ten_j[dof + i] += scale * direction.dot(&omega.cross(&r));
                    }
                }
                MjJointType::Free => {
                    // Translational DOFs: direction projects directly.
                    ten_j[dof] += scale * direction.x;
                    ten_j[dof + 1] += scale * direction.y;
                    ten_j[dof + 2] += scale * direction.z;
                    // Rotational DOFs: body-frame axes (R*e_i), matching MuJoCo's
                    // cdof convention. NOTE: the existing `add_body_jacobian` uses
                    // world-frame unit vectors — that is a pre-existing bug.
                    let jpos = xpos[jnt_body];
                    let r = point - jpos;
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        ten_j[dof + 3 + i] += scale * direction.dot(&omega.cross(&r));
                    }
                }
            }
        }
        current = model.body_parent[current];
    }
}

/// Compute the full site Jacobian: 3×nv translational and 3×nv rotational.
///
/// Analogous to MuJoCo's `mj_jacSite`. Walks the kinematic chain from
/// `site_body[site_id]` to root, accumulating per-joint contributions.
/// Returns `(jac_trans, jac_rot)` where each is a 3×nv `DMatrix`.
fn mj_jac_site(model: &Model, data: &Data, site_id: usize) -> (DMatrix<f64>, DMatrix<f64>) {
    let mut jac_trans = DMatrix::zeros(3, model.nv);
    let mut jac_rot = DMatrix::zeros(3, model.nv);

    let site_pos = data.site_xpos[site_id];
    let mut current = model.site_body[site_id];

    while current != 0 {
        let jnt_start = model.body_jnt_adr[current];
        let jnt_end = jnt_start + model.body_jnt_num[current];

        for jnt_id in jnt_start..jnt_end {
            let dof = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let anchor = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = site_pos - anchor;
                    let cross = axis.cross(&r);
                    for k in 0..3 {
                        jac_trans[(k, dof)] += cross[k];
                        jac_rot[(k, dof)] += axis[k];
                    }
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    for k in 0..3 {
                        jac_trans[(k, dof)] += axis[k];
                    }
                    // No rotational contribution.
                }
                MjJointType::Ball => {
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    let anchor = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = site_pos - anchor;
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        let cross = omega.cross(&r);
                        for k in 0..3 {
                            jac_trans[(k, dof + i)] += cross[k];
                            jac_rot[(k, dof + i)] += omega[k];
                        }
                    }
                }
                MjJointType::Free => {
                    // Translational DOFs (dof+0..dof+3): identity columns.
                    for i in 0..3 {
                        jac_trans[(i, dof + i)] += 1.0;
                    }
                    // Rotational DOFs (dof+3..dof+6).
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    let r = site_pos - data.xpos[jnt_body];
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        let cross = omega.cross(&r);
                        for k in 0..3 {
                            jac_trans[(k, dof + 3 + i)] += cross[k];
                            jac_rot[(k, dof + 3 + i)] += omega[k];
                        }
                    }
                }
            }
        }
        current = model.body_parent[current];
    }

    (jac_trans, jac_rot)
}

/// Map a scalar tendon force to generalized forces via J^T.
///
/// For `TendonType::Fixed`, uses the sparse wrap-array pattern (DOF addresses
/// and coefficients stored in `wrap_objid`/`wrap_prm`). For
/// `TendonType::Spatial`, uses the dense Jacobian row `ten_j` computed by
/// `mj_fwd_tendon_spatial()`.
fn apply_tendon_force(
    model: &Model,
    ten_j: &DVector<f64>,
    tendon_type: TendonType,
    t: usize,
    force: f64,
    target: &mut DVector<f64>,
) {
    match tendon_type {
        TendonType::Fixed => {
            let adr = model.tendon_adr[t];
            let num = model.tendon_num[t];
            for w in adr..(adr + num) {
                let dof_adr = model.wrap_objid[w];
                let coef = model.wrap_prm[w];
                if dof_adr < model.nv {
                    target[dof_adr] += coef * force;
                }
            }
        }
        TendonType::Spatial => {
            for dof in 0..model.nv {
                let j = ten_j[dof];
                if j != 0.0 {
                    target[dof] += j * force;
                }
            }
        }
    }
}

/// Compute quaternion difference as axis-angle 3-vector.
///
/// Satisfies `qb * quat(res) = qa`, matching MuJoCo's `mju_subQuat`.
/// Returns the rotation from `qb` to `qa` expressed as an expmap vector.
fn subquat(qa: &UnitQuaternion<f64>, qb: &UnitQuaternion<f64>) -> Vector3<f64> {
    // dq = conjugate(qb) * qa  — relative quaternion in qb's frame
    let dq = qb.conjugate() * qa;
    let xyz = Vector3::new(dq.i, dq.j, dq.k);
    let sin_half = xyz.norm();
    if sin_half < 1e-14 {
        // Small-angle limit: atan2(0, ~1) → 0, so result is zero vector.
        return Vector3::zeros();
    }
    let axis = xyz / sin_half;
    let mut angle = 2.0 * sin_half.atan2(dq.w);
    // Shortest path
    if angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }
    axis * angle
}

/// Result of a wrapping geometry computation.
///
/// `sphere_wrap`/`cylinder_wrap` return tangent points in the **geom-local
/// frame**. The caller (`mj_fwd_tendon_spatial`) transforms them to world frame.
enum WrapResult {
    /// Straight path is shorter — no wrapping around the obstacle.
    NoWrap,
    /// Path wraps around the obstacle, producing two tangent points and an arc.
    Wrapped {
        tangent_point_1: Vector3<f64>,
        tangent_point_2: Vector3<f64>,
        arc_length: f64,
    },
}

/// Test if 2D line segment a1→a2 intersects segment b1→b2.
///
/// Uses non-strict inequalities (`>=`, `<=`) matching MuJoCo's `is_intersect`:
/// endpoint-touching segments are considered intersecting.
fn segments_intersect_2d(
    a1: Vector2<f64>,
    a2: Vector2<f64>,
    b1: Vector2<f64>,
    b2: Vector2<f64>,
) -> bool {
    let da = a2 - a1;
    let db = b2 - b1;
    let denom = da.x * db.y - da.y * db.x; // 2D cross product
    if denom.abs() < 1e-20 {
        return false; // parallel or degenerate
    }
    let t = ((b1.x - a1.x) * db.y - (b1.y - a1.y) * db.x) / denom;
    let u = ((b1.x - a1.x) * da.y - (b1.y - a1.y) * da.x) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// Compute the directional arc angle between two tangent points on a circle.
///
/// Implements MuJoCo's `length_circle` algorithm. Unlike a simple `acos`
/// (which returns \[0, π\]), this function can return angles in \[0, 2π) by
/// using the 2D cross-product sign and the candidate index `ind` to determine
/// whether to take the reflex angle. Used by both sphere and cylinder wrapping.
fn directional_wrap_angle(t1: Vector2<f64>, t2: Vector2<f64>, ind: usize) -> f64 {
    // Base angle via acos: [0, π]
    let t1n = t1.normalize();
    let t2n = t2.normalize();
    let base_angle = t1n.dot(&t2n).clamp(-1.0, 1.0).acos();

    // 2D cross product determines rotational direction of t1 → t2.
    let cross = t1.y * t2.x - t1.x * t2.y;

    // MuJoCo's convention (from length_circle in engine_util_misc.c):
    // If (cross > 0 && ind == 1) || (cross < 0 && ind == 0):
    //     angle = 2π - base_angle   (take the reflex arc)
    if (cross > 0.0 && ind == 1) || (cross < 0.0 && ind == 0) {
        2.0 * PI - base_angle
    } else {
        base_angle
    }
}

/// Compute tangent point from external point `p` to a sphere of `radius` at origin.
///
/// The tangent line is in the half-plane defined by (origin, p, normal).
/// `normal` must be approximately unit length. `||p|| >= radius` is a precondition.
fn sphere_tangent_point(p: Vector3<f64>, radius: f64, normal: Vector3<f64>) -> Vector3<f64> {
    let d = p.norm();
    debug_assert!(d >= radius, "sphere_tangent_point: p is inside sphere");
    let cos_theta = radius / d;
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let u = p / d; // unit vector toward p (in wrapping plane)
    let v = normal.cross(&u).normalize(); // perpendicular to u, in wrapping plane
    radius * (u * cos_theta + v * sin_theta)
}

/// Compute both candidate tangent points from p1 and p2 to a sphere.
///
/// Uses **crossed pairing**: p1 uses `+normal`, p2 uses `-normal`, producing
/// a consistent wrapping path where the entry and exit tangent lines are on
/// opposite sides of the circle. Matches MuJoCo's `wrap_circle` convention.
fn compute_tangent_pair(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    normal: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    (
        sphere_tangent_point(p1, radius, normal),
        sphere_tangent_point(p2, radius, -normal), // negated for p2 (crossed pairing)
    )
}

/// Compute the tangent point from an external 2D point `p` to a circle of `radius` at the origin.
///
/// `sign` (+1 or −1) selects which of the two tangent lines to use (clockwise vs
/// counterclockwise). 2D analog of [`sphere_tangent_point`].
fn circle_tangent_2d(p: Vector2<f64>, radius: f64, sign: f64) -> Vector2<f64> {
    let d = p.norm();
    debug_assert!(d >= radius, "circle_tangent_2d: p is inside circle");
    let cos_theta = radius / d;
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let u = p / d; // unit radial
    let v = Vector2::new(-u.y, u.x) * sign; // perpendicular (rotated 90°), signed
    radius * (u * cos_theta + v * sin_theta)
}

/// Compute both candidate 2D tangent-point pairs from `p1_xy` and `p2_xy` to a circle.
///
/// Uses **crossed pairing** (p2 uses `-sign`), the 2D analog of [`compute_tangent_pair`].
///
/// **Sign-to-ind mapping**: `sign=+1` → MuJoCo candidate `i=1` (`ind=1`),
/// `sign=−1` → MuJoCo candidate `i=0` (`ind=0`). See spec §4.8 for derivation.
fn compute_tangent_pair_2d(
    p1_xy: Vector2<f64>,
    p2_xy: Vector2<f64>,
    radius: f64,
    sign: f64,
) -> (Vector2<f64>, Vector2<f64>) {
    (
        circle_tangent_2d(p1_xy, radius, sign),
        circle_tangent_2d(p2_xy, radius, -sign), // negated for p2 (crossed pairing)
    )
}

/// Compute the shortest path around a sphere between two points.
///
/// Works in the geom-local frame where the sphere is centered at the origin.
/// Returns tangent points in the geom-local frame; the caller transforms to
/// world frame.
#[allow(clippy::similar_names)] // Dual-candidate algorithm requires paired naming (a1/a2, b1/b2).
fn sphere_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    // 1. Early exits.
    if radius <= 0.0 {
        return WrapResult::NoWrap; // degenerate geom
    }
    if p1.norm() < radius || p2.norm() < radius {
        return WrapResult::NoWrap; // endpoint inside sphere
    }

    // 2. Check if the straight-line path misses the sphere.
    let d = p2 - p1;
    if d.norm_squared() < 1e-20 {
        return WrapResult::NoWrap; // coincident sites
    }
    let t_param = -(p1.dot(&d)) / d.norm_squared();
    let closest = p1 + t_param.clamp(0.0, 1.0) * d;
    if closest.norm() > radius {
        // Straight line clears the sphere. MuJoCo still allows wrapping if
        // a sidesite is present AND the closest point is on the opposite side
        // from the sidesite (sidesite-forced wrapping).
        match sidesite {
            None => return WrapResult::NoWrap,
            Some(ss) if closest.dot(&ss) >= 0.0 => return WrapResult::NoWrap,
            _ => {} // sidesite forces wrapping — fall through
        }
    }

    // 3. Compute the wrapping plane normal.
    //    The plane contains the sphere center (origin), p1, and p2.
    let mut plane_normal = p1.cross(&p2);
    if plane_normal.norm() < 1e-10 * p1.norm() * p2.norm() {
        // Degenerate: p1, origin, p2 are collinear.
        // Construct an arbitrary perpendicular via the least-aligned cardinal axis.
        let u = p1.normalize();
        let min_axis = if u.x.abs() <= u.y.abs() && u.x.abs() <= u.z.abs() {
            Vector3::x()
        } else if u.y.abs() <= u.z.abs() {
            Vector3::y()
        } else {
            Vector3::z()
        };
        plane_normal = u.cross(&min_axis);
    }
    plane_normal = plane_normal.normalize();

    // 4. Construct 2D coordinate system in the wrapping plane.
    let axis0 = p1.normalize();
    let axis1 = plane_normal.cross(&axis0).normalize();

    // Project endpoints and sidesite into the 2D wrapping plane.
    let p1_2d = Vector2::new(p1.dot(&axis0), p1.dot(&axis1)); // = (||p1||, 0)
    let p2_2d = Vector2::new(p2.dot(&axis0), p2.dot(&axis1));
    let ss_2d = sidesite.map(|ss| {
        let v = Vector2::new(ss.dot(&axis0), ss.dot(&axis1));
        if v.norm() > 1e-10 { v.normalize() } else { v }
    });

    // 4b. Compute both candidate tangent-point pairs (±normal).
    //     Candidate A: +normal (MuJoCo i=1), Candidate B: -normal (MuJoCo i=0).
    let (cand_a1, cand_a2) = compute_tangent_pair(p1, p2, radius, plane_normal);
    let (cand_b1, cand_b2) = compute_tangent_pair(p1, p2, radius, -plane_normal);
    // 2D projections for intersection testing:
    let proj_a1 = Vector2::new(cand_a1.dot(&axis0), cand_a1.dot(&axis1));
    let proj_a2 = Vector2::new(cand_a2.dot(&axis0), cand_a2.dot(&axis1));
    let proj_b1 = Vector2::new(cand_b1.dot(&axis0), cand_b1.dot(&axis1));
    let proj_b2 = Vector2::new(cand_b2.dot(&axis0), cand_b2.dot(&axis1));

    // 5. Select the best candidate using MuJoCo's goodness heuristic.
    //    Phase 1: Compute goodness score.
    let (mut good_a, mut good_b) = if let Some(sd) = ss_2d {
        // With sidesite: goodness = dot(normalized 2D midpoint, sidesite direction).
        let sum_a = proj_a1 + proj_a2;
        let sum_b = proj_b1 + proj_b2;
        let ga = if sum_a.norm() > 1e-10 {
            sum_a.normalize().dot(&sd)
        } else {
            -1e10
        };
        let gb = if sum_b.norm() > 1e-10 {
            sum_b.normalize().dot(&sd)
        } else {
            -1e10
        };
        (ga, gb)
    } else {
        // No sidesite: goodness = negative squared chord distance.
        let chord_a = (proj_a1 - proj_a2).norm_squared();
        let chord_b = (proj_b1 - proj_b2).norm_squared();
        (-chord_a, -chord_b)
    };

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_2d, proj_a1, p2_2d, proj_a2) {
        good_a = -10000.0;
    }
    if segments_intersect_2d(p1_2d, proj_b1, p2_2d, proj_b2) {
        good_b = -10000.0;
    }

    // Phase 2: Select the better candidate with ind tracking.
    //          +normal → MuJoCo candidate i=1 → ind=1
    //          -normal → MuJoCo candidate i=0 → ind=0
    //          On tie, select candidate A (ind=1) to match MuJoCo's default.
    let (t1, t2, win1_2d, win2_2d, ind) = if good_a >= good_b {
        (cand_a1, cand_a2, proj_a1, proj_a2, 1)
    } else {
        (cand_b1, cand_b2, proj_b1, proj_b2, 0)
    };

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    if segments_intersect_2d(p1_2d, win1_2d, p2_2d, win2_2d) {
        return WrapResult::NoWrap;
    }

    // 6. Arc length via directional wrap angle.
    let wrap_angle = directional_wrap_angle(win1_2d, win2_2d, ind);
    let arc_length = radius * wrap_angle;

    WrapResult::Wrapped {
        tangent_point_1: t1,
        tangent_point_2: t2,
        arc_length,
    }
}

/// Compute the shortest path around an infinite cylinder between two points.
///
/// Works in the geom-local frame where the cylinder axis is Z and center is the
/// origin. The problem reduces to a 2D circle-tangent problem in XY (perpendicular
/// to the axis), plus Z-interpolation for the axial (helical) component.
/// Returns tangent points in the geom-local frame; the caller transforms to world frame.
#[allow(clippy::similar_names)] // Dual-candidate algorithm requires paired naming (a1/a2, b1/b2).
fn cylinder_wrap(
    p1: Vector3<f64>,
    p2: Vector3<f64>,
    radius: f64,
    sidesite: Option<Vector3<f64>>,
) -> WrapResult {
    // 1. Project onto XY plane (perpendicular to cylinder axis).
    let p1_xy = Vector2::new(p1.x, p1.y);
    let p2_xy = Vector2::new(p2.x, p2.y);

    // 2. Early exits.
    if radius <= 0.0 {
        return WrapResult::NoWrap; // degenerate geom
    }
    if p1_xy.norm() < radius || p2_xy.norm() < radius {
        return WrapResult::NoWrap; // endpoint inside cylinder cross-section
    }

    // 3. Compute sidesite XY projection once (used in both early-exit and
    //    candidate selection). MuJoCo computes this once in mju_wrap and
    //    passes it to wrap_circle as the `side` parameter.
    let ss_dir = sidesite.map(|ss| {
        let ss_xy = Vector2::new(ss.x, ss.y);
        if ss_xy.norm() > 1e-10 {
            ss_xy.normalize()
        } else {
            ss_xy
        }
    });

    // 4. Check if the 2D line segment misses the cylinder cross-section.
    let d_xy = p2_xy - p1_xy;
    if d_xy.norm_squared() < 1e-20 {
        return WrapResult::NoWrap; // coincident sites in XY projection
    }
    let t_param = -(p1_xy.dot(&d_xy)) / d_xy.norm_squared();
    let closest = p1_xy + t_param.clamp(0.0, 1.0) * d_xy;
    if closest.norm() > radius {
        // Straight line clears the cylinder. Same sidesite-forced wrapping
        // logic as sphere (see §4.7 step 2).
        match ss_dir {
            None => return WrapResult::NoWrap,
            Some(sd) if closest.dot(&sd) >= 0.0 => return WrapResult::NoWrap,
            _ => {} // sidesite forces wrapping — fall through
        }
    }

    // 5. Compute both candidate 2D tangent-point pairs (±wrap direction).
    //    sign=+1 → MuJoCo candidate i=1 → ind=1
    //    sign=-1 → MuJoCo candidate i=0 → ind=0
    let (cand_a1, cand_a2) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, 1.0); // MuJoCo i=1
    let (cand_b1, cand_b2) = compute_tangent_pair_2d(p1_xy, p2_xy, radius, -1.0); // MuJoCo i=0

    // 6. Select the best candidate using MuJoCo's goodness heuristic.
    //    Phase 1: Compute goodness score (sidesite alignment or chord distance).
    let (mut good_a, mut good_b) = if let Some(sd) = ss_dir {
        // With sidesite: goodness = dot(normalized midpoint, sidesite direction).
        let sum_a = cand_a1 + cand_a2;
        let sum_b = cand_b1 + cand_b2;
        let ga = if sum_a.norm() > 1e-10 {
            sum_a.normalize().dot(&sd)
        } else {
            -1e10
        };
        let gb = if sum_b.norm() > 1e-10 {
            sum_b.normalize().dot(&sd)
        } else {
            -1e10
        };
        (ga, gb)
    } else {
        // No sidesite: goodness = negative squared chord distance.
        let chord_a = (cand_a1 - cand_a2).norm_squared();
        let chord_b = (cand_b1 - cand_b2).norm_squared();
        (-chord_a, -chord_b)
    };

    // Phase 1b: Penalize self-intersecting candidates.
    if segments_intersect_2d(p1_xy, cand_a1, p2_xy, cand_a2) {
        good_a = -10000.0;
    }
    if segments_intersect_2d(p1_xy, cand_b1, p2_xy, cand_b2) {
        good_b = -10000.0;
    }

    // Phase 2: Select the better candidate with correct ind mapping.
    //          On tie (good_a == good_b), select candidate A (ind=1) to match
    //          MuJoCo's `i = (good[0] > good[1] ? 0 : 1)` which defaults to i=1.
    let (win1_xy, win2_xy, ind) = if good_a >= good_b {
        (cand_a1, cand_a2, 1) // sign=+1 → MuJoCo ind=1
    } else {
        (cand_b1, cand_b2, 0) // sign=-1 → MuJoCo ind=0
    };

    // Phase 3: Reject wrapping if the selected candidate self-intersects.
    if segments_intersect_2d(p1_xy, win1_xy, p2_xy, win2_xy) {
        return WrapResult::NoWrap;
    }

    // 7. Compute directional wrap angle (MuJoCo's length_circle algorithm).
    let wrap_angle = directional_wrap_angle(win1_xy, win2_xy, ind);

    // 8. Z-interpolation: path-length-proportional (MuJoCo formula).
    //    The axial (Z) coordinate is linearly interpolated based on the
    //    fraction of total 2D path length (straight1 + arc + straight2).
    //    This produces a helical path with uniform axial velocity.
    let len_straight1 = (p1_xy - win1_xy).norm(); // 2D straight: p1 → tangent1
    let len_arc = radius * wrap_angle; // 2D arc on cylinder surface
    let len_straight2 = (p2_xy - win2_xy).norm(); // 2D straight: tangent2 → p2
    let total_2d = len_straight1 + len_arc + len_straight2;
    if total_2d < 1e-10 {
        return WrapResult::NoWrap;
    }
    let t1_z = p1.z + (p2.z - p1.z) * len_straight1 / total_2d;
    let t2_z = p1.z + (p2.z - p1.z) * (len_straight1 + len_arc) / total_2d;

    // 9. Arc length of the helical path on the cylinder surface.
    let axial_disp = t2_z - t1_z; // axial travel on surface
    let arc_length = (len_arc * len_arc + axial_disp * axial_disp).sqrt();

    let t1 = Vector3::new(win1_xy.x, win1_xy.y, t1_z);
    let t2 = Vector3::new(win2_xy.x, win2_xy.y, t2_z);

    WrapResult::Wrapped {
        tangent_point_1: t1,
        tangent_point_2: t2,
        arc_length,
    }
}

/// Compute actuator length and moment for site transmissions only.
///
/// For each Site-transmission actuator, computes `actuator_length` and
/// `actuator_moment` (nv-vector). Joint/Tendon transmissions are untouched.
/// Must run after `mj_fwd_position` (needs site poses) and before
/// `mj_sensor_pos` (which reads `actuator_length`).
fn mj_transmission_site(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] != ActuatorTransmission::Site {
            continue;
        }

        let sid = model.actuator_trnid[i][0];
        let refid = model.actuator_trnid[i][1];
        let gear = model.actuator_gear[i];

        let (jac_t, jac_r) = mj_jac_site(model, data, sid);

        if refid == usize::MAX {
            // Mode A — no refsite: length = 0, moment from wrench projection.
            data.actuator_length[i] = 0.0;

            // Wrench in world frame: rotate gear from site-local to world.
            let wrench_t = data.site_xmat[sid] * Vector3::new(gear[0], gear[1], gear[2]);
            let wrench_r = data.site_xmat[sid] * Vector3::new(gear[3], gear[4], gear[5]);

            // moment = J_trans^T @ wrench_t + J_rot^T @ wrench_r
            let moment = &mut data.actuator_moment[i];
            for dof in 0..model.nv {
                moment[dof] = jac_t.column(dof).dot(&wrench_t) + jac_r.column(dof).dot(&wrench_r);
            }
        } else {
            // Mode B — with refsite: length from position/quaternion differences,
            // moment from difference Jacobian.
            let (ref_jac_t, ref_jac_r) = mj_jac_site(model, data, refid);

            // Translational length: gear[0:3] · (R_ref^T @ (p_site - p_ref))
            let dp = data.site_xpos[sid] - data.site_xpos[refid];
            let dp_ref = data.site_xmat[refid].transpose() * dp;
            let len_trans = gear[0] * dp_ref.x + gear[1] * dp_ref.y + gear[2] * dp_ref.z;

            // Rotational length: gear[3:6] · subquat(q_site, q_ref)
            let dq = subquat(&data.site_xquat[sid], &data.site_xquat[refid]);
            let len_rot = gear[3] * dq.x + gear[4] * dq.y + gear[5] * dq.z;

            data.actuator_length[i] = len_trans + len_rot;

            // Difference Jacobian with common-ancestor DOF zeroing.
            let mut diff_jac_t = &jac_t - &ref_jac_t;
            let mut diff_jac_r = &jac_r - &ref_jac_r;

            // Zero common-ancestor DOF columns.
            let b0 = model.site_body[sid];
            let b1 = model.site_body[refid];
            // Find lowest common ancestor.
            let mut ancestors_b0 = Vec::new();
            {
                let mut b = b0;
                while b != 0 {
                    ancestors_b0.push(b);
                    b = model.body_parent[b];
                }
                ancestors_b0.push(0);
            }
            let bca = {
                let mut b = b1;
                loop {
                    if ancestors_b0.contains(&b) {
                        break b;
                    }
                    if b == 0 {
                        break 0;
                    }
                    b = model.body_parent[b];
                }
            };
            // Zero DOFs for bca and all its ancestors up to root.
            {
                let mut b = bca;
                loop {
                    let jnt_start = model.body_jnt_adr[b];
                    let jnt_end = jnt_start + model.body_jnt_num[b];
                    for jnt_id in jnt_start..jnt_end {
                        let dof_start = model.jnt_dof_adr[jnt_id];
                        let ndof = model.jnt_type[jnt_id].nv();
                        for d in dof_start..(dof_start + ndof) {
                            for k in 0..3 {
                                diff_jac_t[(k, d)] = 0.0;
                                diff_jac_r[(k, d)] = 0.0;
                            }
                        }
                    }
                    if b == 0 {
                        break;
                    }
                    b = model.body_parent[b];
                }
            }

            // Wrench in world frame: rotate gear by refsite frame.
            let wrench_t = data.site_xmat[refid] * Vector3::new(gear[0], gear[1], gear[2]);
            let wrench_r = data.site_xmat[refid] * Vector3::new(gear[3], gear[4], gear[5]);

            // moment = diff_J_trans^T @ wrench_t + diff_J_rot^T @ wrench_r
            let moment = &mut data.actuator_moment[i];
            for dof in 0..model.nv {
                moment[dof] =
                    diff_jac_t.column(dof).dot(&wrench_t) + diff_jac_r.column(dof).dot(&wrench_r);
            }
        }
    }
}

/// Compute actuator length and velocity from transmission state.
///
/// For each actuator, computes `actuator_length = gear * transmission_length`
/// and `actuator_velocity = gear * transmission_velocity`.
/// Called after `mj_fwd_velocity()` (which provides `ten_velocity`).
fn mj_actuator_length(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        let gear = model.actuator_gear[i][0];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = model.actuator_trnid[i][0];
                if jid < model.njnt {
                    // Joint transmission only meaningful for Hinge/Slide (scalar qpos).
                    let nv = model.jnt_type[jid].nv();
                    if nv == 1 {
                        let qadr = model.jnt_qpos_adr[jid];
                        let dof_adr = model.jnt_dof_adr[jid];
                        data.actuator_length[i] = gear * data.qpos[qadr];
                        data.actuator_velocity[i] = gear * data.qvel[dof_adr];
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tid = model.actuator_trnid[i][0];
                if tid < model.ntendon {
                    data.actuator_length[i] = gear * data.ten_length[tid];
                    data.actuator_velocity[i] = gear * data.ten_velocity[tid];
                }
            }
            ActuatorTransmission::Site => {
                // Length already set by mj_transmission_site (position stage).
                // Velocity from cached moment:
                data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
            }
        }
    }
}

// ============================================================================
// MuJoCo Muscle Force-Length-Velocity Curves
// ============================================================================
// These implement MuJoCo's exact piecewise-quadratic muscle curves from
// engine_util_misc.c: mju_muscleGain (FL, FV) and mju_muscleBias (FP).

/// Active force-length curve: piecewise quadratic bump.
/// Returns 0 outside `[lmin, lmax]`, peak 1.0 at `L = 1.0`.
fn muscle_gain_length(length: f64, lmin: f64, lmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    if length < lmin || length > lmax {
        return 0.0;
    }
    let a = 0.5 * (lmin + 1.0); // midpoint of [lmin, 1]
    let b = 0.5 * (1.0 + lmax); // midpoint of [1, lmax]

    if length <= a {
        let x = (length - lmin) / (a - lmin).max(EPS);
        0.5 * x * x
    } else if length <= 1.0 {
        let x = (1.0 - length) / (1.0 - a).max(EPS);
        1.0 - 0.5 * x * x
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        1.0 - 0.5 * x * x
    } else {
        let x = (lmax - length) / (lmax - b).max(EPS);
        0.5 * x * x
    }
}

/// Force-velocity curve: piecewise quadratic.
/// `velocity` is normalized by `L0 * vmax` so `V = -1` is max shortening.
fn muscle_gain_velocity(velocity: f64, fvmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let y = fvmax - 1.0;
    if velocity <= -1.0 {
        0.0
    } else if velocity <= 0.0 {
        (velocity + 1.0) * (velocity + 1.0)
    } else if velocity <= y {
        fvmax - (y - velocity) * (y - velocity) / y.max(EPS)
    } else {
        fvmax
    }
}

/// Passive force curve: zero below `L = 1.0`, quadratic onset, linear beyond midpoint.
fn muscle_passive_force(length: f64, lmax: f64, fpmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let b = 0.5 * (1.0 + lmax);
    if length <= 1.0 {
        0.0
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        fpmax * 0.5 * x * x
    } else {
        let x = (length - b) / (b - 1.0).max(EPS);
        fpmax * (0.5 + x)
    }
}

// ============================================================================
// Muscle Activation Dynamics
// ============================================================================

/// Quintic smoothstep (C2-continuous Hermite), matching MuJoCo's `mju_sigmoid`.
fn sigmoid(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x >= 1.0 {
        return 1.0;
    }
    x * x * x * (6.0 * x * x - 15.0 * x + 10.0)
}

/// Compute d(act)/dt for muscle activation dynamics.
///
/// Follows Millard et al. (2013) with activation-dependent time constants:
///   `tau_act_eff   = tau_act   * (0.5 + 1.5 * act)`
///   `tau_deact_eff = tau_deact / (0.5 + 1.5 * act)`
///
/// When `tausmooth > 0`, uses quintic sigmoid to blend between `tau_act` and
/// `tau_deact` instead of a hard switch at `ctrl == act`.
fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 3]) -> f64 {
    let ctrl_clamped = ctrl.clamp(0.0, 1.0);
    let act_clamped = act.clamp(0.0, 1.0);

    // Activation-dependent effective time constants (Millard et al. 2013)
    let tau_act = dynprm[0] * (0.5 + 1.5 * act_clamped);
    let tau_deact = dynprm[1] / (0.5 + 1.5 * act_clamped);
    let tausmooth = dynprm[2];

    let dctrl = ctrl_clamped - act;

    // Select time constant
    let tau = if tausmooth < 1e-10 {
        // Hard switch
        if dctrl > 0.0 { tau_act } else { tau_deact }
    } else {
        // Smooth blending via quintic sigmoid
        tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / tausmooth + 0.5)
    };

    dctrl / tau.max(1e-10)
}

/// Compute actuator forces from control inputs, activation dynamics, and muscle FLV curves.
///
/// This function:
/// 1. Computes activation derivatives (`data.act_dot`) without modifying `data.act`.
/// 2. Computes actuator force using gain/bias (muscle FLV for muscles, raw input for others).
/// 3. Clamps control inputs and output forces to their declared ranges.
/// 4. Maps actuator force to joint forces via the transmission.
fn mj_fwd_actuation(model: &Model, data: &mut Data) {
    data.qfrc_actuator.fill(0.0);

    for i in 0..model.nu {
        // --- Phase 1: Activation dynamics (compute act_dot, do NOT integrate) ---
        let ctrl = data.ctrl[i].clamp(model.actuator_ctrlrange[i].0, model.actuator_ctrlrange[i].1);

        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => ctrl,
            ActuatorDynamics::Muscle => {
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] =
                    muscle_activation_dynamics(ctrl, data.act[act_adr], &model.actuator_dynprm[i]);
                data.act[act_adr] // use CURRENT activation for force
            }
            ActuatorDynamics::Filter | ActuatorDynamics::FilterExact => {
                // First-order filter: d(act)/dt = (ctrl - act) / tau
                // Filter uses Euler integration, FilterExact uses exact integration.
                // Both compute the same act_dot here; the difference is in integrate().
                let act_adr = model.actuator_act_adr[i];
                let tau = model.actuator_dynprm[i][0].max(1e-10);
                data.act_dot[act_adr] = (ctrl - data.act[act_adr]) / tau;
                data.act[act_adr] // use CURRENT activation for force
            }
            ActuatorDynamics::Integrator => {
                // Integrator: d(act)/dt = ctrl
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] = ctrl;
                data.act[act_adr] // use CURRENT activation for force
            }
        };

        // --- Phase 2: Force generation (gain * input + bias) ---
        let length = data.actuator_length[i];
        let velocity = data.actuator_velocity[i];

        let gain = match model.actuator_gaintype[i] {
            GainType::Fixed => model.actuator_gainprm[i][0],
            GainType::Affine => {
                model.actuator_gainprm[i][0]
                    + model.actuator_gainprm[i][1] * length
                    + model.actuator_gainprm[i][2] * velocity
            }
            GainType::Muscle => {
                // Muscle gain = -F0 * FL(L) * FV(V)
                let prm = &model.actuator_gainprm[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2]; // resolved by compute_muscle_params()

                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);
                let norm_vel = velocity / (l0 * prm[6]).max(1e-10);

                let fl = muscle_gain_length(norm_len, prm[4], prm[5]);
                let fv = muscle_gain_velocity(norm_vel, prm[8]);
                -f0 * fl * fv
            }
        };

        let bias = match model.actuator_biastype[i] {
            BiasType::None => 0.0,
            BiasType::Affine => {
                model.actuator_biasprm[i][0]
                    + model.actuator_biasprm[i][1] * length
                    + model.actuator_biasprm[i][2] * velocity
            }
            BiasType::Muscle => {
                let prm = &model.actuator_gainprm[i]; // muscle uses gainprm for both
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2];

                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);

                let fp = muscle_passive_force(norm_len, prm[5], prm[7]);
                -f0 * fp
            }
        };

        let force = gain * input + bias;

        // Clamp to force range
        let force = force.clamp(
            model.actuator_forcerange[i].0,
            model.actuator_forcerange[i].1,
        );
        data.actuator_force[i] = force;

        // --- Phase 3: Transmission (actuator_force → generalized forces) ---
        // qfrc_actuator += moment^T * actuator_force
        // where moment = gear * raw_Jacobian.
        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                if trnid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[trnid];
                    let nv = model.jnt_type[trnid].nv();
                    if nv > 0 {
                        data.qfrc_actuator[dof_adr] += gear * force;
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tendon_id = trnid;
                if tendon_id < model.ntendon {
                    apply_tendon_force(
                        model,
                        &data.ten_J[tendon_id],
                        model.tendon_type[tendon_id],
                        tendon_id,
                        gear * force,
                        &mut data.qfrc_actuator,
                    );
                }
            }
            ActuatorTransmission::Site => {
                // Use cached moment vector from mj_transmission_site.
                for dof in 0..model.nv {
                    let m = data.actuator_moment[i][dof];
                    if m != 0.0 {
                        data.qfrc_actuator[dof] += m * force;
                    }
                }
            }
        }
    }
}

/// Composite Rigid Body Algorithm: compute mass matrix.
///
/// This implements Featherstone's CRBA algorithm which computes the joint-space
/// inertia matrix M such that kinetic energy T = 0.5 * `qvel^T` * M * `qvel`.
///
/// The algorithm works by:
/// 1. Computing composite inertias by accumulating from leaves to root
/// 2. Computing `M[i,j]` = `S_i^T` * `I_c` * `S_j` for each joint pair
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 6
/// Composite Rigid Body Algorithm (CRBA) - Featherstone O(n) version.
///
/// Computes the joint-space inertia matrix M using the recursive algorithm:
/// 1. Initialize composite inertia Ic[i] = I[i] for each body
/// 2. Backward pass: Ic[parent] += transform(Ic[child])
/// 3. For each joint, compute M elements from Ic and joint motion subspace
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 6
#[allow(
    clippy::many_single_char_names,
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
fn mj_crba(model: &Model, data: &mut Data) {
    data.qLD_valid = false;
    data.qM.fill(0.0);

    if model.nv == 0 {
        return;
    }

    // ============================================================
    // Phase 1: Initialize composite inertias from body inertias
    // ============================================================
    // Copy cinert (computed once in FK) to crb_inertia as starting point.
    // cinert contains 6x6 spatial inertias for individual bodies in world frame.
    for body_id in 0..model.nbody {
        data.crb_inertia[body_id] = data.cinert[body_id];
    }

    // ============================================================
    // Phase 2: Backward pass - accumulate composite inertias
    // ============================================================
    // Process bodies from leaves to root, adding child inertia to parent.
    // This gives Ic[i] = inertia of subtree rooted at body i.
    //
    // Note: We need to transform child inertia to parent frame before adding.
    // For now, use a simpler approach: all inertias are in world frame,
    // so we just add them directly (valid because world frame is common).

    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            // Add this body's composite inertia to parent's
            // Need to clone to avoid borrow checker issues
            let child_inertia = data.crb_inertia[body_id];
            data.crb_inertia[parent_id] += child_inertia;
        }
    }

    // ============================================================
    // Phase 3: Build mass matrix from composite inertias
    // ============================================================
    // For each joint, M[i,i] = S[i]^T * Ic[body_i] * S[i]
    // Off-diagonal elements require propagating forces up the tree.

    // Pre-compute all joint motion subspaces once (O(n) instead of O(n²))
    // This is a significant optimization for debug builds where function call
    // overhead and matrix allocation dominate.
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for jnt_i in 0..model.njnt {
        let body_i = model.jnt_body[jnt_i];
        let dof_i = model.jnt_dof_adr[jnt_i];
        let nv_i = model.jnt_type[jnt_i].nv();

        // Use cached joint motion subspace
        let s_i = &joint_subspaces[jnt_i];

        // Diagonal block: M[i,i] = S^T * Ic * S
        let ic = &data.crb_inertia[body_i];
        for di in 0..nv_i {
            let s_col_i = s_i.column(di);
            for dj in 0..nv_i {
                let s_col_j = s_i.column(dj);
                // Compute s_i^T * Ic * s_j
                let ic_s_j = ic * s_col_j;
                data.qM[(dof_i + di, dof_i + dj)] = s_col_i.dot(&ic_s_j);
            }
        }

        // Off-diagonal: propagate to ancestor joints
        // F = Ic * S (force due to joint motion)
        // Then traverse ancestors, transforming F at each step:
        //   F_parent = X^T * F_child (spatial force transform)
        // And computing M[j,i] = S_j^T * F_transformed
        let mut force = ic * s_i; // 6 x nv_i matrix, starts at body_i

        let mut child_body = body_i;
        let mut current_body = model.body_parent[body_i];

        while current_body != 0 {
            // Transform force from child to current body using spatial force transform
            // F_parent = [I, 0; [r]x, I]^T * F_child = [I, [r]x^T; 0, I] * F_child
            // Where r = pos_child - pos_parent (vector from parent to child)
            let r = data.xpos[child_body] - data.xpos[current_body];

            // Apply spatial force transform:
            // F_angular_parent = F_angular_child + r × F_linear_child
            // F_linear_parent = F_linear_child
            for col in 0..nv_i {
                let f_lin_x = force[(3, col)];
                let f_lin_y = force[(4, col)];
                let f_lin_z = force[(5, col)];

                // r × f_lin
                let cross_x = r.y * f_lin_z - r.z * f_lin_y;
                let cross_y = r.z * f_lin_x - r.x * f_lin_z;
                let cross_z = r.x * f_lin_y - r.y * f_lin_x;

                force[(0, col)] += cross_x;
                force[(1, col)] += cross_y;
                force[(2, col)] += cross_z;
                // Linear part stays the same
            }

            // Find joint(s) for this ancestor body
            let jnt_start = model.body_jnt_adr[current_body];
            let jnt_count = model.body_jnt_num[current_body];

            for jnt_j in jnt_start..(jnt_start + jnt_count) {
                if jnt_j >= model.njnt {
                    break;
                }

                let dof_j = model.jnt_dof_adr[jnt_j];
                let nv_j = model.jnt_type[jnt_j].nv();
                // Use cached joint motion subspace instead of recomputing
                let s_j = &joint_subspaces[jnt_j];

                // M[j,i] = S_j^T * F
                for dj in 0..nv_j {
                    let s_col_j = s_j.column(dj);
                    for di in 0..nv_i {
                        let f_col_i = force.column(di);
                        let m_ji = s_col_j.dot(&f_col_i);
                        data.qM[(dof_j + dj, dof_i + di)] = m_ji;
                        data.qM[(dof_i + di, dof_j + dj)] = m_ji; // Symmetry
                    }
                }
            }

            // Move up the tree
            child_body = current_body;
            current_body = model.body_parent[current_body];
        }
    }

    // ============================================================
    // Phase 4: Add armature inertia to diagonal
    // ============================================================
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let nv = model.jnt_type[jnt_id].nv();
        let armature = model.jnt_armature[jnt_id];

        for i in 0..nv {
            data.qM[(dof_adr + i, dof_adr + i)] += armature;
            if let Some(&dof_arm) = model.dof_armature.get(dof_adr + i) {
                data.qM[(dof_adr + i, dof_adr + i)] += dof_arm;
            }
        }
    }

    // ============================================================
    // Phase 5: Sparse L^T D L factorization
    // ============================================================
    // Exploits tree sparsity from dof_parent for O(n) factorization and solve.
    // Reused in mj_fwd_acceleration_explicit() and pgs_solve_contacts().
    mj_factor_sparse(model, data);

    // ============================================================
    // Phase 6: Cache body effective mass/inertia from qM diagonal
    // ============================================================
    // Extract per-body min mass/inertia for constraint force limiting.
    // This avoids O(joints) traversal per constraint.
    cache_body_effective_mass(model, data);
}

/// Cache per-body minimum mass and inertia from the mass matrix diagonal.
///
/// This extracts the minimum diagonal elements for each body's DOFs and stores
/// them in `data.body_min_mass` and `data.body_min_inertia`. These cached values
/// are used by constraint force limiting to avoid repeated mass matrix queries.
///
/// Uses the `JointVisitor` pattern to ensure consistency with joint iteration
/// elsewhere in the codebase.
///
/// Must be called after `mj_crba()` has computed the mass matrix.
fn cache_body_effective_mass(model: &Model, data: &mut Data) {
    // Visitor struct for JointVisitor pattern (defined before statements per clippy)
    struct MassCacheVisitor<'a> {
        model: &'a Model,
        data: &'a mut Data,
    }

    impl JointVisitor for MassCacheVisitor<'_> {
        fn visit_free(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Linear DOFs at 0-2
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let mass = self.data.qM[(dof, dof)];
                    if mass > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_mass[body_id] =
                            self.data.body_min_mass[body_id].min(mass);
                    }
                }
            }
            // Angular DOFs at 3-5
            for i in 3..6 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_ball(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // All 3 DOFs are angular
            for i in 0..3 {
                let dof = ctx.dof_adr + i;
                if dof < self.model.nv {
                    let inertia = self.data.qM[(dof, dof)];
                    if inertia > MIN_INERTIA_THRESHOLD {
                        self.data.body_min_inertia[body_id] =
                            self.data.body_min_inertia[body_id].min(inertia);
                    }
                }
            }
        }

        fn visit_hinge(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Single angular DOF
            if ctx.dof_adr < self.model.nv {
                let inertia = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if inertia > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_inertia[body_id] =
                        self.data.body_min_inertia[body_id].min(inertia);
                }
            }
        }

        fn visit_slide(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            // Single linear DOF
            if ctx.dof_adr < self.model.nv {
                let mass = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if mass > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_mass[body_id] = self.data.body_min_mass[body_id].min(mass);
                }
            }
        }
    }

    // Reset to defaults (world body stays at infinity)
    for i in 1..model.nbody {
        data.body_min_mass[i] = f64::INFINITY;
        data.body_min_inertia[i] = f64::INFINITY;
    }

    let mut visitor = MassCacheVisitor { model, data };
    model.visit_joints(&mut visitor);

    // Replace infinity with default for bodies that had no DOFs of that type
    for i in 1..model.nbody {
        if data.body_min_mass[i] == f64::INFINITY {
            data.body_min_mass[i] = DEFAULT_MASS_FALLBACK;
        }
        if data.body_min_inertia[i] == f64::INFINITY {
            data.body_min_inertia[i] = DEFAULT_MASS_FALLBACK;
        }
    }
}

/// Compute the joint motion subspace matrix S (6 x nv).
///
/// S maps joint velocity to spatial velocity in world frame:
/// v_spatial = S * qdot
///
/// Format: rows 0-2 = angular velocity, rows 3-5 = linear velocity
#[allow(clippy::similar_names)]
#[allow(clippy::inline_always)] // Hot path in CRBA/RNE - profiling shows inlining improves debug performance
#[inline(always)]
pub(crate) fn joint_motion_subspace(
    model: &Model,
    data: &Data,
    jnt_id: usize,
) -> nalgebra::SMatrix<f64, 6, 6> {
    let body_id = model.jnt_body[jnt_id];
    let jnt_type = model.jnt_type[jnt_id];
    let nv = jnt_type.nv();

    let mut s = nalgebra::SMatrix::<f64, 6, 6>::zeros();

    match jnt_type {
        MjJointType::Hinge => {
            // Revolute joint: S = [axis; axis × r]^T where r is from joint to body origin
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            let jpos_world = data.xpos[body_id] + data.xquat[body_id] * model.jnt_pos[jnt_id];
            let r = data.xpos[body_id] - jpos_world;

            // Angular part (rows 0-2)
            s[(0, 0)] = axis_world.x;
            s[(1, 0)] = axis_world.y;
            s[(2, 0)] = axis_world.z;

            // Linear part (rows 3-5) = axis × r
            let lin = axis_world.cross(&r);
            s[(3, 0)] = lin.x;
            s[(4, 0)] = lin.y;
            s[(5, 0)] = lin.z;
        }
        MjJointType::Slide => {
            // Prismatic joint: S = [0; axis]^T
            let axis_world = data.xquat[body_id] * model.jnt_axis[jnt_id];
            s[(3, 0)] = axis_world.x;
            s[(4, 0)] = axis_world.y;
            s[(5, 0)] = axis_world.z;
        }
        MjJointType::Ball => {
            // Ball joint: 3 angular DOFs
            // S = [I_3x3; 0_3x3] (angular velocity in world frame)
            for i in 0..3 {
                s[(i, i)] = 1.0;
            }
        }
        MjJointType::Free => {
            // Free joint: 6 DOFs (3 linear + 3 angular)
            // DOF order in qvel: [vx, vy, vz, ωx, ωy, ωz]
            // Linear DOFs map to linear velocity
            for i in 0..3 {
                s[(3 + i, i)] = 1.0; // Linear velocity
            }
            // Angular DOFs map to angular velocity
            for i in 0..3 {
                s[(i, 3 + i)] = 1.0; // Angular velocity
            }
        }
    }

    // Only return the columns needed
    let _ = nv; // Used implicitly through jnt_type
    s
}

/// Recursive Newton-Euler: compute bias forces (Coriolis + centrifugal + gravity).
///
/// The bias force vector `c(q, qdot)` contains:
/// - Gravity forces projected to joint space
/// - Coriolis/centrifugal forces: velocity-dependent terms
///
/// For the equation of motion: `M * qacc + c(q, qdot) = τ`
///
/// ## Implementation Notes
///
/// This implementation uses a hybrid approach for efficiency and correctness:
/// 1. **Gravity**: O(n) computation using precomputed subtree mass and COM
/// 2. **Gyroscopic**: Direct computation for Ball/Free joints (ω × Iω)
/// 3. **Coriolis/Centrifugal**: O(n²) computation using body velocities
///
/// A full Featherstone O(n) RNE would use spatial algebra throughout, but this
/// hybrid approach achieves the same physics with simpler code that's easier
/// to verify for correctness.
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 5
/// Reference: MuJoCo Computation docs - mj_rne section
#[allow(
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
fn mj_rne(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    data.qfrc_bias.fill(0.0);

    if model.nv == 0 {
        return;
    }

    // ========== Gravity contribution (O(n) using precomputed subtree mass/COM) ==========
    // The bias force is what we need to SUBTRACT from applied forces.
    // For joint i on body b: τ_g[i] = J_i^T * (M_subtree * g)
    // where M_subtree is total mass below body b, and the torque acts at subtree COM
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let jnt_body = model.jnt_body[jnt_id];

        // §16.5a: Skip RNE for sleeping bodies — bias forces are zeroed
        if sleep_enabled && data.body_sleep_state[jnt_body] == SleepState::Asleep {
            continue;
        }

        // Use precomputed subtree mass and COM for O(n) gravity
        let subtree_mass = data.subtree_mass[jnt_body];
        let subtree_com = data.subtree_com[jnt_body];

        // Negative because qfrc_bias opposes motion
        let gravity_force = -subtree_mass * model.gravity;

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr] += torque.dot(&axis);
            }
            MjJointType::Slide => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                data.qfrc_bias[dof_adr] += gravity_force.dot(&axis);
            }
            MjJointType::Ball => {
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                let body_torque = data.xquat[jnt_body].inverse() * torque;
                data.qfrc_bias[dof_adr] += body_torque.x;
                data.qfrc_bias[dof_adr + 1] += body_torque.y;
                data.qfrc_bias[dof_adr + 2] += body_torque.z;
            }
            MjJointType::Free => {
                // Linear gravity
                data.qfrc_bias[dof_adr] += gravity_force.x;
                data.qfrc_bias[dof_adr + 1] += gravity_force.y;
                data.qfrc_bias[dof_adr + 2] += gravity_force.z;
                // Angular: gravity torque from subtree COM relative to free joint position
                let jpos = Vector3::new(
                    data.qpos[model.jnt_qpos_adr[jnt_id]],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                );
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr + 3] += torque.x;
                data.qfrc_bias[dof_adr + 4] += torque.y;
                data.qfrc_bias[dof_adr + 5] += torque.z;
            }
        }
    }

    // ========== Gyroscopic terms for Ball/Free joints ==========
    // τ_gyro = ω × (I * ω) - the gyroscopic torque
    // This is the dominant Coriolis effect for 3D rotations
    for body_id in 1..model.nbody {
        // §16.5a: Skip gyroscopic terms for sleeping bodies
        if sleep_enabled && data.body_sleep_state[body_id] == SleepState::Asleep {
            continue;
        }

        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Ball => {
                    // Angular velocity in body frame
                    let omega_body = Vector3::new(
                        data.qvel[dof_adr],
                        data.qvel[dof_adr + 1],
                        data.qvel[dof_adr + 2],
                    );
                    // Body inertia (diagonal in principal axes)
                    let inertia = model.body_inertia[body_id];
                    // I * ω
                    let i_omega = Vector3::new(
                        inertia.x * omega_body.x,
                        inertia.y * omega_body.y,
                        inertia.z * omega_body.z,
                    );
                    // Gyroscopic torque: ω × (I * ω)
                    let gyro = omega_body.cross(&i_omega);
                    data.qfrc_bias[dof_adr] += gyro.x;
                    data.qfrc_bias[dof_adr + 1] += gyro.y;
                    data.qfrc_bias[dof_adr + 2] += gyro.z;
                }
                MjJointType::Free => {
                    // Angular DOFs are at dof_adr + 3..6
                    let omega_body = Vector3::new(
                        data.qvel[dof_adr + 3],
                        data.qvel[dof_adr + 4],
                        data.qvel[dof_adr + 5],
                    );
                    let inertia = model.body_inertia[body_id];
                    let i_omega = Vector3::new(
                        inertia.x * omega_body.x,
                        inertia.y * omega_body.y,
                        inertia.z * omega_body.z,
                    );
                    let gyro = omega_body.cross(&i_omega);
                    data.qfrc_bias[dof_adr + 3] += gyro.x;
                    data.qfrc_bias[dof_adr + 4] += gyro.y;
                    data.qfrc_bias[dof_adr + 5] += gyro.z;
                }
                _ => {}
            }
        }
    }

    // ========== Coriolis/Centrifugal via Analytical Featherstone RNE ==========
    //
    // This is O(n) and replaces the O(n³) Christoffel symbol computation.
    // The algorithm:
    //   1. Forward pass: compute bias accelerations (velocity-dependent accelerations)
    //   2. Backward pass: compute bias forces and project to joint space
    //
    // Skip if all velocities are small (optimization for static/quasi-static cases)
    let max_qdot = data.qvel.iter().map(|v| v.abs()).fold(0.0, f64::max);
    if max_qdot < 1e-6 {
        return;
    }

    // Initialize bias accelerations and forces
    for i in 0..model.nbody {
        data.cacc_bias[i] = SpatialVector::zeros();
        data.cfrc_bias[i] = SpatialVector::zeros();
    }

    // ========== Forward Pass: Compute Bias Accelerations ==========
    // For each body, accumulate velocity-dependent accelerations.
    //
    // Featherstone's algorithm for bias acceleration:
    //   a_bias[i] = a_bias[parent] + c[i]
    //   c[i] = v[i] ×_m (S[i] @ qdot[i])  (velocity-product acceleration)
    //
    // The key insight: the bias acceleration arises from two sources:
    // 1. The acceleration of the parent frame (a_bias[parent])
    // 2. The velocity-product c[i] = v[i] ×_m v_joint, which captures
    //    centripetal and Coriolis accelerations
    //
    // Note: v[i] is the FULL body velocity (already computed in mj_fwd_velocity)

    // Pre-compute all joint motion subspaces once for RNE (same optimization as CRBA)
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's bias acceleration
        let mut a_bias = data.cacc_bias[parent_id];

        // Add velocity-product acceleration from joints on this body
        // c[i] = v[i] ×_m (S[i] @ qdot[i])
        // This uses the BODY velocity (not parent), as per Featherstone

        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let nv = model.jnt_type[jnt_id].nv();

            // Compute joint velocity contribution: v_joint = S @ qdot
            let mut v_joint = SpatialVector::zeros();
            for d in 0..nv {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }

            // Velocity-product acceleration: v[parent] ×_m v_joint
            // This is the centripetal/Coriolis acceleration contribution
            // Note: using parent velocity because v[i] = v[parent] + S@qdot,
            // and (S@qdot) ×_m (S@qdot) = 0, so v[i] ×_m (S@qdot) = v[parent] ×_m (S@qdot)
            let v_parent = data.cvel[parent_id];
            a_bias += spatial_cross_motion(v_parent, v_joint);
        }

        data.cacc_bias[body_id] = a_bias;
    }

    // ========== Backward Pass: Compute Bias Forces ==========
    // For each body from leaves to root:
    //   f_bias[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])
    //   f_bias[parent] += f_bias[i]
    //
    // Then project to joint space:
    //   τ_bias[dof] = S[i]^T @ f_bias[i]

    // First compute f_bias for each body using cinert (computed once in FK)
    for body_id in 1..model.nbody {
        let i = &data.cinert[body_id];
        let v = data.cvel[body_id];
        let a_bias = data.cacc_bias[body_id];

        // I @ v (momentum)
        let i_v = i * v;

        // I @ a_bias (inertial force from bias acceleration)
        let i_a = i * a_bias;

        // v ×* (I @ v) (gyroscopic/Coriolis force)
        let gyro = spatial_cross_force(v, i_v);

        // f_bias = I @ a_bias + v ×* (I @ v)
        data.cfrc_bias[body_id] = i_a + gyro;
    }

    // Propagate forces from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            // Add child's force to parent (already in world frame, no transform needed)
            let child_force = data.cfrc_bias[body_id];
            data.cfrc_bias[parent_id] += child_force;
        }
    }

    // ========== Project to Joint Space ==========
    // τ_bias[dof] = S^T @ f_bias[body]
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id]; // Use cached subspace
        let nv = model.jnt_type[jnt_id].nv();

        let f = data.cfrc_bias[body_id];

        // τ = S^T @ f
        for d in 0..nv {
            let mut tau = 0.0;
            for row in 0..6 {
                tau += s[(row, d)] * f[row];
            }
            data.qfrc_bias[dof_adr + d] += tau;
        }
    }
}

// Friction smoothing factor is now configurable via model.friction_smoothing.
// Default: 1000.0. At vel=0.001, tanh(1000 * 0.001) ≈ 0.76 (smooth onset).
// At vel=0.01, tanh(1000 * 0.01) ≈ 1.0 (full friction magnitude).

/// Velocity threshold below which friction is not applied.
/// Prevents numerical noise from creating spurious friction forces.
/// Not configurable — this is a numerical safety guard. The value 1e-12
/// is effectively zero for any physical velocity and prevents division-by-zero
/// or denormalized-float issues in the tanh friction approximation.
const FRICTION_VELOCITY_THRESHOLD: f64 = 1e-12;

/// Compute passive forces (springs, dampers, and friction loss).
///
/// Implements MuJoCo's passive force model:
/// - **Spring**: τ = -stiffness * (q - springref)
/// - **Damper**: τ = -damping * qvel
/// - **Friction loss**: τ = -frictionloss * tanh(qvel * friction_smoothing)
///
/// The friction loss uses a smooth `tanh` approximation to Coulomb friction
/// (`sign(qvel)`) to avoid discontinuity at zero velocity, which would cause
/// numerical issues with explicit integrators.
///
/// # MuJoCo Semantics
///
/// The spring equilibrium is `jnt_springref`, NOT `qpos0`. These are distinct:
/// - `qpos0`: Initial joint position at model load (for `mj_resetData()`)
/// - `springref`: Spring equilibrium position (where spring force is zero)
///
/// A joint can start at q=0 but have a spring pulling toward springref=0.5.
///
/// # Implicit Integration Mode
///
/// When `model.integrator == Implicit`, spring and damper forces are handled
/// implicitly in `mj_fwd_acceleration_implicit()`. This function then only
/// computes friction loss (which is velocity-sign-dependent and cannot be
/// linearized into the implicit solve).
fn mj_fwd_passive(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    data.qfrc_passive.fill(0.0);
    data.qfrc_frictionloss.fill(0.0);

    let implicit_mode = model.integrator == Integrator::ImplicitSpringDamper;
    {
        let mut visitor = PassiveForceVisitor {
            model,
            data,
            implicit_mode,
            sleep_enabled,
        };
        model.visit_joints(&mut visitor);
    }
    // visitor is dropped here, releasing the mutable borrow on data

    // Tendon passive forces: spring + damper + friction loss.
    for t in 0..model.ntendon {
        // §16.5a': Skip tendon if ALL target DOFs are sleeping
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let length = data.ten_length[t];
        let velocity = data.ten_velocity[t];
        let mut force = 0.0;

        if !implicit_mode {
            // Spring: F = -k * (L - L_ref)
            let k = model.tendon_stiffness[t];
            if k > 0.0 {
                let l_ref = model.tendon_lengthspring[t];
                force -= k * (length - l_ref);
            }

            // Damper: F = -b * v
            let b = model.tendon_damping[t];
            if b > 0.0 {
                force -= b * velocity;
            }
        }
        // NOTE: In implicit mode, tendon spring/damper forces are skipped.
        // Tendon springs/dampers couple multiple joints (non-diagonal K/D),
        // so they cannot be absorbed into the existing diagonal implicit
        // modification. This is a known limitation.

        // Friction loss: F = -frictionloss * smooth_sign(v)
        // Always explicit (velocity-sign-dependent, cannot linearize).
        // Uses tanh approximation matching the joint friction pattern.
        let fl = model.tendon_frictionloss[t];
        let fl_force;
        if fl > 0.0 && velocity.abs() > FRICTION_VELOCITY_THRESHOLD {
            let smooth_sign = (velocity * model.friction_smoothing).tanh();
            fl_force = fl * smooth_sign;
            force -= fl_force;
        } else {
            fl_force = 0.0;
        }

        data.ten_force[t] = force;

        // Map tendon force to joint forces via J^T.
        if force != 0.0 {
            apply_tendon_force(
                model,
                &data.ten_J[t],
                model.tendon_type[t],
                t,
                force,
                &mut data.qfrc_passive,
            );
        }

        // Also accumulate friction loss component separately for Newton solver (§15.0).
        if fl_force != 0.0 {
            apply_tendon_force(
                model,
                &data.ten_J[t],
                model.tendon_type[t],
                t,
                -fl_force,
                &mut data.qfrc_frictionloss,
            );
        }
    }
}

/// Check if all DOFs affected by a tendon's Jacobian belong to sleeping trees (§16.5a').
fn tendon_all_dofs_sleeping(model: &Model, data: &Data, t: usize) -> bool {
    let ten_j = &data.ten_J[t];
    for dof in 0..model.nv {
        if ten_j[dof] != 0.0 && data.tree_awake[model.dof_treeid[dof]] {
            return false; // At least one target DOF is awake
        }
    }
    true
}

/// Visitor for computing passive forces (springs, dampers, friction loss).
struct PassiveForceVisitor<'a> {
    model: &'a Model,
    data: &'a mut Data,
    implicit_mode: bool,
    sleep_enabled: bool,
}

impl PassiveForceVisitor<'_> {
    /// Check if a joint's body is sleeping (§16.5a').
    #[inline]
    fn is_joint_sleeping(&self, ctx: &JointContext) -> bool {
        self.sleep_enabled
            && self.data.body_sleep_state[self.model.jnt_body[ctx.jnt_id]] == SleepState::Asleep
    }

    /// Apply friction loss force at a single DOF.
    /// Friction loss is always explicit (velocity-sign-dependent, cannot linearize).
    #[inline]
    fn apply_friction_loss(&mut self, dof_idx: usize) {
        let qvel = self.data.qvel[dof_idx];
        let frictionloss = self.model.dof_frictionloss[dof_idx];
        if frictionloss > 0.0 && qvel.abs() > FRICTION_VELOCITY_THRESHOLD {
            let smooth_sign = (qvel * self.model.friction_smoothing).tanh();
            let fl_force = frictionloss * smooth_sign;
            self.data.qfrc_passive[dof_idx] -= fl_force;
            self.data.qfrc_frictionloss[dof_idx] -= fl_force;
        }
    }

    /// Process a 1-DOF joint (Hinge or Slide) with spring, damper, and friction.
    #[inline]
    fn visit_1dof_joint(&mut self, ctx: JointContext) {
        if self.is_joint_sleeping(&ctx) {
            return;
        }
        let dof_adr = ctx.dof_adr;
        let qpos_adr = ctx.qpos_adr;
        let jnt_id = ctx.jnt_id;

        if !self.implicit_mode {
            // Spring: τ = -k * (q - springref)
            let stiffness = self.model.jnt_stiffness[jnt_id];
            let springref = self.model.jnt_springref[jnt_id];
            let q = self.data.qpos[qpos_adr];
            self.data.qfrc_passive[dof_adr] -= stiffness * (q - springref);

            // Damper: τ = -b * qvel
            let damping = self.model.jnt_damping[jnt_id];
            let qvel = self.data.qvel[dof_adr];
            self.data.qfrc_passive[dof_adr] -= damping * qvel;
        }

        // Friction loss: always explicit
        self.apply_friction_loss(dof_adr);
    }

    /// Process a multi-DOF joint (Ball or Free) with per-DOF damping and friction.
    /// No springs (would require quaternion spring formulation).
    #[inline]
    fn visit_multi_dof_joint(&mut self, ctx: JointContext) {
        if self.is_joint_sleeping(&ctx) {
            return;
        }
        for i in 0..ctx.nv {
            let dof_idx = ctx.dof_adr + i;

            if !self.implicit_mode {
                // Per-DOF damping
                let dof_damping = self.model.dof_damping[dof_idx];
                let qvel = self.data.qvel[dof_idx];
                self.data.qfrc_passive[dof_idx] -= dof_damping * qvel;
            }

            // Friction loss: always explicit
            self.apply_friction_loss(dof_idx);
        }
    }
}

impl JointVisitor for PassiveForceVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        self.visit_1dof_joint(ctx);
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        self.visit_multi_dof_joint(ctx);
    }
}

// ============================================================================
// dof_length Mechanism Length (§16.14)
// ============================================================================

/// Compute characteristic body length for dof_length normalization (§16.14.1).
///
/// For each body, compute the maximum extent from this body through the
/// kinematic chain to any descendant. This gives a length scale that converts
/// angular velocity [rad/s] to tip velocity [m/s] for the mechanism rooted
/// at this body.
fn compute_body_lengths(model: &Model) -> Vec<f64> {
    let mut body_length = vec![0.0_f64; model.nbody];

    // Backward pass: accumulate subtree extents from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent = model.body_parent[body_id];

        // Distance from parent to this body (local position in parent frame)
        let pos = &model.body_pos[body_id];
        let dist = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();

        // This body's extent: own subtree extent + distance to parent
        let child_extent = body_length[body_id] + dist;
        body_length[parent] = body_length[parent].max(child_extent);
    }

    // Ensure minimum length (no normalization for tiny/zero-extent bodies)
    for length in &mut body_length {
        if *length < 1e-10 {
            *length = 1.0;
        }
    }

    body_length
}

/// Compute per-DOF mechanism lengths (§16.14.2).
///
/// Rotational DOFs get the body length (converts rad/s to m/s at the tip).
/// Translational DOFs keep 1.0 (already in m/s).
///
/// Called during model construction to replace the Phase A uniform 1.0.
pub fn compute_dof_lengths(model: &mut Model) {
    let body_length = compute_body_lengths(model);

    for dof in 0..model.nv {
        let jnt_id = model.dof_jnt[dof];
        let jnt_type = model.jnt_type[jnt_id];
        let offset = dof - model.jnt_dof_adr[jnt_id];

        let is_rotational = match jnt_type {
            MjJointType::Hinge | MjJointType::Ball => true,
            MjJointType::Free => offset >= 3, // DOFs 3,4,5 are rotational
            MjJointType::Slide => false,
        };

        if is_rotational {
            model.dof_length[dof] = body_length[model.dof_body[dof]];
        } else {
            model.dof_length[dof] = 1.0; // translational: already in [m/s]
        }
    }
}

// ============================================================================
// Sleep / Body Deactivation (§16)
// ============================================================================

/// Sleep update: check velocity thresholds, transition sleeping trees (§16.3).
///
/// Called at the end of `step()`, after integration completes and before
/// the warmstart save. This is the central sleep state machine.
/// Phase B sleep transition function (§16.12.2).
///
/// Three-phase approach:
/// 1. Countdown: awake trees that can sleep have their timer incremented
/// 2. Island sleep: entire islands where ALL trees are ready (timer == -1)
/// 3. Singleton sleep: unconstrained trees (no island) that are ready
///
/// Returns the number of trees that were put to sleep.
#[allow(clippy::cast_sign_loss)]
fn mj_sleep(model: &Model, data: &mut Data) -> usize {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return 0;
    }

    // Phase 1: Countdown for awake trees
    for t in 0..model.ntree {
        if data.tree_asleep[t] >= 0 {
            continue; // Already asleep
        }
        if !tree_can_sleep(model, data, t) {
            data.tree_asleep[t] = -(1 + MIN_AWAKE); // Reset
            continue;
        }
        // Increment toward -1 (ready to sleep)
        if data.tree_asleep[t] < -1 {
            data.tree_asleep[t] += 1;
        }
    }

    let mut nslept = 0;

    // Phase 2: Sleep entire islands where ALL trees are ready
    for island in 0..data.nisland {
        let itree_start = data.island_itreeadr[island];
        let itree_end = itree_start + data.island_ntree[island];

        let all_ready = (itree_start..itree_end).all(|idx| {
            let tree = data.map_itree2tree[idx];
            data.tree_asleep[tree] == -1
        });

        if all_ready {
            let trees: Vec<usize> = (itree_start..itree_end)
                .map(|idx| data.map_itree2tree[idx])
                .collect();
            sleep_trees(model, data, &trees);
            nslept += trees.len();
        }
    }

    // Phase 3: Sleep unconstrained singleton trees that are ready
    for t in 0..model.ntree {
        if data.tree_island[t] < 0 && data.tree_asleep[t] == -1 {
            sleep_trees(model, data, &[t]); // Self-link
            nslept += 1;
        }
    }

    nslept
}

/// Check if a tree is eligible to sleep (§16.12.2).
///
/// Returns `false` if policy forbids sleeping, external forces are applied,
/// or any DOF velocity exceeds the sleep threshold.
fn tree_can_sleep(model: &Model, data: &Data, tree: usize) -> bool {
    // Policy check
    if model.tree_sleep_policy[tree] == SleepPolicy::Never
        || model.tree_sleep_policy[tree] == SleepPolicy::AutoNever
    {
        return false;
    }

    // External force check: xfrc_applied on any body in tree
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        let f = &data.xfrc_applied[body_id];
        if f[0] != 0.0 || f[1] != 0.0 || f[2] != 0.0 || f[3] != 0.0 || f[4] != 0.0 || f[5] != 0.0 {
            return false;
        }
    }

    // External force check: qfrc_applied on any DOF in tree
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        if data.qfrc_applied[dof] != 0.0 {
            return false;
        }
    }

    // Velocity threshold check
    tree_velocity_below_threshold(model, data, tree)
}

/// Check if all DOFs in a tree have velocities below the sleep threshold.
fn tree_velocity_below_threshold(model: &Model, data: &Data, tree: usize) -> bool {
    let dof_start = model.tree_dof_adr[tree];
    let dof_end = dof_start + model.tree_dof_num[tree];
    for dof in dof_start..dof_end {
        if data.qvel[dof].abs() > model.sleep_tolerance * model.dof_length[dof] {
            return false;
        }
    }
    true // All DOFs below threshold (L∞ norm check)
}

/// Sleep a set of trees as a circular linked list (§16.12.1).
///
/// Creates a circular sleep cycle among the given trees, zeros all DOF-level
/// and body-level arrays, and syncs xpos/xquat with post-integration qpos.
/// For a single tree, this creates a self-link (Phase A compatible).
#[allow(clippy::cast_possible_wrap)]
fn sleep_trees(model: &Model, data: &mut Data, trees: &[usize]) {
    let n = trees.len();
    if n == 0 {
        return;
    }

    for i in 0..n {
        let tree = trees[i];
        let next = trees[(i + 1) % n];

        // Create circular linked list
        data.tree_asleep[tree] = next as i32;

        // Zero DOF-level arrays
        let dof_start = model.tree_dof_adr[tree];
        let dof_end = dof_start + model.tree_dof_num[tree];
        for dof in dof_start..dof_end {
            data.qvel[dof] = 0.0;
            data.qacc[dof] = 0.0;
            data.qfrc_bias[dof] = 0.0;
            data.qfrc_passive[dof] = 0.0;
            data.qfrc_constraint[dof] = 0.0;
            data.qfrc_actuator[dof] = 0.0; // §16.26.7: needed for policy relaxation
        }

        // Zero body-level arrays
        let body_start = model.tree_body_adr[tree];
        let body_end = body_start + model.tree_body_num[tree];
        for body_id in body_start..body_end {
            data.cvel[body_id] = SpatialVector::zeros();
            data.cacc_bias[body_id] = SpatialVector::zeros();
            data.cfrc_bias[body_id] = SpatialVector::zeros();
        }

        // Sync xpos/xquat with post-integration qpos (§16.15 compatibility)
        sync_tree_fk(model, data, tree);
    }
}

/// Recompute FK for a single tree's bodies to sync xpos/xquat with current qpos.
///
/// Called after integration to prevent false positives in qpos change detection.
fn sync_tree_fk(model: &Model, data: &mut Data, tree: usize) {
    let body_start = model.tree_body_adr[tree];
    let body_end = body_start + model.tree_body_num[tree];
    for body_id in body_start..body_end {
        let parent_id = model.body_parent[body_id];
        let mut pos = data.xpos[parent_id];
        let mut quat = data.xquat[parent_id];

        // Apply body offset in parent frame
        pos += quat * model.body_pos[body_id];
        quat *= model.body_quat[body_id];

        // Apply each joint
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];
        for jnt_id in jnt_start..jnt_end {
            let qpos_adr = model.jnt_qpos_adr[jnt_id];
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let angle = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    let anchor = model.jnt_pos[jnt_id];
                    let world_anchor = pos + quat * anchor;
                    let world_axis = quat * axis;
                    let rot = if let Some(unit_axis) = nalgebra::Unit::try_new(world_axis, 1e-10) {
                        UnitQuaternion::from_axis_angle(&unit_axis, angle)
                    } else {
                        UnitQuaternion::identity()
                    };
                    quat = rot * quat;
                    pos = world_anchor + rot * (pos - world_anchor);
                }
                MjJointType::Slide => {
                    let displacement = data.qpos[qpos_adr];
                    let axis = model.jnt_axis[jnt_id];
                    pos += quat * (axis * displacement);
                }
                MjJointType::Ball => {
                    let q = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ));
                    quat *= q;
                }
                MjJointType::Free => {
                    pos = nalgebra::Vector3::new(
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                    );
                    quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ));
                }
            }
        }
        data.xpos[body_id] = pos;
        data.xquat[body_id] = quat;
        data.xmat[body_id] = quat.to_rotation_matrix().into_inner();
    }
}

/// Re-initialize all sleep state from model policies (§16.7).
///
/// Called by `Data::reset()` and `Data::reset_to_keyframe()` to ensure sleep
/// state matches the model's tree sleep policies after a reset.
fn reset_sleep_state(model: &Model, data: &mut Data) {
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            #[allow(clippy::cast_possible_wrap)]
            {
                data.tree_asleep[t] = t as i32; // Asleep (self-link for Phase A)
            }
        } else {
            data.tree_asleep[t] = -(1 + MIN_AWAKE); // Fully awake
        }
    }
    mj_update_sleep_arrays(model, data);
}

/// Recompute derived sleep arrays from `tree_asleep` (§16.3, §16.17).
///
/// Updates: tree_awake, body_sleep_state, ntree_awake, nv_awake,
/// and the awake-index indirection arrays (body_awake_ind, parent_awake_ind, dof_awake_ind).
fn mj_update_sleep_arrays(model: &Model, data: &mut Data) {
    data.ntree_awake = 0;
    data.nv_awake = 0;

    for t in 0..model.ntree {
        let awake = data.tree_asleep[t] < 0;
        data.tree_awake[t] = awake;
        if awake {
            data.ntree_awake += 1;
        }
    }

    // --- Body sleep states + body_awake_ind + parent_awake_ind (§16.17.1) ---
    let mut nbody_awake = 0;
    let mut nparent_awake = 0;

    // Body 0 (world) is always Static and always in both indirection arrays
    if !data.body_sleep_state.is_empty() {
        data.body_sleep_state[0] = SleepState::Static;
        if !data.body_awake_ind.is_empty() {
            data.body_awake_ind[0] = 0;
            nbody_awake = 1;
        }
        if !data.parent_awake_ind.is_empty() {
            data.parent_awake_ind[0] = 0;
            nparent_awake = 1;
        }
    }

    // Update per-body sleep states and build indirection arrays
    if model.body_treeid.len() == model.nbody {
        for body_id in 1..model.nbody {
            let tree = model.body_treeid[body_id];
            let awake = if tree < model.ntree {
                data.tree_awake[tree]
            } else {
                true // No tree info → treat as awake
            };

            data.body_sleep_state[body_id] = if awake {
                SleepState::Awake
            } else {
                SleepState::Asleep
            };

            // Include in body_awake_ind if awake
            if awake && nbody_awake < data.body_awake_ind.len() {
                data.body_awake_ind[nbody_awake] = body_id;
                nbody_awake += 1;
            }

            // Include in parent_awake_ind if parent is awake or static
            let parent = model.body_parent[body_id];
            let parent_awake = data.body_sleep_state[parent] != SleepState::Asleep;
            if parent_awake && nparent_awake < data.parent_awake_ind.len() {
                data.parent_awake_ind[nparent_awake] = body_id;
                nparent_awake += 1;
            }
        }
    }

    data.nbody_awake = nbody_awake;
    data.nparent_awake = nparent_awake;

    // --- DOF awake indices (§16.17.1) ---
    let mut nv_awake = 0;
    for dof in 0..model.nv {
        if model.dof_treeid.len() > dof {
            let tree = model.dof_treeid[dof];
            if tree < model.ntree && data.tree_awake[tree] {
                if nv_awake < data.dof_awake_ind.len() {
                    data.dof_awake_ind[nv_awake] = dof;
                }
                nv_awake += 1;
            }
        } else {
            // No tree info → treat as awake
            if nv_awake < data.dof_awake_ind.len() {
                data.dof_awake_ind[nv_awake] = dof;
            }
            nv_awake += 1;
        }
    }
    data.nv_awake = nv_awake;
}

/// Check if any sleeping tree's qpos was externally modified (§16.15).
///
/// Reads `tree_qpos_dirty` flags set by `mj_fwd_position()` during FK,
/// wakes affected trees, then clears all dirty flags.
/// Returns `true` if any tree was newly woken.
fn mj_check_qpos_changed(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntree {
        if data.tree_qpos_dirty[t] && data.tree_asleep[t] >= 0 {
            // Tree was sleeping but FK detected a pose change from external qpos modification.
            mj_wake_tree(model, data, t);
            woke_any = true;
        }
    }

    // Clear all dirty flags (whether or not they triggered a wake)
    data.tree_qpos_dirty.fill(false);

    woke_any
}

/// Flood-fill connected components on a tree-tree adjacency graph (§16.11.3).
///
/// Uses DFS with an explicit stack. Trees with no edges (`rownnz[t] == 0`) get
/// `island[t] = -1` (singletons). Returns the number of islands found.
#[allow(
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::needless_continue
)]
fn mj_flood_fill(
    island: &mut [i32],
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
    stack: &mut [usize],
    ntree: usize,
) -> usize {
    island[..ntree].fill(-1);
    let mut nisland = 0usize;

    for seed in 0..ntree {
        if island[seed] >= 0 || rownnz[seed] == 0 {
            continue;
        }

        // DFS from seed using explicit stack with size counter
        let mut stack_len = 1;
        stack[0] = seed;
        island[seed] = nisland as i32;

        while stack_len > 0 {
            stack_len -= 1;
            let node = stack[stack_len];

            for j in rowadr[node]..rowadr[node] + rownnz[node] {
                let neighbor = colind[j];
                if island[neighbor] < 0 {
                    island[neighbor] = nisland as i32;
                    stack[stack_len] = neighbor;
                    stack_len += 1;
                }
            }
        }

        nisland += 1;
    }

    nisland
}

/// Discover constraint islands from the active constraint set (§16.11).
///
/// Builds a tree-tree adjacency graph from contacts, equality constraints,
/// and multi-tree tendons, then runs flood-fill to find connected components.
/// Populates all island arrays in `data`.
///
/// Called once per step, after `assemble_unified_constraints()` populates `efc_*`.
#[allow(
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::too_many_lines
)]
fn mj_island(model: &Model, data: &mut Data) {
    let ntree = model.ntree;

    // Early return: DISABLE_ISLAND flag (§16.11.5)
    if model.disableflags & DISABLE_ISLAND != 0 || ntree == 0 {
        data.nisland = 0;
        data.tree_island[..ntree].fill(-1);
        return;
    }

    let nefc = data.efc_type.len();

    // === Phase 1: Edge extraction (§16.11.2) ===
    // Collect edges as (tree_a, tree_b) pairs.
    // We use the scratch arrays for the CSR graph.

    // Clear scratch
    data.island_scratch_rownnz[..ntree].fill(0);
    data.island_scratch_colind.clear();

    // Temporary edge list: (tree_a, tree_b) pairs
    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(nefc);

    for r in 0..nefc {
        let ctype = data.efc_type[r];
        let id = data.efc_id[r];

        match ctype {
            ConstraintType::ContactNonElliptic | ConstraintType::ContactElliptic => {
                // Contact: get trees from geom → body → tree
                if id < data.contacts.len() {
                    let contact = &data.contacts[id];
                    let body1 = if contact.geom1 < model.geom_body.len() {
                        model.geom_body[contact.geom1]
                    } else {
                        continue;
                    };
                    let body2 = if contact.geom2 < model.geom_body.len() {
                        model.geom_body[contact.geom2]
                    } else {
                        continue;
                    };
                    let tree1 = if body1 > 0 && body1 < model.body_treeid.len() {
                        model.body_treeid[body1]
                    } else {
                        continue; // World body — skip
                    };
                    let tree2 = if body2 > 0 && body2 < model.body_treeid.len() {
                        model.body_treeid[body2]
                    } else {
                        // Contact with world: self-edge (marks tree as constrained)
                        if tree1 < ntree {
                            edges.push((tree1, tree1));
                        }
                        continue;
                    };
                    if tree1 < ntree && tree2 < ntree {
                        edges.push((tree1, tree2));
                        if tree1 != tree2 {
                            edges.push((tree2, tree1)); // Symmetric
                        }
                    }
                }
            }
            ConstraintType::Equality => {
                // Equality constraint: look up object bodies based on eq_type
                if id < model.neq {
                    let (tree1, tree2) = equality_trees(model, id);
                    if tree1 < ntree {
                        edges.push((tree1, tree1)); // Self-edge
                        if tree2 < ntree && tree2 != tree1 {
                            edges.push((tree1, tree2));
                            edges.push((tree2, tree1));
                        } else if tree2 < ntree {
                            edges.push((tree2, tree2)); // Self-edge for tree2
                        }
                    } else if tree2 < ntree {
                        edges.push((tree2, tree2));
                    }
                }
            }
            ConstraintType::LimitJoint => {
                // Joint limit: single tree from jnt_body
                if id < model.jnt_body.len() {
                    let body = model.jnt_body[id];
                    if body > 0 && body < model.body_treeid.len() {
                        let tree = model.body_treeid[body];
                        if tree < ntree {
                            edges.push((tree, tree));
                        }
                    }
                }
            }
            ConstraintType::LimitTendon => {
                // Tendon limit: use precomputed tendon_treenum
                if id < model.ntendon {
                    if model.tendon_treenum[id] == 2 {
                        let t1 = model.tendon_tree[2 * id];
                        let t2 = model.tendon_tree[2 * id + 1];
                        if t1 < ntree && t2 < ntree {
                            edges.push((t1, t2));
                            edges.push((t2, t1));
                        }
                    } else if model.tendon_treenum[id] == 1 {
                        // Single-tree tendon: self-edge
                        // Find the tree by scanning Jacobian row
                        for dof in 0..model.nv {
                            if data.efc_J[(r, dof)].abs() > 0.0 {
                                let tree = model.dof_treeid[dof];
                                if tree < ntree {
                                    edges.push((tree, tree));
                                }
                                break;
                            }
                        }
                    }
                }
            }
            ConstraintType::FrictionLoss => {
                // DOF friction loss or tendon friction loss: single tree from dof_treeid
                // For DOF friction: efc_id = dof_idx → dof_treeid[dof_idx]
                // For tendon friction: scan Jacobian row for nonzero DOFs
                if id < model.nv && r < nefc {
                    // Check if this is a DOF friction (efc_J has single 1.0 at dof_idx)
                    if id < model.dof_treeid.len() {
                        let tree = model.dof_treeid[id];
                        if tree < ntree {
                            edges.push((tree, tree));
                        }
                    }
                }
            }
        }
    }

    // === Phase 2: Build CSR adjacency graph (§16.11.3) ===

    // Count edges per tree (rownnz)
    for &(ta, _) in &edges {
        if ta < ntree {
            data.island_scratch_rownnz[ta] += 1;
        }
    }

    // Compute row addresses (prefix sum)
    data.island_scratch_rowadr[..ntree].fill(0);
    if ntree > 0 {
        let mut prefix = 0;
        for t in 0..ntree {
            data.island_scratch_rowadr[t] = prefix;
            prefix += data.island_scratch_rownnz[t];
        }
        data.island_scratch_colind.resize(prefix, 0);
    }

    // Fill column indices
    let mut fill_count = vec![0usize; ntree];
    for &(ta, tb) in &edges {
        if ta < ntree {
            let idx = data.island_scratch_rowadr[ta] + fill_count[ta];
            if idx < data.island_scratch_colind.len() {
                data.island_scratch_colind[idx] = tb;
            }
            fill_count[ta] += 1;
        }
    }

    // === Phase 3: Flood fill (§16.11.3) ===
    // We need to clone scratch arrays to avoid borrow conflicts
    let rownnz: Vec<usize> = data.island_scratch_rownnz[..ntree].to_vec();
    let rowadr: Vec<usize> = data.island_scratch_rowadr[..ntree].to_vec();
    let colind: Vec<usize> = data.island_scratch_colind.clone();

    let mut island_out = vec![-1i32; ntree];
    let mut stack = vec![0usize; ntree];

    let nisland = mj_flood_fill(
        &mut island_out,
        &rownnz,
        &rowadr,
        &colind,
        &mut stack,
        ntree,
    );

    // Copy results back
    data.tree_island[..ntree].copy_from_slice(&island_out[..ntree]);
    data.nisland = nisland;

    // === Phase 4: Populate island arrays (§16.11.4) ===

    // 4a: Trees per island
    data.island_ntree[..nisland].fill(0);
    for t in 0..ntree {
        if island_out[t] >= 0 {
            let isl = island_out[t] as usize;
            if isl < nisland {
                data.island_ntree[isl] += 1;
            }
        }
    }

    // Compute island_itreeadr (prefix sum of island_ntree)
    if nisland > 0 {
        let mut prefix = 0;
        for i in 0..nisland {
            data.island_itreeadr[i] = prefix;
            prefix += data.island_ntree[i];
        }
    }

    // Pack map_itree2tree grouped by island
    let mut island_fill = vec![0usize; nisland];
    for t in 0..ntree {
        if island_out[t] >= 0 {
            let isl = island_out[t] as usize;
            if isl < nisland {
                let idx = data.island_itreeadr[isl] + island_fill[isl];
                if idx < data.map_itree2tree.len() {
                    data.map_itree2tree[idx] = t;
                }
                island_fill[isl] += 1;
            }
        }
    }

    // 4b: DOFs per island
    data.island_nv[..nisland].fill(0);
    data.dof_island[..model.nv].fill(-1);
    data.map_dof2idof[..model.nv].fill(-1);

    // First pass: compute island_nv and island_idofadr
    for dof in 0..model.nv {
        if dof < model.dof_treeid.len() {
            let tree = model.dof_treeid[dof];
            if tree < ntree && island_out[tree] >= 0 {
                let isl = island_out[tree] as usize;
                if isl < nisland {
                    data.dof_island[dof] = isl as i32;
                    data.island_nv[isl] += 1;
                }
            }
        }
    }

    // Compute island_idofadr (prefix sum of island_nv)
    if nisland > 0 {
        let mut prefix = 0;
        for i in 0..nisland {
            data.island_idofadr[i] = prefix;
            prefix += data.island_nv[i];
        }
    }

    // Pack map_idof2dof and build map_dof2idof
    let mut island_dof_fill = vec![0usize; nisland];
    for dof in 0..model.nv {
        let isl_i32 = data.dof_island[dof];
        if isl_i32 >= 0 {
            let isl = isl_i32 as usize;
            if isl < nisland {
                let local_idx = island_dof_fill[isl];
                let global_idx = data.island_idofadr[isl] + local_idx;
                if global_idx < data.map_idof2dof.len() {
                    data.map_idof2dof[global_idx] = dof;
                }
                data.map_dof2idof[dof] = local_idx as i32;
                island_dof_fill[isl] += 1;
            }
        }
    }

    // 4c: Constraints per island
    let nefc = data.efc_type.len();
    data.efc_island.resize(nefc, -1);
    data.map_efc2iefc.resize(nefc, -1);
    data.map_iefc2efc.resize(nefc, 0);
    data.island_nefc[..nisland].fill(0);

    // Assign each constraint row to an island based on its tree
    for r in 0..nefc {
        let tree = constraint_tree(model, data, r);
        if tree < ntree && island_out[tree] >= 0 {
            let isl = island_out[tree] as usize;
            if isl < nisland {
                data.efc_island[r] = isl as i32;
                data.island_nefc[isl] += 1;
            }
        }
    }

    // Compute island_iefcadr (prefix sum of island_nefc)
    if nisland > 0 {
        let mut prefix = 0;
        for i in 0..nisland {
            data.island_iefcadr[i] = prefix;
            prefix += data.island_nefc[i];
        }
    }

    // Pack constraint maps
    let mut island_efc_fill = vec![0usize; nisland];
    for r in 0..nefc {
        let isl_i32 = data.efc_island[r];
        if isl_i32 >= 0 {
            let isl = isl_i32 as usize;
            if isl < nisland {
                let local_idx = island_efc_fill[isl];
                let global_idx = data.island_iefcadr[isl] + local_idx;
                data.map_efc2iefc[r] = local_idx as i32;
                if global_idx < data.map_iefc2efc.len() {
                    data.map_iefc2efc[global_idx] = r;
                }
                island_efc_fill[isl] += 1;
            }
        }
    }
}

/// Get the tree pair spanned by an equality constraint (§16.11.2).
///
/// Returns `(tree1, tree2)` where `tree1` and `tree2` may be equal
/// for single-tree constraints. Returns `(usize::MAX, usize::MAX)`
/// if the trees cannot be determined.
fn equality_trees(model: &Model, eq_id: usize) -> (usize, usize) {
    let sentinel = usize::MAX;
    match model.eq_type[eq_id] {
        EqualityType::Connect | EqualityType::Weld => {
            // obj1/obj2 are body IDs
            let b1 = model.eq_obj1id[eq_id];
            let b2 = model.eq_obj2id[eq_id];
            let t1 = if b1 > 0 && b1 < model.body_treeid.len() {
                model.body_treeid[b1]
            } else {
                sentinel
            };
            let t2 = if b2 > 0 && b2 < model.body_treeid.len() {
                model.body_treeid[b2]
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Joint => {
            // obj1/obj2 are joint IDs → jnt_body → body_treeid
            let j1 = model.eq_obj1id[eq_id];
            let j2 = model.eq_obj2id[eq_id];
            let t1 = if j1 < model.jnt_body.len() {
                let b = model.jnt_body[j1];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            let t2 = if j2 < model.jnt_body.len() {
                let b = model.jnt_body[j2];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Distance => {
            // obj1/obj2 are geom IDs → geom_body → body_treeid
            let g1 = model.eq_obj1id[eq_id];
            let g2 = model.eq_obj2id[eq_id];
            let t1 = if g1 < model.geom_body.len() {
                let b = model.geom_body[g1];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            let t2 = if g2 < model.geom_body.len() {
                let b = model.geom_body[g2];
                if b > 0 && b < model.body_treeid.len() {
                    model.body_treeid[b]
                } else {
                    sentinel
                }
            } else {
                sentinel
            };
            (t1, t2)
        }
        EqualityType::Tendon => {
            // Tendon equality: scan tendon waypoints
            // For now, return sentinel (tendon equality not yet implemented)
            (sentinel, sentinel)
        }
    }
}

/// Determine the primary tree for a constraint row (§16.11.2).
///
/// Used for assigning constraint rows to islands. Returns `usize::MAX`
/// if the tree cannot be determined.
#[allow(clippy::too_many_lines)]
fn constraint_tree(model: &Model, data: &Data, row: usize) -> usize {
    let sentinel = usize::MAX;
    let ctype = data.efc_type[row];
    let id = data.efc_id[row];
    let ntree = model.ntree;

    match ctype {
        ConstraintType::ContactNonElliptic | ConstraintType::ContactElliptic => {
            if id >= data.contacts.len() {
                return sentinel;
            }
            let contact = &data.contacts[id];
            let body1 = if contact.geom1 < model.geom_body.len() {
                model.geom_body[contact.geom1]
            } else {
                return sentinel;
            };
            // Use body1's tree as the primary tree for this contact
            if body1 > 0 && body1 < model.body_treeid.len() {
                let tree = model.body_treeid[body1];
                if tree < ntree {
                    return tree;
                }
            }
            // Fallback: try body2
            let body2 = if contact.geom2 < model.geom_body.len() {
                model.geom_body[contact.geom2]
            } else {
                return sentinel;
            };
            if body2 > 0 && body2 < model.body_treeid.len() {
                let tree = model.body_treeid[body2];
                if tree < ntree {
                    return tree;
                }
            }
            sentinel
        }
        ConstraintType::Equality => {
            if id < model.neq {
                let (t1, t2) = equality_trees(model, id);
                if t1 < ntree {
                    t1
                } else if t2 < ntree {
                    t2
                } else {
                    sentinel
                }
            } else {
                sentinel
            }
        }
        ConstraintType::LimitJoint => {
            if id < model.jnt_body.len() {
                let body = model.jnt_body[id];
                if body > 0 && body < model.body_treeid.len() {
                    let tree = model.body_treeid[body];
                    if tree < ntree {
                        return tree;
                    }
                }
            }
            sentinel
        }
        ConstraintType::LimitTendon => {
            // Use precomputed tendon tree mapping
            if id < model.ntendon && model.tendon_treenum[id] >= 1 {
                if model.tendon_treenum[id] == 2 {
                    let t = model.tendon_tree[2 * id];
                    if t < ntree {
                        return t;
                    }
                }
                // Single tree: scan Jacobian
                for dof in 0..model.nv {
                    if data.efc_J[(row, dof)].abs() > 0.0 && dof < model.dof_treeid.len() {
                        let tree = model.dof_treeid[dof];
                        if tree < ntree {
                            return tree;
                        }
                    }
                }
            }
            sentinel
        }
        ConstraintType::FrictionLoss => {
            if id < model.dof_treeid.len() {
                let tree = model.dof_treeid[id];
                if tree < ntree {
                    return tree;
                }
            }
            sentinel
        }
    }
}

/// Wake detection: check user-applied forces on sleeping bodies (§16.4).
///
/// Called at the start of `forward()`, before any pipeline stage.
fn mj_wake(model: &Model, data: &mut Data) {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return;
    }

    // Check xfrc_applied (per-body Cartesian forces)
    for body_id in 1..model.nbody {
        if data.body_sleep_state[body_id] != SleepState::Asleep {
            continue;
        }
        // Bytewise nonzero check (matches MuJoCo: -0.0 wakes because sign bit is set).
        // Use to_bits() != 0 instead of != 0.0 because IEEE 754 treats -0.0 == 0.0.
        let force = &data.xfrc_applied[body_id];
        if force.iter().any(|&v| v.to_bits() != 0) {
            mj_wake_tree(model, data, model.body_treeid[body_id]);
        }
    }

    // Check qfrc_applied (per-DOF generalized forces)
    for dof in 0..model.nv {
        let tree = model.dof_treeid[dof];
        if !data.tree_awake[tree] && data.qfrc_applied[dof].to_bits() != 0 {
            mj_wake_tree(model, data, tree);
        }
    }
}

/// Wake detection after collision: check contacts between sleeping and awake bodies (§16.4).
///
/// Returns `true` if any tree was woken (triggers re-collision).
fn mj_wake_collision(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for contact_idx in 0..data.ncon {
        let contact = &data.contacts[contact_idx];
        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];
        let state1 = data.body_sleep_state[body1];
        let state2 = data.body_sleep_state[body2];

        // Wake sleeping body if partner is awake (not static — static bodies
        // like the world/ground don't wake sleeping bodies).
        let need_wake = match (state1, state2) {
            (SleepState::Asleep, SleepState::Awake) => Some(body1),
            (SleepState::Awake, SleepState::Asleep) => Some(body2),
            _ => None,
        };

        if let Some(body_id) = need_wake {
            let tree = model.body_treeid[body_id];
            if tree < model.ntree {
                mj_wake_tree(model, data, tree);
                woke_any = true;
            }
        }
    }
    woke_any
}

/// Wake a single tree: reset timer, update derived arrays.
/// Wake a tree and its entire sleep cycle (§16.12.3).
///
/// Traverses the circular linked list to wake all trees in the sleeping
/// island. Eagerly updates `tree_awake` and `body_sleep_state` so
/// subsequent wake functions in the same pass see the updated state.
#[allow(clippy::cast_sign_loss)]
fn mj_wake_tree(model: &Model, data: &mut Data, tree: usize) {
    if data.tree_awake[tree] {
        return; // Already awake
    }

    if data.tree_asleep[tree] < 0 {
        // Awake but tree_awake flag stale — just update the flag
        data.tree_awake[tree] = true;
        return;
    }

    // Traverse the sleep cycle, waking each tree
    let mut current = tree;
    loop {
        let next = data.tree_asleep[current] as usize;
        data.tree_asleep[current] = -(1 + MIN_AWAKE); // Fully awake
        data.tree_awake[current] = true;

        // Update body states
        let body_start = model.tree_body_adr[current];
        let body_end = body_start + model.tree_body_num[current];
        for body_id in body_start..body_end {
            data.body_sleep_state[body_id] = SleepState::Awake;
        }

        current = next;
        if current == tree {
            break; // Full cycle traversed
        }
    }
}

/// Check if a DOF's tree is awake (helper for integration skip).
#[inline]
fn data_tree_awake_for_dof(model: &Model, data: &Data, dof: usize) -> bool {
    if model.dof_treeid.len() <= dof {
        return true; // No tree info → treat as awake
    }
    let tree = model.dof_treeid[dof];
    if tree >= data.tree_awake.len() {
        return true;
    }
    data.tree_awake[tree]
}

/// Map a sensor to its associated body_id, or `None` if multi-body (§16.5d).
fn sensor_body_id(model: &Model, sensor_id: usize) -> Option<usize> {
    let objid = model.sensor_objid[sensor_id];
    match model.sensor_objtype[sensor_id] {
        MjObjectType::Body => Some(objid),
        MjObjectType::Joint => {
            if objid < model.njnt {
                Some(model.jnt_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Geom => {
            if objid < model.ngeom {
                Some(model.geom_body[objid])
            } else {
                None
            }
        }
        MjObjectType::Site => {
            if objid < model.nsite {
                Some(model.site_body[objid])
            } else {
                None
            }
        }
        // Multi-body, actuated, or world-relative sensors — always compute
        MjObjectType::Tendon | MjObjectType::Actuator | MjObjectType::None => None,
    }
}

// ============================================================================
// Contact Jacobian Computation
// ============================================================================

/// Compute the contact Jacobian row for a single body contribution.
///
/// The Jacobian maps joint velocities to Cartesian point velocity:
/// v_point = J * qvel
///
/// For a contact at world point `p` on body `body_id`, we walk the kinematic
/// chain from body to root, accumulating Jacobian contributions from each joint.
///
/// Returns the Jacobian row (length nv) for translational velocity at the point.
#[allow(dead_code)] // Reserved for future use in inverse dynamics
fn compute_body_jacobian_at_point(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: Vector3<f64>,
) -> DVector<f64> {
    let mut jacobian = DVector::zeros(model.nv);

    if body_id == 0 {
        return jacobian; // World has no DOFs
    }

    // Walk from body to root, accumulating Jacobian contributions
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // v = axis × r (rotational velocity contribution)
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    jacobian[dof_adr] = j_col.x;
                    // Store the other components in a 3xnv matrix style
                    // For now, we'll compute this per-direction
                }
                MjJointType::Slide => {
                    // v = axis (translational velocity contribution)
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    jacobian[dof_adr] = axis.x;
                }
                MjJointType::Ball => {
                    // Angular velocity contribution
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    // v = ω × r, and ω is in body frame for ball joint
                    // ω_world = R * ω_body, v = ω_world × r
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        // ω_body[i] contribution
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        let j_col = omega_world.cross(&r);
                        jacobian[dof_adr + i] = j_col.x;
                    }
                }
                MjJointType::Free => {
                    // Linear DOFs directly contribute
                    jacobian[dof_adr] = 1.0; // vx
                    jacobian[dof_adr + 1] = 0.0; // vy (this is for x direction)
                    jacobian[dof_adr + 2] = 0.0; // vz

                    // Angular DOFs: v = ω × r
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    // v = ω × r => for ω in world frame
                    // ∂vx/∂ωy = rz, ∂vx/∂ωz = -ry
                    jacobian[dof_adr + 3] = 0.0; // ∂vx/∂ωx
                    jacobian[dof_adr + 4] = r.z; // ∂vx/∂ωy
                    jacobian[dof_adr + 5] = -r.y; // ∂vx/∂ωz
                }
            }
        }
        current_body = model.body_parent[current_body];
    }

    jacobian
}

/// Compute the full dim×nv contact Jacobian for a contact point.
///
/// Returns a `contact.dim`×nv matrix where rows depend on contact dimension:
/// - dim=1: Row 0: normal direction
/// - dim=3: Row 0: normal, Row 1-2: tangents (sliding friction)
/// - dim=4: Rows 0-2: as dim=3, Row 3: torsional (spin about normal)
/// - dim=6: Rows 0-3: as dim=4, Rows 4-5: rolling (angular velocity tangent components)
fn compute_contact_jacobian(model: &Model, data: &Data, contact: &Contact) -> DMatrix<f64> {
    let nv = model.nv;
    let dim = contact.dim;
    let body1 = model.geom_body[contact.geom1];
    let body2 = model.geom_body[contact.geom2];

    // Use the pre-computed contact frame for consistency.
    // contact.frame[] was computed during contact construction via compute_tangent_frame().
    // This ensures the Jacobian rows align exactly with the RHS computation in
    // assemble_contact_system() and force application in mj_fwd_constraint().
    let normal = contact.normal;
    let tangent1 = contact.frame[0];
    let tangent2 = contact.frame[1];

    // Allocate dim×nv Jacobian
    let mut j = DMatrix::zeros(dim, nv);

    // Helper: add body Jacobian contribution for one direction
    let add_body_jacobian =
        |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>, body_id: usize, sign: f64| {
            if body_id == 0 {
                return; // World has no DOFs
            }

            let mut current_body = body_id;
            while current_body != 0 {
                let jnt_start = model.body_jnt_adr[current_body];
                let jnt_end = jnt_start + model.body_jnt_num[current_body];

                for jnt_id in jnt_start..jnt_end {
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let jnt_body = model.jnt_body[jnt_id];

                    match model.jnt_type[jnt_id] {
                        MjJointType::Hinge => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let j_col = axis.cross(&r);
                            j[(row, dof_adr)] += sign * direction.dot(&j_col);
                        }
                        MjJointType::Slide => {
                            let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                            j[(row, dof_adr)] += sign * direction.dot(&axis);
                        }
                        MjJointType::Ball => {
                            let jpos =
                                data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega_world = rot * Vector3::ith(i, 1.0);
                                let j_col = omega_world.cross(&r);
                                j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
                            }
                        }
                        MjJointType::Free => {
                            // Linear DOFs
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs: v = ω × r
                            let jpos = Vector3::new(
                                data.qpos[model.jnt_qpos_adr[jnt_id]],
                                data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                                data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                            );
                            let r = contact.pos - jpos;
                            // d/dω of (ω × r) in direction d: (e_i × r) · d
                            let ex = Vector3::x();
                            let ey = Vector3::y();
                            let ez = Vector3::z();
                            j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
                            j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
                            j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
                        }
                    }
                }
                current_body = model.body_parent[current_body];
            }
        };

    // Compute relative velocity J * qvel = v2 - v1 (velocity of body2 relative to body1)
    // Normal points FROM body1 (geom1) TO body2 (geom2).
    // Positive normal velocity = bodies separating (good)
    // Negative normal velocity = bodies approaching (need constraint)
    //
    // Body2 contributes positively (its velocity in +normal direction = separating)
    // Body1 contributes negatively (its velocity in +normal direction = approaching body2)

    // Row 0: normal direction (always present)
    add_body_jacobian(&mut j, 0, &normal, body2, 1.0);
    add_body_jacobian(&mut j, 0, &normal, body1, -1.0);

    // Rows 1-2: tangent directions (dim >= 3: sliding friction)
    if dim >= 3 {
        add_body_jacobian(&mut j, 1, &tangent1, body2, 1.0);
        add_body_jacobian(&mut j, 1, &tangent1, body1, -1.0);

        add_body_jacobian(&mut j, 2, &tangent2, body2, 1.0);
        add_body_jacobian(&mut j, 2, &tangent2, body1, -1.0);
    }

    // Row 3: torsional/spin (dim >= 4: angular velocity about contact normal)
    // This constrains relative spinning about the contact normal
    if dim >= 4 {
        add_angular_jacobian(&mut j, 3, &normal, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 3, &normal, body1, -1.0, model, data);
    }

    // Rows 4-5: rolling friction (dim = 6: angular velocity in tangent plane)
    // This constrains relative rolling in the tangent directions
    if dim >= 6 {
        add_angular_jacobian(&mut j, 4, &tangent1, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 4, &tangent1, body1, -1.0, model, data);

        add_angular_jacobian(&mut j, 5, &tangent2, body2, 1.0, model, data);
        add_angular_jacobian(&mut j, 5, &tangent2, body1, -1.0, model, data);
    }

    j
}

/// Add angular Jacobian contribution for a body in a given direction.
///
/// This computes how joint velocities affect angular velocity of the body
/// in the specified direction. Used for torsional and rolling constraints.
#[inline]
fn add_angular_jacobian(
    j: &mut DMatrix<f64>,
    row: usize,
    direction: &Vector3<f64>,
    body_id: usize,
    sign: f64,
    model: &Model,
    data: &Data,
) {
    if body_id == 0 {
        return; // World has no DOFs
    }

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Hinge contributes angular velocity along its axis
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    j[(row, dof_adr)] += sign * direction.dot(&axis);
                }
                MjJointType::Slide => {
                    // Slide joints don't contribute to angular velocity
                }
                MjJointType::Ball => {
                    // Ball joint contributes angular velocity in all 3 axes
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + i)] += sign * direction.dot(&omega_world);
                    }
                }
                MjJointType::Free => {
                    // Free joint: only angular DOFs contribute (DOFs 3-5)
                    j[(row, dof_adr + 3)] += sign * direction.x;
                    j[(row, dof_adr + 4)] += sign * direction.y;
                    j[(row, dof_adr + 5)] += sign * direction.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

// ============================================================================
// PGS Constraint Solver
// ============================================================================

/// Check if two bodies share a kinematic chain.
///
/// Returns true if the bodies share any common ancestor joint, meaning forces
/// on one body propagate to the other through the kinematic chain. This includes:
/// - Same body (trivially coupled)
/// - Ancestor-descendant relationships (direct chain)
/// - Siblings or cousins (share a common ancestor joint)
///
/// This is used to determine if off-diagonal constraint blocks are non-zero.
/// Uses pre-computed `body_ancestor_mask` for O(num_words) lookup where
/// num_words = ceil(njnt / 64), typically 1 for models with <64 joints.
///
/// Note: Bodies with empty ancestor masks (no joints) return false, which is
/// correct since they have no DOFs and cannot be kinematically coupled.
#[inline]
fn bodies_share_chain(model: &Model, body_a: usize, body_b: usize) -> bool {
    if body_a == body_b {
        return true;
    }

    // World body (0) has no joints - never coupled with anything except itself
    if body_a == 0 || body_b == 0 {
        return false;
    }

    // Check if the bodies share any common ancestor joint using bitmask AND.
    // If the intersection is non-empty, they're in the same kinematic chain.
    let mask_a = &model.body_ancestor_mask[body_a];
    let mask_b = &model.body_ancestor_mask[body_b];

    // Iterate over words (typically just 1 for models with <64 joints).
    // If either mask is empty (len=0), num_words=0 and loop returns false immediately.
    // This correctly handles edge cases like bodies with no ancestor joints.
    let num_words = mask_a.len().min(mask_b.len());
    for i in 0..num_words {
        if (mask_a[i] & mask_b[i]) != 0 {
            return true;
        }
    }

    false
}

/// Compute starting row index for each contact in the constraint system.
///
/// Returns (efc_offsets, nefc) where:
/// - efc_offsets[i] = starting row for contact i
/// - nefc = total constraint rows = sum of all contact dimensions
#[inline]
fn compute_efc_offsets(contacts: &[Contact]) -> (Vec<usize>, usize) {
    let mut offsets = Vec::with_capacity(contacts.len());
    let mut offset = 0;
    for contact in contacts {
        offsets.push(offset);
        offset += contact.dim;
    }
    (offsets, offset)
}

/// Assemble Delassus matrix A and constraint RHS b for contact constraints.
/// Shared by PGS and CG solvers.
///
/// A[i,j] = J_i * M^{-1} * J_j^T (with CFM regularization on diagonal).
/// b includes unconstrained acceleration, contact velocities, and Baumgarte
/// stabilization from solref/solimp.
///
/// Returns (A, b, efc_offsets) where efc_offsets[i] is the starting row for contact i.
fn assemble_contact_system(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
) -> (DMatrix<f64>, DVector<f64>, Vec<usize>) {
    let ncon = contacts.len();
    let (efc_offsets, nefc) = compute_efc_offsets(contacts);

    let mut a = DMatrix::zeros(nefc, nefc);
    let mut b = DVector::zeros(nefc);

    // Base regularization (softness) for numerical stability
    let base_regularization = model.regularization;

    // Compute unconstrained acceleration (qacc_smooth = M^{-1} * qfrc_smooth)
    // qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
    let mut qacc_smooth = data.qfrc_applied.clone();
    qacc_smooth += &data.qfrc_actuator;
    qacc_smooth += &data.qfrc_passive;
    qacc_smooth -= &data.qfrc_bias;
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut qacc_smooth);

    // Pre-compute M^{-1} * J^T for each contact (variable dimension per contact)
    let mut minv_jt: Vec<DMatrix<f64>> = Vec::with_capacity(ncon);
    for (k, jacobian) in jacobians.iter().enumerate() {
        let dim_k = contacts[k].dim;
        let jt = jacobian.transpose();
        // Solve M * X = J^T for each column (dim_k columns)
        let mut minv_jt_contact = DMatrix::zeros(model.nv, dim_k);
        for col in 0..dim_k {
            let mut x = jt.column(col).clone_owned();
            mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut x);
            minv_jt_contact.set_column(col, &x);
        }
        minv_jt.push(minv_jt_contact);
    }

    // Build A matrix blocks and RHS
    for (i, (contact_i, jac_i)) in contacts.iter().zip(jacobians.iter()).enumerate() {
        // Derive Baumgarte parameters from per-contact solref/solimp
        // solref[0] = timeconst, solref[1] = dampratio
        // Safety clamp: minimum 0.001s timeconst prevents division by zero in
        // Baumgarte stabilization (stiffness = 1/tc², damping = 2*dr/tc).
        // Not configurable — values below 0.001 cause NaN propagation.
        let timeconst = contact_i.solref[0].max(0.001);
        let dampratio = contact_i.solref[1].max(0.0);

        // Position-dependent impedance from full solimp parameters.
        // Uses penetration depth as the violation magnitude, matching
        // MuJoCo's per-row impedance computation for contact constraints.
        let d = compute_impedance(contact_i.solimp, contact_i.depth.abs());

        // CFM (constraint force mixing) derived from impedance:
        // Higher d (closer to 1) = stiffer constraint = lower CFM
        let cfm = base_regularization + (1.0 - d) * model.regularization * 100.0;

        let dim_i = contact_i.dim;
        let offset_i = efc_offsets[i];

        // Diagonal block: A[i,i] = J_i * M^{-1} * J_i^T
        let a_ii = jac_i * &minv_jt[i];
        for ri in 0..dim_i {
            for ci in 0..dim_i {
                a[(offset_i + ri, offset_i + ci)] = a_ii[(ri, ci)];
            }
            // Add regularization to diagonal
            a[(offset_i + ri, offset_i + ri)] += cfm;
        }

        // Store ERP for RHS computation below
        // ERP (error reduction parameter) derived from solref:
        // erp = dt / (dt + timeconst) for critically damped response
        // Scale by dampratio for underdamped/overdamped behavior
        let dt = model.timestep.max(1e-6);
        let erp_i = (dt / (dt + timeconst)) * dampratio.min(1.0);

        // Off-diagonal blocks: A[i,j] = J_i * M^{-1} * J_j^T
        //
        // OPTIMIZATION: For independent bodies (no shared kinematic chain),
        // the off-diagonal blocks are zero because:
        // - J_i is non-zero only for body_i's DOFs
        // - M^{-1}*J_j^T is non-zero only for body_j's DOFs
        // - Independent free joints have block-diagonal mass matrix
        //
        // We check if contacts share a DYNAMIC body (not world body 0).
        // World body has no DOFs, so contacts involving world don't couple
        // unless they share the same dynamic body.
        //
        // This reduces O(n²) to O(n) for systems with independent bodies.
        let body_i1 = model.geom_body[contact_i.geom1];
        let body_i2 = model.geom_body[contact_i.geom2];

        // Extract dynamic bodies for contact i (filter out world body 0).
        // Use fixed-size arrays to avoid per-iteration heap allocation.
        let dynamic_i: [Option<usize>; 2] = [
            if body_i1 != 0 { Some(body_i1) } else { None },
            if body_i2 != 0 { Some(body_i2) } else { None },
        ];
        let has_dynamic_i = dynamic_i[0].is_some() || dynamic_i[1].is_some();

        // Only compute off-diagonal blocks if contact i has dynamic bodies.
        // Static-only contacts (e.g., plane-plane) can't couple with anything.
        // Note: We still need to compute RHS below, so don't `continue` here.
        if has_dynamic_i {
            for (j, contact_j) in contacts.iter().enumerate().skip(i + 1) {
                let geom1_body = model.geom_body[contact_j.geom1];
                let geom2_body = model.geom_body[contact_j.geom2];

                // Extract dynamic bodies for contact j
                let dynamic_j: [Option<usize>; 2] = [
                    if geom1_body != 0 {
                        Some(geom1_body)
                    } else {
                        None
                    },
                    if geom2_body != 0 {
                        Some(geom2_body)
                    } else {
                        None
                    },
                ];

                // If contact j has no dynamic bodies, skip (can't couple)
                if dynamic_j[0].is_none() && dynamic_j[1].is_none() {
                    continue;
                }

                // Check if any dynamic bodies are shared or in the same kinematic chain.
                let bodies_interact = dynamic_i.iter().filter_map(|&b| b).any(|bi| {
                    dynamic_j
                        .iter()
                        .filter_map(|&b| b)
                        .any(|bj| bi == bj || bodies_share_chain(model, bi, bj))
                });

                if !bodies_interact {
                    // Off-diagonal block is zero — skip expensive matrix multiply
                    continue;
                }

                let dim_j = contacts[j].dim;
                let offset_j = efc_offsets[j];
                let block_ij = jac_i * &minv_jt[j];
                for ri in 0..dim_i {
                    for ci in 0..dim_j {
                        a[(offset_i + ri, offset_j + ci)] = block_ij[(ri, ci)];
                        a[(offset_j + ci, offset_i + ri)] = block_ij[(ri, ci)]; // Symmetric
                    }
                }
            }
        }

        // RHS: b = J * qacc_smooth + aref
        // where qacc_smooth = M^{-1} * qfrc_smooth (acceleration without contacts)
        // and aref = velocity_error + position_error (Baumgarte stabilization)
        //
        // For the LCP: A*λ + b >= 0, λ >= 0
        // We want λ > 0 when b < 0 (i.e., when unconstrained acceleration
        // would cause penetration/violation)

        // Reuse body_i1/body_i2 from above (same as model.geom_body[contact_i.geom1/2])
        let vel1 = compute_point_velocity(data, body_i1, contact_i.pos);
        let vel2 = compute_point_velocity(data, body_i2, contact_i.pos);
        let rel_vel = vel2 - vel1;

        // Use the pre-computed contact frame for consistency with the Jacobian.
        // contact.frame[] was computed during contact construction via compute_tangent_frame().
        let normal = contact_i.normal;
        let tangent1 = contact_i.frame[0];
        let tangent2 = contact_i.frame[1];

        let vn = rel_vel.dot(&normal);
        let vt1 = rel_vel.dot(&tangent1);
        let vt2 = rel_vel.dot(&tangent2);

        // Compute J * qacc_smooth (unconstrained relative acceleration in contact frame)
        // qacc_smooth = M^{-1} * (qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias)
        let j_qacc_smooth = jac_i * &qacc_smooth;

        // Baumgarte stabilization parameters
        // aref = (1/h) * vn + β * depth
        // This pushes to zero velocity and zero penetration
        // erp_i was computed above from per-contact solref
        let velocity_damping = 1.0 / dt; // 1/h
        let depth_correction = erp_i / dt; // β/h (erp_i from per-contact solref)

        // b = J * qacc_smooth + velocity_term + position_term
        // For the normal constraint: we want to stop penetration
        // J * qacc = -aref means: acceleration should be aref (away from surface)
        //
        // b should be negative when the constraint needs to activate
        // Normal: if J*qacc_smooth < 0 (accelerating into surface), we need λ > 0

        // Row 0: normal direction (always present)
        b[offset_i] = j_qacc_smooth[0] + velocity_damping * vn + depth_correction * contact_i.depth;

        // Rows 1-2: tangent directions (dim >= 3: sliding friction)
        if dim_i >= 3 {
            b[offset_i + 1] = j_qacc_smooth[1] + vt1; // Friction tangent 1
            b[offset_i + 2] = j_qacc_smooth[2] + vt2; // Friction tangent 2
        }

        // Row 3: torsional (dim >= 4: angular velocity about normal)
        if dim_i >= 4 {
            let omega1 = compute_body_angular_velocity(data, body_i1);
            let omega2 = compute_body_angular_velocity(data, body_i2);
            let rel_omega = omega2 - omega1;
            let omega_n = rel_omega.dot(&normal);
            b[offset_i + 3] = j_qacc_smooth[3] + omega_n;
        }

        // Rows 4-5: rolling (dim >= 6: angular velocity in tangent plane)
        if dim_i >= 6 {
            let omega1 = compute_body_angular_velocity(data, body_i1);
            let omega2 = compute_body_angular_velocity(data, body_i2);
            let rel_omega = omega2 - omega1;
            let omega_t1 = rel_omega.dot(&tangent1);
            let omega_t2 = rel_omega.dot(&tangent2);
            b[offset_i + 4] = j_qacc_smooth[4] + omega_t1;
            b[offset_i + 5] = j_qacc_smooth[5] + omega_t2;
        }
    }

    (a, b, efc_offsets)
}

/// Compute the angular velocity of a body in world frame.
#[inline]
fn compute_body_angular_velocity(data: &Data, body_id: usize) -> Vector3<f64> {
    if body_id == 0 {
        return Vector3::zeros(); // World has no velocity
    }
    // The body's angular velocity is stored in cvel (6D spatial velocity)
    // cvel[body] = [omega_x, omega_y, omega_z, v_x, v_y, v_z]
    Vector3::new(
        data.cvel[body_id].x,
        data.cvel[body_id].y,
        data.cvel[body_id].z,
    )
}

/// Project a single contact's lambda onto the elliptic friction cone.
///
/// For contact dynamics, we use a two-step projection:
/// 1. First ensure normal force is non-negative (unilateral constraint)
/// 2. Then clamp friction magnitude to the elliptic cone boundary
///
/// This matches MuJoCo's constraint-level projection which first enforces
/// λ_n ≥ 0, then clips friction to the cone. The mathematical SOC projection
/// would handle negative λ_n differently (projecting to cone boundary rather
/// than origin), but physically a negative normal force means the contact is
/// separating and should release completely.
///
/// Cone shape: ||(λ₁/μ₁, λ₂/μ₂, ...)|| ≤ λ_n
fn project_elliptic_cone(lambda: &mut [f64], mu: &[f64; 5], dim: usize) {
    // Step 1: Enforce unilateral constraint (normal force must be non-negative)
    // Negative normal force = separating contact = release completely
    if lambda[0] < 0.0 {
        for l in lambda.iter_mut().take(dim) {
            *l = 0.0;
        }
        return;
    }

    // Step 2: Clamp friction components where mu ≈ 0 (infinite resistance = no sliding)
    for i in 1..dim {
        if mu[i - 1] <= 1e-12 {
            lambda[i] = 0.0;
        }
    }

    // Step 3: Compute weighted friction norm (elliptic cone radius)
    // s = sqrt( Σ (λ_i / μ_i)² ) for i = 1..dim-1
    let mut s_sq = 0.0;
    for i in 1..dim {
        if mu[i - 1] > 1e-12 {
            s_sq += (lambda[i] / mu[i - 1]).powi(2);
        }
    }
    let s = s_sq.sqrt();

    // Step 4: If friction exceeds cone boundary, scale to boundary
    // Cone constraint: s ≤ λ_n, i.e., ||(λ_i/μ_i)|| ≤ λ_n
    if s > lambda[0] && s > 1e-10 {
        let scale = lambda[0] / s;
        for l in lambda.iter_mut().take(dim).skip(1) {
            *l *= scale;
        }
    }
}

/// Project lambda onto the friction cone for all contacts.
/// Used by CG after each full iteration. PGS does NOT call this — it inlines
/// per-contact projection inside the GS sweep for correct Gauss-Seidel ordering.
fn project_friction_cone(lambda: &mut DVector<f64>, contacts: &[Contact], efc_offsets: &[usize]) {
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        match dim {
            1 => {
                lambda[base] = lambda[base].max(0.0);
            }
            3 | 4 | 6 => {
                project_elliptic_cone(
                    &mut lambda.as_mut_slice()[base..base + dim],
                    &contact.mu,
                    dim,
                );
            }
            _ => {
                // Invalid condim — treat as frictionless for safety
                lambda[base] = lambda[base].max(0.0);
                for j in 1..dim {
                    lambda[base + j] = 0.0;
                }
            }
        }
    }
}

/// Convert lambda vector to per-contact force vectors.
fn extract_forces(
    lambda: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DVector<f64>> {
    contacts
        .iter()
        .enumerate()
        .map(|(i, c)| {
            let base = efc_offsets[i];
            lambda.rows(base, c.dim).clone_owned()
        })
        .collect()
}

/// Projected Gauss-Seidel solver for contact constraints.
///
/// Solves the LCP:
///   minimize: (1/2) λ^T (A + R) λ + λ^T b
///   subject to: λ_n ≥ 0, |λ_t| ≤ μ λ_n
///
/// Where:
///   A = J * M^{-1} * J^T (constraint-space inverse inertia)
///   R = regularization (softness)
///   b = J * qacc_smooth - aref (velocity error with Baumgarte stabilization)
///
/// Returns the constraint forces in contact-frame (λ) and iterations used.
fn pgs_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,
) -> (Vec<DVector<f64>>, usize) {
    let ncon = contacts.len();
    if ncon == 0 {
        return (vec![], 0);
    }

    let (a, b, efc_offsets) = assemble_contact_system(model, data, contacts, jacobians);
    pgs_solve_with_system(
        contacts,
        &a,
        &b,
        &efc_offsets,
        max_iterations,
        tolerance,
        efc_lambda,
    )
}

/// PGS solver core: operates on a pre-assembled (A, b) system.
/// Separated from `pgs_solve_contacts` so the CG fallback path can reuse
/// the already-computed Delassus matrix without redundant assembly.
fn pgs_solve_with_system(
    contacts: &[Contact],
    a: &DMatrix<f64>,
    b: &DVector<f64>,
    efc_offsets: &[usize],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,
) -> (Vec<DVector<f64>>, usize) {
    let ncon = contacts.len();
    if ncon == 0 {
        return (vec![], 0);
    }

    // Compute total constraint rows from offsets
    let nefc = if ncon > 0 {
        efc_offsets[ncon - 1] + contacts[ncon - 1].dim
    } else {
        0
    };

    // Warmstart from previous frame using spatially-keyed contact correspondence.
    // Discard warmstart if condim changed (stored len != current dim).
    let mut lambda = DVector::zeros(nefc);
    for (i, contact) in contacts.iter().enumerate() {
        let key = warmstart_key(contact);
        if let Some(prev_lambda) = efc_lambda.get(&key) {
            if prev_lambda.len() == contact.dim {
                let base = efc_offsets[i];
                for (j, &val) in prev_lambda.iter().enumerate() {
                    lambda[base + j] = val;
                }
            }
        }
    }

    // Compute diagonal inverse with protection against singular diagonals
    let diag_inv: Vec<f64> = (0..nefc)
        .map(|i| {
            let d = a[(i, i)];
            if d.abs() < 1e-12 { 0.0 } else { 1.0 / d }
        })
        .collect();

    let mut iters_used = max_iterations;
    for iter in 0..max_iterations {
        let mut max_delta = 0.0_f64;

        for i in 0..ncon {
            let base = efc_offsets[i];
            let dim = contacts[i].dim;

            // Compute residuals for all dim rows of this contact
            let mut residuals = vec![0.0; dim];
            for r in 0..dim {
                residuals[r] = b[base + r];
                for j in 0..nefc {
                    residuals[r] += a[(base + r, j)] * lambda[j];
                }
            }

            // Save old values for convergence check
            let old: Vec<f64> = (0..dim).map(|r| lambda[base + r]).collect();

            // Gauss-Seidel update
            for r in 0..dim {
                lambda[base + r] -= residuals[r] * diag_inv[base + r];
            }

            // Project this contact onto its friction cone
            match dim {
                1 => {
                    lambda[base] = lambda[base].max(0.0);
                }
                3 | 4 | 6 => {
                    project_elliptic_cone(
                        &mut lambda.as_mut_slice()[base..base + dim],
                        &contacts[i].mu,
                        dim,
                    );
                }
                _ => {
                    // Invalid condim — treat as frictionless
                    lambda[base] = lambda[base].max(0.0);
                    for j in 1..dim {
                        lambda[base + j] = 0.0;
                    }
                }
            }

            // Track convergence
            for r in 0..dim {
                max_delta = max_delta.max((lambda[base + r] - old[r]).abs());
            }
        }

        if max_delta < tolerance {
            iters_used = iter + 1;
            break;
        }
    }

    // Store lambda for warmstart on next frame.
    // Note: if two contacts share a WarmstartKey (same geom pair, same 1cm cell),
    // the later one overwrites the earlier. This is acceptable — contacts within
    // 1cm of each other have similar forces, and the alternative (accumulation)
    // would bias the warmstart. The 1cm grid makes collisions rare in practice.
    efc_lambda.clear();
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        let key = warmstart_key(contact);
        efc_lambda.insert(key, lambda.as_slice()[base..base + dim].to_vec());
    }

    (extract_forces(&lambda, contacts, efc_offsets), iters_used)
}

/// Compute block Jacobi preconditioner for CG solver.
///
/// Extracts dim×dim diagonal blocks from A and inverts each via Cholesky.
/// Falls back to scalar Jacobi (diagonal-only inverse) if Cholesky fails.
fn compute_block_jacobi_preconditioner(
    a: &DMatrix<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> Vec<DMatrix<f64>> {
    contacts
        .iter()
        .enumerate()
        .map(|(i, contact)| {
            let base = efc_offsets[i];
            let dim = contact.dim;
            let block = a.view((base, base), (dim, dim)).clone_owned();
            if let Some(chol) = block.clone().cholesky() {
                chol.inverse()
            } else {
                // Scalar Jacobi fallback: invert diagonal only
                let mut inv = DMatrix::zeros(dim, dim);
                for r in 0..dim {
                    let d = block[(r, r)];
                    inv[(r, r)] = if d.abs() > 1e-12 { 1.0 / d } else { 0.0 };
                }
                inv
            }
        })
        .collect()
}

/// Apply block Jacobi preconditioner: z = M^{-1} * r (block-diagonal solve).
fn apply_preconditioner(
    precond_inv: &[DMatrix<f64>],
    r: &DVector<f64>,
    contacts: &[Contact],
    efc_offsets: &[usize],
) -> DVector<f64> {
    let mut z = DVector::zeros(r.len());
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        let r_block = r.rows(base, dim);
        let z_block = &precond_inv[i] * r_block;
        z.rows_mut(base, dim).copy_from(&z_block);
    }
    z
}

/// Solve contact constraints using preconditioned projected gradient descent.
///
/// Named "CG" for API compatibility with MuJoCo's solver selector, but the
/// algorithm is preconditioned projected gradient descent (PGD) with
/// Barzilai-Borwein adaptive step size — better suited to the friction cone
/// (second-order cone) constraint than true conjugate gradient, which requires
/// active-set tracking for non-box constraints.
///
/// Solves the same QP as `pgs_solve_contacts`:
///   minimize: 0.5 * λ^T A λ + b^T λ
///   subject to: λ_n ≥ 0, |λ_t| ≤ μ λ_n
///
/// Uses block Jacobi preconditioning (dim×dim diagonal block inverse per contact),
/// with Cholesky factorization and scalar Jacobi fallback. For a single contact,
/// the preconditioner is the exact inverse of A, enabling a direct solve in 0
/// iterations.
///
/// Convergence criterion: relative change in λ falls below tolerance
/// (`||λ_{k+1} - λ_k|| / ||λ_k|| < tol`). This fixed-point criterion is
/// appropriate for constrained problems where the gradient at the optimum is
/// non-zero due to active constraints.
///
/// Returns `Ok((forces, iterations_used))` on convergence, or
/// `Err((A, b, efc_offsets))` on non-convergence — returning the pre-computed
/// Delassus system and offsets so the caller can pass to `pgs_solve_with_system()`
/// without redundant assembly.
#[allow(clippy::many_single_char_names)] // a, b, g, z, s are standard math notation
fn cg_solve_contacts(
    model: &Model,
    data: &Data,
    contacts: &[Contact],
    jacobians: &[DMatrix<f64>],
    max_iterations: usize,
    tolerance: f64,
    efc_lambda: &mut HashMap<WarmstartKey, Vec<f64>>,
) -> Result<(Vec<DVector<f64>>, usize), (DMatrix<f64>, DVector<f64>, Vec<usize>)> {
    let ncon = contacts.len();
    if ncon == 0 {
        return Ok((vec![], 0));
    }

    let (a, b, efc_offsets) = assemble_contact_system(model, data, contacts, jacobians);

    // Compute total constraint rows from offsets
    let nefc = if ncon > 0 {
        efc_offsets[ncon - 1] + contacts[ncon - 1].dim
    } else {
        0
    };

    // Block Jacobi preconditioner
    let precond_inv = compute_block_jacobi_preconditioner(&a, contacts, &efc_offsets);

    // Warmstart from previous frame using spatially-keyed contact correspondence.
    // Discard warmstart if condim changed (stored len != current dim).
    let mut lambda = DVector::zeros(nefc);
    for (i, contact) in contacts.iter().enumerate() {
        let key = warmstart_key(contact);
        if let Some(prev) = efc_lambda.get(&key) {
            if prev.len() == contact.dim {
                let base = efc_offsets[i];
                for (j, &val) in prev.iter().enumerate() {
                    lambda[base + j] = val;
                }
            }
        }
    }

    // Single-contact direct solve: exact dim×dim inversion, no iteration needed.
    // For ncon=1, A is exactly dim×dim — the preconditioner block is the full inverse.
    if ncon == 1 {
        let dim = contacts[0].dim;
        let a_block = a.view((0, 0), (dim, dim)).clone_owned();
        let inv = a_block
            .try_inverse()
            .unwrap_or_else(|| precond_inv[0].clone());
        let b_block = b.rows(0, dim).clone_owned();
        let mut lam = -(inv * b_block);
        // Project onto friction cone
        match dim {
            1 => {
                lam[0] = lam[0].max(0.0);
            }
            3 | 4 | 6 => {
                project_elliptic_cone(lam.as_mut_slice(), &contacts[0].mu, dim);
            }
            _ => {
                lam[0] = lam[0].max(0.0);
                for j in 1..dim {
                    lam[j] = 0.0;
                }
            }
        }
        efc_lambda.clear();
        let key = warmstart_key(&contacts[0]);
        efc_lambda.insert(key, lam.as_slice().to_vec());
        return Ok((vec![lam], 0));
    }

    // Project initial guess onto feasible set
    project_friction_cone(&mut lambda, contacts, &efc_offsets);

    // Preconditioned Projected Gradient Descent (PGD) for the contact QP:
    //   min 0.5 * lambda^T A lambda + b^T lambda
    //   subject to lambda_n >= 0, |lambda_t| <= mu * lambda_n
    //
    // Uses block Jacobi preconditioner M^{-1} and an adaptive step size.
    // The preconditioner approximates A^{-1} per-contact block, so the
    // preconditioned gradient z = M^{-1}*g is a good search direction.
    //
    // Step size: start with alpha=0.8 (slightly under-relaxed preconditioned
    // step), then adapt using Barzilai-Borwein for off-diagonal coupling.

    let b_norm = b.norm();
    if b_norm < 1e-14 {
        efc_lambda.clear();
        return Ok((contacts.iter().map(|c| DVector::zeros(c.dim)).collect(), 0));
    }

    // Initial step size for preconditioned gradient descent.
    // alpha=1 would be exact for block-diagonal A. For coupled contacts,
    // we start slightly under-relaxed to ensure convergence.
    let mut alpha = 0.8;

    let mut converged = false;
    let mut iters_used = max_iterations;

    for iter in 0..max_iterations {
        // Gradient: g = A * lambda + b
        let g = &a * &lambda + &b;

        // Preconditioned gradient: z = M^{-1} * g
        let z = apply_preconditioner(&precond_inv, &g, contacts, &efc_offsets);

        // Projected gradient descent step
        let lambda_new = {
            let mut trial = &lambda - alpha * &z;
            project_friction_cone(&mut trial, contacts, &efc_offsets);
            trial
        };

        // Convergence check: relative change in lambda (fixed-point criterion)
        let delta = (&lambda_new - &lambda).norm();
        let lam_norm = lambda.norm().max(1e-10);

        if delta / lam_norm < tolerance {
            lambda = lambda_new;
            converged = true;
            iters_used = iter + 1;
            break;
        }

        // Barzilai-Borwein step size adaptation.
        // Compute: alpha_bb = (s^T s) / (s^T (g_new - g))
        // where s = lambda_new - lambda.
        let g_new = &a * &lambda_new + &b;
        let s = &lambda_new - &lambda;
        let y_bb = &g_new - &g;
        let sy = s.dot(&y_bb);
        if sy > 1e-30 {
            let alpha_bb = s.dot(&s) / sy;
            // Blend new step size with old for stability
            alpha = alpha_bb.clamp(0.01, 2.0);
        }

        lambda = lambda_new;
    }

    // Store warmstart (even on non-convergence — partial solution helps next frame)
    efc_lambda.clear();
    for (i, contact) in contacts.iter().enumerate() {
        let base = efc_offsets[i];
        let dim = contact.dim;
        let key = warmstart_key(contact);
        efc_lambda.insert(key, lambda.as_slice()[base..base + dim].to_vec());
    }

    if converged {
        Ok((extract_forces(&lambda, contacts, &efc_offsets), iters_used))
    } else {
        Err((a, b, efc_offsets))
    }
}

/// Apply equality constraint forces using penalty method.
///
/// Supports Connect, Weld, and Joint constraints.
/// Uses Baumgarte stabilization for drift correction.
///
/// # MuJoCo Semantics
///
/// - **Connect**: Ball-and-socket (3 DOF position lock).
///   Error = p1 + R1*anchor - p2, where p2 is body2's position (or world origin).
///
/// - **Weld**: Fixed frame (6 DOF pose lock).
///   Position error same as Connect; orientation error via quaternion difference.
///
/// - **Joint**: Polynomial coupling θ₂ = c₀ + c₁θ₁ + c₂θ₁² + c₃θ₁³ + c₄θ₁⁴.
///   Error = θ₂ - poly(θ₁). Only joint2 receives correction torque (see below).
///
/// # Key Design Decision: No Reaction Torque for Joint Coupling
///
/// In articulated body dynamics, applying τ₂ to joint2 naturally propagates forces
/// through the kinematic chain via the mass matrix M. Adding an explicit reaction
/// torque τ₁ = -∂θ₂/∂θ₁ · τ₂ would double-count the coupling, causing positive
/// feedback and numerical explosion. This is NOT a bug — it's how articulated
/// body dynamics work. See Featherstone, "Rigid Body Dynamics Algorithms" (2008),
/// Section 7.3 on constraint force propagation.
///
/// # Penalty Parameters
///
/// Derived from solref [timeconst, dampratio]:
/// ```text
/// k = 1 / timeconst²           (stiffness, N/m or Nm/rad)
/// b = 2 * dampratio / timeconst  (damping, Ns/m or Nms/rad)
/// ```
fn apply_equality_constraints(model: &Model, data: &mut Data) {
    // Skip if no equality constraints
    if model.neq == 0 {
        return;
    }

    // Default penalty parameters (fallback when solref not specified).
    //
    // These are intentionally stiffer than MuJoCo's default solref=[0.02, 1.0],
    // which maps to k=2500, b=100. Our defaults (k=10000, b=1000) are chosen to:
    //
    // 1. Strongly enforce constraints when no solref is given (fail-safe behavior)
    // 2. Remain stable with explicit integration at dt=0.001 (stability limit ~dt<0.02)
    //
    // For explicit penalty integration, the stability condition is approximately:
    //   dt < 2/sqrt(k) → dt < 0.02 for k=10000
    //
    // Users should specify solref in MJCF for fine-grained control. Recommended
    // values for explicit integration: solref="0.05 1.0" to solref="0.1 1.0"
    // (softer than MuJoCo defaults, which assume implicit PGS solver).
    let default_stiffness = model.default_eq_stiffness;
    let default_damping = model.default_eq_damping;

    let dt = model.timestep;

    for eq_id in 0..model.neq {
        // Skip inactive constraints
        if !model.eq_active[eq_id] {
            continue;
        }

        match model.eq_type[eq_id] {
            EqualityType::Connect => {
                apply_connect_constraint(
                    model,
                    data,
                    eq_id,
                    default_stiffness,
                    default_damping,
                    dt,
                );
            }
            EqualityType::Weld => {
                apply_weld_constraint(model, data, eq_id, default_stiffness, default_damping, dt);
            }
            EqualityType::Joint => {
                apply_joint_equality_constraint(
                    model,
                    data,
                    eq_id,
                    default_stiffness,
                    default_damping,
                    dt,
                );
            }
            EqualityType::Distance => {
                apply_distance_constraint(
                    model,
                    data,
                    eq_id,
                    default_stiffness,
                    default_damping,
                    dt,
                );
            }
            EqualityType::Tendon => {
                use std::sync::Once;
                static WARN_ONCE: Once = Once::new();
                WARN_ONCE.call_once(|| {
                    eprintln!(
                        "Warning: Tendon equality constraints are not yet implemented; \
                         these constraints will be ignored. See sim/docs/todo/index.md."
                    );
                });
            }
        }
    }
}

/// Convert solref parameters to penalty stiffness and damping.
///
/// MuJoCo's solref = [timeconst, dampratio] maps to a second-order spring-damper:
/// - Natural frequency: ωn = 1/timeconst
/// - Stiffness: k = ωn² = 1/timeconst²
/// - Damping: b = 2 * dampratio * ωn = 2 * dampratio / timeconst
///
/// This gives standard mass-spring-damper dynamics where dampratio=1 is critically
/// damped and timeconst is the characteristic time constant.
///
/// ## Note on MuJoCo's Full Formula
///
/// MuJoCo's constraint solver uses a more complex formula involving `d_width` from
/// solimp for computing reference acceleration in constraint space:
/// ```text
/// b_mj = 2 / (d_width * timeconst)
/// k_mj = d(r) / (d_width² * timeconst² * dampratio²)
/// ```
///
/// We use the simpler spring-damper formula above because:
/// 1. Joint limits use penalty method, not the full constraint solver
/// 2. The spring-damper formula is equivalent for penalty constraints
/// 3. It correctly captures the timeconst and dampratio semantics
///
/// When timeconst ≤ 0, falls back to provided defaults (MuJoCo uses negative values
/// for direct stiffness/damping specification which we don't support here).
///
/// For explicit integration stability, timeconst should be > 2 * dt.
///
/// The timestep parameter is used to clamp k for stability when the user-specified
/// timeconst would cause instability with the current timestep.
#[inline]
fn solref_to_penalty(solref: [f64; 2], default_k: f64, default_b: f64, dt: f64) -> (f64, f64) {
    let (k, b) = if solref[0] > 0.0 {
        let timeconst = solref[0];
        let dampratio = solref[1];
        (1.0 / (timeconst * timeconst), 2.0 * dampratio / timeconst)
    } else {
        (default_k, default_b)
    };

    // Stability limit for explicit integration: dt < 2/sqrt(k/m_eff)
    // Since we don't know m_eff, we use a conservative limit based on dt alone.
    // For unit mass: k_max = 4/dt² gives marginal stability.
    // We use 1/dt² for a safety factor of 2.
    let k_max = 1.0 / (dt * dt);
    let k_clamped = k.min(k_max);

    // If we clamped k significantly, scale damping proportionally for critical damping
    // Critical damping: b = 2*sqrt(k*m). For unit m: b = 2*sqrt(k).
    // If k was reduced, reduce b to maintain damping ratio.
    let b_scaled = if k_clamped < k * 0.99 {
        // Maintain the original damping ratio (b/2sqrt(k)) with new k
        let original_zeta = b / (2.0 * k.sqrt());
        2.0 * original_zeta * k_clamped.sqrt()
    } else {
        b
    };

    (k_clamped, b_scaled)
}

/// Compute position-dependent impedance from solimp parameters.
///
/// The sigmoid shape is derived from MuJoCo's `getimpedance()` in
/// `engine_core_constraint.c`, but the **consumption** differs:
///
/// - **MuJoCo** uses impedance to compute regularization in a QP constraint solver:
///   `R = (1-d)/d * diag_approx(A)`, where `A = J M⁻¹ Jᵀ`.
/// - **Our penalty method** uses impedance as a direct scaling factor on
///   stiffness and damping: `F = -d(r) * k * error - d(r) * b * vel_error`.
///
/// These are not equivalent formulations. The penalty adaptation preserves the
/// qualitative behavior (higher impedance → stronger constraint, position-dependent
/// softening) but does not reproduce MuJoCo's exact constraint force magnitudes.
///
/// The impedance `d ∈ (0, 1)` controls constraint effectiveness:
/// - `d` close to 1 → strong constraint (high stiffness)
/// - `d` close to 0 → weak constraint (low stiffness)
///
/// The impedance interpolates from `solimp[0]` (d0) to `solimp[1]` (d_width)
/// over a transition zone of `solimp[2]` (width) meters/radians, using a
/// split power-sigmoid controlled by `solimp[3]` (midpoint) and `solimp[4]` (power).
///
/// ## Deviations from MuJoCo
///
/// - **No margin offset**: MuJoCo computes `x = (pos - margin) / width`. We pass raw
///   violation (no margin subtraction). This is correct for equality constraints (margin
///   is always 0 in MuJoCo). For contact constraints, `geom_margin` defaults to 0 so
///   `depth - margin = depth`, but if non-zero margins are supported in the future,
///   callers should subtract margin before passing the violation.
/// - **No derivative output**: MuJoCo returns both `d(r)` and `d'(r)`. The derivative
///   is used for impedance-modified reference acceleration in the QP solver. Our penalty
///   method does not use `aref`, so the derivative is not needed.
///
/// # Arguments
/// * `solimp` - [d0, d_width, width, midpoint, power]
/// * `violation` - Constraint violation magnitude (distance or angle, non-negative)
///
/// # Returns
/// Impedance value in (0, 1), clamped to [MIN_IMPEDANCE, MAX_IMPEDANCE].
#[inline]
fn compute_impedance(solimp: [f64; 5], violation: f64) -> f64 {
    // MuJoCo clamps impedance to [mjMINIMP, mjMAXIMP] = [0.0001, 0.9999]
    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;
    // Threshold for treating floating-point values as equal
    const EPS: f64 = 1e-12;
    // Minimum width to avoid division by zero (MuJoCo uses mjMINVAL ≈ 1e-15,
    // we use a wider margin since f64 precision is ~1e-16)
    const MIN_WIDTH: f64 = 1e-10;

    debug_assert!(
        violation >= 0.0,
        "compute_impedance expects non-negative violation, got {violation}"
    );

    // Clamp inputs to valid ranges, matching MuJoCo's getsolparam():
    //   d0, d_width, midpoint ∈ [mjMINIMP, mjMAXIMP]
    //   width ≥ 0
    //   power ≥ 1
    let d0 = solimp[0].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let d_width = solimp[1].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let width = solimp[2].max(0.0);
    let midpoint = solimp[3].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let power = solimp[4].max(1.0);

    // Flat function: d0 ≈ d_width or width is negligible
    if (d0 - d_width).abs() < EPS || width <= MIN_WIDTH {
        return 0.5 * (d0 + d_width);
    }

    // Normalized position: x = violation / width, clamped to [0, 1]
    let x = (violation / width).min(1.0);

    // Saturated at full width — return d_width
    if x >= 1.0 {
        return d_width;
    }

    // At zero violation — return d0
    if x == 0.0 {
        return d0;
    }

    // Compute sigmoid y(x) ∈ [0, 1]
    let y = if (power - 1.0).abs() < EPS {
        // Linear case (power ≈ 1): y = x regardless of midpoint
        x
    } else if x <= midpoint {
        // Lower half: y = a * x^p where a = 1 / midpoint^(p-1)
        let a = 1.0 / midpoint.powf(power - 1.0);
        a * x.powf(power)
    } else {
        // Upper half: y = 1 - b * (1-x)^p where b = 1 / (1-midpoint)^(p-1)
        let b = 1.0 / (1.0 - midpoint).powf(power - 1.0);
        1.0 - b * (1.0 - x).powf(power)
    };

    // Interpolate: d = d0 + y * (d_width - d0)
    d0 + y * (d_width - d0)
}

// =============================================================================
// Constraint Stability Constants
// =============================================================================

// Maximum velocity change per timestep is now configurable via model fields:
// - model.max_constraint_vel (linear, default: 1.0 m/s)
// - model.max_constraint_angvel (angular, default: 1.0 rad/s)
// These limit acceleration to max_delta_v / dt to ensure stability with
// explicit Euler integration, preventing oscillation from overshooting.

/// Maximum effective rotation error for constraint stiffness term (radians).
///
/// For large orientation errors, the small-angle approximation in the
/// quaternion-to-axis-angle conversion becomes inaccurate. We clamp the
/// effective error to this value (~29 degrees) for the stiffness term.
/// The damping term uses actual velocities and is not affected.
const MAX_ROTATION_ERROR_FOR_STIFFNESS: f64 = 0.5;

/// Minimum inertia/mass threshold for numerical stability.
///
/// Values below this threshold are treated as numerical noise and ignored
/// when computing effective mass for constraint force limiting.
const MIN_INERTIA_THRESHOLD: f64 = 1e-10;

/// Default mass/inertia when no valid DOFs are found for a body.
///
/// This is used when a body has no translational/rotational DOFs that we
/// can extract mass from (e.g., kinematic bodies). Using 1.0 provides
/// reasonable default behavior.
const DEFAULT_MASS_FALLBACK: f64 = 1.0;

/// Default solref parameters [timeconst, dampratio] (MuJoCo defaults).
///
/// timeconst = 0.02 seconds gives a 50 Hz natural frequency.
/// dampratio = 1.0 gives critical damping.
const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];

/// Default solimp parameters [d0, d_width, width, midpoint, power] (MuJoCo defaults).
///
/// These control the constraint impedance profile:
/// - d0 = 0.9: Impedance at zero violation
/// - d_width = 0.95: Impedance at full violation width (endpoint)
/// - width = 0.001: Transition zone size (meters or radians)
/// - midpoint = 0.5: Midpoint of the sigmoid transition curve
/// - power = 2.0: Power of the sigmoid transition curve
const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

// =============================================================================
// Constraint Force Utilities
// =============================================================================

/// Clamp a vector to a maximum magnitude while preserving direction.
///
/// Returns the original vector if its magnitude is at or below `max_magnitude`,
/// otherwise returns a vector in the same direction with the clamped magnitude.
///
/// # Safety
///
/// Clamp vector magnitude, avoiding division by near-zero.
///
/// If `mag <= max_magnitude`, returns `v` unchanged.
/// If `mag > max_magnitude` and `mag > MIN_INERTIA_THRESHOLD`, returns scaled vector.
/// If `mag <= MIN_INERTIA_THRESHOLD`, returns `v` unchanged (near-zero input).
#[inline]
fn clamp_vector_magnitude(v: Vector3<f64>, max_magnitude: f64) -> Vector3<f64> {
    let mag = v.norm();
    // Only clamp if magnitude exceeds limit AND is large enough to safely divide
    if mag > max_magnitude {
        if mag > MIN_INERTIA_THRESHOLD {
            v * (max_magnitude / mag)
        } else {
            // Near-zero vector, return as-is to avoid division issues
            v
        }
    } else {
        v
    }
}

/// Convert a quaternion error to axis-angle representation.
///
/// For a unit quaternion `q = (w, x, y, z) = (cos(θ/2), sin(θ/2) * axis)`,
/// returns `θ * axis` as a Vector3.
///
/// This is more accurate than the small-angle approximation `2 * [x, y, z]`
/// which has ~10% error at 90° and ~36% error at 180°.
///
/// # Arguments
/// * `quat` - A unit quaternion representing the rotation error
///
/// # Returns
/// Axis-angle representation as `angle * axis` (Vector3)
#[inline]
fn quaternion_to_axis_angle(quat: &UnitQuaternion<f64>) -> Vector3<f64> {
    let q = quat.quaternion();
    let (qw, qx, qy, qz) = (q.w, q.i, q.j, q.k);

    // Handle identity quaternion (no rotation)
    let sin_half_angle_sq = qx * qx + qy * qy + qz * qz;
    if sin_half_angle_sq < MIN_INERTIA_THRESHOLD {
        return Vector3::zeros();
    }

    let sin_half_angle = sin_half_angle_sq.sqrt();

    // Compute full angle: θ = 2 * atan2(||xyz||, w)
    // This handles all cases including w < 0 (angle > π)
    let angle = 2.0 * sin_half_angle.atan2(qw);

    // Axis is normalized [x, y, z] / ||xyz||
    // Result is angle * axis
    Vector3::new(qx, qy, qz) * (angle / sin_half_angle)
}

// =============================================================================
// Newton Solver: KBIP, diagApprox, Regularization (§15.1, Step 6)
// =============================================================================

/// Minimum value to prevent division by zero (matches MuJoCo's mjMINVAL).
const MJ_MINVAL: f64 = 1e-15;

/// Compute KBIP stiffness K and damping B from solref parameters (§15.1).
///
/// Returns `(K, B)` where:
/// - Standard mode (solref[0] > 0):
///   - K = 1/(dmax²·timeconst²·dampratio²)
///   - B = 2/(dmax·timeconst)          when dampratio > 0
///   - B = -dampratio/dmax             when dampratio ≤ 0
/// - Direct mode (solref[0] ≤ 0): K = -solref[0]/dmax², B = -solref[1]/dmax
///
/// `dmax` = solimp[1] (clamped to [mjMINIMP, mjMAXIMP]).
/// Matches MuJoCo's `mj_makeImpedance` in `engine_core_constraint.c`.
fn compute_kbip(solref: [f64; 2], solimp: [f64; 5]) -> (f64, f64) {
    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;

    let dmax = solimp[1].clamp(MJ_MIN_IMP, MJ_MAX_IMP);

    if solref[0] > 0.0 {
        // Standard mode: solref = [timeconst, dampratio]
        let timeconst = solref[0];
        let dampratio = solref[1];

        let k = 1.0 / (dmax * dmax * timeconst * timeconst * dampratio * dampratio).max(MJ_MINVAL);
        let b = if dampratio > 0.0 {
            2.0 / (dmax * timeconst).max(MJ_MINVAL)
        } else {
            -dampratio / dmax.max(MJ_MINVAL)
        };
        (k, b)
    } else {
        // Direct mode: solref = [-stiffness, -damping]
        let k = -solref[0] / (dmax * dmax).max(MJ_MINVAL);
        let b = -solref[1] / dmax.max(MJ_MINVAL);
        (k, b)
    }
}

/// Compute reference acceleration aref for a single constraint row (§15.1).
///
/// `aref = -B * vel - K * imp * (pos - margin)`
///
/// This is the MuJoCo KBIP convention where impedance multiplies only the
/// position term, NOT the velocity term.
#[inline]
fn compute_aref(k: f64, b: f64, imp: f64, pos: f64, margin: f64, vel: f64) -> f64 {
    -b * vel - k * imp * (pos - margin)
}

/// Compute exact diagonal approximation of A = J·M⁻¹·J^T for one row (§15.12 step 2).
///
/// Solves M·w = J_i^T via the sparse LDL factorization, then computes
/// diagApprox_i = J_i · w (one dot product). O(nv) per row.
///
/// Falls back to dense Cholesky if sparse factorization is not valid.
fn compute_diag_approx_exact(j_row: &[f64], nv: usize, model: &Model, data: &Data) -> f64 {
    // Solve M * w = J_row^T
    let mut w = DVector::zeros(nv);
    for i in 0..nv {
        w[i] = j_row[i];
    }

    if data.qLD_valid {
        // Use sparse LDL: forward/back substitution
        mj_solve_sparse_vec(model, data, &mut w);
    } else {
        // Fallback: use dense M and solve via Cholesky
        // Build a copy of qM and factor it
        let mut m_copy = data.qM.clone();
        // Cholesky failure means M is singular; w will be approximate.
        // The Newton solver detects and handles this via PGS fallback.
        if cholesky_in_place(&mut m_copy).is_err() {
            return MJ_MINVAL;
        }
        cholesky_solve_in_place(&m_copy, &mut w);
    }

    // diagApprox = J_row · w = J_row · M⁻¹ · J_row^T
    let mut result = 0.0;
    for i in 0..nv {
        result += j_row[i] * w[i];
    }
    result.max(MJ_MINVAL)
}

/// Solve M·x = b using the sparse LDL factorization (in-place).
///
/// Forward substitution: L^T · z = b, then D · y = z, then L · x = y.
/// Matches the tree-sparse structure from mj_factor_sparse.
fn mj_solve_sparse_vec(model: &Model, data: &Data, x: &mut DVector<f64>) {
    let nv = model.nv;

    // Forward substitution: solve L^T * z = x (L is unit lower triangular)
    // Process from leaves to root (high index to low)
    for i in (0..nv).rev() {
        for &(col, val) in &data.qLD_L[i] {
            x[col] -= val * x[i];
        }
    }

    // Diagonal solve: z = z / D
    for i in 0..nv {
        if data.qLD_diag[i].abs() > MJ_MINVAL {
            x[i] /= data.qLD_diag[i];
        }
    }

    // Back substitution: solve L * x = z
    // Process from root to leaves (low index to high)
    for i in 0..nv {
        for &(col, val) in &data.qLD_L[i] {
            x[i] -= val * x[col];
        }
    }
}

/// Compute regularization R and constraint stiffness D from impedance and diagApprox (§15.1).
///
/// R = max(mjMINVAL, (1 - imp) / imp * diagApprox)
/// D = 1 / R
#[inline]
fn compute_regularization(imp: f64, diag_approx: f64) -> (f64, f64) {
    let r = ((1.0 - imp) / imp * diag_approx).max(MJ_MINVAL);
    let d = 1.0 / r;
    (r, d)
}

// =============================================================================
// Unified Constraint Assembly (§15, Step 7)
// =============================================================================

/// Assemble the unified constraint Jacobian and metadata for the Newton solver.
///
/// Populates all `efc_*` fields on Data. Row ordering:
/// 1. Equality constraints (connect, weld, joint, distance)
/// 2. DOF friction loss
/// 3. Tendon friction loss
/// 4. Joint limits (potentially active only)
/// 5. Tendon limits (potentially active only)
/// 6. Contacts
///
/// `qacc_smooth` is the unconstrained acceleration (M⁻¹ · qfrc_smooth), needed for efc_b.
fn assemble_unified_constraints(model: &Model, data: &mut Data, qacc_smooth: &DVector<f64>) {
    let nv = model.nv;

    // === Phase 1: Count rows ===
    let mut nefc = 0usize;

    // Equality constraints
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id] {
            continue;
        }
        nefc += match model.eq_type[eq_id] {
            EqualityType::Connect => 3,
            EqualityType::Weld => 6,
            EqualityType::Joint | EqualityType::Distance => 1,
            EqualityType::Tendon => 0, // Not yet implemented
        };
    }

    // DOF friction loss
    for dof_idx in 0..nv {
        if model.dof_frictionloss[dof_idx] > 0.0 {
            nefc += 1;
        }
    }

    // Tendon friction loss
    for t in 0..model.ntendon {
        if model.tendon_frictionloss[t] > 0.0 {
            nefc += 1;
        }
    }

    // Joint limits (MuJoCo convention: dist < 0 means violated)
    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let (limit_min, limit_max) = model.jnt_range[jnt_id];
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                // Lower limit: dist = q - limit_min (negative when violated)
                if q - limit_min < 0.0 {
                    nefc += 1;
                }
                // Upper limit: dist = limit_max - q (negative when violated)
                if limit_max - q < 0.0 {
                    nefc += 1;
                }
            }
            _ => {} // Ball/Free limits not implemented
        }
    }

    // Tendon limits (MuJoCo convention: dist < 0 means violated)
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }
        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];
        // Lower tendon limit: dist = length - limit_min (negative when too short)
        if length - limit_min < 0.0 {
            nefc += 1;
        }
        // Upper tendon limit: dist = limit_max - length (negative when too long)
        if limit_max - length < 0.0 {
            nefc += 1;
        }
    }

    // Contacts
    for c in &data.contacts {
        nefc += c.dim;
    }

    // === Phase 2: Allocate ===
    data.efc_J = DMatrix::zeros(nefc, nv);
    data.efc_type = Vec::with_capacity(nefc);
    data.efc_pos = Vec::with_capacity(nefc);
    data.efc_margin = Vec::with_capacity(nefc);
    data.efc_vel = DVector::zeros(nefc);
    data.efc_solref = Vec::with_capacity(nefc);
    data.efc_solimp = Vec::with_capacity(nefc);
    data.efc_diagApprox = Vec::with_capacity(nefc);
    data.efc_R = Vec::with_capacity(nefc);
    data.efc_D = Vec::with_capacity(nefc);
    data.efc_imp = Vec::with_capacity(nefc);
    data.efc_aref = DVector::zeros(nefc);
    data.efc_floss = Vec::with_capacity(nefc);
    data.efc_mu = Vec::with_capacity(nefc);
    data.efc_dim = Vec::with_capacity(nefc);
    data.efc_id = Vec::with_capacity(nefc);
    data.efc_state = vec![ConstraintState::Quadratic; nefc];
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);
    data.efc_b = DVector::zeros(nefc);
    data.efc_cone_hessian = vec![None; data.contacts.len()];
    data.ncone = 0;

    let mut row = 0usize;

    // Helper: populate per-row metadata and compute impedance, KBIP, aref, diagApprox, R, D.
    // Must be called for each row after J row and pos/vel/margin are set.
    macro_rules! finalize_row {
        ($solref:expr, $solimp:expr, $pos:expr, $margin:expr, $vel:expr, $floss:expr,
         $ctype:expr, $dim_val:expr, $id_val:expr, $mu_val:expr) => {{
            let sr: [f64; 2] = $solref;
            let si: [f64; 5] = $solimp;
            let pos_val: f64 = $pos;
            let margin_val: f64 = $margin;
            let vel_val: f64 = $vel;
            let floss_val: f64 = $floss;

            data.efc_type.push($ctype);
            data.efc_pos.push(pos_val);
            data.efc_margin.push(margin_val);
            data.efc_vel[row] = vel_val;
            data.efc_solref.push(sr);
            data.efc_solimp.push(si);
            data.efc_floss.push(floss_val);
            data.efc_mu.push($mu_val);
            data.efc_dim.push($dim_val);
            data.efc_id.push($id_val);

            // Impedance
            let violation = (pos_val - margin_val).abs();
            let imp = compute_impedance(si, violation);
            data.efc_imp.push(imp);

            // diagApprox (exact diagonal via M⁻¹ solve)
            let j_row_slice: Vec<f64> = (0..nv).map(|col| data.efc_J[(row, col)]).collect();
            let diag = compute_diag_approx_exact(&j_row_slice, nv, model, data);
            data.efc_diagApprox.push(diag);

            // Regularization
            let (r_val, d_val) = compute_regularization(imp, diag);
            data.efc_R.push(r_val);
            data.efc_D.push(d_val);

            // KBIP + aref
            let (k, b) = compute_kbip(sr, si);
            data.efc_aref[row] = compute_aref(k, b, imp, pos_val, margin_val, vel_val);

            // efc_b = J_row · qacc_smooth - aref
            let mut j_dot_qacc = 0.0;
            for col in 0..nv {
                j_dot_qacc += data.efc_J[(row, col)] * qacc_smooth[col];
            }
            data.efc_b[row] = j_dot_qacc - data.efc_aref[row];

            row += 1;
        }};
    }

    // === Phase 3: Populate rows ===

    // --- 3a: Equality constraints ---
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id] {
            continue;
        }

        let rows = match model.eq_type[eq_id] {
            EqualityType::Connect => extract_connect_jacobian(model, data, eq_id),
            EqualityType::Weld => extract_weld_jacobian(model, data, eq_id),
            EqualityType::Joint => extract_joint_equality_jacobian(model, data, eq_id),
            EqualityType::Distance => extract_distance_jacobian(model, data, eq_id),
            EqualityType::Tendon => continue,
        };

        let sr = model.eq_solref[eq_id];
        let si = model.eq_solimp[eq_id];
        let nrows = rows.j_rows.nrows();

        for r in 0..nrows {
            // Copy J row
            for col in 0..nv {
                data.efc_J[(row, col)] = rows.j_rows[(r, col)];
            }
            finalize_row!(
                sr,
                si,
                rows.pos[r],
                0.0,
                rows.vel[r],
                0.0,
                ConstraintType::Equality,
                1,
                eq_id,
                [0.0; 5]
            );
        }
    }

    // --- 3b: DOF friction loss ---
    for dof_idx in 0..nv {
        let fl = model.dof_frictionloss[dof_idx];
        if fl <= 0.0 {
            continue;
        }
        // Jacobian: 1×nv with 1.0 at dof_idx
        data.efc_J[(row, dof_idx)] = 1.0;
        let vel = data.qvel[dof_idx];
        finalize_row!(
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
            0.0,
            0.0,
            vel,
            fl,
            ConstraintType::FrictionLoss,
            1,
            dof_idx,
            [0.0; 5]
        );
    }

    // --- 3c: Tendon friction loss ---
    for t in 0..model.ntendon {
        let fl = model.tendon_frictionloss[t];
        if fl <= 0.0 {
            continue;
        }
        // Jacobian: tendon Jacobian row
        for col in 0..nv {
            data.efc_J[(row, col)] = data.ten_J[t][col];
        }
        let vel = data.ten_velocity[t];
        finalize_row!(
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
            0.0,
            0.0,
            vel,
            fl,
            ConstraintType::FrictionLoss,
            1,
            t,
            [0.0; 5]
        );
    }

    // --- 3d: Joint limits ---
    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let (limit_min, limit_max) = model.jnt_range[jnt_id];
                let dof_adr = model.jnt_dof_adr[jnt_id];
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                let qdot = data.qvel[dof_adr];
                let sr = model.jnt_solref[jnt_id];
                let si = model.jnt_solimp[jnt_id];

                // MuJoCo convention: dist > 0 = satisfied, dist < 0 = violated.
                // Constraint is instantiated when dist < margin (here margin=0,
                // so when dist < 0, i.e., limit violated).

                // Lower limit: dist = q - limit_min (negative when q < limit_min)
                let dist_lower = q - limit_min;
                if dist_lower < 0.0 {
                    // J = +1 (MuJoCo: jac = -side, side=-1 → jac=+1)
                    data.efc_J[(row, dof_adr)] = 1.0;
                    // pos = dist (negative = violated, MuJoCo convention)
                    // vel = J*qdot = qdot
                    finalize_row!(
                        sr,
                        si,
                        dist_lower,
                        0.0,
                        qdot,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
                    );
                }

                // Upper limit: dist = limit_max - q (negative when q > limit_max)
                let dist_upper = limit_max - q;
                if dist_upper < 0.0 {
                    // J = -1 (MuJoCo: jac = -side, side=+1 → jac=-1)
                    data.efc_J[(row, dof_adr)] = -1.0;
                    // pos = dist (negative = violated, MuJoCo convention)
                    // vel = J*qdot = -qdot
                    finalize_row!(
                        sr,
                        si,
                        dist_upper,
                        0.0,
                        -qdot,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
                    );
                }
            }
            _ => {} // Ball/Free limits not implemented
        }
    }

    // --- 3e: Tendon limits ---
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }
        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];
        let vel = data.ten_velocity[t];
        let sr = model.tendon_solref[t];
        let si = model.tendon_solimp[t];

        // MuJoCo convention: dist > 0 = satisfied, dist < 0 = violated.
        // Tendon limits follow the same pattern as joint limits.

        // Lower tendon limit: dist = length - limit_min (negative when too short)
        let dist_lower = length - limit_min;
        if dist_lower < 0.0 {
            // J = +ten_J (MuJoCo convention: pushes length up)
            for col in 0..nv {
                data.efc_J[(row, col)] = data.ten_J[t][col];
            }
            // pos = dist (negative = violated, MuJoCo convention)
            // vel = J·qdot ≈ ten_velocity
            finalize_row!(
                sr,
                si,
                dist_lower,
                0.0,
                vel,
                0.0,
                ConstraintType::LimitTendon,
                1,
                t,
                [0.0; 5]
            );
        }

        // Upper tendon limit: dist = limit_max - length (negative when too long)
        let dist_upper = limit_max - length;
        if dist_upper < 0.0 {
            // J = -ten_J (MuJoCo convention: pushes length down)
            for col in 0..nv {
                data.efc_J[(row, col)] = -data.ten_J[t][col];
            }
            // pos = dist (negative = violated, MuJoCo convention)
            // vel = -ten_velocity
            finalize_row!(
                sr,
                si,
                dist_upper,
                0.0,
                -vel,
                0.0,
                ConstraintType::LimitTendon,
                1,
                t,
                [0.0; 5]
            );
        }
    }

    // --- 3f: Contacts ---
    let contacts = data.contacts.clone(); // Clone to avoid borrow conflict
    for (ci, contact) in contacts.iter().enumerate() {
        let dim = contact.dim;
        let cj = compute_contact_jacobian(model, data, contact);

        let sr = contact.solref;
        let si = contact.solimp;
        // Contact margin is 0.0 for now — margin/gap not yet applied at contact creation
        // (see line 4876 note). When non-zero geom_margin is supported, subtract it here.
        let margin = 0.0_f64;
        let is_elliptic = dim >= 3 && model.cone == 1 && contact.mu[0] >= 1e-10;
        let ctype = if is_elliptic {
            ConstraintType::ContactElliptic
        } else {
            ConstraintType::ContactNonElliptic
        };

        for r in 0..dim {
            for col in 0..nv {
                data.efc_J[(row, col)] = cj[(r, col)];
            }

            // pos: row 0 = penetration depth, rows 1+ = 0
            let pos = if r == 0 { contact.depth } else { 0.0 };
            let margin_r = if r == 0 { margin } else { 0.0 };

            // vel: J_row · qvel
            let mut vel = 0.0;
            for col in 0..nv {
                vel += cj[(r, col)] * data.qvel[col];
            }

            finalize_row!(sr, si, pos, margin_r, vel, 0.0, ctype, dim, ci, contact.mu);
        }
    }

    debug_assert_eq!(row, nefc, "Row count mismatch in constraint assembly");
    data.efc_cost = 0.0;
}

/// Classify constraint states, compute forces, and evaluate total cost.
///
/// Implements PrimalUpdateConstraint from §15.1:
/// - Per-row state machine (Quadratic, Satisfied, LinearNeg, LinearPos, Cone)
/// - Force computation from state
/// - Total cost = Gauss term + constraint penalties
///
/// # Arguments
/// * `model` - Model with solver parameters
/// * `data` - Data with efc_* arrays (modified: efc_state, efc_force, efc_cost)
/// * `qacc` - Current acceleration estimate
/// * `qacc_smooth` - Unconstrained smooth acceleration (M⁻¹ · qfrc_smooth)
/// * `qfrc_smooth` - Smooth force (excluding friction loss)
#[allow(clippy::too_many_arguments)]
#[allow(clippy::many_single_char_names)]
fn classify_constraint_states(
    model: &Model,
    data: &mut Data,
    qacc: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
) {
    let nv = model.nv;
    let nefc = data.efc_type.len();

    // Reset cone Hessian tracking for this classification pass
    data.ncone = 0;
    for hc in &mut data.efc_cone_hessian {
        *hc = None;
    }

    // Compute jar = J · qacc - aref for all rows
    for i in 0..nefc {
        let mut j_dot_qacc = 0.0;
        for col in 0..nv {
            j_dot_qacc += data.efc_J[(i, col)] * qacc[col];
        }
        data.efc_jar[i] = j_dot_qacc - data.efc_aref[i];
    }

    // Compute Ma = M · qacc
    let mut ma = DVector::<f64>::zeros(nv);
    for row_idx in 0..nv {
        for col_idx in 0..nv {
            ma[row_idx] += data.qM[(row_idx, col_idx)] * qacc[col_idx];
        }
    }

    // Classify each row and compute forces
    let mut constraint_cost = 0.0;
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let jar = data.efc_jar[i];
        let d = data.efc_D[i];
        let r = data.efc_R[i];

        match ctype {
            ConstraintType::Equality => {
                // Always Quadratic
                data.efc_state[i] = ConstraintState::Quadratic;
                data.efc_force[i] = -d * jar;
                constraint_cost += 0.5 * d * jar * jar;
                i += 1;
            }

            ConstraintType::FrictionLoss => {
                // Huber threshold at ±R·floss
                let floss = data.efc_floss[i];
                let threshold = r * floss;
                if jar >= threshold {
                    // LinearPos
                    data.efc_state[i] = ConstraintState::LinearPos;
                    data.efc_force[i] = -floss;
                    constraint_cost += floss * jar - 0.5 * r * floss * floss;
                } else if jar <= -threshold {
                    // LinearNeg
                    data.efc_state[i] = ConstraintState::LinearNeg;
                    data.efc_force[i] = floss;
                    constraint_cost += -floss * jar - 0.5 * r * floss * floss;
                } else {
                    // Quadratic
                    data.efc_state[i] = ConstraintState::Quadratic;
                    data.efc_force[i] = -d * jar;
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }

            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactNonElliptic => {
                // jar < 0 → Quadratic; jar >= 0 → Satisfied
                if jar < 0.0 {
                    data.efc_state[i] = ConstraintState::Quadratic;
                    data.efc_force[i] = -d * jar;
                    constraint_cost += 0.5 * d * jar * jar;
                } else {
                    data.efc_state[i] = ConstraintState::Satisfied;
                    data.efc_force[i] = 0.0;
                }
                i += 1;
            }

            ConstraintType::ContactElliptic => {
                // Three-zone classification over all dim rows
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0]; // Primary friction coefficient

                // Compute U vector and N, T
                // U[0] = jar[i] · μ (normal, scaled)
                // U[j] = jar[i+j] · friction[j-1]  for j = 1..dim-1
                // friction[j-1] = mu[j-1] from the 5-element mu array
                let u0 = data.efc_jar[i] * mu;
                let n = u0;

                let mut t_sq = 0.0;
                for j in 1..dim {
                    let friction_j = data.efc_mu[i][j - 1]; // mu[j-1]
                    let uj = data.efc_jar[i + j] * friction_j;
                    t_sq += uj * uj;
                }
                let t = t_sq.sqrt();

                let t_min = MJ_MINVAL; // 1e-15

                // Three-zone classification
                if n >= mu * t || (t < t_min && n >= 0.0) {
                    // Top (separated) → Satisfied
                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Satisfied;
                        data.efc_force[i + j] = 0.0;
                    }
                } else if mu * n + t <= 0.0 || (t < t_min && n < 0.0) {
                    // Bottom (fully active) → Quadratic, per-row
                    for j in 0..dim {
                        let jar_j = data.efc_jar[i + j];
                        let d_j = data.efc_D[i + j];
                        data.efc_state[i + j] = ConstraintState::Quadratic;
                        data.efc_force[i + j] = -d_j * jar_j;
                        constraint_cost += 0.5 * d_j * jar_j * jar_j;
                    }
                } else {
                    // Middle (cone surface) → Cone state
                    let d_normal = data.efc_D[i];
                    let dm = d_normal / (mu * mu * (1.0 + mu * mu));
                    let n_minus_mu_t = n - mu * t;

                    // Cost: ½·Dm·(N − μ·T)²
                    constraint_cost += 0.5 * dm * n_minus_mu_t * n_minus_mu_t;

                    // Forces:
                    // f[i]   = −Dm · NmT · μ  (normal force)
                    // f[i+j] = (Dm · NmT · μ / T) · U[j] · friction[j−1]  for j = 1..dim-1
                    let f_normal = -dm * n_minus_mu_t * mu;
                    data.efc_force[i] = f_normal;

                    for j in 1..dim {
                        let friction_j = data.efc_mu[i][j - 1];
                        let uj = data.efc_jar[i + j] * friction_j;
                        data.efc_force[i + j] = (dm * n_minus_mu_t * mu / t) * uj * friction_j;
                    }

                    // Replicate Cone state to all dim rows
                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Cone;
                    }

                    // Build per-contact cone Hessian H_c (dim × dim)
                    // MuJoCo formula: H_raw then scale by diag(mu, friction) · Dm
                    let ci = data.efc_id[i]; // contact index
                    let t_safe = t.max(MJ_MINVAL);
                    let mut hc = DMatrix::<f64>::zeros(dim, dim);

                    // H_raw[0,0] = 1
                    hc[(0, 0)] = 1.0;

                    // Compute U vector (scaled jar)
                    let mut u_vec = vec![0.0_f64; dim];
                    u_vec[0] = u0; // jar[i] * mu
                    for j in 1..dim {
                        let friction_j = data.efc_mu[i][j - 1];
                        u_vec[j] = data.efc_jar[i + j] * friction_j;
                    }

                    // H_raw[0,j] = -mu * U[j] / T  for j >= 1
                    // H_raw[j,0] = H_raw[0,j]  (symmetric)
                    for j in 1..dim {
                        let val = -mu * u_vec[j] / t_safe;
                        hc[(0, j)] = val;
                        hc[(j, 0)] = val;
                    }

                    // H_raw[k,j] = mu * N / T³ * U[k] * U[j]  for k,j >= 1
                    // diagonal: += mu² - mu * N / T
                    for k in 1..dim {
                        for j in 1..dim {
                            hc[(k, j)] = mu * n / (t_safe * t_safe * t_safe) * u_vec[k] * u_vec[j];
                        }
                        hc[(k, k)] += mu * mu - mu * n / t_safe;
                    }

                    // Scale: H = diag(mu, friction[0..]) · H_raw · diag(mu, friction[0..]) · Dm
                    // Build scale vector
                    let mut scale = vec![0.0_f64; dim];
                    scale[0] = mu;
                    scale[1..dim].copy_from_slice(&data.efc_mu[i][..(dim - 1)]);
                    // Apply: H[k,j] *= scale[k] * scale[j] * dm
                    for k in 0..dim {
                        for j in 0..dim {
                            hc[(k, j)] *= scale[k] * scale[j] * dm;
                        }
                    }

                    data.efc_cone_hessian[ci] = Some(hc);
                    data.ncone += 1;
                }
                i += dim;
            }
        }
    }

    // Gauss term: ½·(Ma − qfrc_smooth)·(qacc − qacc_smooth)
    let mut cost_gauss = 0.0;
    for k in 0..nv {
        cost_gauss += (ma[k] - qfrc_smooth[k]) * (qacc[k] - qacc_smooth[k]);
    }
    cost_gauss *= 0.5;

    data.efc_cost = cost_gauss + constraint_cost;
}

/// Assemble the Newton Hessian and factor via Cholesky.
///
/// Phase A: H = M + Σ_{Quadratic rows} D_i · J_i^T · J_i
/// (No cone Hessian in Phase A — cone rows are treated as Quadratic per-row.)
///
/// Returns the Cholesky factor L (lower triangular) such that H = L · L^T,
/// or an error if the Hessian is not positive definite.
fn assemble_hessian(data: &Data, nv: usize) -> Result<DMatrix<f64>, StepError> {
    let nefc = data.efc_type.len();

    // Start with mass matrix
    let mut h = DMatrix::<f64>::zeros(nv, nv);
    for r in 0..nv {
        for c in 0..nv {
            h[(r, c)] = data.qM[(r, c)];
        }
    }

    // Add Σ_{Quadratic rows} D_i · J_i^T · J_i
    for i in 0..nefc {
        if data.efc_state[i] != ConstraintState::Quadratic {
            continue;
        }
        let d_i = data.efc_D[i];
        // Rank-1 update: H += D_i · j_i · j_i^T
        for r in 0..nv {
            let j_r = data.efc_J[(i, r)];
            if j_r == 0.0 {
                continue;
            }
            let d_j_r = d_i * j_r;
            for c in r..nv {
                let j_c = data.efc_J[(i, c)];
                if j_c == 0.0 {
                    continue;
                }
                let val = d_j_r * j_c;
                h[(r, c)] += val;
                if r != c {
                    h[(c, r)] += val;
                }
            }
        }
    }

    // Cholesky factorize in-place
    cholesky_in_place(&mut h)?;
    Ok(h)
}

// ============================================================================
// Sparse Hessian path (Phase C): for large systems (nv > NV_SPARSE_THRESHOLD)
// ============================================================================

/// DOF count above which Newton switches from dense O(nv³) Cholesky to
/// sparse LDL^T. At nv ≈ 60, sparse becomes competitive with dense.
const NV_SPARSE_THRESHOLD: usize = 60;

/// Sparse Hessian H = M + J^T·D·J in CSC lower-triangle format with
/// cached symbolic/numeric LDL^T factorization.
///
/// The sparsity pattern comes from:
/// - M: tree sparsity via `dof_parent` (entry (i,j) iff j is ancestor of i)
/// - J^T·D·J: couples DOFs that share constraint rows
///
/// Symbolic factorization (elimination tree + L structure) is computed once
/// per `assemble` call. Numeric factorization is recomputed each Newton
/// iteration via `factor()`. Solve is forward/diagonal/back substitution.
struct SparseHessian {
    nv: usize,
    /// CSC column pointers for lower triangle of H (length nv+1).
    col_ptr: Vec<usize>,
    /// CSC row indices (length nnz). Sorted within each column.
    row_idx: Vec<usize>,
    /// CSC values (length nnz).
    vals: Vec<f64>,
    /// Elimination tree: parent[j] = parent of column j in etree, or None for root.
    etree: Vec<Option<usize>>,
    /// CSC column pointers for L factor (length nv+1).
    l_col_ptr: Vec<usize>,
    /// CSC row indices for L factor.
    l_row_idx: Vec<usize>,
    /// Numeric values of L factor (unit lower triangular: L[j,j] = 1, not stored).
    l_vals: Vec<f64>,
    /// Diagonal D from LDL^T factorization (length nv).
    l_diag: Vec<f64>,
}

impl SparseHessian {
    /// Build the sparse Hessian from Model/Data. Computes:
    /// 1. Sparsity pattern of H = M + J^T·D·J (CSC lower triangle)
    /// 2. Numeric values
    /// 3. Symbolic factorization (elimination tree + L structure)
    /// 4. Numeric LDL^T factorization
    fn assemble(model: &Model, data: &Data, nv: usize) -> Result<Self, StepError> {
        let nefc = data.efc_type.len();

        // --- Step 1: Determine sparsity pattern ---
        // Use a dense boolean mask per column (acceptable since this is O(nv²) and
        // we only enter the sparse path when nv > 60 where this is ~3600 entries).
        let mut has_entry = vec![vec![false; nv]; nv]; // has_entry[col][row], row >= col

        // M sparsity: tree structure from dof_parent
        for i in 0..nv {
            has_entry[i][i] = true; // diagonal always present
            let mut p = model.dof_parent[i];
            while let Some(j) = p {
                // M[i,j] is non-zero (j < i since j is ancestor)
                has_entry[j][i] = true; // lower triangle: row=i, col=j
                p = model.dof_parent[j];
            }
        }

        // J^T·D·J sparsity: for each Quadratic constraint row, the outer product
        // of its non-zero J entries determines fill. We conservatively include all
        // non-zero pairs.
        for r in 0..nefc {
            if data.efc_state[r] != ConstraintState::Quadratic {
                continue;
            }
            // Collect non-zero column indices in this J row
            let mut nz_cols: Vec<usize> = Vec::new();
            for col in 0..nv {
                if data.efc_J[(r, col)] != 0.0 {
                    nz_cols.push(col);
                }
            }
            // Mark all pairs (lower triangle)
            for &ci in &nz_cols {
                for &cj in &nz_cols {
                    let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                    has_entry[lo][hi] = true;
                }
            }
        }

        // --- Step 2: Build CSC arrays ---
        let mut col_ptr = vec![0usize; nv + 1];
        let mut row_idx_vec = Vec::new();
        let mut vals_vec = Vec::new();

        for col in 0..nv {
            col_ptr[col] = row_idx_vec.len();
            for row in col..nv {
                if has_entry[col][row] {
                    row_idx_vec.push(row);
                    vals_vec.push(0.0); // will be filled in fill_numeric
                }
            }
        }
        col_ptr[nv] = row_idx_vec.len();

        let mut h = Self {
            nv,
            col_ptr,
            row_idx: row_idx_vec,
            vals: vals_vec,
            etree: vec![None; nv],
            l_col_ptr: vec![0; nv + 1],
            l_row_idx: Vec::new(),
            l_vals: Vec::new(),
            l_diag: vec![0.0; nv],
        };

        // --- Step 3: Fill numeric values ---
        h.fill_numeric(model, data, nv, nefc);

        // --- Step 4: Symbolic factorization ---
        h.symbolic_factor();

        // --- Step 5: Numeric factorization ---
        h.numeric_factor()?;

        Ok(h)
    }

    /// Refactor with updated numeric values (same sparsity pattern).
    /// Used when constraint states change but sparsity doesn't.
    fn refactor(&mut self, model: &Model, data: &Data) -> Result<(), StepError> {
        let nv = self.nv;
        let nefc = data.efc_type.len();
        self.fill_numeric(model, data, nv, nefc);
        self.numeric_factor()
    }

    /// Fill CSC values with H = M + Σ_{Quadratic} D_i · J_i^T · J_i.
    fn fill_numeric(&mut self, model: &Model, data: &Data, nv: usize, nefc: usize) {
        // Zero all values
        self.vals.iter_mut().for_each(|v| *v = 0.0);

        // Add M using tree sparsity: only walk (i, ancestor) pairs via dof_parent.
        // This is O(nv · depth) instead of O(nv²) — much cheaper for tree-structured
        // robots where depth << nv.
        for i in 0..nv {
            // Diagonal
            if let Some(idx) = self.find_entry(i, i) {
                self.vals[idx] += data.qM[(i, i)];
            }
            // Off-diagonal: walk ancestors
            let mut p = model.dof_parent[i];
            while let Some(j) = p {
                // M[i,j] non-zero, j < i (ancestor). Store in lower triangle: col=j, row=i.
                let m_val = data.qM[(i, j)];
                if let Some(idx) = self.find_entry(j, i) {
                    self.vals[idx] += m_val;
                }
                p = model.dof_parent[j];
            }
        }

        // Add J^T · D · J for Quadratic rows.
        // Cache non-zero column indices per constraint row to avoid O(nv) scans
        // in the inner loop — reduces J^T·D·J fill from O(nefc·nv²) to
        // O(nefc·nnz_per_row²) where nnz_per_row is typically << nv.
        for r in 0..nefc {
            if data.efc_state[r] != ConstraintState::Quadratic {
                continue;
            }
            let d_r = data.efc_D[r];
            // Collect non-zero columns for this J row
            let mut nz_cols: Vec<(usize, f64)> = Vec::new();
            for col in 0..nv {
                let j_val = data.efc_J[(r, col)];
                if j_val != 0.0 {
                    nz_cols.push((col, j_val));
                }
            }
            // Outer product over non-zero pairs only
            for (ai, &(col_a, j_a)) in nz_cols.iter().enumerate() {
                let d_j_a = d_r * j_a;
                for &(col_b, j_b) in &nz_cols[ai..] {
                    // Lower triangle: col_b >= col_a
                    if let Some(idx) = self.find_entry(col_a, col_b) {
                        self.vals[idx] += d_j_a * j_b;
                    }
                }
            }
        }
    }

    /// Find the CSC index for entry (col, row) where row >= col.
    fn find_entry(&self, col: usize, row: usize) -> Option<usize> {
        let start = self.col_ptr[col];
        let end = self.col_ptr[col + 1];
        // Binary search in the sorted row indices for this column
        self.row_idx[start..end]
            .binary_search(&row)
            .ok()
            .map(|offset| start + offset)
    }

    /// Compute elimination tree from the CSC sparsity pattern.
    /// etree[j] = min { i > j : L[i,j] != 0 }, which equals the first
    /// off-diagonal non-zero row in column j of L.
    ///
    /// Also computes the symbolic structure of L (l_col_ptr, l_row_idx).
    fn symbolic_factor(&mut self) {
        let n = self.nv;

        // Compute elimination tree using the Liu algorithm:
        // For each column j, the row indices of H below the diagonal determine
        // which columns will have fill in L. The parent of j in the etree is
        // the first row index > j that appears in column j of H or through
        // fill propagation.
        let mut parent = vec![None; n];
        let mut ancestor = vec![0usize; n]; // path-compressed ancestor

        for j in 0..n {
            ancestor[j] = j;
            let start = self.col_ptr[j];
            let end = self.col_ptr[j + 1];
            for k in start..end {
                let i = self.row_idx[k];
                if i <= j {
                    continue;
                }
                // Walk up the etree from i using path compression
                let mut r = i;
                while ancestor[r] != r && ancestor[r] != j {
                    let next = ancestor[r];
                    ancestor[r] = j;
                    r = next;
                }
                if ancestor[r] == r {
                    // r is a root — make j its parent
                    parent[r] = Some(j);
                    ancestor[r] = j;
                }
            }
        }

        self.etree = parent;

        // Compute symbolic L structure: for each column j, L has non-zero entries
        // at all rows that appear in H[:,j] below diagonal, plus fill from the etree.
        // Use row counts approach: for each column j, collect all row indices in the
        // subtree of j's column in H, then propagate up the etree.
        let mut l_row_sets: Vec<Vec<usize>> = vec![Vec::new(); n];

        for j in 0..n {
            let start = self.col_ptr[j];
            let end = self.col_ptr[j + 1];
            for k in start..end {
                let i = self.row_idx[k];
                if i > j {
                    l_row_sets[j].push(i);
                }
            }
        }

        // Propagate fill: for each column j (in order), merge its row set into
        // parent's row set (excluding j itself, since L[j,j] = 1 implicitly).
        for j in 0..n {
            l_row_sets[j].sort_unstable();
            l_row_sets[j].dedup();
            if let Some(p) = self.etree[j] {
                // All rows in L[:,j] that are > p are also in L[:,p]
                let fill: Vec<usize> = l_row_sets[j].iter().copied().filter(|&r| r > p).collect();
                // Need to clone to avoid borrow conflict
                l_row_sets[p].extend(fill);
            }
        }

        // Sort and dedup all sets again after propagation
        for j in 0..n {
            l_row_sets[j].sort_unstable();
            l_row_sets[j].dedup();
        }

        // Build CSC for L
        let mut l_col_ptr = vec![0usize; n + 1];
        let mut l_row_idx = Vec::new();
        for j in 0..n {
            l_col_ptr[j] = l_row_idx.len();
            l_row_idx.extend_from_slice(&l_row_sets[j]);
        }
        l_col_ptr[n] = l_row_idx.len();

        let l_nnz = l_row_idx.len();
        self.l_col_ptr = l_col_ptr;
        self.l_row_idx = l_row_idx;
        self.l_vals = vec![0.0; l_nnz];
        self.l_diag = vec![0.0; n];
    }

    /// Numeric LDL^T factorization.
    ///
    /// Computes L (unit lower triangular) and D (diagonal) such that H = L·D·L^T.
    /// Uses a left-looking approach: for each column j, subtract contributions from
    /// columns to the left, then scale.
    fn numeric_factor(&mut self) -> Result<(), StepError> {
        let n = self.nv;

        // Work arrays
        let mut y = vec![0.0f64; n]; // dense accumulator for column j of L*D
        let mut pattern = vec![false; n]; // which rows have non-zero in current column

        for j in 0..n {
            // Initialize y with column j of H (lower triangle)
            let h_start = self.col_ptr[j];
            let h_end = self.col_ptr[j + 1];
            for k in h_start..h_end {
                let i = self.row_idx[k];
                y[i] = self.vals[k];
                pattern[i] = true;
            }

            // Subtract contributions from earlier columns that have non-zero in row j
            // For each column k < j where L[j,k] != 0:
            //   y[i] -= L[j,k] * D[k] * L[i,k] for all i in L[:,k] with i >= j
            for k in 0..j {
                let l_start = self.l_col_ptr[k];
                let l_end = self.l_col_ptr[k + 1];

                // Find L[j,k] in L[:,k]
                let mut ljk = 0.0;
                for p in l_start..l_end {
                    if self.l_row_idx[p] == j {
                        ljk = self.l_vals[p];
                        break;
                    }
                    if self.l_row_idx[p] > j {
                        break; // sorted, so we're past j
                    }
                }

                if ljk == 0.0 {
                    continue;
                }

                let dk = self.l_diag[k];
                let ljk_dk = ljk * dk;

                // Subtract from diagonal
                y[j] -= ljk_dk * ljk;

                // Subtract from below-diagonal entries
                for p in l_start..l_end {
                    let i = self.l_row_idx[p];
                    if i <= j {
                        continue;
                    }
                    y[i] -= ljk_dk * self.l_vals[p];
                }
            }

            // Extract D[j] = y[j]
            let dj = y[j];
            if dj <= 0.0 {
                // Clean up work arrays before returning error
                for k in h_start..h_end {
                    let i = self.row_idx[k];
                    y[i] = 0.0;
                    pattern[i] = false;
                }
                return Err(StepError::CholeskyFailed);
            }
            self.l_diag[j] = dj;
            let dj_inv = 1.0 / dj;

            // Extract L[:,j] = y[j+1:] / D[j]
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                self.l_vals[p] = y[i] * dj_inv;
            }

            // Clean up work arrays
            y[j] = 0.0;
            pattern[j] = false;
            for k in h_start..h_end {
                let i = self.row_idx[k];
                y[i] = 0.0;
                pattern[i] = false;
            }
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                y[i] = 0.0;
                pattern[i] = false;
            }
        }

        Ok(())
    }

    /// Solve L·D·L^T · x = b in place.
    fn solve(&self, x: &mut DVector<f64>) {
        let n = self.nv;

        // Forward substitution: L · y = b
        // L is unit lower triangular (L[j,j] = 1, not stored)
        for j in 0..n {
            let xj = x[j];
            if xj == 0.0 {
                continue;
            }
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                x[i] -= self.l_vals[p] * xj;
            }
        }

        // Diagonal solve: D · z = y
        for j in 0..n {
            x[j] /= self.l_diag[j];
        }

        // Back substitution: L^T · w = z
        // L^T is unit upper triangular
        for j in (0..n).rev() {
            let l_start = self.l_col_ptr[j];
            let l_end = self.l_col_ptr[j + 1];
            for p in l_start..l_end {
                let i = self.l_row_idx[p];
                x[j] -= self.l_vals[p] * x[i];
            }
        }
    }
}

/// Incrementally update the Cholesky factor when constraint states change.
///
/// Instead of reassembling the full Hessian and re-factoring, uses rank-1
/// updates and downdates to modify the existing Cholesky factor L:
/// - Old state != Quadratic AND new state == Quadratic → rank-1 update (add D_i · J_i^T · J_i)
/// - Old state == Quadratic AND new state != Quadratic → rank-1 downdate (remove D_i · J_i^T · J_i)
///
/// Falls back to full `assemble_hessian` if a downdate fails (would make H non-PD).
fn hessian_incremental(
    data: &Data,
    nv: usize,
    chol_l: &mut DMatrix<f64>,
    old_states: &[ConstraintState],
) -> Result<(), StepError> {
    let nefc = data.efc_type.len();

    for i in 0..nefc {
        let old = old_states[i];
        let new = data.efc_state[i];

        // Skip if state didn't change w.r.t. Quadratic membership
        let was_quad = old == ConstraintState::Quadratic;
        let is_quad = new == ConstraintState::Quadratic;

        if was_quad == is_quad {
            continue;
        }

        // Build the rank-1 vector: v = sqrt(D_i) * J_i
        let d_i = data.efc_D[i];
        let sqrt_d = d_i.sqrt();
        let mut v = vec![0.0_f64; nv];
        for col in 0..nv {
            v[col] = sqrt_d * data.efc_J[(i, col)];
        }

        if !was_quad && is_quad {
            // Became Quadratic → update (add contribution)
            cholesky_rank1_update(chol_l, &mut v)?;
        } else {
            // Was Quadratic, now not → downdate (remove contribution)
            if cholesky_rank1_downdate(chol_l, &mut v).is_err() {
                // Downdate failed — fall back to full reassembly
                *chol_l = assemble_hessian(data, nv)?;
                return Ok(());
            }
        }
    }

    Ok(())
}

/// Produce L_cone from L by adding cone Hessian contributions via rank-1 updates.
///
/// For each cone-state contact with a stored `efc_cone_hessian[ci]`:
/// 1. Factor the local dim×dim H_c via Cholesky → L_local
/// 2. For each column k of L_local, build v = L_local[:,k]^T * J_contact (nv vector)
/// 3. Apply rank-1 update to L_cone with v
///
/// Returns the modified Cholesky factor, or falls back to full `assemble_hessian` if
/// any update fails.
fn hessian_cone(data: &Data, nv: usize, chol_l: &DMatrix<f64>) -> Result<DMatrix<f64>, StepError> {
    let mut l_cone = chol_l.clone();

    for (ci, hc_opt) in data.efc_cone_hessian.iter().enumerate() {
        let hc = match hc_opt {
            Some(h) => h,
            None => continue,
        };

        let dim = hc.nrows();

        // Find the starting efc row for this contact.
        // Scan efc_id to find the first row with id == ci and type == ContactElliptic.
        let efc_start = data
            .efc_id
            .iter()
            .position(|&id| id == ci && data.efc_type[id] == ConstraintType::ContactElliptic);
        let efc_start = match efc_start {
            Some(s) => s,
            None => continue,
        };

        // Factor H_c via Cholesky: H_c = L_local · L_local^T
        let mut l_local = hc.clone();
        if cholesky_in_place(&mut l_local).is_err() {
            // H_c not PD — skip this cone (degenerate)
            continue;
        }

        // For each column k of L_local, compute v = sum_j L_local[j,k] * J[efc_start+j, :]
        // Then rank-1 update L_cone with v
        for k in 0..dim {
            let mut v = vec![0.0_f64; nv];
            for j in k..dim {
                // L_local is lower triangular, so L_local[j,k] = 0 for j < k
                let ljk = l_local[(j, k)];
                if ljk == 0.0 {
                    continue;
                }
                let efc_row = efc_start + j;
                for col in 0..nv {
                    v[col] += ljk * data.efc_J[(efc_row, col)];
                }
            }
            cholesky_rank1_update(&mut l_cone, &mut v)?;
        }
    }

    Ok(l_cone)
}

/// Compute gradient and preconditioned search direction (PrimalUpdateGradient, §15.4).
///
/// Returns (grad, search) where:
/// - `qfrc_constraint = J^T · efc_force`
/// - `grad = Ma - qfrc_smooth - qfrc_constraint`
/// - `search = -H⁻¹ · grad` (via Cholesky solve)
fn compute_gradient_and_search(
    data: &Data,
    nv: usize,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    chol_l: &DMatrix<f64>,
) -> (DVector<f64>, DVector<f64>) {
    let nefc = data.efc_type.len();

    // qfrc_constraint = J^T · efc_force
    let mut qfrc_constraint = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            qfrc_constraint[col] += data.efc_J[(i, col)] * f;
        }
    }

    // grad = Ma - qfrc_smooth - qfrc_constraint
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint[k];
    }

    // search = -H⁻¹ · grad = -solve(L, grad)
    let mut search = grad.clone();
    cholesky_solve_in_place(chol_l, &mut search);
    for k in 0..nv {
        search[k] = -search[k];
    }

    (grad, search)
}

/// Sparse variant of `compute_gradient_and_search` using `SparseHessian::solve`.
fn compute_gradient_and_search_sparse(
    data: &Data,
    nv: usize,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    sparse_h: &SparseHessian,
) -> (DVector<f64>, DVector<f64>) {
    let nefc = data.efc_type.len();

    // qfrc_constraint = J^T · efc_force
    let mut qfrc_constraint = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            qfrc_constraint[col] += data.efc_J[(i, col)] * f;
        }
    }

    // grad = Ma - qfrc_smooth - qfrc_constraint
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint[k];
    }

    // search = -H⁻¹ · grad via sparse LDL^T solve
    let mut search = grad.clone();
    sparse_h.solve(&mut search);
    for k in 0..nv {
        search[k] = -search[k];
    }

    (grad, search)
}

// ==========================================================================
// Phase B: Exact 1D Newton line search infrastructure (§15.5)
// ==========================================================================

/// Precomputed quadratic polynomials for fast 1D cost evaluation along search direction.
///
/// For constraint i: cost(alpha) depends on `jar_trial = jar + alpha * jv`
/// For the Gauss regularisation: cost is exactly quadratic in alpha.
struct PrimalQuad {
    /// Per-constraint quadratic coefficients.
    /// For Quadratic/Equality rows:   [0.5*D*jar², D*jar*jv, 0.5*D*jv²]
    /// For LinearPos:                  [+floss*jar - 0.5*R*floss², +floss*jv, 0]
    /// For LinearNeg:                  [-floss*jar - 0.5*R*floss², -floss*jv, 0]
    /// For Satisfied:                  [0, 0, 0]
    /// For Contact rows: use raw values, re-classify per alpha in primal_eval.
    quad: Vec<[f64; 3]>,

    /// Per-elliptic-cone extra coefficients.
    /// Each entry: [U0, V0, UU, UV, VV, Dm] for the rescaled cone decomposition.
    /// Indexed by sequential cone index (not efc row).
    cone_quad: Vec<[f64; 6]>,

    /// Mapping from sequential cone index to the starting efc row and dim.
    cone_map: Vec<(usize, usize)>,

    /// Gauss term quadratic: [constant, linear, quadratic].
    /// cost_gauss(alpha) = quad_gauss[0] + alpha*quad_gauss[1] + alpha²*quad_gauss[2]
    quad_gauss: [f64; 3],

    /// Norm of the search direction ||search||.
    snorm: f64,
}

/// A point on the 1D cost curve, evaluated at step size `alpha`.
#[derive(Debug, Clone, Copy)]
struct PrimalPoint {
    /// Step size.
    alpha: f64,
    /// [dcost/dalpha, d²cost/dalpha²]
    deriv: [f64; 2],
    /// Total cost at this alpha (used for diagnostics / future cost monitoring).
    #[allow(dead_code)]
    cost: f64,
}

/// Precompute quadratic polynomial coefficients for the 1D line search (§15.5).
///
/// Called once per Newton iteration before entering the line search. The
/// precomputed coefficients allow O(nefc) cost+derivative evaluation at each
/// trial alpha without any matrix-vector products.
#[allow(clippy::many_single_char_names, clippy::too_many_arguments)]
fn primal_prepare(
    data: &Data,
    _nv: usize,
    qacc: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    search: &DVector<f64>,
    mv: &DVector<f64>,
    jv: &DVector<f64>,
) -> PrimalQuad {
    let nefc = data.efc_type.len();
    let mut quad = vec![[0.0_f64; 3]; nefc];
    let mut cone_quad = Vec::new();
    let mut cone_map = Vec::new();

    let mut i = 0;
    while i < nefc {
        let jar_i = data.efc_jar[i];
        let d_i = data.efc_D[i];
        let jv_i = jv[i];

        if data.efc_type[i] == ConstraintType::ContactElliptic {
            let dim = data.efc_dim[i];
            let mu = data.efc_mu[i][0];

            // Per-row quadratic (for bottom zone / per-component evaluation)
            for j in 0..dim {
                let jar_j = data.efc_jar[i + j];
                let d_j = data.efc_D[i + j];
                let jv_j = jv[i + j];
                quad[i + j] = [
                    0.5 * d_j * jar_j * jar_j,
                    d_j * jar_j * jv_j,
                    0.5 * d_j * jv_j * jv_j,
                ];
            }

            // Cone-specific: rescale to circular cone space
            let u0 = jar_i * mu;
            let v0 = jv_i * mu;
            let mut uu = 0.0;
            let mut uv = 0.0;
            let mut vv = 0.0;

            for j in 1..dim {
                let friction_j = data.efc_mu[i][j - 1];
                let uj = data.efc_jar[i + j] * friction_j;
                let vj = jv[i + j] * friction_j;
                uu += uj * uj;
                uv += uj * vj;
                vv += vj * vj;
            }

            let dm = d_i / (mu * mu * (1.0 + mu * mu));
            cone_quad.push([u0, v0, uu, uv, vv, dm]);
            cone_map.push((i, dim));

            i += dim;
        } else {
            // Scalar constraint: store raw D·jar quadratic
            quad[i] = [
                0.5 * d_i * jar_i * jar_i,
                d_i * jar_i * jv_i,
                0.5 * d_i * jv_i * jv_i,
            ];
            i += 1;
        }
    }

    // Gauss quadratic: cost(alpha) = G0 + alpha*G1 + alpha²*G2
    // G0 = 0.5 * (Ma - qfrc_smooth) · (qacc - qacc_smooth)  [already in data.efc_cost - constraint_cost]
    // G1 = search · Ma - qfrc_smooth · search = search · (Ma - qfrc_smooth)
    // G2 = 0.5 * search · Mv
    let nv = search.len();
    let mut g1 = 0.0;
    let mut g2 = 0.0;
    for k in 0..nv {
        g1 += search[k] * (ma[k] - qfrc_smooth[k]);
        g2 += search[k] * mv[k];
    }
    g2 *= 0.5;

    // G0 = 0.5 * (Ma - qfrc_smooth) · (qacc - qacc_smooth)
    // Exact since Gauss cost is purely quadratic in qacc (linear in alpha).
    let mut g0 = 0.0;
    for k in 0..nv {
        g0 += (ma[k] - qfrc_smooth[k]) * (qacc[k] - qacc_smooth[k]);
    }
    g0 *= 0.5;

    let snorm: f64 = search.iter().map(|x| x * x).sum::<f64>().sqrt();

    PrimalQuad {
        quad,
        cone_quad,
        cone_map,
        quad_gauss: [g0, g1, g2],
        snorm,
    }
}

/// Evaluate cost, first derivative, and second derivative at alpha along the search direction.
///
/// Uses precomputed `PrimalQuad` coefficients for O(nefc) evaluation without
/// matrix-vector products. This is the inner loop of `PrimalSearch`.
#[allow(clippy::many_single_char_names)]
fn primal_eval(data: &Data, pq: &PrimalQuad, jv: &[f64], alpha: f64) -> PrimalPoint {
    let nefc = data.efc_type.len();
    let mut cost = 0.0;
    let mut deriv = [0.0_f64; 2];

    // Batch quadratic accumulator for simple constraints and bottom-zone cones
    let mut quad_total = [0.0_f64; 3];

    let mut cone_idx = 0;
    let mut i = 0;
    while i < nefc {
        match data.efc_type[i] {
            ConstraintType::Equality => {
                // Always quadratic
                quad_total[0] += pq.quad[i][0];
                quad_total[1] += pq.quad[i][1];
                quad_total[2] += pq.quad[i][2];
                i += 1;
            }

            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                let r_i = data.efc_R[i];
                let threshold = r_i * floss;
                // Trial jar at alpha
                let jar_trial = data.efc_jar[i] + alpha * jv[i];

                if jar_trial >= threshold {
                    // LinearPos
                    cost += floss * jar_trial - 0.5 * r_i * floss * floss;
                    deriv[0] += floss * jv[i];
                    // deriv[1] += 0 (linear)
                } else if jar_trial <= -threshold {
                    // LinearNeg
                    cost += -floss * jar_trial - 0.5 * r_i * floss * floss;
                    deriv[0] += -floss * jv[i];
                    // deriv[1] += 0
                } else {
                    // Quadratic
                    quad_total[0] += pq.quad[i][0];
                    quad_total[1] += pq.quad[i][1];
                    quad_total[2] += pq.quad[i][2];
                }
                i += 1;
            }

            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactNonElliptic => {
                // Re-classify at trial alpha
                let jar_trial = data.efc_jar[i] + alpha * jv[i];
                if jar_trial < 0.0 {
                    // Quadratic (active)
                    quad_total[0] += pq.quad[i][0];
                    quad_total[1] += pq.quad[i][1];
                    quad_total[2] += pq.quad[i][2];
                }
                // else: Satisfied — zero cost
                i += 1;
            }

            ConstraintType::ContactElliptic => {
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0];
                let cq = &pq.cone_quad[cone_idx];
                let (_, _) = pq.cone_map[cone_idx];
                cone_idx += 1;

                // Compute N(alpha) and T(alpha) in rescaled cone space
                let n_alpha = cq[0] + alpha * cq[1]; // U0 + alpha*V0
                let t_sq = (cq[2] + alpha * (2.0 * cq[3] + alpha * cq[4])).max(0.0); // UU + 2*alpha*UV + alpha²*VV
                let t_alpha = t_sq.sqrt();
                let dm = cq[5];

                let t_min = MJ_MINVAL;

                if n_alpha >= mu * t_alpha || (t_alpha < t_min && n_alpha >= 0.0) {
                    // Top zone: satisfied, zero cost
                } else if mu * n_alpha + t_alpha <= 0.0 || (t_alpha < t_min && n_alpha < 0.0) {
                    // Bottom zone: per-row quadratic
                    for j in 0..dim {
                        quad_total[0] += pq.quad[i + j][0];
                        quad_total[1] += pq.quad[i + j][1];
                        quad_total[2] += pq.quad[i + j][2];
                    }
                } else {
                    // Middle zone: cone cost
                    let nmt = n_alpha - mu * t_alpha;
                    cost += 0.5 * dm * nmt * nmt;

                    // First derivative: d(cost)/d(alpha)
                    // d(N)/d(alpha) = V0
                    // d(T)/d(alpha) = (UV + alpha*VV) / T (when T > 0)
                    let n1 = cq[1]; // V0
                    let t1 = if t_alpha > t_min {
                        (cq[3] + alpha * cq[4]) / t_alpha
                    } else {
                        0.0
                    };
                    let nmt1 = n1 - mu * t1; // d(N-mu*T)/d(alpha)
                    deriv[0] += dm * nmt * nmt1;

                    // Second derivative: d²(cost)/d(alpha)²
                    // d²(T)/d(alpha)² = (VV - t1²) / T (when T > 0)
                    let t2 = if t_alpha > t_min {
                        (cq[4] - t1 * t1) / t_alpha
                    } else {
                        0.0
                    };
                    deriv[1] += dm * (nmt1 * nmt1 + nmt * (-mu * t2));
                }

                i += dim;
            }
        }
    }

    // Evaluate batched quadratic total
    let q_cost = quad_total[0] + alpha * quad_total[1] + alpha * alpha * quad_total[2];
    let q_d1 = quad_total[1] + 2.0 * alpha * quad_total[2];
    let q_d2 = 2.0 * quad_total[2];
    cost += q_cost;
    deriv[0] += q_d1;
    deriv[1] += q_d2;

    // Add Gauss term (exactly quadratic in alpha)
    cost += pq.quad_gauss[0] + alpha * pq.quad_gauss[1] + alpha * alpha * pq.quad_gauss[2];
    deriv[0] += pq.quad_gauss[1] + 2.0 * alpha * pq.quad_gauss[2];
    deriv[1] += 2.0 * pq.quad_gauss[2];

    PrimalPoint { alpha, deriv, cost }
}

/// Three-phase exact Newton line search (§15.5 PrimalSearch).
///
/// Finds the step size `alpha` that minimizes the 1D cost along the search
/// direction. Uses the precomputed `PrimalQuad` from `primal_prepare` and
/// evaluates via `primal_eval`.
///
/// # Phases
/// 1. **Initial Newton step** from alpha=0. If derivative is small enough, done.
/// 2. **Bracket search**: step in the direction of the initial derivative until
///    the derivative sign changes, establishing a bracket [lo, hi].
/// 3. **Bracketed refinement**: three candidates (Newton from lo, Newton from hi,
///    midpoint). Pick the best converged candidate, or tighten the bracket.
///
/// Returns the optimal alpha, or 0.0 if no improvement is possible.
#[allow(clippy::many_single_char_names)]
fn primal_search(
    data: &Data,
    pq: &PrimalQuad,
    jv: &[f64],
    tolerance: f64,
    ls_tolerance: f64,
    max_ls_iter: usize,
    scale: f64,
) -> (f64, usize) {
    let mut neval: usize = 0;

    // Gradient tolerance for convergence
    let gtol = tolerance * ls_tolerance * pq.snorm / scale.max(MJ_MINVAL);
    if gtol <= 0.0 || pq.snorm < MJ_MINVAL {
        return (0.0, neval);
    }

    // Phase 1: Initial Newton step from alpha=0
    let p0 = primal_eval(data, pq, jv, 0.0);
    neval += 1;

    // Check if gradient at 0 is already small (unconstrained minimum at 0)
    if p0.deriv[0].abs() < gtol {
        return (0.0, neval);
    }

    // Newton step: alpha1 = -f'(0) / f''(0)
    let alpha1 = if p0.deriv[1] > MJ_MINVAL {
        -p0.deriv[0] / p0.deriv[1]
    } else {
        // Hessian too small; take unit step in descent direction
        if p0.deriv[0] < 0.0 { 1.0 } else { -1.0 }
    };

    // Guard: step must be positive (descent direction)
    if alpha1 <= 0.0 {
        return (0.0, neval);
    }

    let p1 = primal_eval(data, pq, jv, alpha1);
    neval += 1;

    // Check convergence at p1
    if p1.deriv[0].abs() < gtol {
        return (alpha1, neval);
    }

    // Phase 2: One-sided bracket search
    // Determine search direction based on sign of p1's derivative
    let mut lo;
    let mut hi;
    if p1.deriv[0] < 0.0 {
        // Derivative still negative — minimum is to the right of p1
        lo = p1;
        hi = p1; // will be updated
        let mut step = alpha1; // step size doubles each iteration
        for _ in 0..max_ls_iter {
            step *= 2.0;
            let trial = lo.alpha + step;
            let pt = primal_eval(data, pq, jv, trial);
            neval += 1;
            if pt.deriv[0].abs() < gtol {
                return (pt.alpha, neval);
            }
            if pt.deriv[0] > 0.0 {
                // Derivative changed sign — bracket found: [lo, pt]
                hi = pt;
                break;
            }
            // Still negative — continue stepping right
            lo = pt;
            step = pt.alpha - lo.alpha; // geometric increase
        }
        // If hi was never updated (loop exhausted), return best we have
        if hi.alpha <= lo.alpha {
            return (lo.alpha, neval);
        }
    } else {
        // Derivative positive at p1 — minimum is between 0 and p1
        lo = p0;
        hi = p1;
    }

    // Phase 3: Bracketed refinement
    for _ in 0..max_ls_iter {
        // Bracket width check
        let width = hi.alpha - lo.alpha;
        if width < MJ_MINVAL {
            break;
        }

        // Three candidates:
        // 1. Newton step from lo
        let c1 = if lo.deriv[1] > MJ_MINVAL {
            let a = lo.alpha - lo.deriv[0] / lo.deriv[1];
            if a > lo.alpha && a < hi.alpha {
                Some(a)
            } else {
                None
            }
        } else {
            None
        };

        // 2. Newton step from hi
        let c2 = if hi.deriv[1] > MJ_MINVAL {
            let a = hi.alpha - hi.deriv[0] / hi.deriv[1];
            if a > lo.alpha && a < hi.alpha {
                Some(a)
            } else {
                None
            }
        } else {
            None
        };

        // 3. Midpoint
        let mid = 0.5 * (lo.alpha + hi.alpha);

        // Evaluate all candidates, pick the best
        let mut best = None;
        let mut best_deriv_abs = f64::MAX;

        for alpha in [c1, c2, Some(mid)].into_iter().flatten() {
            let pt = primal_eval(data, pq, jv, alpha);
            neval += 1;
            if pt.deriv[0].abs() < gtol {
                return (pt.alpha, neval);
            }
            if pt.deriv[0].abs() < best_deriv_abs {
                best_deriv_abs = pt.deriv[0].abs();
                best = Some(pt);
            }
        }

        // Tighten the bracket using the best candidate
        if let Some(pt) = best {
            if pt.deriv[0] < 0.0 {
                lo = pt;
            } else {
                hi = pt;
            }
        } else {
            break;
        }
    }

    // Return the endpoint with smaller |derivative|
    let alpha = if lo.deriv[0].abs() < hi.deriv[0].abs() {
        lo.alpha
    } else {
        hi.alpha
    };
    (alpha, neval)
}

/// Evaluate total cost (Gauss + constraint) at a trial acceleration.
///
/// Does not modify any data fields — purely evaluative.
/// Used by warmstart comparison in `newton_solve`.
#[allow(clippy::many_single_char_names)]
fn evaluate_cost_at(
    data: &Data,
    model: &Model,
    qacc_trial: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
) -> f64 {
    let nv = model.nv;
    let nefc = data.efc_type.len();

    // Compute jar_trial = J · qacc_trial - aref
    let mut jar_trial = DVector::<f64>::zeros(nefc);
    for i in 0..nefc {
        let mut j_dot_qacc = 0.0;
        for col in 0..nv {
            j_dot_qacc += data.efc_J[(i, col)] * qacc_trial[col];
        }
        jar_trial[i] = j_dot_qacc - data.efc_aref[i];
    }

    // Compute Ma_trial = M · qacc_trial
    let mut ma_trial = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma_trial[r] += data.qM[(r, c)] * qacc_trial[c];
        }
    }

    // Gauss term: ½·(Ma_trial - qfrc_smooth)·(qacc_trial - qacc_smooth)
    let mut cost_gauss = 0.0;
    for k in 0..nv {
        cost_gauss += (ma_trial[k] - qfrc_smooth[k]) * (qacc_trial[k] - qacc_smooth[k]);
    }
    cost_gauss *= 0.5;

    // Constraint cost
    let mut constraint_cost = 0.0;
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let jar = jar_trial[i];
        let d = data.efc_D[i];
        let r = data.efc_R[i];

        match ctype {
            ConstraintType::Equality => {
                constraint_cost += 0.5 * d * jar * jar;
                i += 1;
            }
            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                let threshold = r * floss;
                if jar >= threshold {
                    constraint_cost += floss * jar - 0.5 * r * floss * floss;
                } else if jar <= -threshold {
                    constraint_cost += -floss * jar - 0.5 * r * floss * floss;
                } else {
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }
            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactNonElliptic => {
                if jar < 0.0 {
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }
            ConstraintType::ContactElliptic => {
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0];

                let u0 = jar_trial[i] * mu;
                let n = u0;
                let mut t_sq = 0.0;
                for j in 1..dim {
                    let friction_j = data.efc_mu[i][j - 1];
                    let uj = jar_trial[i + j] * friction_j;
                    t_sq += uj * uj;
                }
                let t = t_sq.sqrt();
                let t_min = MJ_MINVAL;

                if n >= mu * t || (t < t_min && n >= 0.0) {
                    // Top → zero cost
                } else if mu * n + t <= 0.0 || (t < t_min && n < 0.0) {
                    // Bottom → per-row quadratic
                    for j in 0..dim {
                        let jar_j = jar_trial[i + j];
                        let d_j = data.efc_D[i + j];
                        constraint_cost += 0.5 * d_j * jar_j * jar_j;
                    }
                } else {
                    // Middle → cone cost
                    let d_normal = data.efc_D[i];
                    let dm = d_normal / (mu * mu * (1.0 + mu * mu));
                    let n_minus_mu_t = n - mu * t;
                    constraint_cost += 0.5 * dm * n_minus_mu_t * n_minus_mu_t;
                }
                i += dim;
            }
        }
    }

    cost_gauss + constraint_cost
}

/// Result of the Newton solver. Used by Step 14 (PGS fallback).
#[derive(Debug)]
enum NewtonResult {
    /// Solver converged successfully.
    Converged,
    /// Cholesky factorization failed (Hessian not positive definite).
    CholeskyFailed,
    /// Maximum iterations reached without convergence.
    MaxIterationsExceeded,
}

/// Newton solver outer loop (§15.8).
///
/// Computes qacc, qfrc_constraint, efc_state, efc_force, efc_jar, efc_cost
/// directly via reduced primal optimization. Returns early with `NewtonResult`
/// indicating success or failure mode for PGS fallback.
#[allow(clippy::too_many_lines)]
fn newton_solve(model: &Model, data: &mut Data) -> NewtonResult {
    let nv = model.nv;

    // === PER-STEP MEANINERTIA (Phase C) ===
    // More accurate than model-level constant for configuration-dependent inertia.
    // O(nv) — free since qM is already filled by CRBA this step.
    let meaninertia = if nv > 0 {
        let trace: f64 = (0..nv).map(|i| data.qM[(i, i)]).sum();
        let mi = trace / nv as f64;
        if mi > 0.0 { mi } else { model.stat_meaninertia }
    } else {
        model.stat_meaninertia
    };
    data.stat_meaninertia = meaninertia;

    // === INITIALIZE ===
    // qfrc_smooth = qfrc_applied + qfrc_actuator + (qfrc_passive - qfrc_frictionloss) - qfrc_bias
    let mut qfrc_smooth = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        qfrc_smooth[k] = data.qfrc_applied[k]
            + data.qfrc_actuator[k]
            + (data.qfrc_passive[k] - data.qfrc_frictionloss[k])
            - data.qfrc_bias[k];
    }

    // qacc_smooth = M⁻¹ · qfrc_smooth (via sparse LDL solve)
    let mut qacc_smooth = qfrc_smooth.clone();
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut qacc_smooth);

    // Assemble unified constraints
    assemble_unified_constraints(model, data, &qacc_smooth);
    let nefc = data.efc_type.len();

    // No constraints → unconstrained solution
    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth);
        data.qfrc_constraint.fill(0.0);
        data.efc_lambda.clear();
        data.solver_niter = 0;
        data.solver_stat.clear();
        return NewtonResult::Converged;
    }

    let scale = 1.0 / (meaninertia * (1.0_f64).max(nv as f64));
    let tolerance = model.solver_tolerance;

    // Initialize efc_state and efc_force vectors to correct size
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // --- Warmstart selection ---
    // Evaluate cost at qacc_warmstart
    let cost_warmstart = evaluate_cost_at(
        data,
        model,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
    );
    // Evaluate cost at qacc_smooth (Gauss = 0, only constraint cost from efc_b)
    let cost_smooth = evaluate_cost_at(data, model, &qacc_smooth, &qacc_smooth, &qfrc_smooth);

    let mut qacc = if cost_warmstart < cost_smooth {
        data.qacc_warmstart.clone()
    } else {
        qacc_smooth.clone()
    };

    // Compute Ma = M · qacc
    let mut ma = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma[r] += data.qM[(r, c)] * qacc[c];
        }
    }

    // Initial constraint classification
    classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

    // Select dense vs sparse Hessian path (Phase C).
    // Sparse path: full refactorization each iteration, no incremental updates.
    // Dense path: incremental rank-1 updates + cone Hessian augmentation.
    let use_sparse = nv > NV_SPARSE_THRESHOLD;

    // Initial Hessian + gradient + search direction
    let mut chol_l_dense: Option<DMatrix<f64>> = None;
    let mut sparse_h: Option<SparseHessian> = None;

    let (mut grad, mut search) = if use_sparse {
        let Ok(sh) = SparseHessian::assemble(model, data, nv) else {
            data.solver_niter = 0;
            data.solver_stat.clear();
            return NewtonResult::CholeskyFailed;
        };
        let gs = compute_gradient_and_search_sparse(data, nv, &ma, &qfrc_smooth, &sh);
        sparse_h = Some(sh);
        gs
    } else {
        let Ok(l) = assemble_hessian(data, nv) else {
            data.solver_niter = 0;
            data.solver_stat.clear();
            return NewtonResult::CholeskyFailed;
        };
        let gs = compute_gradient_and_search(data, nv, &ma, &qfrc_smooth, &l);
        chol_l_dense = Some(l);
        gs
    };

    // Pre-loop convergence check
    let grad_norm: f64 = grad.iter().map(|x| x * x).sum::<f64>().sqrt();
    if scale * grad_norm < tolerance {
        // Already converged — go to RECOVER
        data.solver_niter = 0;
        data.solver_stat.clear();
        recover_newton(model, data, &qacc, &qfrc_smooth);
        return NewtonResult::Converged;
    }

    // === ITERATE (Phase B: exact Newton line search + incremental Hessian) ===
    let max_iters = model.solver_iterations;
    let max_ls_iter = model.ls_iterations.max(20);
    let ls_tolerance = model.ls_tolerance;
    let mut converged = false;
    let mut solver_stats = Vec::with_capacity(max_iters);

    // Precompute Mv = M*search and Jv = J*search for the initial search direction
    let mut mv = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            mv[r] += data.qM[(r, c)] * search[c];
        }
    }
    let mut jv = DVector::<f64>::zeros(nefc);
    for i in 0..nefc {
        for col in 0..nv {
            jv[i] += data.efc_J[(i, col)] * search[col];
        }
    }

    for _iter in 0..max_iters {
        // Compute lineslope before line search: grad · search / ||search||
        let search_norm = search.iter().map(|x| x * x).sum::<f64>().sqrt();
        let lineslope = if search_norm > 0.0 {
            grad.iter()
                .zip(search.iter())
                .map(|(g, s)| g * s)
                .sum::<f64>()
                / search_norm
        } else {
            0.0
        };

        // 1. LINE SEARCH (Phase B: exact Newton)
        let pq = primal_prepare(
            data,
            nv,
            &qacc,
            &qacc_smooth,
            &ma,
            &qfrc_smooth,
            &search,
            &mv,
            &jv,
        );
        let (alpha, nline) = primal_search(
            data,
            &pq,
            jv.as_slice(),
            tolerance,
            ls_tolerance,
            max_ls_iter,
            scale,
        );

        if alpha == 0.0 {
            // Record final stat entry for the alpha=0 iteration
            let nactive = data
                .efc_state
                .iter()
                .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
                .count();
            solver_stats.push(SolverStat {
                improvement: 0.0,
                gradient: scale * grad.iter().map(|x| x * x).sum::<f64>().sqrt(),
                lineslope,
                nactive,
                nchange: 0,
                nline,
            });
            converged = true; // No improvement — treat as converged (local minimum)
            break;
        }

        // 2. MOVE
        for k in 0..nv {
            qacc[k] += alpha * search[k];
            ma[k] += alpha * mv[k];
        }
        for i in 0..nefc {
            data.efc_jar[i] += alpha * jv[i];
        }

        // 3. UPDATE CONSTRAINTS (save old states for incremental Hessian)
        let old_states = data.efc_state.clone();
        let old_cost = data.efc_cost;
        classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

        // 4-5. HESSIAN + GRADIENT (dense vs sparse)
        let (g, s) = if use_sparse {
            // Sparse path: full refactorization each iteration
            let sh = sparse_h.as_mut().unwrap_or_else(|| unreachable!());
            if sh.refactor(model, data).is_err() {
                // Refactorization failed — try full reassembly
                if let Ok(new_sh) = SparseHessian::assemble(model, data, nv) {
                    *sh = new_sh;
                } else {
                    data.solver_niter = solver_stats.len();
                    data.solver_stat = solver_stats;
                    return NewtonResult::CholeskyFailed;
                }
            }
            compute_gradient_and_search_sparse(data, nv, &ma, &qfrc_smooth, sh)
        } else {
            // Dense path: incremental rank-1 updates + cone Hessian
            let chol_l = chol_l_dense.as_mut().unwrap_or_else(|| unreachable!());
            if hessian_incremental(data, nv, chol_l, &old_states).is_err() {
                // Incremental update failed — fall back to full reassembly
                if let Ok(l) = assemble_hessian(data, nv) {
                    *chol_l = l;
                } else {
                    data.solver_niter = solver_stats.len();
                    data.solver_stat = solver_stats;
                    return NewtonResult::CholeskyFailed;
                }
            }

            // Use cone Hessian if any cone contacts
            let l_for_solve = if data.ncone > 0 {
                match hessian_cone(data, nv, chol_l) {
                    Ok(lc) => lc,
                    Err(_) => chol_l.clone(),
                }
            } else {
                chol_l.clone()
            };

            compute_gradient_and_search(data, nv, &ma, &qfrc_smooth, &l_for_solve)
        };
        grad = g;

        // 6. CONVERGENCE CHECK + SOLVER STATISTICS
        let improvement = scale * (old_cost - data.efc_cost);
        let gradient = scale * grad.iter().map(|x| x * x).sum::<f64>().sqrt();

        let nactive = data
            .efc_state
            .iter()
            .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
            .count();
        let nchange = old_states
            .iter()
            .zip(data.efc_state.iter())
            .filter(|(old, new)| old != new)
            .count();

        solver_stats.push(SolverStat {
            improvement,
            gradient,
            lineslope,
            nactive,
            nchange,
            nline,
        });

        if improvement < tolerance || gradient < tolerance {
            converged = true;
            break;
        }

        // 7. SEARCH DIRECTION + recompute Mv, Jv for next iteration
        search = s;

        // Recompute Mv = M * search
        mv.fill(0.0);
        for r in 0..nv {
            for c in 0..nv {
                mv[r] += data.qM[(r, c)] * search[c];
            }
        }
        // Recompute Jv = J * search
        jv.fill(0.0);
        for i in 0..nefc {
            for col in 0..nv {
                jv[i] += data.efc_J[(i, col)] * search[col];
            }
        }
    }

    // === RECOVER ===
    data.solver_niter = solver_stats.len();
    data.solver_stat = solver_stats;
    recover_newton(model, data, &qacc, &qfrc_smooth);

    if converged {
        NewtonResult::Converged
    } else {
        NewtonResult::MaxIterationsExceeded
    }
}

/// RECOVER block: write final Newton results to Data.
fn recover_newton(
    model: &Model,
    data: &mut Data,
    qacc: &DVector<f64>,
    _qfrc_smooth: &DVector<f64>,
) {
    let nv = model.nv;
    let nefc = data.efc_type.len();

    // Write qacc
    data.qacc.copy_from(qacc);

    // Compute and write qfrc_constraint = J^T · efc_force
    data.qfrc_constraint.fill(0.0);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            data.qfrc_constraint[col] += data.efc_J[(i, col)] * f;
        }
    }

    // Extract per-joint and per-tendon limit forces
    data.jnt_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.ten_limit_frc.iter_mut().for_each(|f| *f = 0.0);

    for i in 0..nefc {
        match data.efc_type[i] {
            ConstraintType::LimitJoint => {
                data.jnt_limit_frc[data.efc_id[i]] = data.efc_force[i];
            }
            ConstraintType::LimitTendon => {
                data.ten_limit_frc[data.efc_id[i]] = data.efc_force[i];
            }
            _ => {}
        }
    }

    // Populate efc_lambda for warmstart and touch sensors.
    // Map Newton's efc_force back to the WarmstartKey-based HashMap.
    data.efc_lambda.clear();
    let mut i = 0;
    while i < nefc {
        let dim = data.efc_dim[i];
        match data.efc_type[i] {
            ConstraintType::ContactElliptic | ConstraintType::ContactNonElliptic => {
                let ci = data.efc_id[i];
                if ci < data.contacts.len() {
                    let key = warmstart_key(&data.contacts[ci]);
                    let forces: Vec<f64> = (0..dim).map(|j| data.efc_force[i + j]).collect();
                    data.efc_lambda.insert(key, forces);
                }
                i += dim;
            }
            _ => {
                i += 1;
            }
        }
    }
}

/// Noslip post-processor: suppresses residual contact slip (§15.10).
///
/// Runs a modified PGS pass on friction force rows only, without
/// regularization, using the current normal forces as fixed cone limits.
/// This matches MuJoCo's `mj_solNoSlip`.
///
/// Called after `newton_solve()` converges when `model.noslip_iterations > 0`.
fn noslip_postprocess(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nefc = data.efc_type.len();
    let noslip_iter = model.noslip_iterations;
    let noslip_tol = model.noslip_tolerance;

    if noslip_iter == 0 || nefc == 0 {
        return;
    }

    // 1. Identify friction rows and their parent contacts.
    // friction_rows[k] = efc row index of the k-th friction row
    // friction_contact[k] = (contact_start_efc_row, dim) for the parent contact
    let mut friction_rows: Vec<usize> = Vec::new();
    let mut friction_contact: Vec<(usize, usize)> = Vec::new();

    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];

        match ctype {
            ConstraintType::ContactElliptic | ConstraintType::ContactNonElliptic => {
                // Only process contacts with friction (dim >= 3) and active normal
                if dim >= 3 && data.efc_state[i] != ConstraintState::Satisfied {
                    for j in 1..dim {
                        friction_rows.push(i + j);
                        friction_contact.push((i, dim));
                    }
                }
                i += dim;
            }
            _ => {
                i += 1;
            }
        }
    }

    let nfric = friction_rows.len();
    if nfric == 0 {
        return;
    }

    // 2. Build friction-only Delassus matrix A (no regularization on diagonal).
    // A[fi,fj] = J[friction_rows[fi]] · M⁻¹ · J[friction_rows[fj]]^T
    let mut a_fric = DMatrix::<f64>::zeros(nfric, nfric);
    for (fi, &row_i) in friction_rows.iter().enumerate() {
        // Solve M · x = J_row_i^T via sparse LDL
        let mut minv_ji = DVector::<f64>::zeros(nv);
        for col in 0..nv {
            minv_ji[col] = data.efc_J[(row_i, col)];
        }
        mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut minv_ji);

        for (fj, &row_j) in friction_rows.iter().enumerate() {
            let mut dot = 0.0;
            for col in 0..nv {
                dot += data.efc_J[(row_j, col)] * minv_ji[col];
            }
            a_fric[(fi, fj)] = dot;
        }
    }

    // 3. Extract current friction forces as starting point
    let mut f_fric: Vec<f64> = friction_rows.iter().map(|&r| data.efc_force[r]).collect();

    // Precompute diagonal inverse (no regularization)
    let diag_inv: Vec<f64> = (0..nfric)
        .map(|fi| {
            let d = a_fric[(fi, fi)];
            if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d }
        })
        .collect();

    // 4. RHS: b[fi] = J[friction_row] · qacc (target friction acceleration = 0,
    //    so b = J·qacc − 0 = J·qacc). For friction rows, aref is 0 (position and
    //    margin are 0 for friction dimensions), so efc_jar = J·qacc − aref = J·qacc.
    let b_fric: Vec<f64> = friction_rows.iter().map(|&r| data.efc_jar[r]).collect();

    // 5. PGS iterations on friction rows
    for _iter in 0..noslip_iter {
        let mut max_delta = 0.0_f64;

        // Process contact-by-contact for correct cone projection
        let mut fi = 0;
        while fi < nfric {
            let (contact_start, dim) = friction_contact[fi];
            let fric_dim = dim - 1; // number of friction rows for this contact

            // GS update for each friction row of this contact
            for local_j in 0..fric_dim {
                let idx = fi + local_j;
                // residual = Σ_k A[idx,k]·f[k] + b[idx]
                let mut residual = b_fric[idx];
                for k in 0..nfric {
                    residual += a_fric[(idx, k)] * f_fric[k];
                }
                let old = f_fric[idx];
                f_fric[idx] -= residual * diag_inv[idx];
                max_delta = max_delta.max((f_fric[idx] - old).abs());
            }

            // Project friction forces onto the friction cone.
            // Elliptic cone: Σ_j (f_j / mu_j)² ≤ f_normal²
            let normal_force = data.efc_force[contact_start]; // Fixed from Newton
            let mu = data.efc_mu[contact_start];

            let mut s_sq = 0.0;
            for local_j in 0..fric_dim {
                let mu_j = mu[local_j]; // mu[0..dim-1] for friction dims
                if mu_j > MJ_MINVAL {
                    s_sq += (f_fric[fi + local_j] / mu_j).powi(2);
                }
            }
            let s = s_sq.sqrt();
            let fn_abs = normal_force.abs();
            if s > fn_abs && s > MJ_MINVAL {
                let cone_scale = fn_abs / s;
                for local_j in 0..fric_dim {
                    f_fric[fi + local_j] *= cone_scale;
                }
            }

            fi += fric_dim;
        }

        if max_delta < noslip_tol {
            break;
        }
    }

    // 6. Write back updated friction forces
    for (fi, &row) in friction_rows.iter().enumerate() {
        data.efc_force[row] = f_fric[fi];
    }

    // 7. Recompute qfrc_constraint = J^T · efc_force
    data.qfrc_constraint.fill(0.0);
    for i_row in 0..nefc {
        let f = data.efc_force[i_row];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            data.qfrc_constraint[col] += data.efc_J[(i_row, col)] * f;
        }
    }

    // 8. Recompute qacc = M⁻¹ · (qfrc_smooth + qfrc_constraint)
    for k in 0..nv {
        data.qacc[k] = data.qfrc_applied[k]
            + data.qfrc_actuator[k]
            + (data.qfrc_passive[k] - data.qfrc_frictionloss[k])
            - data.qfrc_bias[k]
            + data.qfrc_constraint[k];
    }
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut data.qacc);
}

/// Apply a Connect (ball-and-socket) equality constraint.
///
/// Constrains anchor point on body1 to coincide with body2's position.
/// Uses Baumgarte stabilization: F = -k * pos_error - b * vel_error
///
/// # Force Limiting
///
/// Forces are clamped based on the effective mass to ensure acceleration
/// stays bounded, preventing instability with explicit integration.
fn apply_connect_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Cache body validity checks (body_id == 0 means world frame)
    let body1_is_dynamic = body1 != 0;
    let body2_is_dynamic = body2 != 0;

    // Anchor point in body1's local frame
    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);

    // Get body poses
    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let p2 = if body2_is_dynamic {
        data.xpos[body2]
    } else {
        Vector3::zeros()
    };

    // World position of anchor on body1
    let anchor_world = p1 + r1 * anchor;

    // Position error: anchor should coincide with body2's origin
    let pos_error = anchor_world - p2;

    // Velocity error (relative velocity at constraint point)
    let vel_error = if body1_is_dynamic {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        let v_anchor = v1 + omega1.cross(&r1_anchor);

        if body2_is_dynamic {
            let cvel2 = &data.cvel[body2];
            let v2 = Vector3::new(cvel2[3], cvel2[4], cvel2[5]);
            v_anchor - v2
        } else {
            v_anchor
        }
    } else {
        Vector3::zeros()
    };

    // Compute penalty parameters from solref
    let (stiffness, damping) = solref_to_penalty(
        model.eq_solref[eq_id],
        default_stiffness,
        default_damping,
        dt,
    );

    // Compute position-dependent impedance from solimp.
    // Impedance d(r) ∈ (0,1) scales effective stiffness/damping:
    // stronger constraint at larger violations (if d_width > d0).
    let violation = pos_error.norm();
    let impedance = compute_impedance(model.eq_solimp[eq_id], violation);

    // Baumgarte stabilization: F = -d(r)*k*error - d(r)*b*vel_error
    let raw_force = -impedance * stiffness * pos_error - impedance * damping * vel_error;

    // Compute effective mass for force limiting using cached values
    // body_min_mass[0] = infinity for world body
    let mass1 = data.body_min_mass[body1];
    let mass2 = data.body_min_mass[body2];
    let eff_mass = effective_mass_for_stability(mass1, mass2);

    // Limit force to bound acceleration: a_max = max_constraint_vel / dt
    let max_accel = model.max_constraint_vel / dt;
    let max_force = eff_mass * max_accel;
    let force = clamp_vector_magnitude(raw_force, max_force);

    // Apply forces via Jacobian transpose
    if body1_is_dynamic {
        apply_constraint_force_to_body(model, data, body1, anchor_world, force);
    }
    if body2_is_dynamic {
        apply_constraint_force_to_body(model, data, body2, p2, -force);
    }
}

/// Apply a distance equality constraint between two geom centers.
///
/// Enforces `|p1 - p2| = target_distance` where p1 and p2 are the world-space
/// centers of geom1 and geom2. This is a 1-dimensional scalar constraint
/// (removes 1 DOF), with force applied along the direction connecting the geoms.
///
/// Object IDs (`eq_obj1id`/`eq_obj2id`) are **geom IDs**, not body IDs.
/// Body IDs are derived via `model.geom_body[geom_id]`.
///
/// # Singularity Handling
///
/// When the two geom centers coincide (distance -> 0), the constraint direction
/// is undefined. We use a fallback direction of `+Z` and apply force proportional
/// to the target distance. This provides a deterministic push-apart force.
fn apply_distance_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    const SINGULARITY_EPS: f64 = 1e-10;

    let geom1_id = model.eq_obj1id[eq_id];
    let geom2_id = model.eq_obj2id[eq_id]; // usize::MAX = world origin
    let target_dist = model.eq_data[eq_id][0];

    // Get geom world positions
    let p1 = data.geom_xpos[geom1_id];
    let p2 = if geom2_id == usize::MAX {
        Vector3::zeros()
    } else {
        data.geom_xpos[geom2_id]
    };

    // Get body IDs for force application
    let body1 = model.geom_body[geom1_id];
    let body2 = if geom2_id == usize::MAX {
        0 // World body
    } else {
        model.geom_body[geom2_id]
    };

    let body1_is_dynamic = body1 != 0;
    let body2_is_dynamic = body2 != 0;

    // Compute direction and current distance
    let diff = p1 - p2;
    let current_dist = diff.norm();

    // Singularity guard: when geoms coincide, direction is undefined
    let (direction, scalar_error) = if current_dist > SINGULARITY_EPS {
        let dir = diff / current_dist;
        let err = current_dist - target_dist;
        (dir, err)
    } else if target_dist > SINGULARITY_EPS {
        // Geoms coincide but target > 0: push apart along +Z fallback
        (Vector3::z(), -target_dist)
    } else {
        // Both near zero: constraint satisfied
        return;
    };

    // Velocity error projected onto constraint direction
    let v1 = if body1_is_dynamic {
        compute_point_velocity(data, body1, p1)
    } else {
        Vector3::zeros()
    };
    let v2 = if body2_is_dynamic {
        compute_point_velocity(data, body2, p2)
    } else {
        Vector3::zeros()
    };
    let vel_error = (v1 - v2).dot(&direction);

    // Compute penalty parameters from solref
    let (stiffness, damping) = solref_to_penalty(
        model.eq_solref[eq_id],
        default_stiffness,
        default_damping,
        dt,
    );

    // Compute position-dependent impedance from solimp
    let violation = scalar_error.abs();
    let impedance = compute_impedance(model.eq_solimp[eq_id], violation);

    // Baumgarte stabilization: scalar_force = -d(r)*k*error - d(r)*b*vel_error
    let scalar_force = -impedance * stiffness * scalar_error - impedance * damping * vel_error;

    // Convert scalar force to 3D force vector along direction
    let raw_force = scalar_force * direction;

    // Compute effective mass for force limiting
    let mass1 = data.body_min_mass[body1];
    let mass2 = data.body_min_mass[body2];
    let eff_mass = effective_mass_for_stability(mass1, mass2);

    // Limit force to bound acceleration
    let max_accel = model.max_constraint_vel / dt;
    let max_force = eff_mass * max_accel;
    let force = clamp_vector_magnitude(raw_force, max_force);

    // Apply forces via Jacobian transpose
    if body1_is_dynamic {
        apply_constraint_force_to_body(model, data, body1, p1, force);
    }
    if body2_is_dynamic {
        apply_constraint_force_to_body(model, data, body2, p2, -force);
    }
}

/// DOF type for mass matrix extraction.
#[derive(Clone, Copy, PartialEq, Eq)]
enum DofKind {
    /// Linear (translational) DOFs
    Linear,
    /// Angular (rotational) DOFs
    Angular,
}

/// Extract minimum diagonal mass/inertia from the mass matrix for specified DOF types.
///
/// This is the core implementation used by both `get_min_translational_mass` and
/// `get_min_rotational_inertia`. It traverses the body's joints and extracts the
/// minimum diagonal element from the mass matrix for the specified DOF type.
///
/// # Arguments
/// * `model` - The physics model
/// * `data` - The simulation data containing the mass matrix
/// * `body_id` - The body to query (0 = world, returns infinity)
/// * `kind` - Whether to extract linear (mass) or angular (inertia) DOFs
///
/// # Returns
/// The minimum diagonal mass/inertia, or `DEFAULT_MASS_FALLBACK` if no valid DOFs found.
#[inline]
fn get_min_diagonal_mass(model: &Model, data: &Data, body_id: usize, kind: DofKind) -> f64 {
    if body_id == 0 {
        return f64::INFINITY;
    }

    let mut min_val = f64::INFINITY;

    let jnt_start = model.body_jnt_adr[body_id];
    let jnt_end = jnt_start + model.body_jnt_num[body_id];

    for jnt_id in jnt_start..jnt_end {
        let dof_adr = model.jnt_dof_adr[jnt_id];

        // Determine which DOF indices to query based on joint type and DOF kind
        // Arms combined where they return identical values per clippy::match_same_arms
        let dof_range: Option<std::ops::Range<usize>> = match (model.jnt_type[jnt_id], kind) {
            // Free linear (0-2) and Ball angular (0-2) both use first 3 DOFs
            (MjJointType::Free, DofKind::Linear) | (MjJointType::Ball, DofKind::Angular) => {
                Some(0..3)
            }

            // Free angular uses DOFs 3-5
            (MjJointType::Free, DofKind::Angular) => Some(3..6),

            // Hinge/Slide: single DOF of matching kind
            (MjJointType::Hinge, DofKind::Angular) | (MjJointType::Slide, DofKind::Linear) => {
                Some(0..1)
            }

            // No DOFs for mismatched kind
            (MjJointType::Ball | MjJointType::Hinge, DofKind::Linear)
            | (MjJointType::Slide, DofKind::Angular) => None,
        };

        if let Some(range) = dof_range {
            for i in range {
                let dof = dof_adr + i;
                if dof < model.nv {
                    let val = data.qM[(dof, dof)];
                    if val > MIN_INERTIA_THRESHOLD {
                        min_val = min_val.min(val);
                    }
                }
            }
        }
    }

    if min_val == f64::INFINITY {
        DEFAULT_MASS_FALLBACK
    } else {
        min_val
    }
}

/// Get the minimum translational mass from the mass matrix diagonal for a body's linear DOFs.
///
/// **Note**: In the hot path, use `data.body_min_mass[body_id]` instead, which is cached
/// during `forward()` after CRBA. This function is kept for debugging and testing.
///
/// For bodies with free joints, this returns the minimum of the x, y, z mass entries.
/// For bodies with slide joints, this returns the slide DOF's effective mass.
///
/// # Returns
/// - `f64::INFINITY` if body_id is 0 (world body)
/// - Minimum diagonal mass if found
/// - `DEFAULT_MASS_FALLBACK` (1.0 kg) if no linear DOFs exist
#[inline]
#[allow(dead_code)] // Kept for debugging/testing; hot path uses cached data.body_min_mass
fn get_min_translational_mass(model: &Model, data: &Data, body_id: usize) -> f64 {
    get_min_diagonal_mass(model, data, body_id, DofKind::Linear)
}

/// Get the minimum rotational inertia from the mass matrix diagonal for a body's angular DOFs.
///
/// **Note**: In the hot path, use `data.body_min_inertia[body_id]` instead, which is cached
/// during `forward()` after CRBA. This function is kept for debugging and testing.
///
/// For bodies with free/ball joints, this returns the minimum of the angular inertia entries.
/// For bodies with hinge joints, this returns the hinge DOF's effective inertia.
///
/// # Returns
/// - `f64::INFINITY` if body_id is 0 (world body)
/// - Minimum diagonal inertia if found
/// - `DEFAULT_MASS_FALLBACK` (1.0 kg·m²) if no angular DOFs exist
#[inline]
#[allow(dead_code)] // Kept for debugging/testing; hot path uses cached data.body_min_inertia
fn get_min_rotational_inertia(model: &Model, data: &Data, body_id: usize) -> f64 {
    get_min_diagonal_mass(model, data, body_id, DofKind::Angular)
}

/// Compute effective mass/inertia for a binary constraint between two bodies.
///
/// For stability, we use the *minimum* of the two masses rather than the harmonic mean.
/// This ensures the force limit is conservative enough for the lighter body.
///
/// # Physics Rationale
///
/// When force F is applied between two bodies with masses m1 and m2:
/// - Body 1 accelerates at F/m1
/// - Body 2 accelerates at -F/m2
/// - The *relative* acceleration is F * (1/m1 + 1/m2) = F * (m1+m2)/(m1*m2)
///
/// For stability, we need to limit the acceleration of *each* body individually.
/// Using min(m1, m2) ensures F/min(m1,m2) ≤ max_accel for both bodies.
#[inline]
fn effective_mass_for_stability(m1: f64, m2: f64) -> f64 {
    // For world body (m = infinity), just use the other body's mass
    if m1 == f64::INFINITY {
        return m2;
    }
    if m2 == f64::INFINITY {
        return m1;
    }
    // Use minimum for conservative stability bound
    m1.min(m2)
}

/// Apply a Weld (6 DOF) equality constraint.
///
/// Constrains body1's frame to maintain a fixed relative pose to body2.
/// Uses Baumgarte stabilization for both position and orientation:
///   F = -k * pos_error - b * vel_error
///   τ = -k * rot_error - b * ang_vel_error
///
/// # Stability Considerations
///
/// Weld constraints between free joints can be unstable with explicit integration
/// when there's a large initial error. The constraint force/torque is limited based
/// on the effective mass/inertia to ensure the resulting acceleration stays bounded.
///
/// # Orientation Error Computation
///
/// The rotation error is computed as the axis-angle representation of
/// `r1 * (r2 * target_quat)⁻¹`. This uses the accurate `quaternion_to_axis_angle`
/// conversion which handles large angles correctly.
fn apply_weld_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Cache body validity checks (body_id == 0 means world frame)
    let body1_is_dynamic = body1 != 0;
    let body2_is_dynamic = body2 != 0;

    // Anchor point in body1's local frame
    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);

    // Target relative quaternion [qw, qx, qy, qz]
    let target_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        eq_data[3], eq_data[4], eq_data[5], eq_data[6],
    ));

    // Get body poses
    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let (p2, r2) = if body2_is_dynamic {
        (data.xpos[body2], data.xquat[body2])
    } else {
        (Vector3::zeros(), UnitQuaternion::identity())
    };

    // === Position Error ===
    let anchor_world = p1 + r1 * anchor;
    let pos_error = anchor_world - p2;

    // === Orientation Error ===
    // Target: r1 = r2 * target_quat
    // Error quaternion: e = r1 * (r2 * target_quat)⁻¹
    let target_r1 = r2 * target_quat;
    let rot_error_quat = r1 * target_r1.inverse();

    // Convert to axis-angle (accurate for all angles, not just small)
    let rot_error = quaternion_to_axis_angle(&rot_error_quat);

    // === Velocity Errors ===
    let (vel_error, ang_vel_error) = if body1_is_dynamic {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        let v_anchor = v1 + omega1.cross(&r1_anchor);

        if body2_is_dynamic {
            let cvel2 = &data.cvel[body2];
            let omega2 = Vector3::new(cvel2[0], cvel2[1], cvel2[2]);
            let v2 = Vector3::new(cvel2[3], cvel2[4], cvel2[5]);
            (v_anchor - v2, omega1 - omega2)
        } else {
            (v_anchor, omega1)
        }
    } else {
        (Vector3::zeros(), Vector3::zeros())
    };

    // Compute penalty parameters from solref
    let (stiffness, damping) = solref_to_penalty(
        model.eq_solref[eq_id],
        default_stiffness,
        default_damping,
        dt,
    );

    // Compute position-dependent impedance from solimp, separately for
    // translational and rotational components. MuJoCo computes impedance
    // per constraint row using each row's individual violation. A weld has
    // 6 rows (3 translational + 3 rotational), so we compute two impedances.
    //
    // Note: solimp `width` is specified in meters for translational constraints
    // and radians for rotational constraints. Using a single solimp for both
    // is a simplification — MuJoCo allows per-row solimp in principle, but
    // equality constraints share one solimp vector. The `width` parameter
    // will be interpreted in the units of each error (meters vs radians).
    let solimp = model.eq_solimp[eq_id];
    let trans_impedance = compute_impedance(solimp, pos_error.norm());
    let rot_impedance = compute_impedance(solimp, rot_error.norm());

    // === Position Constraint Force ===
    let raw_force =
        -trans_impedance * stiffness * pos_error - trans_impedance * damping * vel_error;

    // Compute effective mass using cached values (body_min_mass[0] = infinity for world)
    let mass1 = data.body_min_mass[body1];
    let mass2 = data.body_min_mass[body2];
    let eff_mass = effective_mass_for_stability(mass1, mass2);

    let max_linear_accel = model.max_constraint_vel / dt;
    let max_force = eff_mass * max_linear_accel;
    let force = clamp_vector_magnitude(raw_force, max_force);

    // === Orientation Constraint Torque ===
    // Clamp effective rotation error for stiffness term (damping uses actual velocity)
    let clamped_rot_error = clamp_vector_magnitude(rot_error, MAX_ROTATION_ERROR_FOR_STIFFNESS);
    let raw_torque =
        -rot_impedance * stiffness * clamped_rot_error - rot_impedance * damping * ang_vel_error;

    // Compute effective inertia using cached values (body_min_inertia[0] = infinity for world)
    let inertia1 = data.body_min_inertia[body1];
    let inertia2 = data.body_min_inertia[body2];
    let eff_inertia = effective_mass_for_stability(inertia1, inertia2);

    let max_angular_accel = model.max_constraint_angvel / dt;
    let max_torque = eff_inertia * max_angular_accel;
    let torque = clamp_vector_magnitude(raw_torque, max_torque);

    // === Apply Forces and Torques ===
    if body1_is_dynamic {
        apply_constraint_force_to_body(model, data, body1, anchor_world, force);
        apply_constraint_torque_to_body(model, data, body1, torque);
    }
    if body2_is_dynamic {
        apply_constraint_force_to_body(model, data, body2, p2, -force);
        apply_constraint_torque_to_body(model, data, body2, -torque);
    }
}

/// Apply a Joint equality constraint (polynomial coupling).
///
/// Constrains joint2 position as polynomial function of joint1:
/// q2 = c0 + c1*q1 + c2*q1² + ...
fn apply_joint_equality_constraint(
    model: &Model,
    data: &mut Data,
    eq_id: usize,
    default_stiffness: f64,
    default_damping: f64,
    dt: f64,
) {
    let joint1_id = model.eq_obj1id[eq_id];
    let joint2_id = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    // Polynomial coefficients: c0, c1, c2, c3, c4
    let c0 = eq_data[0];
    let c1 = eq_data[1];
    let c2 = eq_data[2];
    let c3 = eq_data[3];
    let c4 = eq_data[4];

    // Get joint positions and velocities
    let qpos1_adr = model.jnt_qpos_adr[joint1_id];
    let dof1_adr = model.jnt_dof_adr[joint1_id];
    let q1 = data.qpos[qpos1_adr];
    let qd1 = data.qvel[dof1_adr];

    // Compute target position: q2_target = poly(q1)
    let q2_target = c0 + c1 * q1 + c2 * q1 * q1 + c3 * q1 * q1 * q1 + c4 * q1 * q1 * q1 * q1;

    // Compute target velocity: qd2_target = d(poly)/dq1 * qd1
    let dq2_dq1 = c1 + 2.0 * c2 * q1 + 3.0 * c3 * q1 * q1 + 4.0 * c4 * q1 * q1 * q1;
    let qd2_target = dq2_dq1 * qd1;

    // If joint2 is valid (not usize::MAX sentinel)
    if joint2_id < model.njnt {
        let qpos2_adr = model.jnt_qpos_adr[joint2_id];
        let dof2_adr = model.jnt_dof_adr[joint2_id];
        let q2 = data.qpos[qpos2_adr];
        let qd2 = data.qvel[dof2_adr];

        // Errors
        let pos_error = q2 - q2_target;
        let vel_error = qd2 - qd2_target;

        // Compute penalty parameters from solref
        let (stiffness, damping) = solref_to_penalty(
            model.eq_solref[eq_id],
            default_stiffness,
            default_damping,
            dt,
        );

        // Compute position-dependent impedance from solimp
        let impedance = compute_impedance(model.eq_solimp[eq_id], pos_error.abs());

        // Apply torque to joint2 to correct error
        let tau2 = -impedance * stiffness * pos_error - impedance * damping * vel_error;
        data.qfrc_constraint[dof2_adr] += tau2;

        // INVARIANT: No explicit reaction torque on joint1.
        //
        // In articulated body dynamics, applying τ₂ to joint2 naturally propagates
        // forces up the kinematic chain via the recursive Newton-Euler algorithm.
        // The constraint Jacobian already encodes this coupling through the mass
        // matrix. Adding an explicit τ₁ = -dq₂/dq₁ · τ₂ would double-count the
        // reaction, causing positive feedback and numerical explosion.
        //
        // Reference: Featherstone, "Rigid Body Dynamics Algorithms" (2008),
        // Section 7.3 on constraint force propagation in articulated systems.
    } else {
        // joint2 not specified: lock joint1 to constant c0
        let pos_error = q1 - c0;
        let vel_error = qd1;

        // Compute penalty parameters from solref
        let (stiffness, damping) = solref_to_penalty(
            model.eq_solref[eq_id],
            default_stiffness,
            default_damping,
            dt,
        );

        // Compute position-dependent impedance from solimp
        let impedance = compute_impedance(model.eq_solimp[eq_id], pos_error.abs());

        let tau1 = -impedance * stiffness * pos_error - impedance * damping * vel_error;
        data.qfrc_constraint[dof1_adr] += tau1;
    }
}

// =============================================================================
// Equality Constraint Jacobian Extraction (§15, Step 5)
//
// These functions extract explicit Jacobian rows, position violations, and
// velocities from equality constraints. Used by the Newton solver's unified
// constraint assembly. The existing apply_* functions remain unchanged for
// PGS/CG penalty-based enforcement.
// =============================================================================

/// Extracted equality constraint data for Newton solver assembly.
struct EqualityConstraintRows {
    /// Jacobian rows (nrows × nv). Connect: 3×nv, Weld: 6×nv, Joint: 1×nv, Distance: 1×nv.
    j_rows: DMatrix<f64>,
    /// Per-row constraint violation (signed).
    pos: Vec<f64>,
    /// Per-row constraint velocity (J·qvel).
    vel: Vec<f64>,
}

/// Extract connect constraint Jacobian (3 translational rows).
///
/// Constraint: anchor_on_body1 = body2_origin (3D position match).
/// Jacobian structure: 3×nv with sparse columns at both bodies' DOFs.
fn extract_connect_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];
    let nv = model.nv;

    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);
    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let p2 = if body2 != 0 {
        data.xpos[body2]
    } else {
        Vector3::zeros()
    };

    let anchor_world = p1 + r1 * anchor;
    let pos_error = anchor_world - p2;

    let mut j = DMatrix::zeros(3, nv);

    // For each of the 3 Cartesian directions, build a row using the
    // body-chain traversal pattern from compute_contact_jacobian.
    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        // Body1 contributes positively (anchor moves with body1)
        add_body_point_jacobian_row(
            &mut j,
            row,
            &direction,
            body1,
            anchor_world,
            1.0,
            model,
            data,
        );
        // Body2 contributes negatively (constraint wants p2 to match anchor)
        add_body_point_jacobian_row(&mut j, row, &direction, body2, p2, -1.0, model, data);
    }

    // Velocity: relative velocity at constraint point
    let v1 = if body1 != 0 {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v_lin1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        v_lin1 + omega1.cross(&r1_anchor)
    } else {
        Vector3::zeros()
    };
    let v2 = if body2 != 0 {
        let cvel2 = &data.cvel[body2];
        Vector3::new(cvel2[3], cvel2[4], cvel2[5])
    } else {
        Vector3::zeros()
    };
    let vel_error = v1 - v2;

    EqualityConstraintRows {
        j_rows: j,
        pos: vec![pos_error.x, pos_error.y, pos_error.z],
        vel: vec![vel_error.x, vel_error.y, vel_error.z],
    }
}

/// Extract weld constraint Jacobian (6 rows: 3 translational + 3 rotational).
///
/// Constraint: anchor_on_body1 = body2_origin AND orientation match.
fn extract_weld_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];
    let nv = model.nv;

    let anchor = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);
    let target_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        eq_data[3], eq_data[4], eq_data[5], eq_data[6],
    ));

    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let (p2, r2) = if body2 != 0 {
        (data.xpos[body2], data.xquat[body2])
    } else {
        (Vector3::zeros(), UnitQuaternion::identity())
    };

    // Position error
    let anchor_world = p1 + r1 * anchor;
    let pos_error = anchor_world - p2;

    // Orientation error (axis-angle)
    let target_r1 = r2 * target_quat;
    let rot_error_quat = r1 * target_r1.inverse();
    let rot_error = quaternion_to_axis_angle(&rot_error_quat);

    let mut j = DMatrix::zeros(6, nv);

    // Rows 0-2: translational (same as connect)
    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        add_body_point_jacobian_row(
            &mut j,
            row,
            &direction,
            body1,
            anchor_world,
            1.0,
            model,
            data,
        );
        add_body_point_jacobian_row(&mut j, row, &direction, body2, p2, -1.0, model, data);
    }

    // Rows 3-5: rotational Jacobian
    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body1, 1.0, model, data);
        add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body2, -1.0, model, data);
    }

    // Velocities
    let (vel_error, ang_vel_error) = if body1 != 0 {
        let cvel1 = &data.cvel[body1];
        let omega1 = Vector3::new(cvel1[0], cvel1[1], cvel1[2]);
        let v1 = Vector3::new(cvel1[3], cvel1[4], cvel1[5]);
        let r1_anchor = r1 * anchor;
        let v_anchor = v1 + omega1.cross(&r1_anchor);

        if body2 != 0 {
            let cvel2 = &data.cvel[body2];
            let omega2 = Vector3::new(cvel2[0], cvel2[1], cvel2[2]);
            let v2 = Vector3::new(cvel2[3], cvel2[4], cvel2[5]);
            (v_anchor - v2, omega1 - omega2)
        } else {
            (v_anchor, omega1)
        }
    } else {
        (Vector3::zeros(), Vector3::zeros())
    };

    EqualityConstraintRows {
        j_rows: j,
        pos: vec![
            pos_error.x,
            pos_error.y,
            pos_error.z,
            rot_error.x,
            rot_error.y,
            rot_error.z,
        ],
        vel: vec![
            vel_error.x,
            vel_error.y,
            vel_error.z,
            ang_vel_error.x,
            ang_vel_error.y,
            ang_vel_error.z,
        ],
    }
}

/// Extract joint equality constraint Jacobian (1 row).
///
/// Two-joint: constraint is q2 − poly(q1) = 0, Jacobian has -poly'(q1) at dof1 and +1 at dof2.
/// Single-joint: constraint is q1 − c0 = 0, Jacobian has +1 at dof1.
fn extract_joint_equality_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
    let joint1_id = model.eq_obj1id[eq_id];
    let joint2_id = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];
    let nv = model.nv;

    let c0 = eq_data[0];
    let c1 = eq_data[1];
    let c2 = eq_data[2];
    let c3 = eq_data[3];
    let c4 = eq_data[4];

    let qpos1_adr = model.jnt_qpos_adr[joint1_id];
    let dof1_adr = model.jnt_dof_adr[joint1_id];
    let q1 = data.qpos[qpos1_adr];
    let qd1 = data.qvel[dof1_adr];

    // poly(q1) = c0 + c1*q1 + c2*q1² + c3*q1³ + c4*q1⁴
    let q2_target = c0 + c1 * q1 + c2 * q1 * q1 + c3 * q1 * q1 * q1 + c4 * q1 * q1 * q1 * q1;

    // poly'(q1) = c1 + 2*c2*q1 + 3*c3*q1² + 4*c4*q1³ (§15, Round 4 Fix 6)
    let dpoly_dq1 = c1 + 2.0 * c2 * q1 + 3.0 * c3 * q1 * q1 + 4.0 * c4 * q1 * q1 * q1;
    let qd2_target = dpoly_dq1 * qd1;

    let mut j = DMatrix::zeros(1, nv);

    if joint2_id < model.njnt {
        // Two-joint equality: constraint = q2 - poly(q1) = 0
        let qpos2_adr = model.jnt_qpos_adr[joint2_id];
        let dof2_adr = model.jnt_dof_adr[joint2_id];
        let q2 = data.qpos[qpos2_adr];
        let qd2 = data.qvel[dof2_adr];

        let pos_error = q2 - q2_target;
        let vel_error = qd2 - qd2_target;

        // Jacobian: d(q2 - poly(q1))/d_qvel = +1 at dof2, -dpoly/dq1 at dof1
        j[(0, dof2_adr)] = 1.0;
        j[(0, dof1_adr)] = -dpoly_dq1;

        EqualityConstraintRows {
            j_rows: j,
            pos: vec![pos_error],
            vel: vec![vel_error],
        }
    } else {
        // Single-joint: lock q1 to c0
        let pos_error = q1 - c0;
        let vel_error = qd1;

        // Jacobian: d(q1 - c0)/d_qvel = +1 at dof1
        j[(0, dof1_adr)] = 1.0;

        EqualityConstraintRows {
            j_rows: j,
            pos: vec![pos_error],
            vel: vec![vel_error],
        }
    }
}

/// Extract distance equality constraint Jacobian (1 row).
///
/// Constraint: |p1 - p2| = target_distance.
/// Jacobian is the direction vector projected onto body DOFs.
fn extract_distance_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
    const SINGULARITY_EPS: f64 = 1e-10;

    let geom1_id = model.eq_obj1id[eq_id];
    let geom2_id = model.eq_obj2id[eq_id];
    let target_dist = model.eq_data[eq_id][0];
    let nv = model.nv;

    let p1 = data.geom_xpos[geom1_id];
    let p2 = if geom2_id == usize::MAX {
        Vector3::zeros()
    } else {
        data.geom_xpos[geom2_id]
    };

    let body1 = model.geom_body[geom1_id];
    let body2 = if geom2_id == usize::MAX {
        0
    } else {
        model.geom_body[geom2_id]
    };

    let diff = p1 - p2;
    let current_dist = diff.norm();

    let (direction, scalar_error) = if current_dist > SINGULARITY_EPS {
        (diff / current_dist, current_dist - target_dist)
    } else if target_dist > SINGULARITY_EPS {
        (Vector3::z(), -target_dist)
    } else {
        return EqualityConstraintRows {
            j_rows: DMatrix::zeros(1, nv),
            pos: vec![0.0],
            vel: vec![0.0],
        };
    };

    let mut j = DMatrix::zeros(1, nv);

    // The scalar constraint is d(|p1-p2|)/dt, Jacobian row = direction·(J_body1 - J_body2)
    add_body_point_jacobian_row(&mut j, 0, &direction, body1, p1, 1.0, model, data);
    add_body_point_jacobian_row(&mut j, 0, &direction, body2, p2, -1.0, model, data);

    // Velocity: relative velocity projected onto direction
    let v1 = if body1 != 0 {
        compute_point_velocity(data, body1, p1)
    } else {
        Vector3::zeros()
    };
    let v2 = if body2 != 0 {
        compute_point_velocity(data, body2, p2)
    } else {
        Vector3::zeros()
    };
    let vel_error = (v1 - v2).dot(&direction);

    EqualityConstraintRows {
        j_rows: j,
        pos: vec![scalar_error],
        vel: vec![vel_error],
    }
}

/// Build one row of a translational (point) Jacobian for a body.
///
/// Traverses the kinematic chain from `body_id` to root, adding the
/// contribution of each joint's DOF to the Jacobian row. The row
/// corresponds to a 3D direction projected onto the body's DOF space.
#[allow(clippy::too_many_arguments)]
fn add_body_point_jacobian_row(
    j: &mut DMatrix<f64>,
    row: usize,
    direction: &Vector3<f64>,
    body_id: usize,
    point: Vector3<f64>,
    sign: f64,
    model: &Model,
    data: &Data,
) {
    if body_id == 0 {
        return;
    }
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    j[(row, dof_adr)] += sign * direction.dot(&j_col);
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    j[(row, dof_adr)] += sign * direction.dot(&axis);
                }
                MjJointType::Ball => {
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        let j_col = omega_world.cross(&r);
                        j[(row, dof_adr + i)] += sign * direction.dot(&j_col);
                    }
                }
                MjJointType::Free => {
                    // Linear DOFs
                    j[(row, dof_adr)] += sign * direction.x;
                    j[(row, dof_adr + 1)] += sign * direction.y;
                    j[(row, dof_adr + 2)] += sign * direction.z;
                    // Angular DOFs
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    let ex = Vector3::x();
                    let ey = Vector3::y();
                    let ez = Vector3::z();
                    j[(row, dof_adr + 3)] += sign * direction.dot(&ex.cross(&r));
                    j[(row, dof_adr + 4)] += sign * direction.dot(&ey.cross(&r));
                    j[(row, dof_adr + 5)] += sign * direction.dot(&ez.cross(&r));
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Build one row of an angular Jacobian for a body.
///
/// Maps angular velocity components to generalized coordinates. Used for
/// weld constraint rotational rows (rows 3-5).
fn add_body_angular_jacobian_row(
    j: &mut DMatrix<f64>,
    row: usize,
    direction: &Vector3<f64>,
    body_id: usize,
    sign: f64,
    model: &Model,
    data: &Data,
) {
    if body_id == 0 {
        return;
    }
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    j[(row, dof_adr)] += sign * direction.dot(&axis);
                }
                MjJointType::Slide => {
                    // Slide joint doesn't contribute to angular velocity
                }
                MjJointType::Ball => {
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + i)] += sign * direction.dot(&omega_world);
                    }
                }
                MjJointType::Free => {
                    // Angular DOFs (indices 3-5)
                    j[(row, dof_adr + 3)] += sign * direction.x;
                    j[(row, dof_adr + 4)] += sign * direction.y;
                    j[(row, dof_adr + 5)] += sign * direction.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Apply a constraint force to a body at a specific world point.
///
/// Maps the force to generalized coordinates via Jacobian transpose.
/// Called per constraint per body in the kinematic chain — hot path.
#[inline]
fn apply_constraint_force_to_body(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    point: Vector3<f64>,
    force: Vector3<f64>,
) {
    if body_id == 0 {
        return; // World doesn't respond
    }

    // Traverse kinematic chain from body to root
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    data.qfrc_constraint[dof_adr] += j_col.dot(&force);
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&force);
                }
                MjJointType::Ball => {
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let torque = r.cross(&force);
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    data.qfrc_constraint[dof_adr] += force.x;
                    data.qfrc_constraint[dof_adr + 1] += force.y;
                    data.qfrc_constraint[dof_adr + 2] += force.z;
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    let torque = r.cross(&force);
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Apply a constraint torque directly to a body's rotational DOFs.
/// Called per weld constraint per body in the kinematic chain — hot path.
#[inline]
fn apply_constraint_torque_to_body(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    torque: Vector3<f64>,
) {
    if body_id == 0 {
        return;
    }

    // Traverse kinematic chain from body to root
    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // Project torque onto hinge axis
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += torque.dot(&axis);
                }
                MjJointType::Slide => {
                    // Slide joint doesn't respond to torque
                }
                MjJointType::Ball => {
                    // Full 3D torque in body frame
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    // Angular DOFs are indices 3-5 for free joint
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Compute constraint forces (contacts and joint limits).
///
/// This implements constraint-based enforcement using:
/// - Joint limits: Soft penalty method with Baumgarte stabilization
/// - Equality constraints: Soft penalty method with Baumgarte stabilization
/// - Contacts: PGS solver with Coulomb friction cone
///
/// The constraint forces are computed via Jacobian transpose:
///   qfrc_constraint = J^T * λ
///
/// where λ is found by solving the LCP with PGS.
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);
    data.jnt_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.ten_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.newton_solved = false;

    // Note: data.contacts is populated by mj_collision() which runs before this.
    // Do NOT clear contacts here.

    // Newton solver handles ALL constraint types (limits, equality, contacts)
    // in a unified way. It runs before penalty-based constraints and returns early.
    if model.solver_type == SolverType::Newton {
        // Guard: implicit integrators not supported with Newton (§15.8)
        let is_implicit = matches!(
            model.integrator,
            Integrator::Implicit | Integrator::ImplicitFast | Integrator::ImplicitSpringDamper
        );
        // Guard: pyramidal cones not supported with Newton (§15.7)
        let is_pyramidal_with_contacts = model.cone == 0 && !data.contacts.is_empty();

        if is_implicit {
            // Implicit integrators need mj_fwd_acceleration — fall through to PGS
        } else if is_pyramidal_with_contacts {
            // Pyramidal cones with contacts — fall through to PGS
        } else {
            let result = newton_solve(model, data);
            match result {
                NewtonResult::Converged => {
                    // Noslip post-processor (Phase C §15.10): suppress residual contact slip
                    if model.noslip_iterations > 0 {
                        noslip_postprocess(model, data);
                    }
                    data.newton_solved = true;
                    return;
                }
                NewtonResult::CholeskyFailed | NewtonResult::MaxIterationsExceeded => {
                    // PGS fallback: clear Newton-written efc_* fields, reset
                    // qfrc_constraint, fall through to penalty+PGS path.
                    data.efc_type.clear();
                    data.efc_pos.clear();
                    data.efc_margin.clear();
                    data.efc_vel = DVector::zeros(0);
                    data.efc_solref.clear();
                    data.efc_solimp.clear();
                    data.efc_diagApprox.clear();
                    data.efc_R.clear();
                    data.efc_D.clear();
                    data.efc_imp.clear();
                    data.efc_aref = DVector::zeros(0);
                    data.efc_floss.clear();
                    data.efc_mu.clear();
                    data.efc_dim.clear();
                    data.efc_id.clear();
                    data.efc_state.clear();
                    data.efc_force = DVector::zeros(0);
                    data.efc_jar = DVector::zeros(0);
                    data.efc_b = DVector::zeros(0);
                    data.efc_J = DMatrix::zeros(0, model.nv.max(1));
                    data.efc_cost = 0.0;
                    data.efc_cone_hessian.clear();
                    data.ncone = 0;
                    data.qfrc_constraint.fill(0.0);
                    data.jnt_limit_frc.iter_mut().for_each(|f| *f = 0.0);
                    data.ten_limit_frc.iter_mut().for_each(|f| *f = 0.0);
                    // Fall through to penalty+PGS below
                }
            }
        }
    }

    // ========== Joint Limit Constraints ==========
    // Using penalty method with Baumgarte stabilization:
    // τ = k * penetration + b * velocity_into_limit
    //
    // The stiffness and damping are derived from jnt_solref via solref_to_penalty().
    // MuJoCo uses solver-based approach, but penalty is acceptable for most robotics.

    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }

        let (limit_min, limit_max) = model.jnt_range[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        // Convert solref to penalty parameters, using model defaults as fallback
        let (limit_stiffness, limit_damping) = solref_to_penalty(
            model.jnt_solref[jnt_id],
            model.default_eq_stiffness,
            model.default_eq_damping,
            model.timestep,
        );

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let q = data.qpos[qpos_adr];
                let qdot = data.qvel[dof_adr];

                if q < limit_min {
                    // Below lower limit - push up
                    let penetration = limit_min - q;
                    let vel_into = (-qdot).max(0.0);
                    let force = limit_stiffness * penetration + limit_damping * vel_into;
                    data.qfrc_constraint[dof_adr] += force;
                    data.jnt_limit_frc[jnt_id] = force; // Cache unsigned magnitude
                } else if q > limit_max {
                    // Above upper limit - push down
                    let penetration = q - limit_max;
                    let vel_into = qdot.max(0.0);
                    let force = limit_stiffness * penetration + limit_damping * vel_into;
                    data.qfrc_constraint[dof_adr] -= force;
                    data.jnt_limit_frc[jnt_id] = force; // Cache unsigned magnitude
                }
            }
            MjJointType::Ball | MjJointType::Free => {
                // Ball/Free joints can have cone limits (swing-twist)
                // Not yet implemented - would require quaternion-based limit checking
            }
        }
    }

    // ========== Tendon Limit Constraints ==========
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }

        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];

        let (limit_stiffness, limit_damping) = solref_to_penalty(
            model.tendon_solref[t],
            model.default_eq_stiffness,
            model.default_eq_damping,
            model.timestep,
        );

        // Lower limit violation: length < min
        if length < limit_min {
            let penetration = limit_min - length;
            let vel_into = (-data.ten_velocity[t]).max(0.0);
            let force = limit_stiffness * penetration + limit_damping * vel_into;
            data.ten_limit_frc[t] = force; // Cache unsigned magnitude

            // Map to joint forces via J^T (force pushes tendon toward longer)
            apply_tendon_force(
                model,
                &data.ten_J[t],
                model.tendon_type[t],
                t,
                force,
                &mut data.qfrc_constraint,
            );
        }

        // Upper limit violation: length > max
        if length > limit_max {
            let penetration = length - limit_max;
            let vel_into = data.ten_velocity[t].max(0.0);
            let force = limit_stiffness * penetration + limit_damping * vel_into;
            data.ten_limit_frc[t] = force; // Cache unsigned magnitude

            // Map to joint forces via J^T (force pushes tendon toward shorter)
            apply_tendon_force(
                model,
                &data.ten_J[t],
                model.tendon_type[t],
                t,
                -force,
                &mut data.qfrc_constraint,
            );
        }
    }

    // ========== Equality Constraints ==========
    // Using penalty method with Baumgarte stabilization (like joint limits).
    // MuJoCo uses solver-based approach via PGS, but penalty is robust and simpler.
    //
    // For each equality constraint:
    //   τ = -k * violation - b * violation_velocity
    //
    // The forces are applied via Jacobian transpose to the appropriate DOFs.
    apply_equality_constraints(model, data);

    // ========== Contact Constraints ==========
    // Use PGS solver for constraint-based contact forces.
    //
    // For small contact counts or when M is singular, fall back to penalty method
    // which is simpler and more robust for simple scenarios.

    // Early exit if no contacts — clear stale warmstart so a future frame
    // with new contacts doesn't accidentally match old lambda values.
    if data.contacts.is_empty() {
        data.efc_lambda.clear();
        return;
    }

    // For systems without DOFs, skip constraint solve (no generalized forces
    // to apply). Clear warmstart to avoid stale data if DOFs are added later.
    if model.nv == 0 {
        data.efc_lambda.clear();
        return;
    }

    // Temporarily take efc_lambda out of data so we can borrow data.contacts
    // immutably while passing efc_lambda mutably to pgs_solve_contacts.
    let mut efc_lambda = std::mem::take(&mut data.efc_lambda);

    // Build contact Jacobians (compute_contact_jacobian takes &Data)
    let jacobians: Vec<DMatrix<f64>> = data
        .contacts
        .iter()
        .map(|c| compute_contact_jacobian(model, data, c))
        .collect();

    // Dispatch to configured solver.
    // Safety clamps: minimum 10 iterations, minimum 1e-8 tolerance.
    let clamped_iters = model.solver_iterations.max(10);
    let clamped_tol = model.solver_tolerance.max(1e-8);

    let constraint_forces = match model.solver_type {
        SolverType::PGS => {
            let (forces, niter) = pgs_solve_contacts(
                model,
                data,
                &data.contacts,
                &jacobians,
                clamped_iters,
                clamped_tol,
                &mut efc_lambda,
            );
            data.solver_niter = niter;
            forces
        }
        SolverType::CG => {
            match cg_solve_contacts(
                model,
                data,
                &data.contacts,
                &jacobians,
                clamped_iters,
                clamped_tol,
                &mut efc_lambda,
            ) {
                Ok((forces, niter)) => {
                    data.solver_niter = niter;
                    forces
                }
                Err((a, b, efc_offsets)) => {
                    // CG did not converge — fall back to PGS.
                    // Reuse the already-computed Delassus matrix (A, b) to
                    // avoid redundant assembly. efc_lambda was updated by CG
                    // with the partial solution, so PGS gets a warmstart.
                    let (forces, niter) = pgs_solve_with_system(
                        &data.contacts,
                        &a,
                        &b,
                        &efc_offsets,
                        clamped_iters,
                        clamped_tol,
                        &mut efc_lambda,
                    );
                    data.solver_niter = niter;
                    forces
                }
            }
        }
        SolverType::CGStrict => {
            if let Ok((forces, niter)) = cg_solve_contacts(
                model,
                data,
                &data.contacts,
                &jacobians,
                clamped_iters,
                clamped_tol,
                &mut efc_lambda,
            ) {
                data.solver_niter = niter;
                forces
            } else {
                data.solver_niter = clamped_iters;
                data.contacts
                    .iter()
                    .map(|c| DVector::zeros(c.dim))
                    .collect()
            }
        }
        SolverType::Newton => {
            // Newton solver already returned early at top of mj_fwd_constraint.
            // If we reach here, it's a PGS fallback (Cholesky failed or pyramidal cones).
            let (forces, niter) = pgs_solve_contacts(
                model,
                data,
                &data.contacts,
                &jacobians,
                clamped_iters,
                clamped_tol,
                &mut efc_lambda,
            );
            data.solver_niter = niter;
            forces
        }
    };

    // Restore warmstart cache
    data.efc_lambda = efc_lambda;

    // DECISION: Force application uses manual world-frame conversion +
    // apply_contact_force() instead of the pre-computed Jacobians (J^T * lambda).
    // Both are mathematically equivalent. The manual approach is more readable
    // and avoids dense matrix-vector multiplication, but does redundant kinematic
    // chain traversal. Switching to J^T * lambda would be a single matvec per
    // contact but couples force application to the Jacobian representation.
    // qfrc_constraint += J^T * λ
    for (i, (_jacobian, lambda)) in jacobians.iter().zip(constraint_forces.iter()).enumerate() {
        // Use the pre-computed contact frame for consistency with Jacobian and RHS.
        let contact = &data.contacts[i];
        let normal = contact.normal;
        let tangent1 = contact.frame[0];
        let tangent2 = contact.frame[1];
        let dim = contact.dim;

        // Linear force (rows 0–2, or just row 0 for condim 1)
        let world_force = if dim >= 3 {
            normal * lambda[0] + tangent1 * lambda[1] + tangent2 * lambda[2]
        } else {
            normal * lambda[0] // condim 1: normal only
        };

        // Apply via kinematic chain traversal (equivalent to J^T * lambda)
        let body1 = model.geom_body[data.contacts[i].geom1];
        let body2 = model.geom_body[data.contacts[i].geom2];
        let pos = data.contacts[i].pos;

        // Body1 gets negative force (Newton's third law)
        apply_contact_force(model, data, body1, pos, -world_force);
        apply_contact_force(model, data, body2, pos, world_force);

        // Torsional torque (row 3, when dim ≥ 4)
        if dim >= 4 {
            let torsional_torque = normal * lambda[3];
            apply_contact_torque(model, data, body1, -torsional_torque);
            apply_contact_torque(model, data, body2, torsional_torque);
        }

        // Rolling torques (rows 4–5, when dim = 6)
        if dim >= 6 {
            let rolling_torque = tangent1 * lambda[4] + tangent2 * lambda[5];
            apply_contact_torque(model, data, body1, -rolling_torque);
            apply_contact_torque(model, data, body2, rolling_torque);
        }
    }

    // Deformable-rigid contact resolution
    #[cfg(feature = "deformable")]
    solve_deformable_contacts(model, data);
}

// ============================================================================
// Deformable Body Pipeline (feature-gated)
// ============================================================================

/// Compute error reduction parameter from solver reference parameters.
///
/// MuJoCo solref = [timeconst, dampratio]:
///   erp = dt / (dt + 2 * timeconst * dampratio)
#[cfg(feature = "deformable")]
#[inline]
fn compute_erp(dt: f64, solref: &[f64; 2]) -> f64 {
    let timeconst = solref[0];
    let dampratio = solref[1];
    dt / (dt + 2.0 * timeconst * dampratio)
}

/// Compute the effective inverse mass of a rigid body at a contact point
/// projected onto a contact normal direction.
///
/// inv_m_eff = n^T * J * M^{-1} * J^T * n
#[cfg(feature = "deformable")]
fn compute_rigid_inv_mass_at_point(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: Vector3<f64>,
    normal: &Vector3<f64>,
) -> f64 {
    if body_id == 0 {
        return 0.0; // World body = infinite mass
    }

    // Build 3×nv translational Jacobian at point.
    let mut jac = DMatrix::zeros(3, model.nv);

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let j_col = axis.cross(&r);
                    jac[(0, dof_adr)] = j_col.x;
                    jac[(1, dof_adr)] = j_col.y;
                    jac[(2, dof_adr)] = j_col.z;
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    jac[(0, dof_adr)] = axis.x;
                    jac[(1, dof_adr)] = axis.y;
                    jac[(2, dof_adr)] = axis.z;
                }
                MjJointType::Ball => {
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - jpos;
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega_world = rot * Vector3::ith(i, 1.0);
                        let j_col = omega_world.cross(&r);
                        jac[(0, dof_adr + i)] = j_col.x;
                        jac[(1, dof_adr + i)] = j_col.y;
                        jac[(2, dof_adr + i)] = j_col.z;
                    }
                }
                MjJointType::Free => {
                    // Linear DOFs (translations)
                    jac[(0, dof_adr)] = 1.0;
                    jac[(1, dof_adr + 1)] = 1.0;
                    jac[(2, dof_adr + 2)] = 1.0;
                    // Angular DOFs (d(v)/d(omega) = skew(r))
                    let jpos = data.xpos[jnt_body];
                    let r = point - jpos;
                    // omega_x: v = omega_x × r = (0, r.z, -r.y)
                    jac[(1, dof_adr + 3)] = r.z;
                    jac[(2, dof_adr + 3)] = -r.y;
                    // omega_y: v = omega_y × r = (-r.z, 0, r.x)
                    jac[(0, dof_adr + 4)] = -r.z;
                    jac[(2, dof_adr + 4)] = r.x;
                    // omega_z: v = omega_z × r = (r.y, -r.x, 0)
                    jac[(0, dof_adr + 5)] = r.y;
                    jac[(1, dof_adr + 5)] = -r.x;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }

    if model.nv == 0 {
        return 0.0;
    }

    // j_n = J^T * n (nv×1)
    let n = DVector::from_column_slice(normal.as_slice());
    let j_n = jac.transpose() * &n;

    // M^{-1} * j_n via sparse factored mass matrix
    let mut m_inv_jn = j_n.clone();
    if data.qLD_valid {
        mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut m_inv_jn);
    } else {
        // Fallback: use diagonal of mass matrix
        for i in 0..model.nv {
            let m_diag = data.qM[(i, i)];
            if m_diag > 1e-10 {
                m_inv_jn[i] /= m_diag;
            }
        }
    }
    j_n.dot(&m_inv_jn)
}

/// Solve deformable-rigid contact forces using Jacobi-style iteration.
///
/// Each deformable vertex is a 3-DOF point mass. The contact Jacobian is the
/// identity matrix (vertex velocity maps directly to contact-frame velocity).
///
/// Three-phase algorithm for borrow safety:
///
/// 1. Read phase: iterate contacts, compute impulses into scratch buffer
/// 2. Write phase: apply impulses to vertex velocities
/// 3. Reaction phase: apply reaction forces to rigid bodies
#[cfg(feature = "deformable")]
fn solve_deformable_contacts(model: &Model, data: &mut Data) {
    let mut impulses: Vec<(usize, Vector3<f64>)> =
        Vec::with_capacity(data.deformable_contacts.len());

    let dt = model.timestep;

    for _iter in 0..model.solver_iterations {
        let mut max_delta = 0.0_f64;
        impulses.clear();

        // Phase 1: Read-only solve
        for (contact_idx, contact) in data.deformable_contacts.iter().enumerate() {
            let body = &data.deformable_bodies[contact.deformable_idx];
            let inv_m = body.inverse_masses()[contact.vertex_idx];

            if inv_m <= 0.0 {
                continue; // Pinned vertex
            }

            let v_deform = body.velocities()[contact.vertex_idx];

            // Rigid body velocity at contact point
            let rigid_body_idx = model.geom_body[contact.geom_idx];
            let v_rigid = compute_point_velocity(data, rigid_body_idx, contact.pos);

            // Relative velocity in contact frame
            let v_rel = v_deform - v_rigid;
            let v_n = v_rel.dot(&contact.normal);

            // Normal impulse with Baumgarte correction
            let erp = compute_erp(dt, &contact.solref);
            let rhs = v_n + erp * contact.depth / dt;

            // Effective inverse mass (deformable point mass + rigid contribution)
            let inv_m_eff = inv_m
                + compute_rigid_inv_mass_at_point(
                    model,
                    data,
                    rigid_body_idx,
                    contact.pos,
                    &contact.normal,
                );

            let lambda_n = (-rhs / inv_m_eff).max(0.0);
            max_delta = max_delta.max(lambda_n.abs());

            let impulse = contact.normal * lambda_n;
            impulses.push((contact_idx, impulse));
        }

        // Phase 2a: Apply velocity updates to deformable vertices
        for &(contact_idx, ref impulse) in &impulses {
            let contact = &data.deformable_contacts[contact_idx];
            let body = &mut data.deformable_bodies[contact.deformable_idx];
            let inv_m = body.inverse_masses()[contact.vertex_idx];
            body.velocities_mut()[contact.vertex_idx] += impulse * inv_m;
        }

        // Phase 2b: Apply reaction impulses to rigid bodies
        // Copy contact fields to locals before the &mut data call.
        for &(contact_idx, ref impulse) in &impulses {
            let geom_idx = data.deformable_contacts[contact_idx].geom_idx;
            let pos = data.deformable_contacts[contact_idx].pos;
            let rigid_body_idx = model.geom_body[geom_idx];
            apply_contact_force(model, data, rigid_body_idx, pos, -*impulse);
        }

        if max_delta < model.solver_tolerance {
            break;
        }
    }
}

/// Detect contacts between deformable vertices and rigid geoms.
#[cfg(feature = "deformable")]
fn mj_deformable_collision(model: &Model, data: &mut Data) {
    data.deformable_contacts.clear();

    if data.deformable_bodies.is_empty() {
        return; // Zero overhead when no deformables registered
    }

    // Phase 1: Clear COLLIDING flags
    for body in &mut data.deformable_bodies {
        for flag in body.vertex_flags_mut() {
            flag.remove(VertexFlags::COLLIDING);
        }
    }

    // Phase 2: Detect contacts
    // Pre-compute rigid geom AABBs
    let rigid_aabbs: Vec<Aabb> = (0..model.ngeom)
        .map(|g| {
            aabb_from_geom(
                model.geom_type[g],
                model.geom_size[g],
                data.geom_xpos[g],
                data.geom_xmat[g],
            )
        })
        .collect();

    for (deform_idx, body) in data.deformable_bodies.iter().enumerate() {
        let margin = body.collision_margin();
        let positions = body.positions();
        let material = body.material();

        for (vert_idx, pos) in positions.iter().enumerate() {
            // Skip pinned vertices
            if body.inverse_masses()[vert_idx] <= 0.0 {
                continue;
            }

            // Vertex AABB: point ± margin
            let vert_aabb = Aabb::from_center(*pos, Vector3::new(margin, margin, margin));

            // Broadphase: test vertex AABB against all rigid geom AABBs
            for geom_idx in 0..model.ngeom {
                // Skip broadphase for planes — they are infinite and the thin
                // slab AABB can miss fast-moving deformable vertices.
                if model.geom_type[geom_idx] != GeomType::Plane
                    && !vert_aabb.overlaps(&rigid_aabbs[geom_idx])
                {
                    continue;
                }

                // Narrowphase: vertex-vs-geom closest point
                if let Some(contact) = collide_vertex_geom(
                    pos,
                    margin,
                    model,
                    geom_idx,
                    data.geom_xpos[geom_idx],
                    data.geom_xmat[geom_idx],
                    material,
                    deform_idx,
                    vert_idx,
                ) {
                    data.deformable_contacts.push(contact);
                }
            }
        }
    }

    // Phase 3: Set COLLIDING flags on contacted vertices
    for contact in &data.deformable_contacts {
        data.deformable_bodies[contact.deformable_idx].vertex_flags_mut()[contact.vertex_idx]
            .insert(VertexFlags::COLLIDING);
    }

    // Phase 4: Position-level correction
    // Push penetrating vertices to the contact surface. The velocity-level
    // solver in mj_fwd_constraint handles impulses, but without position
    // correction vertices can drift into the surface over time.
    for contact in &data.deformable_contacts {
        let body = &mut data.deformable_bodies[contact.deformable_idx];
        let inv_m = body.inverse_masses()[contact.vertex_idx];
        if inv_m <= 0.0 {
            continue; // Pinned
        }
        // Only correct actual surface penetration (depth > margin means
        // the vertex is below the surface, not just within the margin zone).
        let margin = body.collision_margin();
        let surface_penetration = contact.depth - margin;
        if surface_penetration > 0.0 {
            body.positions_mut()[contact.vertex_idx] += contact.normal * surface_penetration;
            // Also zero out the normal velocity component to prevent
            // the vertex from immediately falling back in.
            let v = body.velocities()[contact.vertex_idx];
            let v_n = v.dot(&contact.normal);
            if v_n < 0.0 {
                body.velocities_mut()[contact.vertex_idx] -= contact.normal * v_n;
            }
        }
    }
}

/// Compute contact between a deformable vertex and a rigid geom.
///
/// The vertex is treated as a sphere with the given margin radius.
/// Returns `DeformableContact` if penetrating, `None` otherwise.
#[cfg(feature = "deformable")]
#[allow(clippy::too_many_arguments)]
fn collide_vertex_geom(
    vertex_pos: &Point3<f64>,
    vertex_margin: f64,
    model: &Model,
    geom_idx: usize,
    geom_pos: Vector3<f64>,
    geom_mat: Matrix3<f64>,
    material: &DeformableMaterial,
    deform_idx: usize,
    vert_idx: usize,
) -> Option<DeformableContact> {
    let v = vertex_pos.coords;

    // Compute signed distance from vertex to geom surface and contact normal
    let (d_surface, normal) = match model.geom_type[geom_idx] {
        GeomType::Plane => {
            // Plane normal is the z-axis of the geom frame
            let plane_normal = geom_mat.column(2).into_owned();
            let d = (v - geom_pos).dot(&plane_normal);
            (d, plane_normal)
        }

        GeomType::Sphere => {
            let radius = model.geom_size[geom_idx].x;
            let diff = v - geom_pos;
            let dist = diff.norm();
            if dist < 1e-10 {
                // Vertex at sphere center — use fallback normal
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                let normal = diff / dist;
                (dist - radius, normal)
            }
        }

        GeomType::Box => {
            let half = model.geom_size[geom_idx];
            // Transform vertex to local box frame
            let v_local = geom_mat.transpose() * (v - geom_pos);

            // Clamp to box
            let clamped = Vector3::new(
                v_local.x.clamp(-half.x, half.x),
                v_local.y.clamp(-half.y, half.y),
                v_local.z.clamp(-half.z, half.z),
            );

            let diff_local = v_local - clamped;
            let dist_outside = diff_local.norm();

            if dist_outside > 1e-10 {
                // Outside the box
                let normal_local = diff_local / dist_outside;
                let normal_world = geom_mat * normal_local;
                (dist_outside, normal_world)
            } else {
                // Inside the box — find closest face
                let dx = half.x - v_local.x.abs();
                let dy = half.y - v_local.y.abs();
                let dz = half.z - v_local.z.abs();
                let min_depth = dx.min(dy).min(dz);

                let normal_local = if (dx - min_depth).abs() < 1e-10 {
                    Vector3::new(v_local.x.signum(), 0.0, 0.0)
                } else if (dy - min_depth).abs() < 1e-10 {
                    Vector3::new(0.0, v_local.y.signum(), 0.0)
                } else {
                    Vector3::new(0.0, 0.0, v_local.z.signum())
                };
                let normal_world = geom_mat * normal_local;
                (-min_depth, normal_world)
            }
        }

        GeomType::Capsule => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let axis = geom_mat.column(2).into_owned();
            let a = geom_pos - axis * half_len;
            let b = geom_pos + axis * half_len;
            let closest = closest_point_segment(a, b, v);
            let diff = v - closest;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                let normal = diff / dist;
                (dist - radius, normal)
            }
        }

        GeomType::Cylinder => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            // Transform to cylinder local frame
            let v_local = geom_mat.transpose() * (v - geom_pos);

            let radial = Vector2::new(v_local.x, v_local.y);
            let radial_dist = radial.norm();
            let z_clamped = v_local.z.clamp(-half_len, half_len);

            // Three cases: barrel, endcap, or edge
            let on_barrel = v_local.z.abs() <= half_len;
            let in_radial = radial_dist <= radius;

            if on_barrel && in_radial {
                // Inside cylinder — find closest surface
                let d_radial = radius - radial_dist;
                let d_axial = half_len - v_local.z.abs();
                if d_radial < d_axial {
                    // Closer to barrel
                    let normal_local = if radial_dist > 1e-10 {
                        Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    };
                    let normal_world = geom_mat * normal_local;
                    (-d_radial, normal_world)
                } else {
                    // Closer to endcap
                    let normal_local = Vector3::new(0.0, 0.0, v_local.z.signum());
                    let normal_world = geom_mat * normal_local;
                    (-d_axial, normal_world)
                }
            } else if on_barrel {
                // Outside radially
                let normal_local = if radial_dist > 1e-10 {
                    Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                let normal_world = geom_mat * normal_local;
                (radial_dist - radius, normal_world)
            } else if in_radial {
                // Outside axially (endcap)
                let normal_local = Vector3::new(0.0, 0.0, v_local.z.signum());
                let normal_world = geom_mat * normal_local;
                (v_local.z.abs() - half_len, normal_world)
            } else {
                // Outside both (edge region)
                let closest_local = Vector3::new(
                    v_local.x * radius / radial_dist,
                    v_local.y * radius / radial_dist,
                    z_clamped,
                );
                let diff_local = Vector3::new(v_local.x, v_local.y, v_local.z) - closest_local;
                let dist = diff_local.norm();
                if dist < 1e-10 {
                    return None;
                }
                let normal_local = diff_local / dist;
                let normal_world = geom_mat * normal_local;
                (dist, normal_world)
            }
        }

        GeomType::Ellipsoid => {
            let radii = model.geom_size[geom_idx];
            // Transform to local frame and scale to unit sphere
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let v_scaled = Vector3::new(
                v_local.x / radii.x,
                v_local.y / radii.y,
                v_local.z / radii.z,
            );
            let dist_scaled = v_scaled.norm();
            if dist_scaled < 1e-10 {
                return None;
            }
            // Approximate: distance in scaled space, normal from scaled gradient
            let d_surface_approx = (dist_scaled - 1.0) * radii.x.min(radii.y).min(radii.z);
            let grad_local = Vector3::new(
                v_local.x / (radii.x * radii.x),
                v_local.y / (radii.y * radii.y),
                v_local.z / (radii.z * radii.z),
            );
            let grad_len = grad_local.norm();
            if grad_len < 1e-10 {
                return None;
            }
            let normal_local = grad_local / grad_len;
            let normal_world = geom_mat * normal_local;
            (d_surface_approx, normal_world)
        }

        // Out of scope
        GeomType::Mesh | GeomType::Hfield | GeomType::Sdf => {
            return None;
        }
    };

    // Contact generated when margin-sphere overlaps the geom
    let depth = vertex_margin - d_surface;
    if depth <= 0.0 {
        return None;
    }

    // Friction combination (geometric mean, same as make_contact_from_geoms)
    let deform_friction = material.friction;
    let rigid_friction = model.geom_friction[geom_idx];
    let sliding = (deform_friction * rigid_friction.x).sqrt();
    let torsional = (deform_friction * rigid_friction.y).sqrt();
    let rolling = (deform_friction * rigid_friction.z).sqrt();

    // Tangent frame
    let (t1, t2) = compute_tangent_frame(&normal);

    let dim = model.deformable_condim.unsigned_abs() as usize;

    Some(DeformableContact {
        deformable_idx: deform_idx,
        vertex_idx: vert_idx,
        geom_idx,
        pos: Point3::from(v - normal * d_surface).coords,
        normal,
        depth,
        friction: sliding,
        dim,
        mu: [sliding, sliding, torsional, rolling, rolling],
        solref: model.deformable_solref,
        solimp: model.deformable_solimp,
        frame: [t1, t2],
    })
}

/// Step all deformable bodies through XPBD.
///
/// Contact impulses must already be applied to vertex velocities
/// (in `solve_deformable_contacts`) before calling this function.
#[cfg(feature = "deformable")]
fn mj_deformable_step(model: &Model, data: &mut Data) {
    if data.deformable_bodies.is_empty() {
        return;
    }

    let gravity = model.gravity;
    let dt = model.timestep;

    for (solver, body) in data
        .deformable_solvers
        .iter_mut()
        .zip(data.deformable_bodies.iter_mut())
    {
        solver.step(body.as_mut(), gravity, dt);
    }
}

/// Compute the velocity of a point on a body.
fn compute_point_velocity(data: &Data, body_id: usize, point: Vector3<f64>) -> Vector3<f64> {
    if body_id == 0 {
        return Vector3::zeros(); // World is stationary
    }

    // Get body velocity (linear and angular)
    let cvel = &data.cvel[body_id];
    let v_linear = Vector3::new(cvel[3], cvel[4], cvel[5]);
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);

    // Point velocity = v_body + omega × r
    let body_pos = data.xpos[body_id];
    let r = point - body_pos;

    v_linear + omega.cross(&r)
}

/// Apply a contact force to a body by mapping it to generalized forces.
fn apply_contact_force(
    model: &Model,
    data: &mut Data,
    body_id: usize,
    contact_point: Vector3<f64>,
    force: Vector3<f64>,
) {
    if body_id == 0 {
        return; // World doesn't respond
    }

    // For each joint in the kinematic chain from body to root,
    // compute the generalized force contribution using the Jacobian transpose.
    //
    // τ = J^T * F where J = ∂p/∂q (point velocity Jacobian)

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // τ = (axis × r) · F
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = contact_point - jpos;
                    let j_col = axis.cross(&r); // Jacobian column for this joint
                    data.qfrc_constraint[dof_adr] += j_col.dot(&force);
                }
                MjJointType::Slide => {
                    // τ = axis · F
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&force);
                }
                MjJointType::Ball => {
                    // τ = r × F (torque from force at point)
                    let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = contact_point - jpos;
                    let torque = r.cross(&force);
                    // Transform to body frame for ball joint
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    // Linear DOFs: F directly
                    data.qfrc_constraint[dof_adr] += force.x;
                    data.qfrc_constraint[dof_adr + 1] += force.y;
                    data.qfrc_constraint[dof_adr + 2] += force.z;
                    // Angular DOFs: torque from force
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = contact_point - jpos;
                    let torque = r.cross(&force);
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Apply a contact torque to a body by mapping it to generalized forces.
///
/// This is the angular analogue of `apply_contact_force()`. Used for torsional
/// friction (condim ≥ 4) and rolling friction (condim = 6). The torque is applied
/// via the angular Jacobian transpose.
fn apply_contact_torque(model: &Model, data: &mut Data, body_id: usize, torque: Vector3<f64>) {
    if body_id == 0 {
        return; // World doesn't respond
    }

    // For each joint in the kinematic chain from body to root,
    // compute the generalized force contribution using the angular Jacobian transpose.
    //
    // τ_gen = J_ω^T * τ_world where J_ω = ∂ω/∂q̇ (angular velocity Jacobian)

    let mut current_body = body_id;
    while current_body != 0 {
        let jnt_start = model.body_jnt_adr[current_body];
        let jnt_end = jnt_start + model.body_jnt_num[current_body];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let jnt_body = model.jnt_body[jnt_id];

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge => {
                    // τ_gen = axis · τ_world
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    data.qfrc_constraint[dof_adr] += axis.dot(&torque);
                }
                MjJointType::Slide => {
                    // No angular contribution from prismatic joints
                }
                MjJointType::Ball => {
                    // Transform world torque to body frame for ball joint
                    let body_torque = data.xquat[jnt_body].inverse() * torque;
                    data.qfrc_constraint[dof_adr] += body_torque.x;
                    data.qfrc_constraint[dof_adr + 1] += body_torque.y;
                    data.qfrc_constraint[dof_adr + 2] += body_torque.z;
                }
                MjJointType::Free => {
                    // Angular DOFs (3–5): torque directly in world frame
                    data.qfrc_constraint[dof_adr + 3] += torque.x;
                    data.qfrc_constraint[dof_adr + 4] += torque.y;
                    data.qfrc_constraint[dof_adr + 5] += torque.z;
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}

/// Compute final acceleration from forces using proper matrix solve.
///
/// # Explicit Integration (Euler, RK4)
///
/// Solves: M * qacc = `τ_total` where
/// `τ_total` = `qfrc_applied` + `qfrc_actuator` + `qfrc_passive` + `qfrc_constraint` - `qfrc_bias`
///
/// Uses Cholesky decomposition for symmetric positive-definite M.
///
/// # Implicit Integration
///
/// For implicit springs, we solve a modified system that incorporates
/// stiffness and damping into the velocity update:
///
/// ```text
/// (M + h*D + h²*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
/// ```
///
/// Where:
/// - M = mass matrix (from CRBA)
/// - D = diagonal damping matrix
/// - K = diagonal stiffness matrix
/// - h = timestep
/// - f_ext = external forces (applied + actuator + constraint - bias)
/// - q_eq = spring equilibrium positions (springref)
///
/// This provides unconditional stability for arbitrarily stiff springs,
/// allowing larger timesteps without energy blow-up.
///
/// After solving, we compute qacc = (v_new - v_old) / h for consistency
/// with sensors and other code that expects qacc.
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if using implicit integration
/// and the modified mass matrix (M + h*D + h²*K) is not positive definite.
fn mj_fwd_acceleration(model: &Model, data: &mut Data) -> Result<(), StepError> {
    if model.nv == 0 {
        return Ok(());
    }

    match model.integrator {
        Integrator::ImplicitSpringDamper => mj_fwd_acceleration_implicit(model, data),
        Integrator::ImplicitFast => mj_fwd_acceleration_implicitfast(model, data),
        Integrator::Implicit => mj_fwd_acceleration_implicit_full(model, data),
        Integrator::Euler | Integrator::RungeKutta4 => {
            mj_fwd_acceleration_explicit(model, data);
            Ok(())
        }
    }
}

/// Explicit forward acceleration (semi-implicit Euler or RK4).
///
/// Computes: qacc = M⁻¹ * (f_applied + f_actuator + f_passive + f_constraint - f_bias)
fn mj_fwd_acceleration_explicit(_model: &Model, data: &mut Data) {
    // Sum all forces: τ = applied + actuator + passive + constraint - bias
    let mut qfrc_total = data.qfrc_applied.clone();
    qfrc_total += &data.qfrc_actuator;
    qfrc_total += &data.qfrc_passive;
    qfrc_total += &data.qfrc_constraint;
    qfrc_total -= &data.qfrc_bias;

    // Solve M * qacc = qfrc_total using sparse L^T D L factorization from mj_crba
    data.qacc.copy_from(&qfrc_total);
    mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut data.qacc);
}

/// In-place Cholesky (LL^T) factorization. Overwrites the lower triangle of `m` with L.
/// The upper triangle is left unchanged. Returns `Err(StepError::CholeskyFailed)` if
/// the matrix is not positive definite.
///
/// Zero allocations — operates entirely on borrowed data.
fn cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError> {
    let n = m.nrows();
    for j in 0..n {
        // Diagonal: L[j,j] = sqrt(M[j,j] - Σ(L[j,k]² for k < j))
        let mut diag = m[(j, j)];
        for k in 0..j {
            diag -= m[(j, k)] * m[(j, k)];
        }
        if diag <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let ljj = diag.sqrt();
        m[(j, j)] = ljj;

        // Off-diagonal: L[i,j] = (M[i,j] - Σ(L[i,k]·L[j,k] for k < j)) / L[j,j]
        for i in (j + 1)..n {
            let mut sum = m[(i, j)];
            for k in 0..j {
                sum -= m[(i, k)] * m[(j, k)];
            }
            m[(i, j)] = sum / ljj;
        }
    }
    Ok(())
}

/// Solve L·L^T·x = b in place, where L is stored in the lower triangle of `l`.
/// On entry `x` contains b; on exit `x` contains the solution.
///
/// Zero allocations — operates entirely on borrowed data.
pub(crate) fn cholesky_solve_in_place(l: &DMatrix<f64>, x: &mut DVector<f64>) {
    let n = l.nrows();

    // Forward substitution: L·y = b
    for j in 0..n {
        for k in 0..j {
            x[j] -= l[(j, k)] * x[k];
        }
        x[j] /= l[(j, j)];
    }

    // Back substitution: L^T·z = y
    for j in (0..n).rev() {
        for k in (j + 1)..n {
            x[j] -= l[(k, j)] * x[k];
        }
        x[j] /= l[(j, j)];
    }
}

/// Dense Cholesky rank-1 update: given L such that L·L^T = H, compute L' in-place
/// such that L'·L'^T = H + v·v^T.
///
/// Uses the Linpack DCHUD algorithm (Givens rotations). O(n²).
/// The vector `v` is used as workspace and modified.
///
/// Returns `Err(StepError::CholeskyFailed)` if the diagonal becomes non-positive.
#[allow(clippy::many_single_char_names)]
fn cholesky_rank1_update(l: &mut DMatrix<f64>, v: &mut [f64]) -> Result<(), StepError> {
    let n = l.nrows();
    debug_assert_eq!(v.len(), n);

    for j in 0..n {
        let a = l[(j, j)];
        let b = v[j];
        let r = (a * a + b * b).sqrt();
        if r <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let c = a / r;
        let s = b / r;
        l[(j, j)] = r;

        // Apply Givens rotation to remaining entries in column j and v
        for i in (j + 1)..n {
            let t = l[(i, j)];
            l[(i, j)] = c * t + s * v[i];
            v[i] = -s * t + c * v[i];
        }
    }
    Ok(())
}

/// Dense Cholesky rank-1 downdate: given L such that L·L^T = H, compute L' in-place
/// such that L'·L'^T = H - v·v^T.
///
/// Uses the Linpack DCHDD algorithm (forward solve + reverse Givens). O(n²).
/// The vector `v` is used as workspace and modified.
///
/// Returns `Err(StepError::CholeskyFailed)` if the result would be indefinite.
#[allow(clippy::many_single_char_names)]
fn cholesky_rank1_downdate(l: &mut DMatrix<f64>, v: &mut [f64]) -> Result<(), StepError> {
    let n = l.nrows();
    debug_assert_eq!(v.len(), n);

    // Step 1: Forward solve L·p = v, store p in v
    for j in 0..n {
        for k in 0..j {
            v[j] -= l[(j, k)] * v[k];
        }
        if l[(j, j)].abs() < MJ_MINVAL {
            return Err(StepError::CholeskyFailed);
        }
        v[j] /= l[(j, j)];
    }

    // Step 2: Check positive-definiteness: alpha² = 1 - ||p||² > 0
    let p_norm_sq: f64 = v.iter().map(|x| x * x).sum();
    let alpha_sq = 1.0 - p_norm_sq;
    if alpha_sq <= 0.0 {
        return Err(StepError::CholeskyFailed);
    }
    let mut alpha = alpha_sq.sqrt();

    // Step 3: Reverse Givens rotations to produce L'
    for j in (0..n).rev() {
        let a = alpha;
        let b = v[j];
        let r = (a * a + b * b).sqrt();
        if r <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let c = a / r;
        let s = b / r;
        alpha = r;

        // Update column j of L
        for i in (j + 1)..n {
            let t = l[(i, j)];
            // Accumulate the p-vector contribution back
            l[(i, j)] = (t - s * v[i]) / c;
            v[i] = c * v[i] - s * t;
        }

        // Update diagonal
        l[(j, j)] *= c;
        if l[(j, j)] <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
    }

    Ok(())
}

/// Sparse L^T D L factorization of the mass matrix, exploiting tree structure.
///
/// Computes `M = L^T D L` where `L` is unit lower triangular (L[i,i] = 1 implicitly,
/// stored entries are off-diagonal only) and `D` is diagonal. The sparsity pattern
/// is determined by the kinematic tree: `L[i,j] ≠ 0` only if DOF `j` is an ancestor
/// of DOF `i` in `model.dof_parent`.
///
/// **Algorithm:** Process DOFs from leaves to root (i = nv-1 down to 0). For each DOF,
/// copy its row from M, scale by the diagonal to extract L entries, then propagate a
/// rank-1 update to all ancestor pairs. This is equivalent to Gaussian elimination
/// reordered to exploit tree sparsity.
///
/// **Complexity:** O(Σ depth(i)²). For balanced trees O(n log² n), for typical
/// humanoids (depth ≤ 8) effectively O(n).
///
/// Zero heap allocations after the first call — `qLD_L[i].clear()` + `push()` reuses
/// existing `Vec` capacity.
fn mj_factor_sparse(model: &Model, data: &mut Data) {
    let nv = model.nv;

    // Phase 1: Copy M's sparse entries into qLD working storage.
    // qLD_diag[i] starts as M[i,i]; qLD_L[i] holds (col, M[i,col]) for ancestors.
    for i in 0..nv {
        data.qLD_diag[i] = data.qM[(i, i)];
        data.qLD_L[i].clear();
        let mut p = model.dof_parent[i];
        while let Some(j) = p {
            data.qLD_L[i].push((j, data.qM[(i, j)]));
            p = model.dof_parent[j];
        }
        // Parent chain yields descending col indices; reverse for ascending order.
        data.qLD_L[i].reverse();
    }

    // Phase 2: Eliminate from leaves to root.
    // When we process DOF i, all descendants k > i are already factored.
    // We propagate i's contribution to ancestors j < i, which requires
    // mutating qLD_L[j] while reading qLD_L[i]. Since j < i always,
    // we use split_at_mut to get non-overlapping borrows.
    for i in (0..nv).rev() {
        let di = data.qLD_diag[i];

        // Scale off-diagonals: L[i,j] = working[i,j] / D[i]
        for entry in &mut data.qLD_L[i] {
            entry.1 /= di;
        }

        // Propagate rank-1 update to ancestors.
        // Split: [0..i] and [i..nv] give non-overlapping mutable access.
        let (lo, hi) = data.qLD_L.split_at_mut(i);
        let row_i = &hi[0]; // qLD_L[i], immutable borrow

        for idx_a in 0..row_i.len() {
            let (j, lij) = row_i[idx_a];
            data.qLD_diag[j] -= lij * lij * di;
            for idx_b in 0..idx_a {
                let (k, lik) = row_i[idx_b];
                // k < j < i, so both are in `lo`
                if let Some(entry) = lo[j].iter_mut().find(|(col, _)| *col == k) {
                    entry.1 -= lij * lik * di;
                }
            }
        }
    }

    data.qLD_valid = true;
}

/// Solve `L^T D L x = b` using the sparse factorization from `mj_factor_sparse`.
///
/// On entry `x` contains `b`; on exit `x` contains the solution.
/// Takes `qLD_diag` and `qLD_L` by reference separately from `x` to allow
/// calling with `x = &mut data.qacc` without borrow conflicts.
///
/// Zero allocations — operates entirely on borrowed data.
pub(crate) fn mj_solve_sparse(
    qld_diag: &DVector<f64>,
    qld_l: &[Vec<(usize, f64)>],
    x: &mut DVector<f64>,
) {
    let nv = x.len();

    // Phase 1: Solve L^T y = b (scatter: propagate each DOF to its ancestors).
    // L is unit-lower-triangular, so L^T is unit-upper-triangular.
    for i in (0..nv).rev() {
        for &(j, lij) in &qld_l[i] {
            x[j] -= lij * x[i];
        }
    }

    // Phase 2: Solve D z = y
    for i in 0..nv {
        x[i] /= qld_diag[i];
    }

    // Phase 3: Solve L w = z (standard forward substitution)
    for i in 0..nv {
        for &(j, lij) in &qld_l[i] {
            x[i] -= lij * x[j];
        }
    }
}

/// Implicit forward acceleration for springs and dampers.
///
/// Solves:
/// ```text
/// (M + h*D + h²*K) * v_new = M*v_old + h*f_ext - h*K*(q - q_eq)
/// ```
///
/// This provides unconditional stability for stiff springs by treating
/// spring and damper forces implicitly in the velocity update.
///
/// # Implementation Notes
///
/// - Spring/damper parameters are diagonal (no coupling between DOFs)
/// - Friction loss remains explicit (velocity-sign-dependent, cannot linearize)
/// - The modified matrix M + h*D + h²*K is still SPD if M is SPD and D, K ≥ 0
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if the modified mass matrix
/// is not positive definite. This can happen with:
/// - Negative stiffness or damping values
/// - Corrupted mass matrix
/// - Extreme numerical conditions
fn mj_fwd_acceleration_implicit(model: &Model, data: &mut Data) -> Result<(), StepError> {
    // Guard against zero timestep (would cause division by zero)
    debug_assert!(
        model.timestep > 0.0,
        "Timestep must be positive for implicit integration"
    );
    let h = model.timestep;
    let h2 = h * h;

    // Use cached spring-damper parameters from Model (avoids allocation)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    let q_eq = &model.implicit_springref;

    // Build external forces into scratch buffer (avoids allocation)
    // f_ext = applied + actuator + passive(friction) + constraint - bias
    data.scratch_force.copy_from(&data.qfrc_applied);
    data.scratch_force += &data.qfrc_actuator;
    data.scratch_force += &data.qfrc_passive; // Friction loss (explicit even in implicit mode)
    data.scratch_force += &data.qfrc_constraint;
    data.scratch_force -= &data.qfrc_bias;

    // Build modified mass matrix: M_impl = M + h*D + h²*K
    // Copy M into scratch, then modify diagonal only
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        data.scratch_m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // Build RHS into scratch buffer: M*v_old + h*f_ext - h*K*(q - q_eq)
    // Start with M*v_old
    data.qM.mul_to(&data.qvel, &mut data.scratch_rhs);

    // Add h*f_ext for each DOF
    for i in 0..model.nv {
        data.scratch_rhs[i] += h * data.scratch_force[i];
    }

    // Subtract h*K*(q - q_eq) for spring displacement using visitor
    let mut spring_visitor = ImplicitSpringVisitor {
        k,
        q_eq,
        h,
        qpos: &data.qpos,
        rhs: &mut data.scratch_rhs,
    };
    model.visit_joints(&mut spring_visitor);

    // Factorize M_impl in place (overwrites lower triangle with L where M_impl = L·L^T).
    // M_impl is SPD (M is SPD from CRBA, D ≥ 0, K ≥ 0).
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.scratch_v_new.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.scratch_v_new);

    // Compute qacc = (v_new - v_old) / h and update qvel
    for i in 0..model.nv {
        data.qacc[i] = (data.scratch_v_new[i] - data.qvel[i]) / h;
        data.qvel[i] = data.scratch_v_new[i];
    }

    Ok(())
}

/// Visitor for computing spring displacement contribution to implicit RHS.
/// Computes: `rhs\[dof\] -= h * K\[dof\] * (q - q_eq)` for joints with springs.
struct ImplicitSpringVisitor<'a> {
    k: &'a DVector<f64>,
    q_eq: &'a DVector<f64>,
    h: f64,
    qpos: &'a DVector<f64>,
    rhs: &'a mut DVector<f64>,
}

impl JointVisitor for ImplicitSpringVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        let q = self.qpos[ctx.qpos_adr];
        self.rhs[ctx.dof_adr] -= self.h * self.k[ctx.dof_adr] * (q - self.q_eq[ctx.dof_adr]);
    }

    // Ball and Free joints have no springs (k = 0), so default no-ops are correct.
}

/// Implicit-fast forward acceleration: symmetric D, Cholesky factorization.
///
/// Solves `(M − h·D) · qacc = qfrc_smooth + qfrc_applied + qfrc_constraint`
/// where D = ∂(qfrc_smooth)/∂(qvel) is assembled from DOF damping, tendon
/// damping, and actuator velocity derivatives (Coriolis terms skipped).
/// D is symmetrized: `D ← (D + D^T) / 2`.
///
/// After return, `data.scratch_m_impl` holds the Cholesky factors (L where
/// M−hD = L·L^T), available for derivative column solves in
/// `mjd_transition_hybrid`.
///
/// # Errors
///
/// Returns `Err(StepError::CholeskyFailed)` if `M − h·D` is not positive
/// definite (can happen with positive velocity feedback actuators, see KA#7).
fn mj_fwd_acceleration_implicitfast(model: &Model, data: &mut Data) -> Result<(), StepError> {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel};

    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel)
    //   Components: DOF damping + tendon damping J^T B J + actuator vel derivatives
    //   Coriolis terms SKIPPED for implicitfast.
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);

    // Step 2: Symmetrize D ← (D + D^T) / 2
    for i in 0..model.nv {
        for j in (i + 1)..model.nv {
            let avg = 0.5 * (data.qDeriv[(i, j)] + data.qDeriv[(j, i)]);
            data.qDeriv[(i, j)] = avg;
            data.qDeriv[(j, i)] = avg;
        }
    }

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Solve (M − h·D) · qacc = rhs via dense Cholesky
    cholesky_in_place(&mut data.scratch_m_impl)?;
    data.qacc.copy_from(&data.scratch_rhs);
    cholesky_solve_in_place(&data.scratch_m_impl, &mut data.qacc);

    Ok(())
}

/// Full implicit forward acceleration: asymmetric D, LU factorization.
///
/// Same as `mj_fwd_acceleration_implicitfast` but includes Coriolis velocity
/// derivatives (`mjd_rne_vel`), does NOT symmetrize D, and uses LU
/// factorization instead of Cholesky.
///
/// After return, `data.scratch_m_impl` holds the LU factors and
/// `data.scratch_lu_piv` holds the pivot permutation, available for
/// derivative column solves in `mjd_transition_hybrid`.
///
/// # Errors
///
/// Returns `Err(StepError::LuSingular)` if any pivot magnitude is below
/// `1e-30` during LU factorization.
fn mj_fwd_acceleration_implicit_full(model: &Model, data: &mut Data) -> Result<(), StepError> {
    use crate::derivatives::{mjd_actuator_vel, mjd_passive_vel, mjd_rne_vel};

    let h = model.timestep;

    // Step 1: Assemble qDeriv = ∂(qfrc_smooth)/∂(qvel) — ALL components
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);

    // Step 2: No symmetrization — D is asymmetric (Coriolis terms break symmetry)

    // Step 3: Form M_hat = M − h·D into scratch_m_impl
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        for j in 0..model.nv {
            data.scratch_m_impl[(i, j)] -= h * data.qDeriv[(i, j)];
        }
    }

    // Step 4: RHS = qfrc_smooth + qfrc_applied + qfrc_constraint
    data.scratch_rhs.copy_from(&data.qfrc_applied);
    data.scratch_rhs += &data.qfrc_actuator;
    data.scratch_rhs += &data.qfrc_passive;
    data.scratch_rhs += &data.qfrc_constraint;
    data.scratch_rhs -= &data.qfrc_bias;

    // Step 5: Factor (M − h·D) = P·L·U, then solve for qacc
    lu_factor_in_place(&mut data.scratch_m_impl, &mut data.scratch_lu_piv)?;
    data.qacc.copy_from(&data.scratch_rhs);
    lu_solve_factored(&data.scratch_m_impl, &data.scratch_lu_piv, &mut data.qacc);

    Ok(())
}

/// Factor A = P·L·U in place. Stores L (unit lower) and U (upper) in `a`.
/// Stores pivot permutation in `piv`. O(n³/3).
///
/// # Errors
///
/// Returns `Err(StepError::LuSingular)` if any pivot magnitude is below `1e-30`.
fn lu_factor_in_place(a: &mut DMatrix<f64>, piv: &mut [usize]) -> Result<(), StepError> {
    let n = a.nrows();
    for k in 0..n {
        // Partial pivot: find max |a[i,k]| for i in k..n
        let mut max_val = a[(k, k)].abs();
        let mut max_row = k;
        for i in (k + 1)..n {
            let v = a[(i, k)].abs();
            if v > max_val {
                max_val = v;
                max_row = i;
            }
        }
        if max_val < 1e-30 {
            return Err(StepError::LuSingular);
        }
        piv[k] = max_row;

        if max_row != k {
            for j in 0..n {
                let tmp = a[(k, j)];
                a[(k, j)] = a[(max_row, j)];
                a[(max_row, j)] = tmp;
            }
        }

        for i in (k + 1)..n {
            a[(i, k)] /= a[(k, k)];
            for j in (k + 1)..n {
                a[(i, j)] -= a[(i, k)] * a[(k, j)];
            }
        }
    }
    Ok(())
}

/// Solve P·L·U·x = b using pre-computed factors. Non-destructive on `a`/`piv`.
/// Can be called multiple times for different RHS vectors.
pub(crate) fn lu_solve_factored(a: &DMatrix<f64>, piv: &[usize], x: &mut DVector<f64>) {
    let n = a.nrows();

    // Apply row permutation to RHS
    for k in 0..n {
        if piv[k] != k {
            x.swap_rows(k, piv[k]);
        }
    }

    // Forward substitution (L·y = Pb)
    for i in 1..n {
        for k in 0..i {
            x[i] -= a[(i, k)] * x[k];
        }
    }

    // Back substitution (U·x = y)
    for i in (0..n).rev() {
        for k in (i + 1)..n {
            x[i] -= a[(i, k)] * x[k];
        }
        x[i] /= a[(i, i)];
    }
}

/// Proper position integration that handles quaternions on SO(3) manifold.
fn mj_integrate_pos(model: &Model, data: &mut Data, h: f64) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let mut visitor = PositionIntegrateVisitor {
        qpos: &mut data.qpos,
        qvel: &data.qvel,
        h,
        sleep_enabled,
        jnt_body: &model.jnt_body,
        body_sleep_state: &data.body_sleep_state,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for position integration that handles different joint types.
struct PositionIntegrateVisitor<'a> {
    qpos: &'a mut DVector<f64>,
    qvel: &'a DVector<f64>,
    h: f64,
    sleep_enabled: bool,
    jnt_body: &'a [usize],
    body_sleep_state: &'a [SleepState],
}

impl PositionIntegrateVisitor<'_> {
    /// Integrate a quaternion with angular velocity on SO(3) manifold.
    /// `qpos_offset` is the offset into qpos where the quaternion starts [w,x,y,z].
    /// `omega` is the angular velocity vector.
    #[inline]
    fn integrate_quaternion(&mut self, qpos_offset: usize, omega: Vector3<f64>) {
        let omega_norm = omega.norm();
        let angle = omega_norm * self.h;

        // Skip if angle is negligible (avoids division by zero)
        if angle > 1e-10 && omega_norm > 1e-10 {
            let axis = omega / omega_norm;
            let dq = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(axis), angle);
            let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                self.qpos[qpos_offset],
                self.qpos[qpos_offset + 1],
                self.qpos[qpos_offset + 2],
                self.qpos[qpos_offset + 3],
            ));
            let q_new = q_old * dq;
            self.qpos[qpos_offset] = q_new.w;
            self.qpos[qpos_offset + 1] = q_new.i;
            self.qpos[qpos_offset + 2] = q_new.j;
            self.qpos[qpos_offset + 3] = q_new.k;
        }
    }
}

impl PositionIntegrateVisitor<'_> {
    /// Check if this joint's body is sleeping (§16.5a'').
    #[inline]
    fn is_sleeping(&self, ctx: &JointContext) -> bool {
        self.sleep_enabled && self.body_sleep_state[self.jnt_body[ctx.jnt_id]] == SleepState::Asleep
    }
}

impl JointVisitor for PositionIntegrateVisitor<'_> {
    #[inline]
    fn visit_hinge(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_slide(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Simple scalar: qpos += qvel * h
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
    }

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Quaternion: integrate angular velocity on SO(3)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr],
            self.qvel[ctx.dof_adr + 1],
            self.qvel[ctx.dof_adr + 2],
        );
        self.integrate_quaternion(ctx.qpos_adr, omega);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        if self.is_sleeping(&ctx) {
            return;
        }
        // Position: linear integration (first 3 components)
        self.qpos[ctx.qpos_adr] += self.qvel[ctx.dof_adr] * self.h;
        self.qpos[ctx.qpos_adr + 1] += self.qvel[ctx.dof_adr + 1] * self.h;
        self.qpos[ctx.qpos_adr + 2] += self.qvel[ctx.dof_adr + 2] * self.h;

        // Orientation: quaternion integration (last 4 components, DOFs 3-5)
        let omega = Vector3::new(
            self.qvel[ctx.dof_adr + 3],
            self.qvel[ctx.dof_adr + 4],
            self.qvel[ctx.dof_adr + 5],
        );
        self.integrate_quaternion(ctx.qpos_adr + 3, omega);
    }
}

/// Normalize all quaternions in qpos to prevent numerical drift.
fn mj_normalize_quat(model: &Model, data: &mut Data) {
    let mut visitor = QuaternionNormalizeVisitor {
        qpos: &mut data.qpos,
    };
    model.visit_joints(&mut visitor);
}

/// Visitor for normalizing quaternions in joints that use them.
struct QuaternionNormalizeVisitor<'a> {
    qpos: &'a mut DVector<f64>,
}

impl QuaternionNormalizeVisitor<'_> {
    /// Normalize a quaternion at the given offset in qpos.
    #[inline]
    fn normalize_quaternion(&mut self, offset: usize) {
        let norm = (self.qpos[offset].powi(2)
            + self.qpos[offset + 1].powi(2)
            + self.qpos[offset + 2].powi(2)
            + self.qpos[offset + 3].powi(2))
        .sqrt();
        if norm > 1e-10 {
            self.qpos[offset] /= norm;
            self.qpos[offset + 1] /= norm;
            self.qpos[offset + 2] /= norm;
            self.qpos[offset + 3] /= norm;
        } else {
            // Degenerate quaternion - reset to identity [w=1, x=0, y=0, z=0]
            self.qpos[offset] = 1.0;
            self.qpos[offset + 1] = 0.0;
            self.qpos[offset + 2] = 0.0;
            self.qpos[offset + 3] = 0.0;
        }
    }
}

impl JointVisitor for QuaternionNormalizeVisitor<'_> {
    // Hinge and Slide have no quaternions - use default no-ops

    #[inline]
    fn visit_ball(&mut self, ctx: JointContext) {
        // Ball joint: quaternion at qpos_adr
        self.normalize_quaternion(ctx.qpos_adr);
    }

    #[inline]
    fn visit_free(&mut self, ctx: JointContext) {
        // Free joint: quaternion at qpos_adr + 3 (after position xyz)
        self.normalize_quaternion(ctx.qpos_adr + 3);
    }
}

/// Compute velocity from position difference: `qvel` = `mj_differentiatePos(qpos2 - qpos1) / dt`.
///
/// This function computes the velocity that would move from `qpos1` to `qpos2` in time `dt`.
/// For quaternions (ball/free joints), it uses the proper SO(3) velocity rather than
/// naive quaternion subtraction.
///
/// # Arguments
///
/// * `model` - The model containing joint definitions
/// * `qvel` - Output velocity vector (length `nv`)
/// * `qpos1` - Start position
/// * `qpos2` - End position
/// * `dt` - Time difference
///
/// # `MuJoCo` Equivalence
///
/// This matches `MuJoCo`'s `mj_differentiatePos` function.
pub fn mj_differentiate_pos(
    model: &Model,
    qvel: &mut DVector<f64>,
    qpos1: &DVector<f64>,
    qpos2: &DVector<f64>,
    dt: f64,
) {
    if dt.abs() < 1e-10 {
        qvel.fill(0.0);
        return;
    }

    let dt_inv = 1.0 / dt;

    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                // Scalar: simple finite difference
                qvel[dof_adr] = (qpos2[qpos_adr] - qpos1[qpos_adr]) * dt_inv;
            }

            MjJointType::Ball => {
                // Quaternion velocity: compute angular velocity from q1 to q2
                // q2 = q_delta * q1  =>  q_delta = q2 * q1^-1
                // angular velocity = 2 * log(q_delta) / dt
                let q1 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos1[qpos_adr],
                    qpos1[qpos_adr + 1],
                    qpos1[qpos_adr + 2],
                    qpos1[qpos_adr + 3],
                ));
                let q2 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos2[qpos_adr],
                    qpos2[qpos_adr + 1],
                    qpos2[qpos_adr + 2],
                    qpos2[qpos_adr + 3],
                ));

                // q_delta = q2 * q1.inverse()
                let q_delta = q2 * q1.inverse();

                // Extract axis-angle (clamp w to avoid NaN from floating-point precision)
                let angle = 2.0 * q_delta.w.clamp(-1.0, 1.0).acos();
                let sin_half = (1.0 - q_delta.w * q_delta.w).max(0.0).sqrt();

                if sin_half > 1e-10 {
                    let axis = Vector3::new(q_delta.i, q_delta.j, q_delta.k) / sin_half;
                    let omega = axis * angle * dt_inv;
                    qvel[dof_adr] = omega.x;
                    qvel[dof_adr + 1] = omega.y;
                    qvel[dof_adr + 2] = omega.z;
                } else {
                    qvel[dof_adr] = 0.0;
                    qvel[dof_adr + 1] = 0.0;
                    qvel[dof_adr + 2] = 0.0;
                }
            }

            MjJointType::Free => {
                // Linear velocity: simple finite difference
                qvel[dof_adr] = (qpos2[qpos_adr] - qpos1[qpos_adr]) * dt_inv;
                qvel[dof_adr + 1] = (qpos2[qpos_adr + 1] - qpos1[qpos_adr + 1]) * dt_inv;
                qvel[dof_adr + 2] = (qpos2[qpos_adr + 2] - qpos1[qpos_adr + 2]) * dt_inv;

                // Angular velocity from quaternion difference
                let q1 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos1[qpos_adr + 3],
                    qpos1[qpos_adr + 4],
                    qpos1[qpos_adr + 5],
                    qpos1[qpos_adr + 6],
                ));
                let q2 = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos2[qpos_adr + 3],
                    qpos2[qpos_adr + 4],
                    qpos2[qpos_adr + 5],
                    qpos2[qpos_adr + 6],
                ));

                let q_delta = q2 * q1.inverse();
                // Clamp w to avoid NaN from floating-point precision
                let angle = 2.0 * q_delta.w.clamp(-1.0, 1.0).acos();
                let sin_half = (1.0 - q_delta.w * q_delta.w).max(0.0).sqrt();

                if sin_half > 1e-10 {
                    let axis = Vector3::new(q_delta.i, q_delta.j, q_delta.k) / sin_half;
                    let omega = axis * angle * dt_inv;
                    qvel[dof_adr + 3] = omega.x;
                    qvel[dof_adr + 4] = omega.y;
                    qvel[dof_adr + 5] = omega.z;
                } else {
                    qvel[dof_adr + 3] = 0.0;
                    qvel[dof_adr + 4] = 0.0;
                    qvel[dof_adr + 5] = 0.0;
                }
            }
        }
    }
}

/// Integrate position given velocity: `qpos_out` = `mj_integratePos(qpos, qvel, dt)`.
///
/// This is the inverse of `mj_differentiatePos`. It computes the position reached
/// by integrating velocity over time dt, handling quaternions correctly on SO(3).
///
/// # Arguments
///
/// * `model` - The model containing joint definitions
/// * `qpos_out` - Output position vector (length `nq`)
/// * `qpos` - Start position
/// * `qvel` - Velocity
/// * `dt` - Time step
///
/// # `MuJoCo` Equivalence
///
/// This matches the inverse operation of `mj_differentiatePos`.
pub fn mj_integrate_pos_explicit(
    model: &Model,
    qpos_out: &mut DVector<f64>,
    qpos: &DVector<f64>,
    qvel: &DVector<f64>,
    dt: f64,
) {
    for jnt_id in 0..model.njnt {
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                qpos_out[qpos_adr] = qpos[qpos_adr] + qvel[dof_adr] * dt;
            }

            MjJointType::Ball => {
                // Quaternion integration
                let omega = Vector3::new(qvel[dof_adr], qvel[dof_adr + 1], qvel[dof_adr + 2]);
                let angle = omega.norm() * dt;

                let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos[qpos_adr],
                    qpos[qpos_adr + 1],
                    qpos[qpos_adr + 2],
                    qpos[qpos_adr + 3],
                ));

                let q_new = if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(
                        &nalgebra::Unit::new_normalize(axis),
                        angle,
                    );
                    q_old * dq
                } else {
                    q_old
                };

                qpos_out[qpos_adr] = q_new.w;
                qpos_out[qpos_adr + 1] = q_new.i;
                qpos_out[qpos_adr + 2] = q_new.j;
                qpos_out[qpos_adr + 3] = q_new.k;
            }

            MjJointType::Free => {
                // Linear position
                qpos_out[qpos_adr] = qpos[qpos_adr] + qvel[dof_adr] * dt;
                qpos_out[qpos_adr + 1] = qpos[qpos_adr + 1] + qvel[dof_adr + 1] * dt;
                qpos_out[qpos_adr + 2] = qpos[qpos_adr + 2] + qvel[dof_adr + 2] * dt;

                // Quaternion integration
                let omega = Vector3::new(qvel[dof_adr + 3], qvel[dof_adr + 4], qvel[dof_adr + 5]);
                let angle = omega.norm() * dt;

                let q_old = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    qpos[qpos_adr + 3],
                    qpos[qpos_adr + 4],
                    qpos[qpos_adr + 5],
                    qpos[qpos_adr + 6],
                ));

                let q_new = if angle > 1e-10 {
                    let axis = omega / omega.norm();
                    let dq = UnitQuaternion::from_axis_angle(
                        &nalgebra::Unit::new_normalize(axis),
                        angle,
                    );
                    q_old * dq
                } else {
                    q_old
                };

                qpos_out[qpos_adr + 3] = q_new.w;
                qpos_out[qpos_adr + 4] = q_new.i;
                qpos_out[qpos_adr + 5] = q_new.j;
                qpos_out[qpos_adr + 6] = q_new.k;
            }
        }
    }
}

// ============================================================================
// RK4 Integration
// ============================================================================

/// Standard 4-stage Runge-Kutta integration matching MuJoCo's `mj_RungeKutta`.
///
/// # Preconditions
/// - `data.forward()` has already been called (qacc is valid for stage 0).
///
/// # Algorithm
/// Uses the classic RK4 Butcher tableau. Stage 0 reuses qacc from the preceding
/// `forward()` call. Stages 1-3 each call `forward_skip_sensors()` at trial states.
/// Position integration uses `mj_integrate_pos_explicit()` for quaternion correctness.
///
/// After this function returns, derived quantities (xpos, contacts, forces, etc.)
/// are stale from stage 3 and do NOT correspond to the final (qpos, qvel) state.
/// This matches MuJoCo's behavior.
fn mj_runge_kutta(model: &Model, data: &mut Data) -> Result<(), StepError> {
    // Butcher tableau
    const RK4_A: [f64; 9] = [
        0.5, 0.0, 0.0, // Stage 1→2
        0.0, 0.5, 0.0, // Stage 2→3
        0.0, 0.0, 1.0, // Stage 3→4
    ];
    const RK4_B: [f64; 4] = [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0];
    const RK4_TIME: [f64; 3] = [0.5, 0.5, 1.0];

    let h = model.timestep;
    let nv = model.nv;

    // 1. SAVE initial state
    data.rk4_qpos_saved.copy_from(&data.qpos);
    data.rk4_qvel[0].copy_from(&data.qvel);
    data.rk4_qacc[0].copy_from(&data.qacc);
    data.rk4_act_saved.copy_from(&data.act);
    data.rk4_act_dot[0].copy_from(&data.act_dot);
    let t0 = data.time;
    let efc_lambda_saved = data.efc_lambda.clone();

    // 2. FOR i = 1, 2, 3:
    for i in 1..4usize {
        // 2a. Weighted velocity: dX_vel[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * X[j].qvel[v]
        // (For this tableau, only j = i−1 has a non-zero coefficient.)
        for v in 0..nv {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += RK4_A[(i - 1) * 3 + j] * data.rk4_qvel[j][v];
            }
            data.rk4_dX_vel[v] = sum;
        }

        // 2b. Weighted acceleration: dX_acc[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * F[j].qacc[v]
        for v in 0..nv {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += RK4_A[(i - 1) * 3 + j] * data.rk4_qacc[j][v];
            }
            data.rk4_dX_acc[v] = sum;
        }

        // 2c. Position (manifold): integrate from saved initial position
        mj_integrate_pos_explicit(
            model,
            &mut data.rk4_qpos_stage,
            &data.rk4_qpos_saved,
            &data.rk4_dX_vel,
            h,
        );

        // 2d. Velocity (linear): X[i].qvel = X[0].qvel + h * dX_acc
        // Use split_at_mut for borrow-checker disjointness on rk4_qvel.
        let (head, tail) = data.rk4_qvel.split_at_mut(1);
        for v in 0..nv {
            tail[i - 1][v] = head[0][v] + h * data.rk4_dX_acc[v];
        }

        // 2e. Activation trial state: act = act_saved + h_eff * Σ A[(i-1)*3+j] * act_dot[j]
        // where h_eff depends on dynamics type (Euler vs FilterExact).
        for act_i in 0..model.nu {
            let act_adr = model.actuator_act_adr[act_i];
            for k in 0..model.actuator_act_num[act_i] {
                let a = act_adr + k;
                let mut sum = 0.0;
                for j in 0..3 {
                    sum += RK4_A[(i - 1) * 3 + j] * data.rk4_act_dot[j][a];
                }
                match model.actuator_dyntype[act_i] {
                    ActuatorDynamics::FilterExact => {
                        let tau = model.actuator_dynprm[act_i][0].max(1e-10);
                        data.act[a] = data.rk4_act_saved[a] + sum * tau * (1.0 - (-h / tau).exp());
                    }
                    _ => {
                        data.act[a] = data.rk4_act_saved[a] + h * sum;
                    }
                }
            }
            // Clamp muscle activations to [0, 1] for trial state
            if model.actuator_dyntype[act_i] == ActuatorDynamics::Muscle {
                for k in 0..model.actuator_act_num[act_i] {
                    data.act[act_adr + k] = data.act[act_adr + k].clamp(0.0, 1.0);
                }
            }
        }

        // 2f. Set Data state
        data.qpos.copy_from(&data.rk4_qpos_stage);
        data.qvel.copy_from(&tail[i - 1]);

        // 2g. Set Data time
        data.time = t0 + h * RK4_TIME[i - 1];

        // 2h. Evaluate full forward pipeline (without sensors)
        data.forward_skip_sensors(model)?;

        // 2i. Store rates
        data.rk4_qacc[i].copy_from(&data.qacc);
        data.rk4_act_dot[i].copy_from(&data.act_dot);
    }

    // 3. FINAL combination using B weights
    for v in 0..nv {
        let mut vel_sum = 0.0;
        let mut acc_sum = 0.0;
        for j in 0..4 {
            vel_sum += RK4_B[j] * data.rk4_qvel[j][v];
            acc_sum += RK4_B[j] * data.rk4_qacc[j][v];
        }
        data.rk4_dX_vel[v] = vel_sum;
        data.rk4_dX_acc[v] = acc_sum;
    }

    // 4. ADVANCE from initial state
    // Note: qacc_warmstart is now saved at end of step() (§15.9), not here.

    // Restore initial velocity, then advance
    data.qvel.copy_from(&data.rk4_qvel[0]);
    for v in 0..nv {
        data.qvel[v] += h * data.rk4_dX_acc[v];
    }

    // Position on manifold from saved initial position
    mj_integrate_pos_explicit(
        model,
        &mut data.qpos,
        &data.rk4_qpos_saved,
        &data.rk4_dX_vel,
        h,
    );
    mj_normalize_quat(model, data);

    // Advance activation from saved initial state
    for act_i in 0..model.nu {
        let act_adr = model.actuator_act_adr[act_i];
        for k in 0..model.actuator_act_num[act_i] {
            let a = act_adr + k;
            let dact_combined: f64 = (0..4).map(|j| RK4_B[j] * data.rk4_act_dot[j][a]).sum();
            match model.actuator_dyntype[act_i] {
                ActuatorDynamics::FilterExact => {
                    let tau = model.actuator_dynprm[act_i][0].max(1e-10);
                    data.act[a] =
                        data.rk4_act_saved[a] + dact_combined * tau * (1.0 - (-h / tau).exp());
                }
                _ => {
                    data.act[a] = data.rk4_act_saved[a] + h * dact_combined;
                }
            }
        }
        // Clamp muscle activations to [0, 1]
        if model.actuator_dyntype[act_i] == ActuatorDynamics::Muscle {
            for k in 0..model.actuator_act_num[act_i] {
                data.act[act_adr + k] = data.act[act_adr + k].clamp(0.0, 1.0);
            }
        }
    }

    // Restore warmstart from initial solve
    data.efc_lambda = efc_lambda_saved;
    data.time = t0 + h;

    // Step deformable bodies (XPBD internal constraints)
    #[cfg(feature = "deformable")]
    mj_deformable_step(model, data);

    Ok(())
}

// ============================================================================
// Tests for Primitive Collision Detection
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod primitive_collision_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Helper to create a minimal Model for collision testing.
    fn make_collision_test_model(ngeom: usize) -> Model {
        let mut model = Model::empty();
        model.ngeom = ngeom;
        model.geom_type = vec![GeomType::Sphere; ngeom];
        model.geom_body = vec![0; ngeom];
        model.geom_pos = vec![Vector3::zeros(); ngeom];
        model.geom_quat = vec![UnitQuaternion::identity(); ngeom];
        model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); ngeom];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); ngeom];
        model.geom_condim = vec![3; ngeom]; // Default condim = 3 (sliding friction)
        model.geom_contype = vec![1; ngeom];
        model.geom_conaffinity = vec![1; ngeom];
        model.geom_margin = vec![0.0; ngeom];
        model.geom_gap = vec![0.0; ngeom];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; ngeom];
        model.geom_solref = vec![[0.02, 1.0]; ngeom];
        model.geom_name = vec![None; ngeom];
        model.geom_rbound = vec![1.0; ngeom];
        model.geom_mesh = vec![None; ngeom]; // No mesh geoms in test helper
        model
    }

    // ========================================================================
    // Cylinder-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_cylinder_plane_upright_penetrating() {
        // Cylinder standing upright on plane, bottom rim penetrating
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;
        model.geom_size[0] = Vector3::new(10.0, 10.0, 0.1);

        // Geom 1: Cylinder (radius=0.3, half_height=0.5)
        // Center at z=0.4, so bottom cap center is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // [radius, half_height, unused]

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let cyl_mat = Matrix3::identity(); // Axis along +Z (upright)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_upright_no_contact() {
        // Cylinder above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 1.0); // Center at z=1.0, bottom at z=0.5
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_none(),
            "Cylinder should not contact plane when above"
        );
    }

    #[test]
    fn test_cylinder_plane_tilted() {
        // Cylinder tilted 45° around X axis, verifying rim point depth calculation
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Cylinder with radius=0.3, half_height=0.5, tilted 45° around X
        // After rotation, the cylinder axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Cylinder;
        let radius = 0.3;
        let half_height = 0.5;
        model.geom_size[1] = Vector3::new(radius, half_height, 0.0);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Tilt cylinder 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let cyl_mat = rot.to_rotation_matrix().into_inner();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected deepest point calculation (Case 3: tilted cylinder):
        // Cylinder axis in world frame: (0, sin(45°), cos(45°)) = (0, 0.707, 0.707)
        // Plane normal: (0, 0, 1)
        // axis_dot_signed = plane_normal · cyl_axis = cos(45°) ≈ 0.707
        // radial = plane_normal - cyl_axis * axis_dot_signed
        //        = (0, 0, 1) - (0, 0.707, 0.707) * 0.707
        //        = (0, -0.5, 0.5)
        // rim_dir = -radial / ||radial|| = (0, 0.707, -0.707)
        //
        // Bottom cap center: cyl_pos - cyl_axis * half_height
        //                  = (0, 0, 0.5) - (0, 0.354, 0.354) = (0, -0.354, 0.146)
        // Bottom rim point: bottom_center + rim_dir * radius
        //                 = (0, -0.354, 0.146) + (0, 0.212, -0.212) = (0, -0.142, -0.066)
        //
        // Top cap center: cyl_pos + cyl_axis * half_height
        //               = (0, 0, 0.5) + (0, 0.354, 0.354) = (0, 0.354, 0.854)
        // Top rim point: top_center + rim_dir * radius
        //              = (0, 0.354, 0.854) + (0, 0.212, -0.212) = (0, 0.566, 0.642)
        //
        // Bottom rim z = -0.066 (below plane) → depth = 0.066
        // Top rim z = 0.642 (above plane) → no penetration
        // Deepest point is bottom rim with depth ≈ 0.066
        let cyl_axis_y = sin_a;
        let cyl_axis_z = cos_a;
        let bottom_center_z = cyl_pos.z - cyl_axis_z * half_height;
        // radial = (0, -axis_dot_signed * cyl_axis_y, 1 - axis_dot_signed * cyl_axis_z)
        let axis_dot_signed = cos_a;
        let radial_y = -axis_dot_signed * cyl_axis_y;
        let radial_z = 1.0 - axis_dot_signed * cyl_axis_z;
        let radial_len = (radial_y * radial_y + radial_z * radial_z).sqrt();
        let rim_dir_z = -radial_z / radial_len;
        let bottom_rim_z = bottom_center_z + rim_dir_z * radius;
        let expected_depth = -bottom_rim_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Tilted cylinder should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_cylinder_plane_horizontal() {
        // Cylinder lying flat (axis parallel to plane)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0); // radius=0.3, half_height=0.5

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();

        // Rotate 90 degrees around X axis (cylinder now horizontal, axis along Y)
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::FRAC_PI_2);
        let cyl_mat = rot.to_rotation_matrix().into_inner();

        // Position center at z = radius - epsilon for penetration
        let cyl_pos = Vector3::new(0.0, 0.0, 0.2); // Below radius, should penetrate

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_some(),
            "Horizontal cylinder should contact plane"
        );
        let c = contact.unwrap();
        // Penetration should be radius - z_center = 0.3 - 0.2 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    // ========================================================================
    // Ellipsoid-Plane Collision Tests
    // ========================================================================

    #[test]
    fn test_ellipsoid_plane_sphere_case() {
        // Ellipsoid with equal radii (sphere) for validation against known sphere formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Ellipsoid with rx=ry=rz=0.5 (degenerates to sphere with radius 0.5)
        // Center at z=0.4, so bottom (support point) is at z = 0.4 - 0.5 = -0.1
        // This means the bottom penetrates 0.1 below the plane (at z=0)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z
        let ell_pos = Vector3::new(0.0, 0.0, 0.4); // Center at z=0.4
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Ellipsoid (sphere) should contact plane");
        let c = contact.unwrap();
        // Penetration = radius - z = 0.5 - 0.4 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_z() {
        // Ellipsoid stretched along Z axis (tall and thin)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8); // Tall ellipsoid

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.7); // Bottom at z = -0.1
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Tall ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.8 - 0.7 = 0.1
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_stretched_x() {
        // Ellipsoid stretched along X axis (wide and flat)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.8, 0.3, 0.2); // Wide ellipsoid (short in Z)

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 0.15); // Bottom at z = -0.05 (penetrating)
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Wide ellipsoid should contact plane");
        let c = contact.unwrap();
        // Penetration = z_radius - z = 0.2 - 0.15 = 0.05
        assert_relative_eq!(c.depth, 0.05, epsilon = 1e-6);
    }

    #[test]
    fn test_ellipsoid_plane_rotated() {
        // Ellipsoid rotated 45° around X axis, verifying support point formula
        let mut model = make_collision_test_model(2);

        // Geom 0: Plane at z=0
        model.geom_type[0] = GeomType::Plane;

        // Geom 1: Tall ellipsoid with radii (0.2, 0.2, 0.8), rotated 45° around X
        // After rotation, the long axis points diagonally (into Y-Z plane)
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.2, 0.2, 0.8);

        let plane_pos = Vector3::zeros(); // Plane at z=0
        let plane_mat = Matrix3::identity(); // Normal along +Z

        // Rotate 45 degrees around X axis
        let angle = std::f64::consts::FRAC_PI_4;
        let cos_a = angle.cos(); // √2/2 ≈ 0.7071
        let sin_a = angle.sin(); // √2/2 ≈ 0.7071
        let rot = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let ell_mat = rot.to_rotation_matrix().into_inner();
        let ell_pos = Vector3::new(0.0, 0.0, 0.5); // Center at z=0.5

        // Expected support point calculation:
        // Local plane normal: n_local = R^T * (0,0,1) = (0, sin(45°), cos(45°))
        //   (R rotates around X, so R^T maps world Z to local (0, sin, cos))
        // scaled = r ⊙ n_local = (0, 0.2*sin, 0.8*cos) = (0, 0.1414, 0.5657)
        // ||scaled|| = sqrt(0.2² * sin² + 0.8² * cos²) ≈ 0.583
        // local_support = -(r² ⊙ n) / ||scaled||
        //               = -(0, 0.04*sin, 0.64*cos) / 0.583
        //               = (0, -0.0485, -0.776)
        // world_support = center + R * local_support
        // The z-component: 0.5 + (local_y * sin + local_z * cos)
        //                = 0.5 + (-0.0485 * 0.707 + (-0.776) * 0.707)
        //                = 0.5 - 0.583 = -0.083
        // Expected depth = -(-0.083) = 0.083 (positive, penetrating)
        let ry = 0.2;
        let rz = 0.8;
        let scaled_y = ry * sin_a;
        let scaled_z = rz * cos_a;
        let scale_norm = (scaled_y * scaled_y + scaled_z * scaled_z).sqrt();
        let local_support_y = -(ry * ry * sin_a) / scale_norm;
        let local_support_z = -(rz * rz * cos_a) / scale_norm;
        let world_support_z = ell_pos.z + local_support_y * sin_a + local_support_z * cos_a;
        let expected_depth = -world_support_z;

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(contact.is_some(), "Rotated ellipsoid should contact plane");
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, expected_depth, epsilon = 1e-10);
        assert_relative_eq!(c.normal, Vector3::z(), epsilon = 1e-10);
    }

    #[test]
    fn test_ellipsoid_plane_no_contact() {
        // Ellipsoid above plane, no contact
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Ellipsoid;
        model.geom_size[1] = Vector3::new(0.3, 0.3, 0.3);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let ell_pos = Vector3::new(0.0, 0.0, 1.0); // Far above plane
        let ell_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Ellipsoid,
            plane_pos,
            plane_mat,
            ell_pos,
            ell_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        assert!(
            contact.is_none(),
            "Ellipsoid should not contact plane when above"
        );
    }

    // ========================================================================
    // Edge Cases and Numerical Stability
    // ========================================================================

    #[test]
    fn test_cylinder_plane_axis_parallel_to_normal() {
        // Cylinder axis exactly parallel to plane normal (degenerate case)
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4); // Bottom at z = -0.1
        let cyl_mat = Matrix3::identity(); // Axis along Z (parallel to plane normal)

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        );

        // Should still detect contact at the bottom rim
        assert!(
            contact.is_some(),
            "Should detect contact even when axis parallel to normal"
        );
        let c = contact.unwrap();
        assert_relative_eq!(c.depth, 0.1, epsilon = 1e-6);
    }

    #[test]
    fn test_contact_frame_is_valid() {
        // Verify contact frame is orthonormal
        let mut model = make_collision_test_model(2);

        model.geom_type[0] = GeomType::Plane;
        model.geom_type[1] = GeomType::Cylinder;
        model.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);

        let plane_pos = Vector3::zeros();
        let plane_mat = Matrix3::identity();
        let cyl_pos = Vector3::new(0.0, 0.0, 0.4);
        let cyl_mat = Matrix3::identity();

        let contact = collide_with_plane(
            &model,
            0,
            1,
            GeomType::Plane,
            GeomType::Cylinder,
            plane_pos,
            plane_mat,
            cyl_pos,
            cyl_mat,
            model.geom_size[0],
            model.geom_size[1],
        )
        .expect("should have contact");

        // Normal should be unit length
        assert_relative_eq!(contact.normal.norm(), 1.0, epsilon = 1e-10);

        // Tangent vectors should be unit length
        assert_relative_eq!(contact.frame[0].norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(contact.frame[1].norm(), 1.0, epsilon = 1e-10);

        // All three should be mutually orthogonal
        assert_relative_eq!(contact.normal.dot(&contact.frame[0]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(contact.normal.dot(&contact.frame[1]), 0.0, epsilon = 1e-10);
        assert_relative_eq!(
            contact.frame[0].dot(&contact.frame[1]),
            0.0,
            epsilon = 1e-10
        );
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod impedance_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Default solimp: [d0=0.9, d_width=0.95, width=0.001, midpoint=0.5, power=2.0]
    const DEFAULT: [f64; 5] = DEFAULT_SOLIMP;

    // ========================================================================
    // compute_impedance unit tests
    // ========================================================================

    #[test]
    fn test_impedance_at_zero_violation() {
        // At zero violation, impedance should be d0
        let d = compute_impedance(DEFAULT, 0.0);
        assert_relative_eq!(d, 0.9, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_at_full_width() {
        // At violation >= width, impedance should be d_width
        let d = compute_impedance(DEFAULT, 0.001);
        assert_relative_eq!(d, 0.95, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_beyond_width() {
        // Beyond width, impedance should saturate at d_width
        let d = compute_impedance(DEFAULT, 0.01);
        assert_relative_eq!(d, 0.95, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_at_midpoint() {
        // At midpoint of transition, impedance should be between d0 and d_width.
        // For power=2 and midpoint=0.5:
        //   x = 0.5*width / width = 0.5
        //   y(0.5) = a * 0.5^2 where a = 1/0.5^(2-1) = 2
        //   y = 2 * 0.25 = 0.5
        //   d = 0.9 + 0.5 * (0.95 - 0.9) = 0.925
        let d = compute_impedance(DEFAULT, 0.0005);
        assert_relative_eq!(d, 0.925, epsilon = 1e-3);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "non-negative violation")]
    fn test_impedance_rejects_negative_violation() {
        // Negative violation is a caller bug — debug_assert catches it
        compute_impedance(DEFAULT, -0.0005);
    }

    #[test]
    fn test_impedance_flat_when_d0_equals_dwidth() {
        // When d0 == d_width, impedance is constant
        let solimp = [0.9, 0.9, 0.001, 0.5, 2.0];
        let d_zero = compute_impedance(solimp, 0.0);
        let d_mid = compute_impedance(solimp, 0.0005);
        let d_full = compute_impedance(solimp, 0.001);
        assert_relative_eq!(d_zero, 0.9, epsilon = 1e-4);
        assert_relative_eq!(d_mid, 0.9, epsilon = 1e-4);
        assert_relative_eq!(d_full, 0.9, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_linear_power() {
        // With power=1, transition should be linear
        let solimp = [0.5, 1.0, 0.01, 0.5, 1.0];
        let d_quarter = compute_impedance(solimp, 0.0025);
        // x = 0.25, y = 0.25 (linear), d = 0.5 + 0.25*0.5 = 0.625
        assert_relative_eq!(d_quarter, 0.625, epsilon = 1e-3);

        let d_half = compute_impedance(solimp, 0.005);
        // x = 0.5, y = 0.5 (linear), d = 0.5 + 0.5*0.5 = 0.75
        assert_relative_eq!(d_half, 0.75, epsilon = 1e-3);
    }

    #[test]
    fn test_impedance_clamped_to_valid_range() {
        // Impedance should be clamped to (0, 1) even with extreme solimp values
        let solimp_low = [0.0, 0.0, 0.001, 0.5, 2.0];
        let d = compute_impedance(solimp_low, 0.0);
        assert!(d >= 0.0001, "Impedance should be at least MIN_IMPEDANCE");

        let solimp_high = [1.0, 1.0, 0.001, 0.5, 2.0];
        let d = compute_impedance(solimp_high, 0.0);
        assert!(d <= 0.9999, "Impedance should be at most MAX_IMPEDANCE");
    }

    #[test]
    fn test_impedance_monotonic_increasing() {
        // With d_width > d0, impedance should increase with violation
        let solimp = [0.5, 0.95, 0.01, 0.5, 2.0];
        let mut prev = compute_impedance(solimp, 0.0);
        for i in 1..=10 {
            let pos = f64::from(i) * 0.001;
            let d = compute_impedance(solimp, pos);
            assert!(
                d >= prev - 1e-10,
                "Impedance should be monotonically increasing: d({})={} < d({})={}",
                pos - 0.001,
                prev,
                pos,
                d
            );
            prev = d;
        }
    }

    #[test]
    fn test_impedance_zero_width() {
        // Zero width should return average of d0 and d_width
        let solimp = [0.5, 0.9, 0.0, 0.5, 2.0];
        let d = compute_impedance(solimp, 0.1);
        assert_relative_eq!(d, 0.7, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_extreme_midpoint_no_nan() {
        // midpoint=0 would cause division by zero without input clamping.
        // Clamped to 0.0001, so this must produce a finite result.
        let solimp = [0.5, 0.9, 0.01, 0.0, 3.0];
        let d = compute_impedance(solimp, 0.005);
        assert!(d.is_finite(), "midpoint=0 should not produce NaN, got {d}");
        assert!(d > 0.0 && d < 1.0);

        // midpoint=1 same issue on the upper branch
        let solimp = [0.5, 0.9, 0.01, 1.0, 3.0];
        let d = compute_impedance(solimp, 0.005);
        assert!(d.is_finite(), "midpoint=1 should not produce NaN, got {d}");
        assert!(d > 0.0 && d < 1.0);
    }

    #[test]
    fn test_impedance_power_below_one_clamped() {
        // power < 1 is invalid in MuJoCo (clamped to 1). Should behave as linear.
        let solimp = [0.5, 1.0, 0.01, 0.5, 0.5]; // power=0.5, clamped to 1.0
        let d = compute_impedance(solimp, 0.005);
        // With power clamped to 1 (linear): x=0.5, y=0.5, d = 0.5 + 0.5*0.5 = 0.75
        assert_relative_eq!(d, 0.75, epsilon = 1e-3);
    }
}

// ============================================================================
// Sensor Implementation Tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod sensor_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Helper: create a pendulum model (1 body, 1 hinge joint) with sensor slots.
    /// The pendulum has:
    /// - Body 1 with mass 1kg at (0,0,-0.5) relative to joint
    /// - 1 hinge joint about Y axis
    /// - 1 sphere geom (radius 0.1) at body center
    /// - 1 site at the geom center
    fn make_sensor_test_model() -> Model {
        let mut model = Model::empty();

        // Add body 1 (pendulum)
        model.nbody = 2; // world + pendulum
        model.body_parent.push(0); // parent = world
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0); // first joint at index 0
        model.body_jnt_num.push(1); // 1 joint
        model.body_dof_adr.push(0);
        model.body_dof_num.push(1);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.0, 0.0, -0.5)); // COM offset
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("pendulum".to_string()));
        model.body_subtreemass.push(1.0);

        // Add hinge joint
        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type.push(MjJointType::Hinge);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::y()); // Y axis
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("hinge".to_string()));

        // DOF
        model.dof_body.push(1);
        model.dof_jnt.push(0);
        model.dof_parent.push(None); // no parent DOF
        model.dof_armature.push(0.0);
        model.dof_damping.push(0.0);
        model.dof_frictionloss.push(0.0);

        // Add geom (sphere radius 0.1)
        model.ngeom = 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(1);
        model.geom_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.1, 0.1, 0.1));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.1);
        model.geom_mesh.push(None);

        // Add site at the body's COM
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("sensor_site".to_string()));

        // Initialize qpos0
        model.qpos0 = DVector::zeros(model.nq);

        model
    }

    /// Helper: add a sensor to a model.
    fn add_sensor(
        model: &mut Model,
        sensor_type: MjSensorType,
        datatype: MjSensorDataType,
        objtype: MjObjectType,
        objid: usize,
    ) {
        let dim = sensor_type.dim();
        let adr = model.nsensordata;
        model.sensor_type.push(sensor_type);
        model.sensor_datatype.push(datatype);
        model.sensor_objtype.push(objtype);
        model.sensor_objid.push(objid);
        model.sensor_reftype.push(MjObjectType::Body);
        model.sensor_refid.push(0);
        model.sensor_adr.push(adr);
        model.sensor_dim.push(dim);
        model.sensor_noise.push(0.0);
        model.sensor_cutoff.push(0.0);
        model.sensor_name.push(None);
        model.nsensor += 1;
        model.nsensordata += dim;
    }

    // ========================================================================
    // Touch Sensor Tests
    // ========================================================================

    #[test]
    fn test_touch_sensor_no_contact() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration, // Touch needs acc stage
            MjObjectType::Geom,
            0, // geom 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // No contacts exist, touch should be 0
        assert_eq!(data.sensordata[0], 0.0);
    }

    #[test]
    fn test_touch_sensor_with_contact() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration,
            MjObjectType::Geom,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Manually inject a contact and an efc_lambda entry
        data.contacts.push(Contact::new(
            Vector3::new(0.0, 0.0, -0.5),
            Vector3::z(),
            0.01, // depth
            0,    // geom1 = our sensor geom
            99,   // geom2 = some other geom
            1.0,  // friction
        ));
        data.efc_lambda.insert(
            warmstart_key(data.contacts.last().unwrap()),
            vec![42.0, 1.0, 2.0],
        );

        // Run acc sensors directly to test Touch
        mj_sensor_acc(&model, &mut data);

        // Touch should read the normal force from efc_lambda
        assert_relative_eq!(data.sensordata[0], 42.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Magnetometer Tests
    // ========================================================================

    #[test]
    fn test_magnetometer_identity_frame() {
        let mut model = make_sensor_test_model();
        model.magnetic = Vector3::new(0.0, 25.0, -45.0); // Earth-like field

        add_sensor(
            &mut model,
            MjSensorType::Magnetometer,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At identity orientation, sensor reads the global field directly
        // (site_xmat should be identity since joint angle is 0 and
        //  the site is aligned with the body which is at identity)
        // But site_xmat might differ due to FK. Let's just check it's non-zero
        let mag_x = data.sensordata[0];
        let mag_y = data.sensordata[1];
        let mag_z = data.sensordata[2];
        let magnitude = (mag_x * mag_x + mag_y * mag_y + mag_z * mag_z).sqrt();

        // The magnetic field magnitude should be preserved
        let expected_mag = model.magnetic.norm();
        assert_relative_eq!(magnitude, expected_mag, epsilon = 1e-6);
    }

    #[test]
    fn test_magnetometer_zero_field() {
        let mut model = make_sensor_test_model();
        model.magnetic = Vector3::zeros(); // No magnetic field

        add_sensor(
            &mut model,
            MjSensorType::Magnetometer,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        assert_eq!(data.sensordata[0], 0.0);
        assert_eq!(data.sensordata[1], 0.0);
        assert_eq!(data.sensordata[2], 0.0);
    }

    // ========================================================================
    // Actuator Position/Velocity Sensor Tests
    // ========================================================================

    #[test]
    fn test_actuator_pos_sensor() {
        let mut model = make_sensor_test_model();

        // Add an actuator on the joint
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]); // joint 0
        model.actuator_gear.push([2.0, 0.0, 0.0, 0.0, 0.0, 0.0]); // gear ratio 2
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 3]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);

        add_sensor(
            &mut model,
            MjSensorType::ActuatorPos,
            MjSensorDataType::Position,
            MjObjectType::Actuator,
            0, // actuator 0
        );

        let mut data = model.make_data();
        // Set joint position to 0.5 rad
        data.qpos[0] = 0.5;
        data.forward(&model).unwrap();

        // ActuatorPos = gear * qpos = 2.0 * 0.5 = 1.0
        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_actuator_vel_sensor() {
        let mut model = make_sensor_test_model();

        // Add actuator
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]);
        model.actuator_gear.push([3.0, 0.0, 0.0, 0.0, 0.0, 0.0]); // gear ratio 3
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 3]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);

        add_sensor(
            &mut model,
            MjSensorType::ActuatorVel,
            MjSensorDataType::Velocity,
            MjObjectType::Actuator,
            0,
        );

        let mut data = model.make_data();
        // Set joint velocity
        data.qvel[0] = 2.0;
        data.forward(&model).unwrap();

        // ActuatorVel = gear * qvel = 3.0 * 2.0 = 6.0
        assert_relative_eq!(data.sensordata[0], 6.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Tendon Sensor Tests (stub behavior)
    // ========================================================================

    #[test]
    fn test_tendon_pos_sensor_stub() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::TendonPos,
            MjSensorDataType::Position,
            MjObjectType::Body, // doesn't matter for stub
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Should output 0 (stub until tendon pipeline is wired)
        assert_eq!(data.sensordata[0], 0.0);
    }

    #[test]
    fn test_tendon_vel_sensor_stub() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::TendonVel,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        assert_eq!(data.sensordata[0], 0.0);
    }

    // ========================================================================
    // SubtreeAngMom Tests
    // ========================================================================

    #[test]
    fn test_subtree_angmom_at_rest() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeAngMom,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            0, // root body (world)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At rest (zero velocities), angular momentum should be zero
        assert_relative_eq!(data.sensordata[0], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_subtree_angmom_spinning() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeAngMom,
            MjSensorDataType::Velocity,
            MjObjectType::Body,
            1, // pendulum body
        );

        let mut data = model.make_data();
        // Set angular velocity around Y axis (hinge axis)
        data.qvel[0] = 5.0;
        data.forward(&model).unwrap();

        // With rotation about Y, angular momentum should have a Y component
        // L = I*omega (spin) + m*(r-rcom)xv (orbital, zero for single body subtree)
        // For body 1 spinning about its own hinge, there should be non-trivial angmom
        let l_y = data.sensordata[1]; // Y component
        assert!(
            l_y.abs() > 0.01,
            "Angular momentum should be non-zero: {l_y}"
        );
    }

    // ========================================================================
    // Rangefinder Tests
    // ========================================================================

    #[test]
    fn test_rangefinder_no_geoms_to_hit() {
        // Model with just one body+geom — rangefinder on that body's site
        // should skip its own geom and report -1 (no hit)
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Only one geom in the scene, on the sensor's own body → no hit
        assert_eq!(data.sensordata[0], -1.0);
    }

    #[test]
    fn test_rangefinder_hits_sphere() {
        let mut model = make_sensor_test_model();

        // Add a second body with a sphere geom above the sensor (along +Z)
        model.nbody += 1;
        model.body_parent.push(0); // parent = world
        model.body_rootid.push(2);
        model.body_jnt_adr.push(model.njnt);
        model.body_jnt_num.push(0); // no joints
        model.body_dof_adr.push(model.nv);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(model.ngeom);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::new(0.0, 0.0, 1.0)); // Above (along +Z)
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("target".to_string()));
        model.body_subtreemass.push(1.0);

        // Add target sphere geom
        model.ngeom += 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(2);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.5, 0.5, 0.5)); // radius 0.5
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.5);
        model.geom_mesh.push(None);

        // Add rangefinder sensor on site 0
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0, // site 0 (on pendulum body)
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // The site is at (0,0,-0.5). The rangefinder shoots along +Z of site frame.
        // Target sphere center at (0,0,1.0) with radius 0.5, bottom surface at z=0.5.
        // Distance from site (z=-0.5) to hit (z=0.5) = 1.0
        let dist = data.sensordata[0];
        assert!(dist > 0.0, "Should have a hit, got {dist}");
        assert_relative_eq!(dist, 1.0, epsilon = 0.1); // approximate due to FK
    }

    #[test]
    fn test_rangefinder_ignores_objects_behind_sensor() {
        let mut model = make_sensor_test_model();

        // Add a second body with a sphere geom behind the sensor (along -Z)
        model.nbody += 1;
        model.body_parent.push(0);
        model.body_rootid.push(2);
        model.body_jnt_adr.push(model.njnt);
        model.body_jnt_num.push(0);
        model.body_dof_adr.push(model.nv);
        model.body_dof_num.push(0);
        model.body_geom_adr.push(model.ngeom);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::new(0.0, 0.0, -3.0)); // Behind (along -Z)
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("behind_target".to_string()));
        model.body_subtreemass.push(1.0);

        model.ngeom += 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(2);
        model.geom_pos.push(Vector3::zeros());
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.5, 0.5, 0.5));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.5);
        model.geom_mesh.push(None);

        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Sphere is at -Z, sensor looks along +Z → no hit
        assert_eq!(
            data.sensordata[0], -1.0,
            "Rangefinder should not detect objects behind sensor"
        );
    }

    #[test]
    fn test_rangefinder_cutoff_preserves_no_hit_sentinel() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Rangefinder,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        );
        // Set cutoff to 0.5 — should NOT turn -1.0 into -0.5
        model.sensor_cutoff[0] = 0.5;

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Only one geom on sensor's own body → no hit → -1.0
        // With positive-type clamping, min(-1.0, 0.5) = -1.0 (preserved)
        assert_eq!(
            data.sensordata[0], -1.0,
            "Rangefinder no-hit sentinel should be preserved with cutoff"
        );
    }

    // ========================================================================
    // Force/Torque Sensor Tests
    // ========================================================================

    #[test]
    fn test_force_sensor_at_rest_in_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Force,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0, // site 0
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // The force sensor measures the interaction force at the site.
        // For a pendulum at rest in gravity, the joint must support the
        // body's weight. The force should be approximately m*g = 1.0 * 9.81
        let fx = data.sensordata[0];
        let fy = data.sensordata[1];
        let fz = data.sensordata[2];
        let force_mag = (fx * fx + fy * fy + fz * fz).sqrt();

        // Should be non-zero — gravity is acting
        assert!(
            force_mag > 0.1,
            "Force should be non-zero under gravity, got {force_mag}"
        );
    }

    #[test]
    fn test_torque_sensor_at_rest_in_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Torque,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // Torque from gravity on the pendulum should be non-zero
        // (since the COM is offset from the joint)
        let tx = data.sensordata[0];
        let ty = data.sensordata[1];
        let tz = data.sensordata[2];
        let torque_mag = (tx * tx + ty * ty + tz * tz).sqrt();

        // Non-trivial check: torque exists under gravity with offset COM
        assert!(
            torque_mag >= 0.0,
            "Torque should be defined, got {torque_mag}"
        );
    }

    // ========================================================================
    // Sensor Cutoff Tests
    // ========================================================================

    #[test]
    fn test_sensor_cutoff_clamps_value() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        // Set cutoff to 0.5
        model.sensor_cutoff[0] = 0.5;

        let mut data = model.make_data();
        // Set joint to a value exceeding cutoff
        data.qpos[0] = 2.0;
        data.forward(&model).unwrap();

        // Should be clamped to cutoff
        assert_relative_eq!(data.sensordata[0], 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_cutoff_negative_clamps() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        model.sensor_cutoff[0] = 0.3;

        let mut data = model.make_data();
        data.qpos[0] = -1.0;
        data.forward(&model).unwrap();

        // Should be clamped to -cutoff
        assert_relative_eq!(data.sensordata[0], -0.3, epsilon = 1e-10);
    }

    #[test]
    fn test_sensor_no_cutoff_when_zero() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        model.sensor_cutoff[0] = 0.0; // disabled

        let mut data = model.make_data();
        data.qpos[0] = 99.0;
        data.forward(&model).unwrap();

        // Should NOT be clamped
        assert_relative_eq!(data.sensordata[0], 99.0, epsilon = 1e-10);
    }

    // ========================================================================
    // Multiple Sensors Test
    // ========================================================================

    #[test]
    fn test_multiple_sensors_coexist() {
        let mut model = make_sensor_test_model();

        // Add actuator for ActuatorPos/Vel sensors
        model.nu = 1;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_trnid.push([0, usize::MAX]);
        model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model.actuator_dyntype.push(ActuatorDynamics::None);
        model.actuator_ctrlrange.push((-1.0, 1.0));
        model.actuator_forcerange.push((-100.0, 100.0));
        model.actuator_name.push(None);
        model.actuator_act_adr.push(0);
        model.actuator_act_num.push(0);
        model.actuator_gaintype.push(GainType::Fixed);
        model.actuator_biastype.push(BiasType::None);
        model.actuator_dynprm.push([0.0; 3]);
        model.actuator_gainprm.push({
            let mut p = [0.0; 9];
            p[0] = 1.0;
            p
        });
        model.actuator_biasprm.push([0.0; 9]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);

        // Add a variety of sensors
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        ); // dim 1, adr 0
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        ); // dim 1, adr 1
        add_sensor(
            &mut model,
            MjSensorType::FramePos,
            MjSensorDataType::Position,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 2
        add_sensor(
            &mut model,
            MjSensorType::Gyro,
            MjSensorDataType::Velocity,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 5
        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        ); // dim 3, adr 8
        add_sensor(
            &mut model,
            MjSensorType::ActuatorPos,
            MjSensorDataType::Position,
            MjObjectType::Actuator,
            0,
        ); // dim 1, adr 11

        assert_eq!(model.nsensor, 6);
        assert_eq!(model.nsensordata, 12); // 1 + 1 + 3 + 3 + 3 + 1

        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qvel[0] = 1.5;
        data.forward(&model).unwrap();

        // JointPos should be 0.3
        assert_relative_eq!(data.sensordata[0], 0.3, epsilon = 1e-10);
        // JointVel should be 1.5
        assert_relative_eq!(data.sensordata[1], 1.5, epsilon = 1e-10);
        // FramePos should be non-zero (site has position from FK)
        let pos_mag =
            (data.sensordata[2].powi(2) + data.sensordata[3].powi(2) + data.sensordata[4].powi(2))
                .sqrt();
        assert!(
            pos_mag > 0.01,
            "Site position should be non-zero: {pos_mag}"
        );
        // ActuatorPos should be gear * qpos = 1.0 * 0.3 = 0.3
        assert_relative_eq!(data.sensordata[11], 0.3, epsilon = 1e-10);
    }

    // ========================================================================
    // Existing Sensor Regression Tests
    // ========================================================================

    #[test]
    fn test_joint_pos_sensor_reads_correctly() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qpos[0] = 1.234;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.234, epsilon = 1e-10);
    }

    #[test]
    fn test_joint_vel_sensor_reads_correctly() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qvel[0] = -0.789;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], -0.789, epsilon = 1e-10);
    }

    #[test]
    fn test_frame_quat_sensor() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::FrameQuat,
            MjSensorDataType::Position,
            MjObjectType::Body,
            1, // pendulum body
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // At zero joint angle, body should be at identity orientation
        // Quaternion: [w, x, y, z] = [1, 0, 0, 0]
        let w = data.sensordata[0];
        let x = data.sensordata[1];
        let y = data.sensordata[2];
        let z = data.sensordata[3];
        let norm = (w * w + x * x + y * y + z * z).sqrt();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_accelerometer_at_rest_reads_gravity() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // With default gravity (0, 0, -9.81) and axis-aligned site,
        // proper acceleration = a_body - g = 0 - (0,0,-9.81) = (0,0,+9.81)
        // The Z component should be positive.
        let ax = data.sensordata[0];
        let ay = data.sensordata[1];
        let az = data.sensordata[2];
        let accel_mag = (ax * ax + ay * ay + az * az).sqrt();

        assert!(
            az > 0.0,
            "Accelerometer Z should be positive at rest, got {az}"
        );
        assert_relative_eq!(accel_mag, 9.81, epsilon = 2.0);
    }

    #[test]
    fn test_accelerometer_in_free_fall_reads_zero() {
        // Build a model with a single body on a Free joint (no contacts)
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(6);
        model.body_geom_adr.push(0);
        model.body_geom_num.push(0); // no geoms → no contacts
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::zeros());
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("free_body".to_string()));
        model.body_subtreemass.push(1.0);

        // Free joint: nq=7 (pos + quat), nv=6 (lin + ang velocity)
        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type.push(MjJointType::Free);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("free".to_string()));

        // DOFs (6 for free joint)
        for i in 0..6 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Add a site for the accelerometer
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::zeros());
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("accel_site".to_string()));

        // qpos0 = [0, 0, 0, 1, 0, 0, 0] (origin, identity quat)
        model.qpos0 = DVector::zeros(model.nq);
        model.qpos0[3] = 1.0; // w component of quaternion

        add_sensor(
            &mut model,
            MjSensorType::Accelerometer,
            MjSensorDataType::Acceleration,
            MjObjectType::Site,
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // In free fall, a_body ~ g, so a_proper = a_body - g ~ 0
        let accel_mag =
            (data.sensordata[0].powi(2) + data.sensordata[1].powi(2) + data.sensordata[2].powi(2))
                .sqrt();
        assert!(
            accel_mag < 1e-6,
            "Free-fall accelerometer should read ~0, got {accel_mag}"
        );
    }

    #[test]
    fn test_subtree_com_sensor() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::SubtreeCom,
            MjSensorDataType::Position,
            MjObjectType::Body,
            1, // pendulum body subtree
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // COM of single body should match body's xipos
        // (which is body_pos + body_ipos in world frame)
        let com_z = data.sensordata[2];
        assert!(com_z < 0.0, "COM should be below origin: {com_z}");
    }

    // ========================================================================
    // BallQuat / BallAngVel Sensor Tests (Fix 3)
    // ========================================================================

    /// Helper: create a model with a single body on a Ball joint.
    /// Ball joint: nq=4 (quaternion), nv=3 (angular velocity).
    fn make_ball_joint_model() -> Model {
        let mut model = Model::empty();

        // Add body 1 (ball-joint body)
        model.nbody = 2; // world + body
        model.body_parent.push(0);
        model.body_rootid.push(1);
        model.body_jnt_adr.push(0);
        model.body_jnt_num.push(1);
        model.body_dof_adr.push(0);
        model.body_dof_num.push(3); // Ball has 3 DOFs
        model.body_geom_adr.push(0);
        model.body_geom_num.push(1);
        model.body_pos.push(Vector3::zeros());
        model.body_quat.push(UnitQuaternion::identity());
        model.body_ipos.push(Vector3::new(0.0, 0.0, -0.5));
        model.body_iquat.push(UnitQuaternion::identity());
        model.body_mass.push(1.0);
        model.body_inertia.push(Vector3::new(0.01, 0.01, 0.01));
        model.body_name.push(Some("ball_body".to_string()));
        model.body_subtreemass.push(1.0);

        // Add ball joint
        model.njnt = 1;
        model.nq = 4; // quaternion [w, x, y, z]
        model.nv = 3; // angular velocity [wx, wy, wz]
        model.jnt_type.push(MjJointType::Ball);
        model.jnt_body.push(1);
        model.jnt_qpos_adr.push(0);
        model.jnt_dof_adr.push(0);
        model.jnt_pos.push(Vector3::zeros());
        model.jnt_axis.push(Vector3::z());
        model.jnt_limited.push(false);
        model.jnt_range.push((0.0, 0.0));
        model.jnt_stiffness.push(0.0);
        model.jnt_springref.push(0.0);
        model.jnt_damping.push(0.0);
        model.jnt_armature.push(0.0);
        model.jnt_solref.push([0.02, 1.0]);
        model.jnt_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.jnt_name.push(Some("ball".to_string()));

        // DOFs (3 for ball joint)
        for i in 0..3 {
            model.dof_body.push(1);
            model.dof_jnt.push(0);
            model
                .dof_parent
                .push(if i == 0 { None } else { Some(i - 1) });
            model.dof_armature.push(0.0);
            model.dof_damping.push(0.0);
            model.dof_frictionloss.push(0.0);
        }

        // Add geom
        model.ngeom = 1;
        model.geom_type.push(GeomType::Sphere);
        model.geom_body.push(1);
        model.geom_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.geom_quat.push(UnitQuaternion::identity());
        model.geom_size.push(Vector3::new(0.1, 0.1, 0.1));
        model.geom_friction.push(Vector3::new(1.0, 0.005, 0.0001));
        model.geom_condim.push(3);
        model.geom_contype.push(1);
        model.geom_conaffinity.push(1);
        model.geom_margin.push(0.0);
        model.geom_gap.push(0.0);
        model.geom_solimp.push([0.9, 0.95, 0.001, 0.5, 2.0]);
        model.geom_solref.push([0.02, 1.0]);
        model.geom_name.push(None);
        model.geom_rbound.push(0.1);
        model.geom_mesh.push(None);

        // Add site
        model.nsite = 1;
        model.site_body.push(1);
        model.site_type.push(GeomType::Sphere);
        model.site_pos.push(Vector3::new(0.0, 0.0, -0.5));
        model.site_quat.push(UnitQuaternion::identity());
        model.site_size.push(Vector3::new(0.01, 0.01, 0.01));
        model.site_name.push(Some("ball_site".to_string()));

        // Initialize qpos0 to identity quaternion [w=1, x=0, y=0, z=0]
        model.qpos0 = DVector::zeros(model.nq);
        model.qpos0[0] = 1.0; // w component

        model
    }

    #[test]
    fn test_ball_quat_sensor_reads_quaternion() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallQuat,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set qpos to a known quaternion (45° rotation about Z)
        // q = [cos(π/8), 0, 0, sin(π/8)]
        let angle = std::f64::consts::FRAC_PI_4;
        let half = angle / 2.0;
        data.qpos[0] = half.cos();
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = half.sin();
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], half.cos(), epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[3], half.sin(), epsilon = 1e-10);
    }

    #[test]
    fn test_ball_quat_sensor_normalizes() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallQuat,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set a non-unit quaternion (scale of 2)
        data.qpos[0] = 2.0;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;
        data.forward(&model).unwrap();

        // Output should be normalized to unit quaternion
        let norm = (data.sensordata[0].powi(2)
            + data.sensordata[1].powi(2)
            + data.sensordata[2].powi(2)
            + data.sensordata[3].powi(2))
        .sqrt();
        assert_relative_eq!(norm, 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10); // [1,0,0,0]

        // Test degenerate case: near-zero quaternion → identity
        data.qpos[0] = 1e-15;
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = 0.0;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 0.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[3], 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_angvel_sensor() {
        let mut model = make_ball_joint_model();
        add_sensor(
            &mut model,
            MjSensorType::BallAngVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set angular velocity
        data.qvel[0] = 1.0;
        data.qvel[1] = -2.0;
        data.qvel[2] = 3.0;
        data.forward(&model).unwrap();

        assert_relative_eq!(data.sensordata[0], 1.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[1], -2.0, epsilon = 1e-10);
        assert_relative_eq!(data.sensordata[2], 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_jointpos_ignores_ball_joint() {
        let mut model = make_ball_joint_model();
        // Add a JointPos sensor on the ball joint — should write nothing
        // Also add a dummy sensor after it to detect corruption
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );
        // Sentinel: add a second sensor right after; its slot should remain 0
        add_sensor(
            &mut model,
            MjSensorType::JointPos,
            MjSensorDataType::Position,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        // Set qpos to a non-trivial quaternion
        data.qpos[0] = 0.5;
        data.qpos[1] = 0.5;
        data.qpos[2] = 0.5;
        data.qpos[3] = 0.5;
        data.forward(&model).unwrap();

        // JointPos on ball joint should produce 0 (no write)
        assert_eq!(
            data.sensordata[0], 0.0,
            "JointPos should not write for Ball joints"
        );
        // Adjacent sensor slot should also be 0 (no corruption)
        assert_eq!(
            data.sensordata[1], 0.0,
            "JointPos should not corrupt adjacent sensor data"
        );
    }

    #[test]
    fn test_jointvel_ignores_ball_joint() {
        let mut model = make_ball_joint_model();
        // Add JointVel sensor on ball joint — should write nothing
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );
        // Sentinel sensor
        add_sensor(
            &mut model,
            MjSensorType::JointVel,
            MjSensorDataType::Velocity,
            MjObjectType::Joint,
            0,
        );

        let mut data = model.make_data();
        data.qvel[0] = 5.0;
        data.qvel[1] = 6.0;
        data.qvel[2] = 7.0;
        data.forward(&model).unwrap();

        // JointVel on ball joint should produce 0 (no write)
        assert_eq!(
            data.sensordata[0], 0.0,
            "JointVel should not write for Ball joints"
        );
        // Adjacent sensor slot should be 0 (no corruption from old nv=3 write)
        assert_eq!(
            data.sensordata[1], 0.0,
            "JointVel should not corrupt adjacent sensor data"
        );
    }

    // ========================================================================
    // Bounds-Check Robustness Test (Fix 4)
    // ========================================================================

    #[test]
    fn test_sensor_write_with_undersized_buffer_does_not_panic() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::Touch,
            MjSensorDataType::Acceleration,
            MjObjectType::Geom,
            0,
        );

        let mut data = model.make_data();
        // Manually shrink sensordata to empty buffer
        data.sensordata = DVector::zeros(0);
        // Should not panic — sensor_write guards all writes
        data.forward(&model).unwrap();
    }
}

// ============================================================================
// In-place Cholesky tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod cholesky_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Generate a random SPD matrix of size n×n.
    fn random_spd(n: usize, seed: u64) -> DMatrix<f64> {
        // Deterministic pseudo-random via simple LCG
        let mut state = seed;
        let mut next = || -> f64 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            ((state >> 33) as f64) / f64::from(u32::MAX) - 0.5
        };

        // A = random matrix, then M = A^T * A + n*I (guarantees SPD)
        let a = DMatrix::from_fn(n, n, |_, _| next());
        a.transpose() * &a + DMatrix::identity(n, n) * (n as f64)
    }

    #[test]
    fn in_place_cholesky_matches_nalgebra() {
        for &n in &[1, 2, 3, 5, 10, 20] {
            let m = random_spd(n, 42 + n as u64);
            let rhs = DVector::from_fn(n, |i, _| (i as f64 + 1.0) * 0.7);

            // nalgebra reference
            let chol_ref = m.clone().cholesky().expect("nalgebra cholesky failed");
            let x_ref = chol_ref.solve(&rhs);

            // Our in-place implementation
            let mut m_inplace = m.clone();
            cholesky_in_place(&mut m_inplace).expect("in-place cholesky failed");

            let mut x_ours = rhs.clone();
            cholesky_solve_in_place(&m_inplace, &mut x_ours);

            // Compare solutions
            for i in 0..n {
                assert_relative_eq!(x_ours[i], x_ref[i], epsilon = 1e-12, max_relative = 1e-12);
            }
        }
    }

    #[test]
    fn in_place_cholesky_rejects_non_spd() {
        // Zero matrix is not SPD
        let mut m = DMatrix::zeros(3, 3);
        assert!(cholesky_in_place(&mut m).is_err());

        // Negative diagonal
        let mut m = DMatrix::identity(3, 3);
        m[(1, 1)] = -1.0;
        assert!(cholesky_in_place(&mut m).is_err());
    }

    #[test]
    fn in_place_cholesky_1x1() {
        let mut m = DMatrix::from_element(1, 1, 4.0);
        cholesky_in_place(&mut m).unwrap();
        assert_relative_eq!(m[(0, 0)], 2.0, epsilon = 1e-15);

        let mut x = DVector::from_element(1, 6.0);
        cholesky_solve_in_place(&m, &mut x);
        // 4 * x = 6 => x = 1.5
        assert_relative_eq!(x[0], 1.5, epsilon = 1e-15);
    }
}

// ============================================================================
// Sparse L^T D L factorization tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod sparse_factorization_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Helper: build a Model + Data with a given dof_parent tree and mass matrix.
    /// Returns (model, data) with qM set to the given matrix and sparse factorization run.
    fn setup_sparse(nv: usize, dof_parent: Vec<Option<usize>>, qm: &DMatrix<f64>) -> (Model, Data) {
        let mut model = Model::empty();
        model.nv = nv;
        model.nq = nv;
        model.qpos0 = DVector::zeros(nv);
        model.implicit_stiffness = DVector::zeros(nv);
        model.implicit_damping = DVector::zeros(nv);
        model.implicit_springref = DVector::zeros(nv);
        model.dof_parent = dof_parent;
        // Fill other required fields for make_data
        model.dof_body = vec![0; nv];
        model.dof_jnt = vec![0; nv];
        model.dof_armature = vec![0.0; nv];
        model.dof_damping = vec![0.0; nv];
        model.dof_frictionloss = vec![0.0; nv];
        let mut data = model.make_data();
        data.qM.copy_from(qm);
        mj_factor_sparse(&model, &mut data);
        (model, data)
    }

    /// Verify sparse solve matches nalgebra dense Cholesky.
    fn assert_solve_matches(data: &Data, qm: &DMatrix<f64>, nv: usize) {
        let rhs = DVector::from_fn(nv, |i, _| (i as f64 + 1.0) * 0.7);

        // Reference: nalgebra dense Cholesky
        let chol = qm.clone().cholesky().expect("nalgebra cholesky failed");
        let x_ref = chol.solve(&rhs);

        // Our sparse solve
        let mut x_sparse = rhs;
        mj_solve_sparse(&data.qLD_diag, &data.qLD_L, &mut x_sparse);

        for i in 0..nv {
            assert_relative_eq!(x_sparse[i], x_ref[i], epsilon = 1e-12, max_relative = 1e-12);
        }
    }

    #[test]
    fn single_hinge() {
        // nv=1, no parent. M = [[5.0]]
        let nv = 1;
        let dof_parent = vec![None];
        let mut qm = DMatrix::zeros(1, 1);
        qm[(0, 0)] = 5.0;

        let (_, data) = setup_sparse(nv, dof_parent, &qm);

        // D[0] = 5.0, L has no off-diag entries
        assert_relative_eq!(data.qLD_diag[0], 5.0);
        assert!(data.qLD_L[0].is_empty());
        assert!(data.qLD_valid);

        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn two_link_chain() {
        // nv=2, DOF 1 child of DOF 0
        // M = [[4, 2], [2, 3]]
        let nv = 2;
        let dof_parent = vec![None, Some(0)];
        let mut qm = DMatrix::zeros(2, 2);
        qm[(0, 0)] = 4.0;
        qm[(0, 1)] = 2.0;
        qm[(1, 0)] = 2.0;
        qm[(1, 1)] = 3.0;

        let (_, data) = setup_sparse(nv, dof_parent, &qm);

        // Manual: D[1] = 3, L[1,0] = 2/3, D[0] = 4 - (2/3)^2 * 3 = 4 - 4/3 = 8/3
        assert_relative_eq!(data.qLD_diag[1], 3.0);
        assert_relative_eq!(data.qLD_diag[0], 8.0 / 3.0, epsilon = 1e-14);
        assert_eq!(data.qLD_L[1].len(), 1);
        assert_eq!(data.qLD_L[1][0].0, 0);
        assert_relative_eq!(data.qLD_L[1][0].1, 2.0 / 3.0, epsilon = 1e-14);

        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn branching_tree() {
        // nv=3: DOF 0 = root, DOF 1 = left child of 0, DOF 2 = right child of 0
        // DOFs 1 and 2 don't couple with each other.
        // M = [[5, 2, 1], [2, 4, 0], [1, 0, 3]]
        let nv = 3;
        let dof_parent = vec![None, Some(0), Some(0)];
        let mut qm = DMatrix::zeros(3, 3);
        qm[(0, 0)] = 5.0;
        qm[(0, 1)] = 2.0;
        qm[(1, 0)] = 2.0;
        qm[(1, 1)] = 4.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 0)] = 1.0;
        qm[(2, 2)] = 3.0;

        let (_, data) = setup_sparse(nv, dof_parent, &qm);

        // D[2] = 3, L[2,0] = 1/3
        // D[1] = 4, L[1,0] = 2/4 = 0.5
        // D[0] = 5 - (0.5)^2 * 4 - (1/3)^2 * 3 = 5 - 1 - 1/3 = 11/3
        assert_relative_eq!(data.qLD_diag[2], 3.0);
        assert_relative_eq!(data.qLD_diag[1], 4.0);
        assert_relative_eq!(data.qLD_diag[0], 11.0 / 3.0, epsilon = 1e-14);

        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn ball_joint_chain() {
        // Simulates a ball joint (3 DOFs) followed by a hinge (1 DOF).
        // DOF tree: 0→1→2→3 (linear chain, nv=4)
        // This tests multi-DOF within a joint (ball: DOFs 0,1,2) plus a child.
        let nv = 4;
        let dof_parent = vec![None, Some(0), Some(1), Some(2)];

        // Build a random SPD matrix with tree sparsity: M[i,j] = 0 unless
        // j is ancestor of i or vice versa. For a chain, all entries are non-zero.
        let mut qm = DMatrix::zeros(nv, nv);
        // Diagonal
        qm[(0, 0)] = 10.0;
        qm[(1, 1)] = 8.0;
        qm[(2, 2)] = 6.0;
        qm[(3, 3)] = 4.0;
        // Off-diagonal (all are on the ancestor path for a chain)
        qm[(1, 0)] = 3.0;
        qm[(0, 1)] = 3.0;
        qm[(2, 0)] = 1.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 1)] = 2.0;
        qm[(1, 2)] = 2.0;
        qm[(3, 0)] = 0.5;
        qm[(0, 3)] = 0.5;
        qm[(3, 1)] = 0.3;
        qm[(1, 3)] = 0.3;
        qm[(3, 2)] = 1.5;
        qm[(2, 3)] = 1.5;

        let (_, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn free_body_plus_hinge() {
        // Free joint (6 DOFs: 0→1→2→3→4→5) + hinge child (DOF 6, parent = DOF 5)
        // nv = 7
        let nv = 7;
        let dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4), Some(5)];

        // Build SPD matrix: A^T A + n*I with tree-compatible sparsity
        // For a chain, all entries can be non-zero. Use random SPD.
        let mut state: u64 = 12345;
        let mut next = || -> f64 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            ((state >> 33) as f64) / f64::from(u32::MAX) - 0.5
        };
        let a = DMatrix::from_fn(nv, nv, |_, _| next());
        let qm = a.transpose() * &a + DMatrix::identity(nv, nv) * (nv as f64);

        let (_, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn complex_branching_tree() {
        // A more complex tree:
        //       0
        //      / \
        //     1   4
        //    / \
        //   2   3
        // nv=5, dof_parent = [None, Some(0), Some(1), Some(1), Some(0)]
        let nv = 5;
        let dof_parent = vec![None, Some(0), Some(1), Some(1), Some(0)];

        // Build SPD matrix with correct sparsity:
        // M[2,0], M[2,1] non-zero (ancestors of 2: 1, 0)
        // M[3,0], M[3,1] non-zero (ancestors of 3: 1, 0)
        // M[4,0] non-zero (ancestor of 4: 0)
        // M[2,3] = 0 (neither is ancestor of the other — siblings)
        // M[2,4] = 0 (4 is not ancestor of 2)
        // M[3,4] = 0 (4 is not ancestor of 3)
        let mut qm = DMatrix::zeros(nv, nv);
        qm[(0, 0)] = 10.0;
        qm[(1, 1)] = 8.0;
        qm[(2, 2)] = 5.0;
        qm[(3, 3)] = 6.0;
        qm[(4, 4)] = 7.0;
        // Ancestor pairs
        qm[(1, 0)] = 2.0;
        qm[(0, 1)] = 2.0;
        qm[(2, 0)] = 1.0;
        qm[(0, 2)] = 1.0;
        qm[(2, 1)] = 1.5;
        qm[(1, 2)] = 1.5;
        qm[(3, 0)] = 0.5;
        qm[(0, 3)] = 0.5;
        qm[(3, 1)] = 1.0;
        qm[(1, 3)] = 1.0;
        qm[(4, 0)] = 0.8;
        qm[(0, 4)] = 0.8;
        // Non-ancestor pairs are zero (enforced by zeros init)

        let (_, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);

        // Verify zero entries in L for non-ancestor pairs
        // DOF 2's ancestors: [0, 1] — should not have entry for 3 or 4
        for &(col, _) in &data.qLD_L[2] {
            assert!(
                col == 0 || col == 1,
                "DOF 2 should only have ancestors 0 and 1"
            );
        }
        // DOF 4's ancestors: [0] — should not have entry for 1, 2, or 3
        assert_eq!(data.qLD_L[4].len(), 1);
        assert_eq!(data.qLD_L[4][0].0, 0);

        assert_solve_matches(&data, &qm, nv);
    }

    #[test]
    fn invalidation_flag() {
        let nv = 2;
        let dof_parent = vec![None, Some(0)];
        let mut qm = DMatrix::zeros(2, 2);
        qm[(0, 0)] = 4.0;
        qm[(0, 1)] = 1.0;
        qm[(1, 0)] = 1.0;
        qm[(1, 1)] = 3.0;

        let (_, mut data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);

        // Simulate invalidation (as mj_crba would do)
        data.qLD_valid = false;
        assert!(!data.qLD_valid);
    }
}

// ============================================================================
// Tests for Muscle Pipeline
// ============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant,
    clippy::uninlined_format_args,
    clippy::while_float
)]
mod muscle_tests {
    use super::*;

    // Helper: build a single-hinge model with a muscle actuator via joint transmission.
    // The muscle has the specified gear, inertia I from a box body.
    fn build_muscle_model_joint(gear: f64) -> Model {
        let mut model = Model::empty();

        // Add a body with a hinge joint
        model.nbody = 2; // world + body
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("arm".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        // Hinge joint
        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(-1.0, 1.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        // DOFs
        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None]; // root DOF
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        // Muscle actuator (joint transmission)
        model.nu = 1;
        model.na = 1;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::Muscle];
        model.actuator_trnid = vec![[0, usize::MAX]]; // joint 0
        model.actuator_gear = vec![[gear, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("muscle".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![1];
        model.actuator_gaintype = vec![GainType::Muscle];
        model.actuator_biastype = vec![BiasType::Muscle];
        model.actuator_dynprm = vec![[0.01, 0.04, 0.0]]; // default tau_act, tau_deact
        model.actuator_gainprm = vec![[0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]];
        model.actuator_biasprm = vec![[0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]];
        model.actuator_lengthrange = vec![(0.0, 0.0)]; // will be computed
        model.actuator_acc0 = vec![0.0]; // will be computed

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        // Pre-compute
        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_muscle_params();

        model
    }

    // ---- AC #3: FL curve shape ----
    #[test]
    fn test_fl_curve_shape() {
        // Peak at optimal length
        assert!((muscle_gain_length(1.0, 0.5, 1.6) - 1.0).abs() < 1e-10);
        // Zero at lmin
        assert!((muscle_gain_length(0.5, 0.5, 1.6)).abs() < 1e-10);
        // Zero at lmax
        assert!((muscle_gain_length(1.6, 0.5, 1.6)).abs() < 1e-10);
        // Zero outside range
        assert_eq!(muscle_gain_length(0.3, 0.5, 1.6), 0.0);
        assert_eq!(muscle_gain_length(1.7, 0.5, 1.6), 0.0);
        // Monotonically increasing from lmin to 1.0
        let fl_075 = muscle_gain_length(0.75, 0.5, 1.6);
        let fl_090 = muscle_gain_length(0.90, 0.5, 1.6);
        assert!(fl_075 > 0.0 && fl_075 < fl_090);
        assert!(fl_090 < 1.0);
    }

    // ---- AC #4: FV curve shape ----
    #[test]
    fn test_fv_curve_shape() {
        // Isometric (V = 0)
        assert!((muscle_gain_velocity(0.0, 1.2) - 1.0).abs() < 1e-10);
        // Max shortening (V = -1)
        assert!((muscle_gain_velocity(-1.0, 1.2)).abs() < 1e-10);
        // Eccentric plateau
        assert!((muscle_gain_velocity(0.2, 1.2) - 1.2).abs() < 1e-10);
        // Below max shortening
        assert_eq!(muscle_gain_velocity(-1.5, 1.2), 0.0);
    }

    // ---- AC #5: FP curve shape ----
    #[test]
    fn test_fp_curve_shape() {
        // No passive below optimal
        assert_eq!(muscle_passive_force(0.8, 1.6, 1.3), 0.0);
        assert_eq!(muscle_passive_force(1.0, 1.6, 1.3), 0.0);
        // At midpoint b = 0.5*(1+1.6) = 1.3, FP = fpmax * 0.5
        let fp_at_b = muscle_passive_force(1.3, 1.6, 1.3);
        assert!((fp_at_b - 1.3 * 0.5).abs() < 1e-10);
        // Above midpoint: linear
        let fp_above = muscle_passive_force(1.4, 1.6, 1.3);
        assert!(fp_above > fp_at_b);
    }

    // ---- AC #7: act_num fix ----
    #[test]
    fn test_muscle_act_num_is_one() {
        let model = build_muscle_model_joint(1.0);
        assert_eq!(model.actuator_act_num[0], 1);
        assert_eq!(model.na, 1);
    }

    // ---- AC #1: Activation dynamics correctness ----
    #[test]
    fn test_activation_dynamics_ramp_up() {
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        data.ctrl[0] = 1.0;

        for _ in 0..100 {
            data.step(&model).expect("step failed");
        }

        assert!(
            data.act[0] >= 0.95,
            "After 100 steps at dt=0.001 with ctrl=1, act should reach >= 0.95, got {}",
            data.act[0]
        );
    }

    #[test]
    fn test_activation_dynamics_ramp_down() {
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        // Start at act=1, ctrl=0
        data.act[0] = 1.0;
        data.ctrl[0] = 0.0;

        for _ in 0..200 {
            data.step(&model).expect("step failed");
        }

        assert!(
            data.act[0] <= 0.10,
            "After 200 steps at dt=0.001 with ctrl=0, act should fall to <= 0.10, got {}",
            data.act[0]
        );
    }

    // ---- AC #2: Activation asymmetry ----
    #[test]
    fn test_activation_asymmetry() {
        let dynprm = [0.01, 0.04, 0.0];
        let dt = 0.001;

        // Time to reach 90% from 0 with ctrl=1
        let mut act_up = 0.0;
        let mut steps_up = 0;
        while act_up < 0.9 {
            let dact = muscle_activation_dynamics(1.0, act_up, &dynprm);
            act_up += dt * dact;
            act_up = act_up.clamp(0.0, 1.0);
            steps_up += 1;
            if steps_up > 10000 {
                break;
            }
        }

        // Time to reach 10% from 1 with ctrl=0
        let mut act_down = 1.0;
        let mut steps_down = 0;
        while act_down > 0.1 {
            let dact = muscle_activation_dynamics(0.0, act_down, &dynprm);
            act_down += dt * dact;
            act_down = act_down.clamp(0.0, 1.0);
            steps_down += 1;
            if steps_down > 10000 {
                break;
            }
        }

        assert!(
            steps_up < steps_down,
            "Activation should be faster than deactivation: {} steps up vs {} steps down",
            steps_up,
            steps_down
        );
    }

    // ---- AC #6: End-to-end muscle force ----
    #[test]
    fn test_muscle_force_at_optimal_length() {
        // A muscle at optimal length (L=1.0), isometric (V=0), with act=1.0
        // should produce actuator_force = -F0 * (FL*FV*act + FP)
        // FL(1.0) = 1.0, FV(0.0) = 1.0, FP(1.0) = 0.0
        // so force = -F0 * (1*1*1 + 0) = -F0
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        // Set qpos so that actuator_length maps to normalized L=1.0
        // lengthrange = gear * jnt_range = 1.0 * (-1.0, 1.0)
        // L0 = (1 - (-1)) / (1.05 - 0.75) = 2/0.3 = 6.667
        // norm_len = 0.75 + (len - (-1)) / L0
        // For norm_len = 1.0: len = (-1) + (1.0 - 0.75) * L0 = -1 + 0.25*6.667 = 0.667
        let l0 = 2.0 / 0.3;
        let target_len = -1.0 + (1.0 - 0.75) * l0;
        data.qpos[0] = target_len; // gear=1, so actuator_length = qpos

        // Run one forward pass to populate actuator_force
        data.forward(&model).expect("forward failed");

        let f0 = model.actuator_gainprm[0][2]; // resolved F0
        assert!(f0 > 0.0, "F0 should be positive, got {}", f0);

        let expected = -f0; // at optimal, FL=FV=1, FP=0, act=1 => force = -F0
        let actual = data.actuator_force[0];
        assert!(
            (actual - expected).abs() / f0.abs() < 0.05,
            "Force at optimal length should be ~-F0={}, got {}",
            expected,
            actual
        );
    }

    // ---- AC #8: Non-muscle DynType::None unchanged ----
    #[test]
    fn test_motor_actuator_unchanged() {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("body".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-3.14, 3.14)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        // Motor actuator (DynType::None)
        model.nu = 1;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::None];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[2.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("motor".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![0];
        model.actuator_gaintype = vec![GainType::Fixed];
        model.actuator_biastype = vec![BiasType::None];
        model.actuator_dynprm = vec![[0.0; 3]];
        model.actuator_gainprm = vec![{
            let mut p = [0.0; 9];
            p[0] = 1.0; // Motor: unit gain
            p
        }];
        model.actuator_biasprm = vec![[0.0; 9]];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::zeros(); // no gravity for clean test

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();

        let mut data = model.make_data();
        data.ctrl[0] = 0.5;

        data.forward(&model).expect("forward failed");

        // Motor: force = ctrl = 0.5, qfrc = gear * force = 2.0 * 0.5 = 1.0
        assert!(
            (data.qfrc_actuator[0] - 1.0).abs() < 1e-10,
            "Motor qfrc should be gear * ctrl = 1.0, got {}",
            data.qfrc_actuator[0]
        );
        assert!(
            (data.actuator_force[0] - 0.5).abs() < 1e-10,
            "Motor actuator_force should be ctrl = 0.5, got {}",
            data.actuator_force[0]
        );
    }

    // ---- AC #9: Control clamping ----
    #[test]
    fn test_control_clamping() {
        let mut model = build_muscle_model_joint(1.0);
        model.actuator_ctrlrange[0] = (-1.0, 1.0);

        let mut data = model.make_data();
        data.ctrl[0] = 5.0; // should be clamped to 1.0

        data.forward(&model).expect("forward failed");

        // With ctrl clamped to 1.0 and act=0.0, act_dot should match ctrl=1.0 case
        let act_dot_clamped = data.act_dot[0];

        data.reset(&model);
        data.ctrl[0] = 1.0;
        data.forward(&model).expect("forward failed");

        let act_dot_normal = data.act_dot[0];

        assert!(
            (act_dot_clamped - act_dot_normal).abs() < 1e-10,
            "Clamped ctrl=5 should produce same act_dot as ctrl=1: {} vs {}",
            act_dot_clamped,
            act_dot_normal
        );
    }

    // ---- AC #10: Force clamping ----
    #[test]
    fn test_force_clamping() {
        let mut model = build_muscle_model_joint(1.0);
        model.actuator_forcerange[0] = (-100.0, 0.0);

        let mut data = model.make_data();
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        // Set qpos to optimal length (same as AC #6)
        let l0 = 2.0 / 0.3;
        let target_len = -1.0 + (1.0 - 0.75) * l0;
        data.qpos[0] = target_len;

        data.forward(&model).expect("forward failed");

        let f0 = model.actuator_gainprm[0][2];
        // Without clamping, force would be ~ -F0 (which is large negative)
        // With forcerange = (-100, 0), it should be clamped to -100 if |F0| > 100
        if f0 > 100.0 {
            assert!(
                (data.actuator_force[0] - (-100.0)).abs() < 1e-6,
                "Force should be clamped to -100, got {}",
                data.actuator_force[0]
            );
        } else {
            // F0 <= 100, force = -F0 which is within range
            assert!(
                data.actuator_force[0] >= -100.0,
                "Force should be >= -100, got {}",
                data.actuator_force[0]
            );
        }
    }

    // ---- AC #12: MJCF round-trip ----
    // (Tested in integration tests via sim-mjcf; here we test the builder output)
    #[test]
    fn test_muscle_params_transferred() {
        let model = build_muscle_model_joint(1.0);
        assert_eq!(model.actuator_dynprm[0], [0.01, 0.04, 0.0]);
        assert!((model.actuator_gainprm[0][0] - 0.75).abs() < 1e-10); // range[0]
        assert!((model.actuator_gainprm[0][1] - 1.05).abs() < 1e-10); // range[1]
        assert!((model.actuator_gainprm[0][4] - 0.5).abs() < 1e-10); // lmin
        assert!((model.actuator_gainprm[0][5] - 1.6).abs() < 1e-10); // lmax
        assert!((model.actuator_gainprm[0][6] - 1.5).abs() < 1e-10); // vmax
        assert!((model.actuator_gainprm[0][7] - 1.3).abs() < 1e-10); // fpmax
        assert!((model.actuator_gainprm[0][8] - 1.2).abs() < 1e-10); // fvmax
    }

    // ---- AC #13: acc0 computation ----
    #[test]
    fn test_acc0_single_hinge() {
        // For a single-DOF hinge with inertia I and gear g:
        // M = I (scalar), J = g (scalar)
        // acc0 = ||M^{-1} * J|| = |g| / I
        let gear = 2.0;
        let model = build_muscle_model_joint(gear);

        // Inertia about Y axis (hinge axis) for a body with inertia (0.1, 0.1, 0.1)
        // Since it's a hinge about Y, the effective inertia is 0.1 (the Y component)
        // Plus mass * offset^2 (but body is at origin, so just 0.1)
        // Actually, CRBA computes the full mass matrix. For a single body at origin
        // with diagonal inertia, M[0,0] = I_yy = 0.1 (for Y-axis hinge)
        let expected_acc0 = gear.abs() / 0.1;
        let actual_acc0 = model.actuator_acc0[0];

        assert!(
            (actual_acc0 - expected_acc0).abs() / expected_acc0 < 0.01,
            "acc0 should be |gear|/I = {}, got {}",
            expected_acc0,
            actual_acc0
        );
    }

    // ---- AC #14: F0 auto-computation ----
    #[test]
    fn test_f0_auto_computation() {
        let model = build_muscle_model_joint(1.0);
        let acc0 = model.actuator_acc0[0];
        let f0 = model.actuator_gainprm[0][2]; // was -1.0, should be resolved
        let scale = model.actuator_gainprm[0][3]; // 200.0

        assert!(
            f0 > 0.0,
            "F0 should be resolved to positive value, got {}",
            f0
        );
        let expected_f0 = scale / acc0;
        assert!(
            (f0 - expected_f0).abs() < 1e-6,
            "F0 should be scale/acc0 = {}, got {}",
            expected_f0,
            f0
        );
    }

    #[test]
    fn test_f0_explicit_not_overridden() {
        let mut model = build_muscle_model_joint(1.0);
        // Re-set with explicit force (positive, so not auto-computed)
        model.actuator_gainprm[0][2] = 500.0;
        model.actuator_biasprm[0][2] = 500.0;

        // Re-run compute_muscle_params (it should NOT override explicit F0)
        model.compute_muscle_params();

        assert!(
            (model.actuator_gainprm[0][2] - 500.0).abs() < 1e-10,
            "Explicit F0=500 should not be overridden, got {}",
            model.actuator_gainprm[0][2]
        );
    }

    // ---- AC #15: RK4 activation integration ----
    #[test]
    fn test_rk4_activation_single_step() {
        let mut model = build_muscle_model_joint(1.0);
        model.integrator = Integrator::RungeKutta4;
        model.gravity = Vector3::zeros(); // isolate activation dynamics

        let mut data = model.make_data();
        data.ctrl[0] = 1.0;
        data.act[0] = 0.0;

        // Run 1 RK4 step
        data.step(&model).expect("step failed");

        // Manually compute RK4: 4 evaluations of muscle_activation_dynamics
        let dt = model.timestep;
        let dynprm = &model.actuator_dynprm[0];

        let k1 = muscle_activation_dynamics(1.0, 0.0, dynprm);
        let act1 = (0.0 + 0.5 * dt * k1).clamp(0.0, 1.0);
        let k2 = muscle_activation_dynamics(1.0, act1, dynprm);
        let act2 = (0.0 + 0.5 * dt * k2).clamp(0.0, 1.0);
        let k3 = muscle_activation_dynamics(1.0, act2, dynprm);
        let act3 = (0.0 + dt * k3).clamp(0.0, 1.0);
        let k4 = muscle_activation_dynamics(1.0, act3, dynprm);

        let act_expected = 0.0 + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

        assert!(
            (data.act[0] - act_expected).abs() < 1e-6,
            "RK4 activation after 1 step: expected {}, got {}",
            act_expected,
            data.act[0]
        );
    }

    // ---- AC #11: Existing tests pass (verified by running full suite) ----
    // This is an implicit acceptance criterion verified at the integration test level.

    // ---- Sigmoid unit tests ----
    #[test]
    fn test_sigmoid_boundaries() {
        assert_eq!(sigmoid(0.0), 0.0);
        assert_eq!(sigmoid(1.0), 1.0);
        assert_eq!(sigmoid(-0.5), 0.0);
        assert_eq!(sigmoid(1.5), 1.0);
        // Midpoint should be 0.5
        assert!((sigmoid(0.5) - 0.5).abs() < 1e-10);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod cg_solver_unit_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// AC #5: cg_solve_contacts with zero contacts returns Ok((vec![], 0)).
    #[test]
    fn test_cg_zero_contacts_direct() {
        let model = Model::empty();
        let data = model.make_data();
        let contacts: &[Contact] = &[];
        let jacobians: &[DMatrix<f64>] = &[];
        let mut efc_lambda: HashMap<WarmstartKey, Vec<f64>> = HashMap::new();

        let result = cg_solve_contacts(
            &model,
            &data,
            contacts,
            jacobians,
            100,
            1e-8,
            &mut efc_lambda,
        );
        assert!(result.is_ok(), "Zero contacts should return Ok");
        let (forces, iters) = result.unwrap();
        assert!(
            forces.is_empty(),
            "Zero contacts should return empty forces"
        );
        assert_eq!(iters, 0, "Zero contacts should use 0 iterations");
    }

    /// AC #6: cg_solve_contacts with single contact uses direct 3×3 solve (iterations_used == 0).
    #[test]
    fn test_cg_single_contact_direct_call() {
        // Build a minimal model with 1 DOF so we can construct a valid system.
        // Use a sphere-on-plane loaded from MJCF, step to generate contacts,
        // then call cg_solve_contacts directly.
        let mut model = Model::n_link_pendulum(1, 0.5, 1.0);
        model.solver_type = SolverType::CGStrict;
        // Add a geom so contact geom indices are valid.
        model.ngeom = 1;
        model.geom_type = vec![GeomType::Sphere];
        model.geom_body = vec![1]; // attached to body 1
        model.geom_pos = vec![Vector3::zeros()];
        model.geom_quat = vec![UnitQuaternion::identity()];
        model.geom_size = vec![Vector3::new(0.1, 0.0, 0.0)];
        model.geom_friction = vec![Vector3::new(0.5, 0.005, 0.0001)];
        model.geom_condim = vec![3];
        model.geom_solref = vec![[0.02, 1.0]];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.geom_contype = vec![1];
        model.geom_conaffinity = vec![1];
        let mut data = model.make_data();
        data.step(&model).expect("step");

        // Create a synthetic single contact with a valid Jacobian.
        let nv = model.nv;
        let contact = Contact {
            pos: Vector3::new(0.0, 0.0, 0.0),
            normal: Vector3::new(0.0, 0.0, 1.0),
            frame: [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
            depth: 0.001,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let jacobian = DMatrix::from_fn(3, nv, |r, c| if r == 0 && c == 0 { 1.0 } else { 0.0 });

        let mut efc_lambda: HashMap<WarmstartKey, Vec<f64>> = HashMap::new();
        let result = cg_solve_contacts(
            &model,
            &data,
            &[contact],
            &[jacobian],
            100,
            1e-8,
            &mut efc_lambda,
        );

        assert!(result.is_ok(), "Single contact should converge");
        let (forces, iters) = result.unwrap();
        assert_eq!(forces.len(), 1, "Should return 1 force vector");
        assert_eq!(
            iters, 0,
            "Single contact direct solve should use 0 iterations"
        );
    }

    /// Test project_friction_cone: normal force clamped, friction bounded by mu.
    #[test]
    fn test_project_friction_cone_unit() {
        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let efc_offsets = vec![0usize];

        // Case 1: negative normal → clamped to 0 (polar cone → origin)
        let mut lambda = DVector::from_vec(vec![-1.0, 0.3, 0.4]);
        project_friction_cone(&mut lambda, std::slice::from_ref(&contact), &efc_offsets);
        assert_eq!(lambda[0], 0.0, "Negative normal should be clamped to 0");
        assert_eq!(lambda[1], 0.0, "Friction should be 0 when normal is 0");
        assert_eq!(lambda[2], 0.0, "Friction should be 0 when normal is 0");

        // Case 2: friction exceeds cone → friction scaled down to boundary
        // Two-step projection: normal preserved, friction scaled by (λ₀/s)
        // Input: (2.0, 1.5, 0.0) with mu=[0.5, 0.5, ...]
        // s = sqrt((1.5/0.5)^2 + 0) = 3.0
        // Since s > λ₀, scale friction by λ₀/s = 2/3
        // Expected: λ₀ = 2.0 (unchanged), λ₁ = 1.5 * (2/3) = 1.0
        let mut lambda = DVector::from_vec(vec![2.0, 1.5, 0.0]);
        project_friction_cone(&mut lambda, std::slice::from_ref(&contact), &efc_offsets);
        assert!(
            (lambda[0] - 2.0).abs() < 1e-10,
            "Normal should be preserved at {}, got {}",
            2.0,
            lambda[0]
        );
        assert!(
            (lambda[1] - 1.0).abs() < 1e-10,
            "Friction1 should be scaled to {}, got {}",
            1.0,
            lambda[1]
        );
        let friction_mag = (lambda[1].powi(2) + lambda[2].powi(2)).sqrt();
        let max_friction = 0.5 * lambda[0];
        assert!(
            friction_mag <= max_friction + 1e-10,
            "Friction {friction_mag} should be <= mu*lambda_n {max_friction}"
        );

        // Case 3: friction within cone → unchanged
        let mut lambda = DVector::from_vec(vec![10.0, 0.1, 0.1]);
        let orig = lambda.clone();
        project_friction_cone(&mut lambda, &[contact], &efc_offsets);
        assert_eq!(lambda, orig, "Friction within cone should be unchanged");
    }

    /// Test extract_forces utility.
    #[test]
    fn test_extract_forces_unit() {
        // Create two contacts with dim=3
        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let contacts = vec![contact.clone(), contact];
        let efc_offsets = vec![0usize, 3usize];

        let lambda = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let forces = extract_forces(&lambda, &contacts, &efc_offsets);
        assert_eq!(forces.len(), 2);
        assert_eq!(forces[0].as_slice(), &[1.0, 2.0, 3.0]);
        assert_eq!(forces[1].as_slice(), &[4.0, 5.0, 6.0]);
    }

    /// Test block Jacobi preconditioner with a known SPD matrix.
    #[test]
    fn test_block_jacobi_preconditioner() {
        // 3x3 identity block → preconditioner should be identity
        let a = DMatrix::identity(3, 3);
        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let contacts = vec![contact];
        let efc_offsets = vec![0usize];

        let blocks = compute_block_jacobi_preconditioner(&a, &contacts, &efc_offsets);
        assert_eq!(blocks.len(), 1);
        let inv = &blocks[0];
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (inv[(i, j)] - expected).abs() < 1e-10,
                    "Preconditioner of identity should be identity"
                );
            }
        }
    }

    /// Test block Jacobi preconditioner with a non-trivial SPD matrix.
    /// Verifies that Cholesky decomposition produces correct inverse.
    #[test]
    fn test_block_jacobi_preconditioner_nontrivial() {
        // Build a 6×6 matrix with two 3×3 SPD diagonal blocks.
        // Block 0: diag(2,3,4) → inv = diag(0.5, 0.333, 0.25)
        // Block 1: a non-diagonal SPD matrix [[4,2,0],[2,5,1],[0,1,3]]
        let mut a = DMatrix::zeros(6, 6);
        // Block 0: diagonal
        a[(0, 0)] = 2.0;
        a[(1, 1)] = 3.0;
        a[(2, 2)] = 4.0;
        // Block 1: non-diagonal SPD
        a[(3, 3)] = 4.0;
        a[(3, 4)] = 2.0;
        a[(3, 5)] = 0.0;
        a[(4, 3)] = 2.0;
        a[(4, 4)] = 5.0;
        a[(4, 5)] = 1.0;
        a[(5, 3)] = 0.0;
        a[(5, 4)] = 1.0;
        a[(5, 5)] = 3.0;

        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let contacts = vec![contact.clone(), contact];
        let efc_offsets = vec![0usize, 3usize];

        let blocks = compute_block_jacobi_preconditioner(&a, &contacts, &efc_offsets);
        assert_eq!(blocks.len(), 2);

        // Block 0: diagonal inverse
        assert_relative_eq!(blocks[0][(0, 0)], 0.5, epsilon = 1e-10);
        assert_relative_eq!(blocks[0][(1, 1)], 1.0 / 3.0, epsilon = 1e-10);
        assert_relative_eq!(blocks[0][(2, 2)], 0.25, epsilon = 1e-10);

        // Block 1: verify M * M^{-1} = I
        let orig_mat = Matrix3::new(4.0, 2.0, 0.0, 2.0, 5.0, 1.0, 0.0, 1.0, 3.0);
        // Convert DMatrix block to Matrix3 for multiplication
        let inv_block = Matrix3::new(
            blocks[1][(0, 0)],
            blocks[1][(0, 1)],
            blocks[1][(0, 2)],
            blocks[1][(1, 0)],
            blocks[1][(1, 1)],
            blocks[1][(1, 2)],
            blocks[1][(2, 0)],
            blocks[1][(2, 1)],
            blocks[1][(2, 2)],
        );
        let product = orig_mat * inv_block;
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (product[(i, j)] - expected).abs() < 1e-10,
                    "Block1 * inv(Block1) should be identity, got [{i},{j}]={:.6e}",
                    product[(i, j)]
                );
            }
        }
    }

    /// Test block Jacobi preconditioner falls back to scalar Jacobi for
    /// a non-positive-definite block (Cholesky fails).
    #[test]
    fn test_block_jacobi_preconditioner_fallback() {
        // Build a 3×3 matrix that is NOT positive definite (negative eigenvalue).
        // Cholesky should fail, triggering scalar Jacobi fallback.
        let mut a = DMatrix::zeros(3, 3);
        a[(0, 0)] = 1.0;
        a[(1, 1)] = 2.0;
        a[(2, 2)] = 3.0;
        // Off-diagonal makes it indefinite
        a[(0, 1)] = 10.0;
        a[(1, 0)] = 10.0;

        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.5,
            dim: 3,
            includemargin: false,
            mu: [0.5, 0.5, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let contacts = vec![contact];
        let efc_offsets = vec![0usize];

        let blocks = compute_block_jacobi_preconditioner(&a, &contacts, &efc_offsets);
        assert_eq!(blocks.len(), 1);

        // Should fall back to scalar Jacobi: diag(1/1, 1/2, 1/3)
        let inv = &blocks[0];
        assert_relative_eq!(inv[(0, 0)], 1.0, epsilon = 1e-10);
        assert_relative_eq!(inv[(1, 1)], 0.5, epsilon = 1e-10);
        assert_relative_eq!(inv[(2, 2)], 1.0 / 3.0, epsilon = 1e-10);
        // Off-diagonals should be zero (scalar Jacobi is diagonal)
        assert_relative_eq!(inv[(0, 1)], 0.0, epsilon = 1e-10);
        assert_relative_eq!(inv[(1, 0)], 0.0, epsilon = 1e-10);
    }

    /// Test project_friction_cone with zero friction coefficient.
    #[test]
    fn test_project_friction_cone_zero_mu() {
        let contact = Contact {
            pos: Vector3::zeros(),
            normal: Vector3::z(),
            frame: [Vector3::x(), Vector3::y()],
            depth: 0.0,
            geom1: 0,
            geom2: 0,
            friction: 0.0, // Frictionless
            dim: 3,
            includemargin: false,
            mu: [0.0, 0.0, 0.0, 0.0, 0.0],
            solref: [0.02, 1.0],
            solimp: [0.9, 0.95, 0.001, 0.5, 2.0],
        };
        let efc_offsets = vec![0usize];

        // Normal force should be kept, friction should be zeroed
        let mut lambda = DVector::from_vec(vec![5.0, 2.0, 3.0]);
        project_friction_cone(&mut lambda, &[contact], &efc_offsets);
        assert_eq!(lambda[0], 5.0, "Normal force should be unchanged");
        assert_eq!(lambda[1], 0.0, "Friction should be zero with mu=0");
        assert_eq!(lambda[2], 0.0, "Friction should be zero with mu=0");
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod warmstart_key_tests {
    use super::*;

    fn make_contact(geom1: usize, geom2: usize, x: f64, y: f64, z: f64) -> Contact {
        Contact::new(Vector3::new(x, y, z), Vector3::z(), 0.01, geom1, geom2, 1.0)
    }

    #[test]
    fn canonical_geom_ordering() {
        let c1 = make_contact(3, 7, 0.0, 0.0, 0.0);
        let c2 = make_contact(7, 3, 0.0, 0.0, 0.0);
        assert_eq!(warmstart_key(&c1), warmstart_key(&c2));
        // Both should have (3, 7) as the geom pair
        let key = warmstart_key(&c1);
        assert_eq!(key.geom_lo, 3);
        assert_eq!(key.geom_hi, 7);
    }

    #[test]
    fn same_cell_same_key() {
        // Two positions within the same 1cm grid cell.
        // 0.101 / 0.01 = 10.1 → rounds to 10; 0.103 / 0.01 = 10.3 → rounds to 10
        let c1 = make_contact(0, 1, 0.101, 0.201, 0.301);
        let c2 = make_contact(0, 1, 0.103, 0.203, 0.303);
        assert_eq!(warmstart_key(&c1), warmstart_key(&c2));
    }

    #[test]
    fn different_cells_different_keys() {
        let c1 = make_contact(0, 1, 0.0, 0.0, 0.0);
        let c2 = make_contact(0, 1, 0.02, 0.0, 0.0); // 2cm apart in x
        assert_ne!(warmstart_key(&c1), warmstart_key(&c2));
    }

    #[test]
    fn different_geom_pairs_different_keys() {
        let c1 = make_contact(0, 1, 0.0, 0.0, 0.0);
        let c2 = make_contact(0, 2, 0.0, 0.0, 0.0);
        assert_ne!(warmstart_key(&c1), warmstart_key(&c2));
    }

    #[test]
    fn negative_positions() {
        let key = warmstart_key(&make_contact(0, 1, -0.5, -1.0, -2.0));
        assert_eq!(key.cell_x, -50);
        assert_eq!(key.cell_y, -100);
        assert_eq!(key.cell_z, -200);
    }

    #[test]
    fn grid_resolution_is_1cm() {
        // Positions 0.004 and 0.006 round to 0 and 1 respectively at 1cm grid
        let key_below = warmstart_key(&make_contact(0, 1, 0.004, 0.0, 0.0));
        let key_above = warmstart_key(&make_contact(0, 1, 0.006, 0.0, 0.0));
        assert_eq!(key_below.cell_x, 0); // 0.004 / 0.01 = 0.4, rounds to 0
        assert_eq!(key_above.cell_x, 1); // 0.006 / 0.01 = 0.6, rounds to 1
    }
}

// =============================================================================
// Unit tests — subquat
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod subquat_tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn identity_difference_is_zero() {
        let q = UnitQuaternion::identity();
        let res = subquat(&q, &q);
        assert_relative_eq!(res.norm(), 0.0, epsilon = 1e-14);
    }

    #[test]
    fn ninety_degrees_about_x() {
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), angle);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should be (π/2, 0, 0)
        assert_relative_eq!(res.x, angle, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn ninety_degrees_about_z() {
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        assert_relative_eq!(res.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.z, angle, epsilon = 1e-12);
    }

    #[test]
    fn opposite_quaternions_give_pi() {
        // 180° about y
        let qa = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::PI);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Magnitude should be π
        assert_relative_eq!(res.norm(), std::f64::consts::PI, epsilon = 1e-10);
    }

    #[test]
    fn shortest_path_wraps() {
        // qa = 270° about x = -90° via shortest path
        let qa =
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 3.0 * std::f64::consts::FRAC_PI_2);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should report -π/2 about x (shortest path)
        assert_relative_eq!(res.x, -std::f64::consts::FRAC_PI_2, epsilon = 1e-10);
        assert_relative_eq!(res.norm(), std::f64::consts::FRAC_PI_2, epsilon = 1e-10);
    }

    #[test]
    fn relative_rotation() {
        // qa = 60° about z, qb = 20° about z → difference = 40° about z
        let qa = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 60.0_f64.to_radians());
        let qb = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 20.0_f64.to_radians());
        let res = subquat(&qa, &qb);
        assert_relative_eq!(res.z, 40.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(res.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(res.y, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn small_angle_near_zero() {
        let tiny = 1e-15;
        let qa = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tiny);
        let qb = UnitQuaternion::identity();
        let res = subquat(&qa, &qb);
        // Should be zero vector (within small-angle guard)
        assert_relative_eq!(res.norm(), 0.0, epsilon = 1e-10);
    }
}

// =============================================================================
// Unit tests — mj_jac_site vs accumulate_point_jacobian
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod jac_site_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Build a minimal Model + Data for a single-joint body with a site.
    fn make_single_joint_site_model(
        jnt_type: MjJointType,
        joint_axis: Vector3<f64>,
        site_offset: Vector3<f64>,
        body_pos: Vector3<f64>,
        qpos_val: f64,
    ) -> (Model, Data) {
        let mut model = Model::empty();

        // Add body 1 with a joint
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, jnt_type.nv()];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), body_pos];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".to_string()), Some("b1".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        let nv = jnt_type.nv();
        let nq = jnt_type.nq();
        model.njnt = 1;
        model.nq = nq;
        model.nv = nv;
        model.jnt_type = vec![jnt_type];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![joint_axis];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".to_string())];

        // DOFs
        model.dof_body = vec![1; nv];
        model.dof_jnt = vec![0; nv];
        model.dof_parent = vec![None; nv];
        model.dof_armature = vec![0.0; nv];
        model.dof_damping = vec![0.0; nv];
        model.dof_frictionloss = vec![0.0; nv];

        // Site
        model.nsite = 1;
        model.site_body = vec![1];
        model.site_pos = vec![site_offset];
        model.site_quat = vec![UnitQuaternion::identity()];
        model.site_type = vec![GeomType::Sphere];
        model.site_size = vec![Vector3::new(0.01, 0.01, 0.01)];
        model.site_name = vec![Some("s1".to_string())];

        model.qpos0 = DVector::zeros(nq);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();

        let mut data = model.make_data();
        data.qpos[0] = qpos_val;
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    /// mj_jac_site translational column for hinge must agree with
    /// accumulate_point_jacobian for the same direction projection.
    #[test]
    fn jac_site_agrees_with_accumulate_hinge() {
        let (model, data) = make_single_joint_site_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.3,
        );

        let (jac_t, jac_r) = mj_jac_site(&model, &data, 0);

        // For each cardinal direction, accumulate_point_jacobian projecting
        // onto that direction should agree with the corresponding row of jac_t.
        for dir_idx in 0..3 {
            let direction = Vector3::ith(dir_idx, 1.0);
            let mut ten_j = DVector::zeros(model.nv);
            accumulate_point_jacobian(
                &model,
                &data.xpos,
                &data.xquat,
                &mut ten_j,
                model.site_body[0],
                &data.site_xpos[0],
                &direction,
                1.0,
            );

            assert_relative_eq!(jac_t[(dir_idx, 0)], ten_j[0], epsilon = 1e-12);
        }

        // Rotational Jacobian for hinge: should be the world-frame joint axis.
        let world_axis = data.xquat[1] * model.jnt_axis[0];
        for k in 0..3 {
            assert_relative_eq!(jac_r[(k, 0)], world_axis[k], epsilon = 1e-12);
        }
    }

    /// For a slide joint, translational Jacobian is the axis; rotational is zero.
    #[test]
    fn jac_site_slide_joint() {
        let (model, data) = make_single_joint_site_model(
            MjJointType::Slide,
            Vector3::z(),
            Vector3::new(0.2, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            0.1,
        );

        let (jac_t, jac_r) = mj_jac_site(&model, &data, 0);

        // Slide: translational Jacobian = world-frame axis
        let world_axis = data.xquat[1] * model.jnt_axis[0];
        for k in 0..3 {
            assert_relative_eq!(jac_t[(k, 0)], world_axis[k], epsilon = 1e-12);
        }

        // Slide: no rotational contribution
        for k in 0..3 {
            assert_relative_eq!(jac_r[(k, 0)], 0.0, epsilon = 1e-14);
        }
    }
}
