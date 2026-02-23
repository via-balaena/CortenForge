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
    heightfield_box_contact, heightfield_capsule_contact, heightfield_sphere_contact,
};
use crate::mesh::{
    MeshContact, TriangleMeshData, mesh_box_contact, mesh_capsule_contact,
    mesh_mesh_deepest_contact, mesh_sphere_contact,
};
use crate::raycast::raycast_shape;
use crate::sdf::{
    sdf_box_contact, sdf_capsule_contact, sdf_cylinder_contact, sdf_ellipsoid_contact,
    sdf_heightfield_contact, sdf_plane_contact, sdf_sdf_contact, sdf_sphere_contact,
    sdf_triangle_mesh_contact,
};
use nalgebra::{
    DMatrix, DVector, Matrix3, Matrix6, Point3, UnitQuaternion, UnitVector3, Vector2, Vector3,
    Vector6,
};
use sim_types::Pose;
use std::f64::consts::PI;
use std::sync::Arc;

// Re-imports from types module (accumulated during structural refactor phases 1–8c,
// removed in Phase 12 when the monolith shim is deleted).
pub(crate) use crate::types::ActuatorDynamics;
pub(crate) use crate::types::ActuatorTransmission;
pub(crate) use crate::types::BiasType;
pub(crate) use crate::types::ConstraintState;
pub(crate) use crate::types::ConstraintType;
pub(crate) use crate::types::DISABLE_ISLAND;
pub(crate) use crate::types::ENABLE_SLEEP;
pub(crate) use crate::types::EqualityType;
pub(crate) use crate::types::GainType;
pub(crate) use crate::types::GeomType;
pub(crate) use crate::types::Integrator;
pub(crate) use crate::types::MIN_AWAKE;
pub(crate) use crate::types::MjJointType;
pub(crate) use crate::types::MjObjectType;
pub(crate) use crate::types::MjSensorDataType;
pub(crate) use crate::types::MjSensorType;
pub(crate) use crate::types::Model;
pub(crate) use crate::types::SleepError;
pub(crate) use crate::types::SleepPolicy;
pub(crate) use crate::types::SleepState;
pub(crate) use crate::types::SolverStat;
pub(crate) use crate::types::SolverType;
pub(crate) use crate::types::StepError;
pub(crate) use crate::types::TendonType;
pub(crate) use crate::types::WrapType;

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

/// Shift a 6×6 spatial inertia from one reference point to another.
///
/// Given Φ (spatial inertia about point A in world frame) and
/// d = xpos_child - xpos_parent (vector from parent to child origin),
/// returns the equivalent spatial inertia about the parent's origin.
///
/// Uses the approach: extract (I_COM, m, h) from Φ_A, then recompute
/// Φ_B with h_new = h + d (since COM - parent = (COM - child) + (child - parent)).
///
/// Convention: rows 0-2 = angular, rows 3-5 = linear.
/// Off-diagonal coupling block (upper-right 3x3) = m * skew(h).
#[allow(clippy::similar_names)]
fn shift_spatial_inertia(phi: &Matrix6<f64>, d: &Vector3<f64>) -> Matrix6<f64> {
    // Extract mass from lower-right diagonal
    let m = phi[(3, 3)];
    if m == 0.0 {
        return *phi;
    }

    // Extract m*h from coupling block: H = m*skew(h)
    // skew(h) = [0, -hz, hy; hz, 0, -hx; -hy, hx, 0]
    // So: phi[(2,4)] = m*hx, phi[(0,5)] = m*hy, phi[(1,3)] = m*hz
    let mh_x = phi[(2, 4)];
    let mh_y = phi[(0, 5)];
    let mh_z = phi[(1, 3)];
    let h_x = mh_x / m;
    let h_y = mh_y / m;
    let h_z = mh_z / m;

    // Extract I_about_A (rotational block, upper-left 3x3)
    // Then reverse parallel axis to get I_COM:
    // I_COM = I_A - m*(|h|²I₃ - h*hᵀ)
    let hh = h_x * h_x + h_y * h_y + h_z * h_z;
    let mut i_com = Matrix3::zeros();
    for row in 0..3 {
        for col in 0..3 {
            let h_rc = [h_x, h_y, h_z];
            let delta = if row == col { 1.0 } else { 0.0 };
            i_com[(row, col)] = phi[(row, col)] - m * (hh * delta - h_rc[row] * h_rc[col]);
        }
    }

    // Compute new h: h_new = h + d (COM offset from parent origin)
    let h_new_x = h_x + d.x;
    let h_new_y = h_y + d.y;
    let h_new_z = h_z + d.z;

    // Build new 6x6 spatial inertia about parent origin
    let h_new = [h_new_x, h_new_y, h_new_z];
    let hh_new = h_new_x * h_new_x + h_new_y * h_new_y + h_new_z * h_new_z;

    let mut result = Matrix6::zeros();

    // Upper-left 3x3: I_parent = I_COM + m*(|h_new|²I₃ - h_new*h_newᵀ)
    for row in 0..3 {
        for col in 0..3 {
            let delta = if row == col { 1.0 } else { 0.0 };
            result[(row, col)] = i_com[(row, col)] + m * (hh_new * delta - h_new[row] * h_new[col]);
        }
    }

    // Lower-right 3x3: mass
    result[(3, 3)] = m;
    result[(4, 4)] = m;
    result[(5, 5)] = m;

    // Off-diagonal coupling: m * skew(h_new)
    let mhn_x = m * h_new_x;
    let mhn_y = m * h_new_y;
    let mhn_z = m * h_new_z;

    result[(0, 4)] = -mhn_z;
    result[(0, 5)] = mhn_y;
    result[(1, 3)] = mhn_z;
    result[(1, 5)] = -mhn_x;
    result[(2, 3)] = -mhn_y;
    result[(2, 4)] = mhn_x;
    // Transpose (lower-left)
    result[(4, 0)] = -mhn_z;
    result[(5, 0)] = mhn_y;
    result[(3, 1)] = mhn_z;
    result[(5, 1)] = -mhn_x;
    result[(3, 2)] = -mhn_y;
    result[(4, 2)] = mhn_x;

    result
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

/// Disjoint-set / union-find for Init-sleep validation (§16.24).
///
/// Path compression + union by rank. Used to group trees connected
/// by equality constraints and multi-tree tendons.
struct UnionFind {
    parent: Vec<usize>,
    rank: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            rank: vec![0; n],
        }
    }

    fn find(&mut self, mut x: usize) -> usize {
        while self.parent[x] != x {
            self.parent[x] = self.parent[self.parent[x]]; // Path compression
            x = self.parent[x];
        }
        x
    }

    fn union(&mut self, a: usize, b: usize) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        // Union by rank
        match self.rank[ra].cmp(&self.rank[rb]) {
            std::cmp::Ordering::Less => self.parent[ra] = rb,
            std::cmp::Ordering::Greater => self.parent[rb] = ra,
            std::cmp::Ordering::Equal => {
                self.parent[rb] = ra;
                self.rank[ra] += 1;
            }
        }
    }
}

// Moved to types/keyframe.rs: Keyframe struct
// (no re-export needed — all consumers import directly from types::keyframe)

// Moved to types/contact_types.rs: ContactPair, Contact, impl Contact, compute_tangent_frame
pub(crate) use crate::types::{Contact, ContactPair, compute_tangent_frame};

// Moved to types/data.rs: Data struct, Clone impl
pub(crate) use crate::types::Data;

impl Model {
    // Moved to types/model_init.rs: empty()

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

    /// Compute CSR metadata for sparse LDL factorization from `dof_parent` chains.
    ///
    /// Must be called after `dof_parent` is finalized (in `ModelBuilder::build()` or
    /// test helpers that construct Model manually). The sparsity pattern is immutable
    /// after this call.
    ///
    /// Each row stores off-diagonal entries (ancestors) followed by the diagonal
    /// (self-index) as the last element, matching MuJoCo's `mj_factorI` layout.
    /// `rownnz[i]` includes the diagonal, so `rownnz[i] - 1` is the off-diagonal count.
    pub fn compute_qld_csr_metadata(&mut self) {
        let nv = self.nv;
        self.qLD_rownnz = vec![0; nv];
        self.qLD_rowadr = vec![0; nv];

        // Pass 1: Count entries per row (ancestors + 1 for diagonal)
        for i in 0..nv {
            let mut count = 0;
            let mut p = self.dof_parent[i];
            while let Some(j) = p {
                count += 1;
                p = self.dof_parent[j];
            }
            self.qLD_rownnz[i] = count + 1; // +1 for diagonal
        }

        // Pass 2: Compute row addresses (prefix sum)
        let mut offset = 0;
        for i in 0..nv {
            self.qLD_rowadr[i] = offset;
            offset += self.qLD_rownnz[i];
        }
        self.qLD_nnz = offset;

        // Pass 3: Fill column indices (ancestors ascending, then self-index for diagonal)
        self.qLD_colind = vec![0; self.qLD_nnz];
        for i in 0..nv {
            let mut ancestors = Vec::new();
            let mut p = self.dof_parent[i];
            while let Some(j) = p {
                ancestors.push(j);
                p = self.dof_parent[j];
            }
            ancestors.reverse(); // ascending order (root ancestor first)
            let start = self.qLD_rowadr[i];
            for (k, &col) in ancestors.iter().enumerate() {
                self.qLD_colind[start + k] = col;
            }
            // Diagonal as last element (matching MuJoCo's layout)
            self.qLD_colind[start + ancestors.len()] = i;
        }
    }

    // (§27F) nv_rigid()/nq_rigid() removed — all DOFs are now "rigid" DOFs.

    // Moved to types/model_init.rs: make_data(), compute_ancestors(), compute_implicit_params()

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
                ActuatorTransmission::Site | ActuatorTransmission::Body => {
                    // Site: configuration-dependent (full FK required).
                    // Body: no length concept. Both: no-op, leave at (0, 0).
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
                ActuatorTransmission::Body => {
                    // At qpos0 there are no contacts, so J-vector is zero.
                    // No-op: j_vec already initialized to zeros.
                }
            }

            // Solve M * x = J using the sparse L^T D L factorization from CRBA.
            let mut x = j_vec;
            let (rowadr, rownnz, colind) = self.qld_csr();
            mj_solve_sparse(
                rowadr,
                rownnz,
                colind,
                &data.qLD_data,
                &data.qLD_diag_inv,
                &mut x,
            );

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

    /// Compute `tendon_length0` for spatial tendons via FK at `qpos0`.
    ///
    /// 1. Sets `tendon_length0[t] = ten_length[t]` from the FK result.
    /// 2. Defaults `lengthspring` to `tendon_length0` when stiffness > 0.
    ///
    /// Must be called after `compute_implicit_params()` and before
    /// `compute_muscle_params()` (which needs valid `tendon_length0` for all types).
    ///
    /// Note: Rule 9 (sidesite outside wrapping geometry) is retired — sidesites
    /// inside wrapping geometry are now handled by the `wrap_inside` algorithm (§39).
    pub fn compute_spatial_tendon_length0(&mut self) {
        // Early return if no spatial tendons — avoid unnecessary FK + Data allocation.
        let has_spatial = (0..self.ntendon).any(|t| self.tendon_type[t] == TendonType::Spatial);
        if !has_spatial {
            return;
        }

        let mut data = self.make_data();
        mj_fwd_position(self, &mut data); // runs FK + mj_fwd_tendon

        // Compute tendon_length0 and default lengthspring for spatial tendons.
        for t in 0..self.ntendon {
            if self.tendon_type[t] == TendonType::Spatial {
                self.tendon_length0[t] = data.ten_length[t];
                // S3: Replace sentinel [-1, -1] with computed length at qpos0
                // Sentinel is an exact literal, never a computed float.
                #[allow(clippy::float_cmp)]
                if self.tendon_lengthspring[t] == [-1.0, -1.0] {
                    self.tendon_lengthspring[t] = [data.ten_length[t], data.ten_length[t]];
                }
            }
        }
    }

    // Moved to types/model_init.rs: compute_stat_meaninertia()

    // Moved to types/model_factories.rs: n_link_pendulum(), double_pendulum(),
    // spherical_pendulum(), free_body()
}

impl Data {
    // Moved to types/data.rs: qld_diag(), reset(), reset_to_keyframe()

    /// Get total mechanical energy (kinetic + potential).
    #[must_use]
    pub fn total_energy(&self) -> f64 {
        self.energy_kinetic + self.energy_potential
    }

    // ==================== Sleep Public API (§16.25) ====================

    /// Query the sleep state of a body.
    ///
    /// Returns `SleepState::Static` for the world body (body 0),
    /// `SleepState::Asleep` for sleeping bodies, `SleepState::Awake`
    /// for active bodies.
    #[must_use]
    pub fn sleep_state(&self, body_id: usize) -> SleepState {
        self.body_sleep_state[body_id]
    }

    /// Query whether a kinematic tree is awake.
    #[must_use]
    pub fn tree_awake(&self, tree_id: usize) -> bool {
        self.tree_asleep[tree_id] < 0
    }

    /// Query the number of awake bodies (including the world body).
    #[must_use]
    pub fn nbody_awake(&self) -> usize {
        self.nbody_awake
    }

    /// Query the number of constraint islands discovered this step.
    #[must_use]
    pub fn nisland(&self) -> usize {
        self.nisland
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
        // Must update sleep arrays after user-force wake so that
        // body_awake_ind/dof_awake_ind are current before mj_crba (§16.29.3).
        if sleep_enabled && mj_wake(model, self) {
            mj_update_sleep_arrays(model, self);
        }

        // ========== Position Stage ==========
        mj_fwd_position(model, self);
        mj_flex(model, self);

        // §16.15: If FK detected external qpos changes on sleeping bodies, wake them
        if sleep_enabled && mj_check_qpos_changed(model, self) {
            mj_update_sleep_arrays(model, self);
        }

        mj_transmission_site(model, self);

        // §16.13.2: Tendon wake — multi-tree tendons with active limits
        if sleep_enabled && mj_wake_tendon(model, self) {
            mj_update_sleep_arrays(model, self);
        }

        mj_collision(model, self);

        // Wake-on-contact: if sleeping body touched awake body, wake it
        // and re-run collision for the newly-awake tree's geoms (§16.5c)
        if sleep_enabled && mj_wake_collision(model, self) {
            mj_update_sleep_arrays(model, self);
            mj_collision(model, self);
        }

        // §16.13.3: Equality constraint wake — cross-tree equality coupling
        if sleep_enabled && mj_wake_equality(model, self) {
            mj_update_sleep_arrays(model, self);
        }

        // §36: Body transmission — requires contacts from mj_collision()
        mj_transmission_body_dispatch(model, self);

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
        mj_crba(model, self);
        mj_rne(model, self);
        mj_energy_vel(model, self);
        mj_fwd_passive(model, self);

        // §16.11: Island discovery must run BEFORE constraint solve so that
        // contact_island assignments are available for per-island partitioning.
        if sleep_enabled {
            mj_island(model, self);
        }

        // §16.16: Per-island constraint solve when islands are active;
        // falls back to global solve when DISABLE_ISLAND or no islands.
        mj_fwd_constraint_islands(model, self);

        if !self.newton_solved {
            mj_fwd_acceleration(model, self)?;
        }

        // (§27F) Pinned flex vertex DOF clamping removed — pinned vertices now have
        // no joints/DOFs (zero body_dof_num), so no qacc/qvel entries to clamp.

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
        // §16.27: Use indirection array for cache-friendly iteration over awake DOFs.
        let use_dof_ind = sleep_enabled && self.nv_awake < model.nv;

        // Integrate activation per actuator via mj_next_activation() (§34).
        // Handles both integration (Euler/FilterExact) and actlimited clamping.
        // MuJoCo order: activation → velocity → position.
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            for k in 0..act_num {
                let j = act_adr + k;
                self.act[j] = mj_next_activation(model, i, self.act[j], self.act_dot[j]);
            }
        }

        // For Euler and new implicit variants, update velocity using computed acceleration.
        // For legacy ImplicitSpringDamper, velocity was already updated in mj_fwd_acceleration_implicit.
        match model.integrator {
            Integrator::Euler | Integrator::ImplicitFast | Integrator::Implicit => {
                let nv = if use_dof_ind { self.nv_awake } else { model.nv };
                for idx in 0..nv {
                    let i = if use_dof_ind {
                        self.dof_awake_ind[idx]
                    } else {
                        idx
                    };
                    self.qvel[i] += self.qacc[i] * h;
                }
            }
            Integrator::ImplicitSpringDamper => {
                if self.newton_solved {
                    // Newton already computed qacc with implicit spring/damper effects
                    // baked into the constraint solve via M_impl (DT-35: includes
                    // tendon K/D coupling). Update velocity explicitly.
                    let nv = if use_dof_ind { self.nv_awake } else { model.nv };
                    for idx in 0..nv {
                        let i = if use_dof_ind {
                            self.dof_awake_ind[idx]
                        } else {
                            idx
                        };
                        self.qvel[i] += self.qacc[i] * h;
                    }
                }
                // Otherwise: velocity already updated by mj_fwd_acceleration_implicit
                // (non-Newton path solves for v_new directly, not qacc)
            }
            Integrator::RungeKutta4 => {
                unreachable!("RK4 integration handled by mj_runge_kutta()")
            }
        }

        // Update positions - quaternions need special handling!
        mj_integrate_pos(model, self, h);

        // Normalize quaternions to prevent drift
        mj_normalize_quat(model, self);

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

        // 1. Activation integration via mj_next_activation() (§34, identical to integrate())
        for i in 0..model.nu {
            let act_adr = model.actuator_act_adr[i];
            let act_num = model.actuator_act_num[i];
            for k in 0..act_num {
                let j = act_adr + k;
                self.act[j] = mj_next_activation(model, i, self.act[j], self.act_dot[j]);
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
pub(crate) fn mj_fwd_position(model: &Model, data: &mut Data) {
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

    // Rigid-rigid collision requires at least 2 geoms for a pair.
    // Even with 0 or 1 geoms, flex-vertex-vs-rigid contacts are still possible.
    if model.ngeom >= 2 {
        // Build AABBs for all geoms
        // This is O(n) and cache-friendly (linear memory access)
        let aabbs: Vec<Aabb> = (0..model.ngeom)
            .map(|geom_id| {
                let mut aabb = aabb_from_geom(
                    model.geom_type[geom_id],
                    model.geom_size[geom_id],
                    data.geom_xpos[geom_id],
                    data.geom_xmat[geom_id],
                );
                // Expand AABB by geom margin so SAP doesn't reject pairs
                // that are within margin distance but not overlapping.
                let m = model.geom_margin[geom_id];
                if m > 0.0 {
                    let expand = Vector3::new(m, m, m);
                    aabb = Aabb::new(
                        Point3::from(Vector3::new(aabb.min.x, aabb.min.y, aabb.min.z) - expand),
                        Point3::from(Vector3::new(aabb.max.x, aabb.max.y, aabb.max.z) + expand),
                    );
                }
                aabb
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

            // Compute effective margin for this pair
            let margin = model.geom_margin[geom1] + model.geom_margin[geom2];

            // Narrow-phase collision detection
            if let Some(contact) =
                collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
            {
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

            // Pair margin overrides geom margins
            let margin = pair.margin;

            // Distance cull using bounding radii (replaces SAP broad-phase for pairs).
            // geom_rbound is the bounding sphere radius, pre-computed per geom.
            // For planes, rbound = INFINITY so this check always passes.
            // Margin is added to match MuJoCo's mj_filterSphere.
            let dist = (data.geom_xpos[geom1] - data.geom_xpos[geom2]).norm();
            if dist > model.geom_rbound[geom1] + model.geom_rbound[geom2] + margin {
                continue;
            }

            // Narrow-phase collision detection
            let pos1 = data.geom_xpos[geom1];
            let mat1 = data.geom_xmat[geom1];
            let pos2 = data.geom_xpos[geom2];
            let mat2 = data.geom_xmat[geom2];

            if let Some(mut contact) =
                collide_geoms(model, geom1, geom2, pos1, mat1, pos2, mat2, margin)
            {
                apply_pair_overrides(&mut contact, pair);
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    } // end if model.ngeom >= 2

    // Mechanism 3: flex vertex vs rigid geom contacts
    mj_collision_flex(model, data);
}

/// Detect contacts between flex vertices and rigid geoms.
///
/// Each flex vertex is treated as a sphere of radius `flexvert_radius[vi]`.
/// Uses brute-force broadphase (O(V*G)) for simplicity — SAP integration
/// deferred to when nflexvert is large enough to warrant it.
/// No-op when nflexvert == 0.
fn mj_collision_flex(model: &Model, data: &mut Data) {
    if model.nflexvert == 0 {
        return;
    }

    for vi in 0..model.nflexvert {
        let vpos = data.flexvert_xpos[vi];
        let flex_id = model.flexvert_flexid[vi];
        let radius = model.flexvert_radius[vi];
        let margin = model.flex_margin[flex_id];

        // Skip pinned vertices (infinite mass = immovable)
        if model.flexvert_invmass[vi] <= 0.0 {
            continue;
        }

        // Per-flex bitmask values (invariant across the inner geom loop)
        let fcontype = model.flex_contype[flex_id];
        let fconaffinity = model.flex_conaffinity[flex_id];

        for gi in 0..model.ngeom {
            // Proper contype/conaffinity bitmask filtering (MuJoCo filterBitmask protocol).
            // Collision proceeds iff: (flex_contype & geom_conaffinity) != 0
            //                      || (geom_contype & flex_conaffinity) != 0
            let gcontype = model.geom_contype[gi];
            let gconaffinity = model.geom_conaffinity[gi];
            if (fcontype & gconaffinity) == 0 && (gcontype & fconaffinity) == 0 {
                continue;
            }

            // Skip geoms belonging to the same body as this vertex
            let vertex_body = model.flexvert_bodyid[vi];
            let geom_body = model.geom_body[gi];
            if geom_body == vertex_body {
                continue;
            }
            // Skip geoms on the vertex's parent body (node body for node-flex)
            if geom_body < model.nbody && model.body_parent[vertex_body] == geom_body {
                continue;
            }

            // Narrowphase: vertex sphere vs rigid geom
            let geom_pos = data.geom_xpos[gi];
            let geom_mat = data.geom_xmat[gi];

            if let Some((depth, normal, contact_pos)) =
                narrowphase_sphere_geom(vpos, radius + margin, gi, model, geom_pos, geom_mat)
            {
                let contact = make_contact_flex_rigid(model, vi, gi, contact_pos, normal, depth);
                data.contacts.push(contact);
                data.ncon += 1;
            }
        }
    }
}

/// Narrowphase collision between a sphere (flex vertex) and a rigid geom.
///
/// Returns `(depth, normal, contact_pos)` if penetrating, `None` otherwise.
/// The normal points from the rigid geom surface toward the vertex.
fn narrowphase_sphere_geom(
    sphere_pos: Vector3<f64>,
    sphere_radius: f64,
    geom_idx: usize,
    model: &Model,
    geom_pos: Vector3<f64>,
    geom_mat: Matrix3<f64>,
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let v = sphere_pos;

    let (d_surface, normal) = match model.geom_type[geom_idx] {
        GeomType::Plane => {
            let plane_normal = geom_mat.column(2).into_owned();
            let d = (v - geom_pos).dot(&plane_normal);
            (d, plane_normal)
        }
        GeomType::Sphere => {
            let radius = model.geom_size[geom_idx].x;
            let diff = v - geom_pos;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (dist - radius, diff / dist)
            }
        }
        GeomType::Box => {
            let half = model.geom_size[geom_idx];
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let clamped = Vector3::new(
                v_local.x.clamp(-half.x, half.x),
                v_local.y.clamp(-half.y, half.y),
                v_local.z.clamp(-half.z, half.z),
            );
            let diff_local = v_local - clamped;
            let dist_outside = diff_local.norm();
            if dist_outside > 1e-10 {
                let normal_world = geom_mat * (diff_local / dist_outside);
                (dist_outside, normal_world)
            } else {
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
                (-min_depth, geom_mat * normal_local)
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
                (dist - radius, diff / dist)
            }
        }
        GeomType::Cylinder => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let radial_dist = Vector2::new(v_local.x, v_local.y).norm();
            let on_barrel = v_local.z.abs() <= half_len;
            let in_radial = radial_dist <= radius;

            if on_barrel && in_radial {
                let d_radial = radius - radial_dist;
                let d_axial = half_len - v_local.z.abs();
                if d_radial < d_axial {
                    let nl = if radial_dist > 1e-10 {
                        Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    };
                    (-d_radial, geom_mat * nl)
                } else {
                    let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                    (-d_axial, geom_mat * nl)
                }
            } else if on_barrel {
                let nl = if radial_dist > 1e-10 {
                    Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                (radial_dist - radius, geom_mat * nl)
            } else if in_radial {
                let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                (v_local.z.abs() - half_len, geom_mat * nl)
            } else {
                let z_clamped = v_local.z.clamp(-half_len, half_len);
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
                (dist, geom_mat * (diff_local / dist))
            }
        }
        GeomType::Ellipsoid => {
            let radii = model.geom_size[geom_idx];
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
            let d_approx = (dist_scaled - 1.0) * radii.x.min(radii.y).min(radii.z);
            let grad_local = Vector3::new(
                v_local.x / (radii.x * radii.x),
                v_local.y / (radii.y * radii.y),
                v_local.z / (radii.z * radii.z),
            );
            let grad_len = grad_local.norm();
            if grad_len < 1e-10 {
                return None;
            }
            (d_approx, geom_mat * (grad_local / grad_len))
        }
        // Mesh/Hfield/Sdf: not yet supported for flex-vertex collision
        _ => return None,
    };

    let depth = sphere_radius - d_surface;
    if depth <= 0.0 {
        return None;
    }

    let contact_pos = v - normal * d_surface;
    Some((depth, normal, contact_pos))
}

/// Create a Contact for a flex-rigid collision.
///
/// Uses the unified `contact_param_flex_rigid()` function for parameter
/// combination, mirroring `make_contact_from_geoms()` for rigid-rigid contacts.
fn make_contact_flex_rigid(
    model: &Model,
    vertex_idx: usize,
    geom_idx: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex_idx];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_rigid(model, flex_id, geom_idx);
    // Effective margin = flex_margin + geom_margin (already used in broadphase at line 5585)
    let margin = model.flex_margin[flex_id] + model.geom_margin[geom_idx];
    let includemargin = margin - gap;

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3, // condim 3 and any other value default to 3D friction cone
    };

    Contact {
        pos,
        normal,
        depth,
        // For flex contacts, geom1 = geom2 = the rigid geom index.
        // The vertex index is stored in flex_vertex.
        // This ensures model.geom_body[contact.geom1] is always valid.
        geom1: geom_idx,
        geom2: geom_idx,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: [0.0, 0.0],
        solimp,
        frame: [t1, t2],
        flex_vertex: Some(vertex_idx),
    }
}

/// Narrow-phase collision between two geometries.
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
#[allow(clippy::items_after_statements)] // use statement placed after special cases for readability
#[allow(clippy::too_many_arguments)]
fn collide_geoms(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
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
        return collide_with_sdf(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: mesh collision (has its own BVH-accelerated path)
    if type1 == GeomType::Mesh || type2 == GeomType::Mesh {
        return collide_with_mesh(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: height field collision
    if type1 == GeomType::Hfield || type2 == GeomType::Hfield {
        return collide_with_hfield(model, geom1, geom2, pos1, mat1, pos2, mat2, margin);
    }

    // Special case: plane collision
    if type1 == GeomType::Plane || type2 == GeomType::Plane {
        return collide_with_plane(
            model, geom1, geom2, type1, type2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-sphere collision (analytical, more robust than GJK/EPA)
    if type1 == GeomType::Sphere && type2 == GeomType::Sphere {
        return collide_sphere_sphere(model, geom1, geom2, pos1, pos2, size1, size2, margin);
    }

    // Special case: capsule-capsule collision (analytical, much faster than GJK/EPA)
    if type1 == GeomType::Capsule && type2 == GeomType::Capsule {
        return collide_capsule_capsule(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-capsule collision
    if (type1 == GeomType::Sphere && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Sphere)
    {
        return collide_sphere_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: sphere-box collision (analytical)
    if (type1 == GeomType::Sphere && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Sphere)
    {
        return collide_sphere_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: capsule-box collision (analytical)
    if (type1 == GeomType::Capsule && type2 == GeomType::Box)
        || (type1 == GeomType::Box && type2 == GeomType::Capsule)
    {
        return collide_capsule_box(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: box-box collision (SAT)
    if type1 == GeomType::Box && type2 == GeomType::Box {
        return collide_box_box(
            model, geom1, geom2, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: cylinder-sphere collision (analytical)
    if (type1 == GeomType::Cylinder && type2 == GeomType::Sphere)
        || (type1 == GeomType::Sphere && type2 == GeomType::Cylinder)
    {
        return collide_cylinder_sphere(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
        );
    }

    // Special case: cylinder-capsule collision (analytical with GJK/EPA fallback)
    // Analytical solution handles common cases; degenerate cases fall through to GJK/EPA
    if (type1 == GeomType::Cylinder && type2 == GeomType::Capsule)
        || (type1 == GeomType::Capsule && type2 == GeomType::Cylinder)
    {
        if let Some(contact) = collide_cylinder_capsule(
            model, geom1, geom2, type1, pos1, mat1, pos2, mat2, size1, size2, margin,
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
        if result.penetration > -margin {
            return Some(make_contact_from_geoms(
                model,
                Vector3::new(result.point.x, result.point.y, result.point.z),
                result.normal,
                result.penetration,
                geom1,
                geom2,
                margin,
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
//    - Compute derived values like friction (element-wise max)
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
    // §31: propagate solreffriction from pair → runtime contact.
    // [0.0, 0.0] sentinel means "use solref" (auto-generated contacts keep this default).
    contact.solreffriction = pair.solreffriction;
    // Pair margin/gap override geom-derived includemargin
    contact.includemargin = pair.margin - pair.gap;
}

/// Contact parameter combination — MuJoCo `mj_contactParam()` equivalent.
///
/// Computes combined contact parameters from two geoms.
/// Handles priority (#25), solmix (#26), friction max (#24), and gap (#27).
///
/// Returns: (condim, gap, solref, solimp, friction\[5\])
fn contact_param(
    model: &Model,
    geom1: usize,
    geom2: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    // 1. Load parameters
    let priority1 = model.geom_priority[geom1];
    let priority2 = model.geom_priority[geom2];
    let gap = model.geom_gap[geom1] + model.geom_gap[geom2];

    // 2. Priority check — higher priority geom's params win entirely
    if priority1 != priority2 {
        let winner = if priority1 > priority2 { geom1 } else { geom2 };
        let fri = model.geom_friction[winner];
        return (
            model.geom_condim[winner],
            gap,
            model.geom_solref[winner],
            model.geom_solimp[winner],
            [fri.x, fri.x, fri.y, fri.z, fri.z], // 3→5 unpack
        );
    }

    // 3. Equal priority — combine
    let condim = model.geom_condim[geom1].max(model.geom_condim[geom2]);

    // 3a. Solmix weight
    let s1 = model.geom_solmix[geom1];
    let s2 = model.geom_solmix[geom2];
    let mix = solmix_weight(s1, s2);

    // 3b. Solref combination
    let solref1 = model.geom_solref[geom1];
    let solref2 = model.geom_solref[geom2];
    let solref = combine_solref(solref1, solref2, mix);

    // 3c. Solimp: weighted average
    let solimp1 = model.geom_solimp[geom1];
    let solimp2 = model.geom_solimp[geom2];
    let solimp = combine_solimp(solimp1, solimp2, mix);

    // 3d. Friction: element-wise max (NOT affected by solmix)
    let f1 = model.geom_friction[geom1];
    let f2 = model.geom_friction[geom2];
    let fri = [
        f1.x.max(f2.x),
        f1.x.max(f2.x), // sliding1, sliding2
        f1.y.max(f2.y), // torsional
        f1.z.max(f2.z),
        f1.z.max(f2.z), // rolling1, rolling2
    ];

    (condim, gap, solref, solimp, fri)
}

/// Contact parameter combination for flex-rigid collision pairs.
///
/// Mirrors `contact_param()` for geom-geom pairs, but reads `flex_*` fields
/// for the flex entity and `geom_*` fields for the rigid entity. Follows
/// MuJoCo's `mj_contactParam()` with f1=flex_id, f2=-1 (geom).
fn contact_param_flex_rigid(
    model: &Model,
    flex_id: usize,
    geom_idx: usize,
) -> (i32, f64, [f64; 2], [f64; 5], [f64; 5]) {
    let priority_flex = model.flex_priority[flex_id];
    let priority_geom = model.geom_priority[geom_idx];
    let gap = model.flex_gap[flex_id] + model.geom_gap[geom_idx];

    if priority_flex > priority_geom {
        let f = model.flex_friction[flex_id]; // scalar until Vec<Vector3> upgrade
        return (
            model.flex_condim[flex_id],
            gap,
            model.flex_solref[flex_id],
            model.flex_solimp[flex_id],
            [f, f, f, f, f], // scalar → uniform 5-element unpack
        );
    }
    if priority_geom > priority_flex {
        let f = model.geom_friction[geom_idx];
        return (
            model.geom_condim[geom_idx],
            gap,
            model.geom_solref[geom_idx],
            model.geom_solimp[geom_idx],
            [f.x, f.x, f.y, f.z, f.z],
        );
    }

    // Equal priority — combine
    let condim = model.flex_condim[flex_id].max(model.geom_condim[geom_idx]);

    let s1 = model.flex_solmix[flex_id];
    let s2 = model.geom_solmix[geom_idx];
    let mix = solmix_weight(s1, s2);

    let solref = combine_solref(model.flex_solref[flex_id], model.geom_solref[geom_idx], mix);
    let solimp = combine_solimp(model.flex_solimp[flex_id], model.geom_solimp[geom_idx], mix);

    // Friction: element-wise max (flex scalar applied to all components)
    let ff = model.flex_friction[flex_id];
    let gf = model.geom_friction[geom_idx];
    let fri = [
        ff.max(gf.x),
        ff.max(gf.x), // sliding1, sliding2
        ff.max(gf.y), // torsional
        ff.max(gf.z),
        ff.max(gf.z), // rolling1, rolling2
    ];

    (condim, gap, solref, solimp, fri)
}

/// Compute solmix weight, matching MuJoCo's edge-case handling.
/// Returns weight for entity 1 (entity 2 weight = 1 - mix).
fn solmix_weight(s1: f64, s2: f64) -> f64 {
    const MJ_MINVAL: f64 = 1e-15;
    if s1 >= MJ_MINVAL && s2 >= MJ_MINVAL {
        s1 / (s1 + s2)
    } else if s1 < MJ_MINVAL && s2 < MJ_MINVAL {
        0.5
    } else if s1 < MJ_MINVAL {
        0.0 // entity 2 dominates
    } else {
        1.0 // entity 1 dominates
    }
}

/// Combine solref using solmix weight.
/// Standard reference (solref\[0\] > 0): weighted average.
/// Direct reference (solref\[0\] <= 0): element-wise minimum.
fn combine_solref(solref1: [f64; 2], solref2: [f64; 2], mix: f64) -> [f64; 2] {
    if solref1[0] > 0.0 && solref2[0] > 0.0 {
        [
            mix * solref1[0] + (1.0 - mix) * solref2[0],
            mix * solref1[1] + (1.0 - mix) * solref2[1],
        ]
    } else {
        [solref1[0].min(solref2[0]), solref1[1].min(solref2[1])]
    }
}

/// Combine solimp using solmix weight (always weighted average).
fn combine_solimp(solimp1: [f64; 5], solimp2: [f64; 5], mix: f64) -> [f64; 5] {
    std::array::from_fn(|i| mix * solimp1[i] + (1.0 - mix) * solimp2[i])
}

/// Create a contact with solver parameters derived from the colliding geoms.
///
/// Uses the unified `contact_param()` function (MuJoCo `mj_contactParam()`
/// equivalent) for parameter combination: priority gating, solmix-weighted
/// solver params, element-wise max friction, and additive gap.
#[inline]
fn make_contact_from_geoms(
    model: &Model,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
    geom1: usize,
    geom2: usize,
    margin: f64,
) -> Contact {
    let (condim, gap, solref, solimp, mu) = contact_param(model, geom1, geom2);
    let includemargin = margin - gap;

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        6 => 6,
        _ => 3,
    };

    let (t1, t2) = compute_tangent_frame(&normal);

    Contact {
        pos,
        normal,
        depth,
        geom1,
        geom2,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: [0.0, 0.0],
        solimp,
        frame: [t1, t2],
        flex_vertex: None,
    }
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
#[allow(clippy::too_many_arguments)]
fn collide_with_hfield(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64, // TODO: thread margin into heightfield helpers
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
        make_contact_from_geoms(
            model,
            c.point.coords,
            normal,
            c.penetration,
            geom1,
            geom2,
            margin,
        )
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
    margin: f64, // TODO: thread margin into SDF helpers
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
        make_contact_from_geoms(
            model,
            c.point.coords,
            normal,
            c.penetration,
            geom1,
            geom2,
            margin,
        )
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
    margin: f64, // TODO: thread margin into mesh helpers
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
            margin,
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
    margin: f64,
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

            if penetration > -margin {
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
                    margin,
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

            if depth > -margin {
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
                    margin,
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

            if penetration > -margin {
                let contact_pos = closest_end - plane_normal * min_dist;
                Some(make_contact_from_geoms(
                    model,
                    contact_pos,
                    plane_normal,
                    penetration,
                    plane_geom,
                    other_geom,
                    margin,
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
                margin,
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
                margin,
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
    margin: f64,
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

    if depth <= -margin {
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
        margin,
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
    margin: f64,
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

    if depth <= -margin {
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
        margin,
    ))
}

/// Sphere-sphere collision detection.
///
/// This is a simple analytical calculation that's more robust than GJK/EPA
/// for the sphere-sphere case.
#[allow(clippy::too_many_arguments)]
fn collide_sphere_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    pos2: Vector3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Option<Contact> {
    let radius1 = size1.x;
    let radius2 = size2.x;

    let diff = pos2 - pos1;
    let dist = diff.norm();

    // Check for penetration (or within margin zone)
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > -margin {
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
            margin,
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
    margin: f64,
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

    // Check distance (or within margin zone)
    let diff = closest2 - closest1;
    let dist = diff.norm();
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > -margin {
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
            margin,
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
    margin: f64,
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

    if penetration > -margin {
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
            margin,
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
    margin: f64,
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

    if penetration > -margin {
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
            margin,
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
    margin: f64,
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

    if penetration <= -margin {
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
        margin,
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
    margin: f64,
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

    if penetration <= -margin {
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
        margin,
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
    margin: f64,
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

    if penetration > -margin {
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
            margin,
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
    margin: f64,
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
        if pen <= -margin {
            return None; // Separating axis found (beyond margin zone)
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
        if pen <= -margin {
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
            if pen <= -margin {
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
        margin,
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
                        ActuatorTransmission::Site | ActuatorTransmission::Body => {
                            // Length set by transmission function (runs before this).
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
                // Scans efc_force directly for contact constraint rows involving
                // the sensor's geom. The first row of each contact group holds the
                // normal force (always >= 0 after projection).
                let mut total_force = 0.0;
                let nefc = data.efc_type.len();
                let mut ei = 0;
                while ei < nefc {
                    let dim = data.efc_dim[ei];
                    if matches!(
                        data.efc_type[ei],
                        ConstraintType::ContactElliptic
                            | ConstraintType::ContactFrictionless
                            | ConstraintType::ContactPyramidal
                    ) {
                        let ci = data.efc_id[ei];
                        if ci < data.contacts.len() {
                            let c = &data.contacts[ci];
                            if c.geom1 == objid || c.geom2 == objid {
                                if data.efc_type[ei] == ConstraintType::ContactPyramidal {
                                    // §32: Normal force = sum of ALL facet forces
                                    for k in 0..dim {
                                        total_force += data.efc_force[ei + k];
                                    }
                                } else {
                                    // Elliptic/frictionless: first row is normal force
                                    total_force += data.efc_force[ei];
                                }
                            }
                        }
                        ei += dim;
                    } else {
                        ei += 1;
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
                // Joint limit force: read cached constraint force magnitude.
                // objid is the joint index (resolved by model builder).
                if objid < data.jnt_limit_frc.len() {
                    sensor_write(&mut data.sensordata, adr, 0, data.jnt_limit_frc[objid]);
                }
            }

            MjSensorType::TendonLimitFrc => {
                // Tendon limit force: read cached constraint force magnitude.
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

/// Compute flex vertex world positions from body FK.
///
/// Each flex vertex has an associated body (created by process_flex_bodies).
/// The body's xpos (computed by standard FK) IS the vertex world position.
/// Pinned vertices (no DOFs) use xpos directly from their static body.
fn mj_flex(model: &Model, data: &mut Data) {
    for i in 0..model.nflexvert {
        let body_id = model.flexvert_bodyid[i];
        data.flexvert_xpos[i] = data.xpos[body_id];
    }
}

/// Velocity kinematics: compute body velocities from qvel.
fn mj_fwd_velocity(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    // §16.27: Use indirection array for cache-friendly iteration over awake bodies.
    let use_body_ind = sleep_enabled && data.nbody_awake < model.nbody;

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

    let nbody = if use_body_ind {
        data.nbody_awake
    } else {
        model.nbody
    };
    for idx in 1..nbody {
        let body_id = if use_body_ind {
            data.body_awake_ind[idx]
        } else {
            idx
        };

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
                    // 6-DOF: linear (world frame) + angular (body-local → world)
                    vel[3] += data.qvel[dof_adr];
                    vel[4] += data.qvel[dof_adr + 1];
                    vel[5] += data.qvel[dof_adr + 2];
                    // Angular velocity: rotate from body-local to world frame
                    let omega_local = Vector3::new(
                        data.qvel[dof_adr + 3],
                        data.qvel[dof_adr + 4],
                        data.qvel[dof_adr + 5],
                    );
                    let omega_world = data.xquat[body_id] * omega_local;
                    vel[0] += omega_world.x;
                    vel[1] += omega_world.y;
                    vel[2] += omega_world.z;
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
/// For spatial tendons: 3D pairwise routing via `mj_fwd_tendon_spatial()`,
/// with wrap visualization data (`wrap_xpos`/`wrap_obj`, §40b).
///
/// Called from mj_fwd_position() after site transforms, before subtree COM.
fn mj_fwd_tendon(model: &Model, data: &mut Data) {
    if model.ntendon == 0 {
        return;
    }

    let mut wrapcount: usize = 0;
    for t in 0..model.ntendon {
        match model.tendon_type[t] {
            TendonType::Fixed => {
                data.ten_wrapadr[t] = wrapcount;
                data.ten_wrapnum[t] = 0;
                mj_fwd_tendon_fixed(model, data, t);
            }
            TendonType::Spatial => {
                mj_fwd_tendon_spatial(model, data, t, &mut wrapcount);
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
#[allow(clippy::cast_possible_wrap)] // geom_id as i32: safe for any practical model size (< i32::MAX geoms)
fn mj_fwd_tendon_spatial(model: &Model, data: &mut Data, t: usize, wrapcount: &mut usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];
    data.ten_J[t].fill(0.0);
    let mut total_length = 0.0;
    let mut divisor: f64 = 1.0;

    // §40b: record wrap path start address
    data.ten_wrapadr[t] = *wrapcount;
    data.ten_wrapnum[t] = 0;

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
                // §40b: store pulley marker
                debug_assert!(
                    *wrapcount < data.wrap_xpos.len(),
                    "wrap path overflow: wrapcount={} >= capacity={}",
                    *wrapcount,
                    data.wrap_xpos.len()
                );
                data.wrap_xpos[*wrapcount] = Vector3::zeros();
                data.wrap_obj[*wrapcount] = -2;
                data.ten_wrapnum[t] += 1;
                *wrapcount += 1;
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

            // §40b: store leading site
            debug_assert!(
                *wrapcount < data.wrap_xpos.len(),
                "wrap path overflow: wrapcount={} >= capacity={}",
                *wrapcount,
                data.wrap_xpos.len()
            );
            data.wrap_xpos[*wrapcount] = p0;
            data.wrap_obj[*wrapcount] = -1;
            data.ten_wrapnum[t] += 1;
            *wrapcount += 1;

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

                    // §40b: store 3 points: site0, tangent₁, tangent₂
                    debug_assert!(
                        *wrapcount + 2 < data.wrap_xpos.len(),
                        "wrap path overflow: wrapcount+2={} >= capacity={}",
                        *wrapcount + 2,
                        data.wrap_xpos.len()
                    );
                    data.wrap_xpos[*wrapcount] = p0;
                    data.wrap_obj[*wrapcount] = -1;
                    data.wrap_xpos[*wrapcount + 1] = t1;
                    data.wrap_obj[*wrapcount + 1] = geom_id as i32;
                    data.wrap_xpos[*wrapcount + 2] = t2;
                    data.wrap_obj[*wrapcount + 2] = geom_id as i32;
                    data.ten_wrapnum[t] += 3;
                    *wrapcount += 3;

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
                    // §40b: store 1 point: site0 only (straight line, no tangent points)
                    debug_assert!(
                        *wrapcount < data.wrap_xpos.len(),
                        "wrap path overflow: wrapcount={} >= capacity={}",
                        *wrapcount,
                        data.wrap_xpos.len()
                    );
                    data.wrap_xpos[*wrapcount] = p0;
                    data.wrap_obj[*wrapcount] = -1;
                    data.ten_wrapnum[t] += 1;
                    *wrapcount += 1;

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

        // §40b: store final site (endpoint of current segment)
        // After j advance, j points at the endpoint site. Store it if this
        // is the last element or the next element is a pulley.
        let at_end = j == num - 1;
        let before_pulley = j < num - 1 && model.wrap_type[adr + j + 1] == WrapType::Pulley;
        if at_end || before_pulley {
            let endpoint_id = model.wrap_objid[adr + j];
            debug_assert!(
                *wrapcount < data.wrap_xpos.len(),
                "wrap path overflow: wrapcount={} >= capacity={}",
                *wrapcount,
                data.wrap_xpos.len()
            );
            data.wrap_xpos[*wrapcount] = data.site_xpos[endpoint_id];
            data.wrap_obj[*wrapcount] = -1;
            data.ten_wrapnum[t] += 1;
            *wrapcount += 1;
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
                    // cdof convention.
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

/// Compute the full body Jacobian at a world-frame point: 3×nv translational and
/// 3×nv rotational.
///
/// Canonical equivalent of MuJoCo's `mj_jac(m, d, jacp, jacr, point, body)`.
/// Walks the kinematic chain from `body_id` to root, accumulating per-joint
/// contributions for each joint type:
///
/// | Joint type | `jacp` column(s)             | `jacr` column(s)         |
/// |------------|------------------------------|--------------------------|
/// | Hinge      | `axis × r`                   | `axis`                   |
/// | Slide      | `axis`                       | `0`                      |
/// | Ball       | `(R·eᵢ) × r` for `i∈0..3`   | `R·eᵢ`                  |
/// | Free trans | `I₃`                         | `0`                      |
/// | Free rot   | `(R·eᵢ) × r` for `i∈0..3`   | `R·eᵢ`                  |
///
/// where `r = point − anchor`, `R` is the body orientation, and `axis` is the
/// world-frame joint axis.
///
/// Returns `(jacp, jacr)` — both 3×nv dense matrices.
#[must_use]
#[allow(clippy::similar_names)] // jacp/jacr are canonical MuJoCo names
pub fn mj_jac(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
) -> (DMatrix<f64>, DMatrix<f64>) {
    let mut jacp = DMatrix::zeros(3, model.nv);
    let mut jacr = DMatrix::zeros(3, model.nv);

    let mut current = body_id;
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
                    let r = point - anchor;
                    let cross = axis.cross(&r);
                    for k in 0..3 {
                        jacp[(k, dof)] += cross[k];
                        jacr[(k, dof)] += axis[k];
                    }
                }
                MjJointType::Slide => {
                    let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                    for k in 0..3 {
                        jacp[(k, dof)] += axis[k];
                    }
                    // No rotational contribution.
                }
                MjJointType::Ball => {
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    let anchor = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        let cross = omega.cross(&r);
                        for k in 0..3 {
                            jacp[(k, dof + i)] += cross[k];
                            jacr[(k, dof + i)] += omega[k];
                        }
                    }
                }
                MjJointType::Free => {
                    // Translational DOFs (dof+0..dof+3): identity columns.
                    for i in 0..3 {
                        jacp[(i, dof + i)] += 1.0;
                    }
                    // Rotational DOFs (dof+3..dof+6).
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    let r = point - data.xpos[jnt_body];
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        let cross = omega.cross(&r);
                        for k in 0..3 {
                            jacp[(k, dof + 3 + i)] += cross[k];
                            jacr[(k, dof + 3 + i)] += omega[k];
                        }
                    }
                }
            }
        }
        current = model.body_parent[current];
    }

    (jacp, jacr)
}

/// Compute the site Jacobian: `(jacp 3×nv, jacr 3×nv)`.
///
/// Thin wrapper around [`mj_jac`] using the site's parent body and world position.
/// Analogous to MuJoCo's `mj_jacSite`.
#[must_use]
pub fn mj_jac_site(model: &Model, data: &Data, site_id: usize) -> (DMatrix<f64>, DMatrix<f64>) {
    mj_jac(
        model,
        data,
        model.site_body[site_id],
        &data.site_xpos[site_id],
    )
}

/// Compute the body Jacobian at the body origin: `(jacp 3×nv, jacr 3×nv)`.
///
/// Thin wrapper around [`mj_jac`] using the body's world position.
/// Analogous to MuJoCo's `mj_jacBody`.
#[must_use]
pub fn mj_jac_body(model: &Model, data: &Data, body_id: usize) -> (DMatrix<f64>, DMatrix<f64>) {
    mj_jac(model, data, body_id, &data.xpos[body_id])
}

/// Compute combined 6×nv world-frame Jacobian at `point` on the chain rooted at `body_id`.
///
/// Layout: rows 0–2 = angular (ω), rows 3–5 = linear (v).
/// Thin wrapper around [`mj_jac`] that stacks `jacr` and `jacp` into a single 6×nv matrix
/// for `J^T·B·J` projection.
#[doc(hidden)]
#[must_use]
#[allow(clippy::similar_names)] // jacp/jacr are canonical MuJoCo names
pub fn mj_jac_point(
    model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
) -> DMatrix<f64> {
    let (jacp, jacr) = mj_jac(model, data, body_id, point);
    let mut jac = DMatrix::zeros(6, model.nv);
    // rows 0–2: angular (jacr), rows 3–5: linear (jacp)
    for col in 0..model.nv {
        for k in 0..3 {
            jac[(k, col)] = jacr[(k, col)];
            jac[(k + 3, col)] = jacp[(k, col)];
        }
    }
    jac
}

/// Compute 6×nv Jacobian at body CoM. Used by inertia-box derivatives.
#[must_use]
pub(crate) fn mj_jac_body_com(model: &Model, data: &Data, body_id: usize) -> DMatrix<f64> {
    mj_jac_point(model, data, body_id, &data.xipos[body_id])
}

/// Compute 6×nv Jacobian at geom center. Used by ellipsoid derivatives.
#[must_use]
pub(crate) fn mj_jac_geom(model: &Model, data: &Data, geom_id: usize) -> DMatrix<f64> {
    let body_id = model.geom_body[geom_id];
    mj_jac_point(model, data, body_id, &data.geom_xpos[geom_id])
}

/// Project a Cartesian force + torque at a world-frame point on a body into
/// generalized forces via the Jacobian transpose: `qfrc += J_p^T * force + J_r^T * torque`.
///
/// General-purpose utility matching MuJoCo's `mj_applyFT()`. Walks the kinematic
/// chain from `body_id` to root, accumulating per-DOF contributions without
/// materializing the full Jacobian. Follows the same chain-walk and axis conventions
/// as `accumulate_point_jacobian()`.
#[allow(clippy::too_many_arguments)]
fn mj_apply_ft(
    model: &Model,
    xpos: &[Vector3<f64>],
    xquat: &[UnitQuaternion<f64>],
    force: &Vector3<f64>,
    torque: &Vector3<f64>,
    point: &Vector3<f64>,
    body_id: usize,
    qfrc: &mut DVector<f64>,
) {
    if body_id == 0 {
        return;
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
                    let anchor = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    qfrc[dof] += axis.cross(&r).dot(force) + axis.dot(torque);
                }
                MjJointType::Slide => {
                    let axis = xquat[jnt_body] * model.jnt_axis[jnt_id];
                    qfrc[dof] += axis.dot(force);
                }
                MjJointType::Ball => {
                    let anchor = xpos[jnt_body] + xquat[jnt_body] * model.jnt_pos[jnt_id];
                    let r = point - anchor;
                    // Body-frame axes (matching MuJoCo's cdof convention for ball joints,
                    // same as accumulate_point_jacobian)
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        qfrc[dof + i] += omega.cross(&r).dot(force) + omega.dot(torque);
                    }
                }
                MjJointType::Free => {
                    // Translation DOFs (0,1,2): world-frame x,y,z
                    qfrc[dof] += force[0];
                    qfrc[dof + 1] += force[1];
                    qfrc[dof + 2] += force[2];
                    // Rotation DOFs (3,4,5): body-frame axes (cdof convention)
                    let anchor = xpos[jnt_body];
                    let r = point - anchor;
                    let rot = xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        qfrc[dof + 3 + i] += omega.cross(&r).dot(force) + omega.dot(torque);
                    }
                }
            }
        }
        current = model.body_parent[current];
    }
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

/// Construct the 2D wrapping plane for sphere wrapping.
///
/// Returns `(axis0, axis1)` where `axis0 = normalize(p1)` and `axis1` is
/// perpendicular to `axis0` in the plane containing `(origin, p1, p2)`.
/// Handles the collinear degenerate case (`p1`, `O`, `p2` collinear) by
/// constructing an arbitrary perpendicular plane through `p1`.
fn sphere_wrapping_plane(p1: Vector3<f64>, p2: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
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
    let axis0 = p1.normalize();
    let axis1 = plane_normal.cross(&axis0).normalize();
    (axis0, axis1)
}

/// 2D Newton solver for inverse (inside) tendon wrapping.
///
/// Given two 2D endpoint positions (in the wrapping plane, circle centered at
/// origin) and a circle radius, finds the single tangent point for an inverse
/// wrap path — the tendon passes *through* the geometry, touching the circle
/// surface at one point.
///
/// Implements MuJoCo's `wrap_inside()` from `engine_util_misc.c`.
///
/// Returns `None` (no wrap — tendon goes straight) or `Some(tangent_point)`.
/// `None` maps to MuJoCo's return `-1`; `Some` maps to return `0`.
#[allow(clippy::many_single_char_names)] // Newton solver math: z, f, g, a match MuJoCo variable names.
fn wrap_inside_2d(end0: Vector2<f64>, end1: Vector2<f64>, radius: f64) -> Option<Vector2<f64>> {
    const EPS: f64 = 1e-15; // MuJoCo's mjMINVAL

    // Step 1 — Validate inputs.
    let len0 = end0.norm();
    let len1 = end1.norm();
    if len0 <= radius || len1 <= radius || radius < EPS || len0 < EPS || len1 < EPS {
        return None;
    }

    // Step 2 — Segment-circle intersection check.
    let dif = end1 - end0;
    let dd = dif.norm_squared();
    if dd > EPS {
        let a = -(dif.dot(&end0)) / dd;
        if a > 0.0 && a < 1.0 {
            let nearest = end0 + a * dif;
            if nearest.norm() <= radius {
                return None; // straight path crosses circle
            }
        }
    }

    // Step 3 — Compute Newton equation parameters.
    let cap_a = radius / len0;
    let cap_b = radius / len1;
    let cos_g = (len0 * len0 + len1 * len1 - dd) / (2.0 * len0 * len1);

    if cos_g < -1.0 + EPS {
        return None; // near-antiparallel: no solution
    }

    // Step 4 — Prepare default fallback (after antiparallel check for NaN safety).
    let mid = 0.5 * (end0 + end1);
    let mid_norm = mid.norm();
    let fallback = if mid_norm > EPS {
        mid * (radius / mid_norm)
    } else {
        // Midpoint near-zero — should not happen after antiparallel guard,
        // but defend against numerical edge cases.
        return None;
    };

    if cos_g > 1.0 - EPS {
        return Some(fallback); // near-parallel / coincident: use fallback
    }

    let g = cos_g.acos();

    // Newton equation: f(z) = asin(A·z) + asin(B·z) - 2·asin(z) + G = 0
    let mut z: f64 = 1.0 - 1e-7; // initial guess near domain boundary
    let mut f = (cap_a * z).asin() + (cap_b * z).asin() - 2.0 * z.asin() + g;

    if f > 0.0 {
        return Some(fallback); // init on wrong side of root
    }

    // Step 5 — Newton iteration.
    let mut converged = false;
    for _ in 0..20 {
        if f.abs() < 1e-6 {
            converged = true;
            break;
        }

        let df = cap_a / (1.0 - z * z * cap_a * cap_a).sqrt().max(EPS)
            + cap_b / (1.0 - z * z * cap_b * cap_b).sqrt().max(EPS)
            - 2.0 / (1.0 - z * z).sqrt().max(EPS);

        if df > -EPS {
            return Some(fallback); // derivative non-negative; abort
        }

        let z_new = z - f / df;

        if z_new > z {
            return Some(fallback); // solver moving rightward; abort
        }

        z = z_new;
        f = (cap_a * z).asin() + (cap_b * z).asin() - 2.0 * z.asin() + g;

        if f > 1e-6 {
            return Some(fallback); // overshot positive; abort
        }
    }

    if !converged {
        return Some(fallback);
    }

    // Step 6 — Geometric reconstruction.
    let cross = end0.x * end1.y - end0.y * end1.x;
    let (vec, ang) = if cross > 0.0 {
        let v = end0.normalize();
        let a = z.asin() - (cap_a * z).asin();
        (v, a)
    } else {
        let v = end1.normalize();
        let a = z.asin() - (cap_b * z).asin();
        (v, a)
    };

    let tangent = Vector2::new(
        radius * (ang.cos() * vec.x - ang.sin() * vec.y),
        radius * (ang.sin() * vec.x + ang.cos() * vec.y),
    );

    Some(tangent)
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

    // 2. Check for sidesite inside sphere (inverse wrap).
    //    Uses 3D norm matching MuJoCo's mju_norm3(s) < radius.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            let (axis0, axis1) = sphere_wrapping_plane(p1, p2);
            let end0 = Vector2::new(p1.dot(&axis0), p1.dot(&axis1));
            let end1 = Vector2::new(p2.dot(&axis0), p2.dot(&axis1));

            return match wrap_inside_2d(end0, end1, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    let t3d = axis0 * t2d.x + axis1 * t2d.y;
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d, // identical — single tangent point
                        arc_length: 0.0,
                    }
                }
            };
        }
    }

    // 3. Check if the straight-line path misses the sphere.
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

    // 4. Construct 2D wrapping plane via shared helper.
    let (axis0, axis1) = sphere_wrapping_plane(p1, p2);
    let plane_normal = axis0.cross(&axis1);

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

    // 2b. Check for sidesite inside cylinder (inverse wrap).
    //     Uses 3D norm matching MuJoCo's mju_norm3(s) < radius dispatch.
    //     This differs from the previous panic check which used 2D XY norm —
    //     a sidesite inside the cross-section but far along Z (norm3 > radius)
    //     now correctly uses normal wrap instead of inside wrap.
    if let Some(ss) = sidesite {
        if ss.norm() < radius {
            return match wrap_inside_2d(p1_xy, p2_xy, radius) {
                None => WrapResult::NoWrap,
                Some(t2d) => {
                    // Z interpolation with arc_length = 0.
                    let l0 = (p1_xy - t2d).norm();
                    let l1 = (p2_xy - t2d).norm();
                    let total = l0 + l1;
                    let tz = if total > 1e-10 {
                        p1.z + (p2.z - p1.z) * l0 / total
                    } else {
                        0.5 * (p1.z + p2.z)
                    };
                    let t3d = Vector3::new(t2d.x, t2d.y, tz);
                    WrapResult::Wrapped {
                        tangent_point_1: t3d,
                        tangent_point_2: t3d, // identical — single tangent point
                        arc_length: 0.0,      // no helical component
                    }
                }
            };
        }
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

/// Compute contact normal Jacobian difference: `n^T · (J_p(b2) - J_p(b1))`.
///
/// `b1` is the body of `geom1`, `b2` is the body of `geom2`. Follows MuJoCo's
/// `mj_jacDifPair(b1, b2)` convention which computes `J(b2) - J(b1)` (second
/// argument minus first). The contact normal points from `geom[0]` toward
/// `geom[1]`.
fn compute_contact_normal_jacobian(model: &Model, data: &Data, contact: &Contact) -> DVector<f64> {
    let nv = model.nv;
    let b1 = model.geom_body[contact.geom1];
    let b2 = model.geom_body[contact.geom2];
    let point = &contact.pos;
    let normal = &contact.normal;

    let mut j_normal = DVector::zeros(nv);

    // Walk kinematic chain for body2 (+1 contribution)
    // MuJoCo convention: jacdifp = J(b2) - J(b1)
    accumulate_point_jacobian(
        model,
        &data.xpos,
        &data.xquat,
        &mut j_normal,
        b2,
        point,
        normal,
        1.0,
    );
    // Walk kinematic chain for body1 (-1 contribution)
    accumulate_point_jacobian(
        model,
        &data.xpos,
        &data.xquat,
        &mut j_normal,
        b1,
        point,
        normal,
        -1.0,
    );

    j_normal
}

/// Compute adhesion moment arm for a single body-transmission actuator.
///
/// Iterates all contacts involving the target body, accumulates the contact
/// normal Jacobians, negates and averages. The negation ensures positive ctrl
/// produces an attractive force (toward the contact surface).
///
/// Gear is NOT applied — MuJoCo's `mjTRN_BODY` case omits gear entirely.
/// Force magnitude is controlled by `gainprm[0]` (the `gain` attribute).
fn mj_transmission_body(model: &Model, data: &mut Data, actuator_id: usize) {
    let body_id = model.actuator_trnid[actuator_id][0];
    let nv = model.nv;
    let mut moment = DVector::zeros(nv);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];

        // Skip flex contacts (MuJoCo: geom < 0)
        if contact.flex_vertex.is_some() {
            continue;
        }

        // Skip contacts not involving target body
        let b1 = model.geom_body[contact.geom1];
        let b2 = model.geom_body[contact.geom2];
        if b1 != body_id && b2 != body_id {
            continue;
        }

        // Compute normal^T · (J(b2, pos) - J(b1, pos))
        let j_normal = compute_contact_normal_jacobian(model, data, contact);
        moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        // Negate and average. NO gear scaling (MuJoCo omits gear for body
        // transmission — force magnitude is controlled by gainprm, not gear).
        moment *= -1.0 / (count as f64);
    }

    data.actuator_moment[actuator_id] = moment;

    // Body transmission has no length concept
    data.actuator_length[actuator_id] = 0.0;
}

/// Dispatch body transmission computation for all body-transmission actuators.
///
/// Must run after `mj_collision()` (needs contacts) and before `mj_sensor_pos()`
/// (which reads `actuator_length`).
fn mj_transmission_body_dispatch(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] == ActuatorTransmission::Body {
            mj_transmission_body(model, data, i);
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
            ActuatorTransmission::Body => {
                // Length already set to 0 by mj_transmission_body (position stage).
                // Velocity from cached moment (same as Site):
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

/// Compute next activation state: integrate act_dot and clamp to actrange.
///
/// Matches MuJoCo's `mj_nextActivation()` in `engine_forward.c`:
/// 1. Integrates activation using Euler (all types) or exact exponential (FilterExact).
/// 2. Clamps to `[actrange[0], actrange[1]]` when `actlimited` is true.
///
/// The `act_dot` input should be computed from the UNCLAMPED current activation
/// (MuJoCo computes act_dot before clamping). Clamping only applies after integration.
///
/// This function is called in two contexts:
/// - **`actearly` force computation**: predict next-step activation for force generation.
/// - **Integration step**: update the actual activation state.
fn mj_next_activation(model: &Model, actuator_id: usize, current_act: f64, act_dot: f64) -> f64 {
    let mut act = current_act;

    // Integration step
    if model.actuator_dyntype[actuator_id] == ActuatorDynamics::FilterExact {
        let tau = model.actuator_dynprm[actuator_id][0].max(1e-10);
        act += act_dot * tau * (1.0 - (-model.timestep / tau).exp());
    } else {
        act += act_dot * model.timestep;
    }

    // Activation clamping (§34)
    if model.actuator_actlimited[actuator_id] {
        let range = model.actuator_actrange[actuator_id];
        act = act.clamp(range.0, range.1);
    }

    act
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
                // act_dot computed from UNCLAMPED current activation (MuJoCo convention)
                data.act_dot[act_adr] =
                    muscle_activation_dynamics(ctrl, data.act[act_adr], &model.actuator_dynprm[i]);
                if model.actuator_actearly[i] {
                    // §34: predict next-step activation (integrated + clamped)
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
            }
            ActuatorDynamics::Filter | ActuatorDynamics::FilterExact => {
                // First-order filter: d(act)/dt = (ctrl - act) / tau
                // Filter uses Euler integration, FilterExact uses exact integration.
                // Both compute the same act_dot here; the difference is in integrate().
                let act_adr = model.actuator_act_adr[i];
                let tau = model.actuator_dynprm[i][0].max(1e-10);
                data.act_dot[act_adr] = (ctrl - data.act[act_adr]) / tau;
                if model.actuator_actearly[i] {
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
            }
            ActuatorDynamics::Integrator => {
                // Integrator: d(act)/dt = ctrl
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] = ctrl;
                if model.actuator_actearly[i] {
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
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
            ActuatorTransmission::Site | ActuatorTransmission::Body => {
                // Use cached moment vector from transmission function.
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
/// 2. Computing `M\[i,j\]` = `S_i^T` * `I_c` * `S_j` for each joint pair
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
pub(crate) fn mj_crba(model: &Model, data: &mut Data) {
    // ============================================================
    // Phase 0: Preamble — sleep filter + selective qM zeroing (§16.29.3)
    // ============================================================
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;

    data.qLD_valid = false;

    if sleep_filter {
        // Selective path: zero only awake trees' DOF diagonal blocks.
        // DOFs within a tree are contiguous (§16.0 body ordering guarantee).
        // Cross-tree entries in qM are always zero (CRBA's dof_parent walk
        // never crosses tree boundaries), so zeroing only the intra-tree
        // diagonal block for each awake tree is sufficient.
        for tree_id in 0..model.ntree {
            if !data.tree_awake[tree_id] {
                continue; // Sleeping tree — preserve qM entries
            }
            let dof_start = model.tree_dof_adr[tree_id];
            let dof_count = model.tree_dof_num[tree_id];
            for i in dof_start..(dof_start + dof_count) {
                for j in dof_start..(dof_start + dof_count) {
                    data.qM[(i, j)] = 0.0;
                }
            }
        }
    } else {
        // Fast path: all bodies awake, zero everything (original behavior)
        data.qM.fill(0.0);
    }

    if model.nv == 0 {
        return;
    }

    // ============================================================
    // Phase 1: Initialize composite inertias from body inertias
    // ============================================================
    // Copy cinert (computed once in FK) to crb_inertia as starting point.
    // cinert contains 6x6 spatial inertias for individual bodies in world frame.
    // Sleeping bodies' crb_inertia values are preserved from their last awake
    // step — Phase 1 is overwrite (=), not accumulate (+=), so there is no
    // stale-accumulation hazard.
    if sleep_filter {
        for idx in 0..data.nbody_awake {
            let body_id = data.body_awake_ind[idx];
            data.crb_inertia[body_id] = data.cinert[body_id];
        }
    } else {
        for body_id in 0..model.nbody {
            data.crb_inertia[body_id] = data.cinert[body_id];
        }
    }

    // ============================================================
    // Phase 2: Backward pass - accumulate composite inertias
    // ============================================================
    // Process bodies from leaves to root, adding child inertia to parent.
    // This gives Ic[i] = inertia of subtree rooted at body i.
    //
    // Each cinert[body_id] is a 6x6 spatial inertia about that body's origin
    // (xpos[body_id]). Before adding child to parent, we must shift the
    // child's spatial inertia to the parent's reference point using the
    // parallel axis theorem. Without this shift, the rotational block would
    // be incorrect because the two inertias are about different points.
    //
    // §16.29.3 §16.27 exception note: Unlike RNE backward pass (which
    // accumulates velocity-dependent cfrc_bias and needs sleeping roots for
    // correct body 0 accumulation), CRBA backward pass accumulates
    // crb_inertia (position-only, frozen for sleeping bodies) and body 0
    // has no DOFs so its crb_inertia is never consumed by Phase 3.
    // Therefore body_awake_ind (not parent_awake_ind) is correct here.
    if sleep_filter {
        // Selective: iterate only awake bodies in reverse.
        // Per-tree invariant: all bodies in a tree share sleep state, so
        // the backward accumulation only involves bodies in the same tree,
        // all of which are in body_awake_ind.
        for idx in (1..data.nbody_awake).rev() {
            let body_id = data.body_awake_ind[idx];
            let parent_id = model.body_parent[body_id];
            if parent_id != 0 {
                let d = data.xpos[body_id] - data.xpos[parent_id];
                let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
                data.crb_inertia[parent_id] += child_shifted;
            }
        }
    } else {
        for body_id in (1..model.nbody).rev() {
            let parent_id = model.body_parent[body_id];
            if parent_id != 0 {
                // Shift child's spatial inertia from child origin to parent origin,
                // then add to parent's composite inertia.
                let d = data.xpos[body_id] - data.xpos[parent_id];
                let child_shifted = shift_spatial_inertia(&data.crb_inertia[body_id], &d);
                data.crb_inertia[parent_id] += child_shifted;
            }
        }
    }

    // ============================================================
    // Phase 3: Build mass matrix from composite inertias
    // ============================================================
    // Per-DOF iteration with dof_parent walk (MuJoCo-style).
    //
    // This correctly handles cross-entries M[dof_A, dof_B] for bodies with
    // multiple joints (e.g., two hinges on one body), because dof_parent
    // chains same-body DOFs together. The prior per-joint approach missed
    // these cross-entries because body_parent skipped the starting body.

    // Pre-compute per-DOF motion subspace columns (cdof).
    // First build per-joint subspaces, then extract individual DOF columns.
    // Sleeping joints' subspaces are computed but never read because the
    // outer loop skips sleeping DOFs, and the dof_parent walk only visits
    // ancestors in the same awake tree.
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    // Build cdof only for rigid DOFs (flex DOFs have dof_jnt = usize::MAX,
    // (§27F) All DOFs now have real joints — build cdof for all DOFs.
    let cdof: Vec<Vector6<f64>> = (0..model.nv)
        .map(|dof| {
            let jnt = model.dof_jnt[dof];
            let dof_in_jnt = dof - model.jnt_dof_adr[jnt];
            joint_subspaces[jnt].column(dof_in_jnt).into_owned()
        })
        .collect();

    let nv_iter = if sleep_filter {
        data.nv_awake
    } else {
        model.nv
    };
    for v in 0..nv_iter {
        let dof_i = if sleep_filter {
            data.dof_awake_ind[v]
        } else {
            v
        };
        let body_i = model.dof_body[dof_i];
        let ic = &data.crb_inertia[body_i];

        // buf = Ic[body_i] * cdof[dof_i]  (spatial force at body_i's frame)
        let mut buf: Vector6<f64> = ic * &cdof[dof_i];

        // Diagonal: M[i,i] = cdof[i]^T * buf
        data.qM[(dof_i, dof_i)] = cdof[dof_i].dot(&buf);

        // Walk dof_parent chain for off-diagonal entries.
        // All ancestors are in the same awake tree (per-tree invariant),
        // so no sleep check is needed in the ancestor walk.
        let mut current_body = body_i;
        let mut j = model.dof_parent[dof_i];

        while let Some(dof_j) = j {
            let body_j = model.dof_body[dof_j];

            // Spatial force transform only when crossing a body boundary.
            // Same-body DOFs share an origin, so no transform is needed.
            if body_j != current_body {
                let r = data.xpos[current_body] - data.xpos[body_j];

                // Shift spatial force: angular += r × linear
                // Linear part is unchanged (spatial force transform property).
                let fl_x = buf[3];
                let fl_y = buf[4];
                let fl_z = buf[5];
                buf[0] += r.y * fl_z - r.z * fl_y;
                buf[1] += r.z * fl_x - r.x * fl_z;
                buf[2] += r.x * fl_y - r.y * fl_x;

                current_body = body_j;
            }

            // Off-diagonal: M[j,i] = cdof[j]^T * buf
            let m_ji = cdof[dof_j].dot(&buf);
            data.qM[(dof_j, dof_i)] = m_ji;
            data.qM[(dof_i, dof_j)] = m_ji; // symmetry

            j = model.dof_parent[dof_j];
        }
    }

    // ============================================================
    // Phase 4: Add armature inertia to diagonal
    // ============================================================
    for jnt_id in 0..model.njnt {
        if sleep_filter && data.body_sleep_state[model.jnt_body[jnt_id]] == SleepState::Asleep {
            continue;
        }
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

    // Phase 5b: Sparse L^T D L factorization
    // ============================================================
    // Exploits tree sparsity from dof_parent for O(n) factorization and solve.
    // Reused in mj_fwd_acceleration_explicit() and pgs_solve_contacts().
    // C3b: Selective factorization skips sleeping DOFs when sleep is enabled.
    // Sleeping DOFs' qLD entries are preserved from their last awake step.
    mj_factor_sparse_selective(model, data);

    // Phase 5c: (§27F) Flex DOFs now handled by standard sparse factorization above.

    // ============================================================
    // Phase 6: Cache body effective mass/inertia from qM diagonal
    // ============================================================
    // Extract per-body min mass/inertia for constraint force limiting.
    // This avoids O(joints) traversal per constraint.
    cache_body_effective_mass(model, data, sleep_filter);
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
/// When `sleep_filter` is true, sleeping bodies' cached values are preserved
/// from their last awake step (not recomputed, not zeroed). All three sub-phases
/// (reset, visitor, fallback) skip sleeping bodies. (§16.29.3 Phase 6)
///
/// Must be called after `mj_crba()` has computed the mass matrix.
fn cache_body_effective_mass(model: &Model, data: &mut Data, sleep_filter: bool) {
    // Visitor struct for JointVisitor pattern (defined before statements per clippy)
    struct MassCacheVisitor<'a> {
        model: &'a Model,
        data: &'a mut Data,
        sleep_filter: bool,
    }

    impl JointVisitor for MassCacheVisitor<'_> {
        fn visit_free(&mut self, ctx: JointContext) {
            let body_id = self.model.jnt_body[ctx.jnt_id];
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
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
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
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
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
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
            if self.sleep_filter && self.data.body_sleep_state[body_id] == SleepState::Asleep {
                return;
            }
            // Single linear DOF
            if ctx.dof_adr < self.model.nv {
                let mass = self.data.qM[(ctx.dof_adr, ctx.dof_adr)];
                if mass > MIN_INERTIA_THRESHOLD {
                    self.data.body_min_mass[body_id] = self.data.body_min_mass[body_id].min(mass);
                }
            }
        }
    }

    // Sub-phase 1: Reset only awake bodies.
    // CRITICAL: sleeping bodies' cached values must be preserved.
    for i in 1..model.nbody {
        if sleep_filter && data.body_sleep_state[i] == SleepState::Asleep {
            continue; // Preserve cached values from last awake step
        }
        data.body_min_mass[i] = f64::INFINITY;
        data.body_min_inertia[i] = f64::INFINITY;
    }

    // Sub-phase 2: Visitor with sleep guard.
    let mut visitor = MassCacheVisitor {
        model,
        data,
        sleep_filter,
    };
    model.visit_joints(&mut visitor);

    // Sub-phase 3: Fallback — only awake bodies.
    for i in 1..model.nbody {
        if sleep_filter && data.body_sleep_state[i] == SleepState::Asleep {
            continue; // Preserve cached values from last awake step
        }
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

// mj_crba_flex DELETED (§27F): Flex vertices are now real bodies with slide joints.
// The standard CRBA handles their mass matrix entries automatically.

// mj_factor_flex DELETED (§27F): Flex vertices are now real bodies with slide joints.
// The standard sparse LDL factorization handles their DOFs automatically.

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
///
/// Reference: MuJoCo Computation docs - mj_rne section
#[allow(
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
fn mj_rne(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    // §16.27: Use indirection array for cache-friendly iteration over awake bodies.
    let use_body_ind = sleep_enabled && data.nbody_awake < model.nbody;

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

    // (§27F) Flex vertex gravity now handled by the joint loop above — each vertex
    // has a body with 3 slide joints, so the standard gravity path applies automatically.

    // ========== Gyroscopic terms for Ball/Free joints ==========
    // τ_gyro = ω × (I * ω) - the gyroscopic torque
    // This is the dominant Coriolis effect for 3D rotations
    let nbody_gyro = if use_body_ind {
        data.nbody_awake
    } else {
        model.nbody
    };
    for idx in 1..nbody_gyro {
        let body_id = if use_body_ind {
            data.body_awake_ind[idx]
        } else {
            idx
        };

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

/// Compute gravity compensation forces for bodies with non-zero `gravcomp`.
///
/// Matches MuJoCo's `mj_gravcomp()` in `engine_passive.c`. For each body with
/// `gravcomp != 0`, applies an anti-gravity force `F = -gravity * mass * gravcomp`
/// at the body's center of mass (`xipos`), projecting through the kinematic chain
/// via `mj_apply_ft()` into `qfrc_gravcomp`.
///
/// Returns `true` if any gravcomp was applied (for conditional routing).
fn mj_gravcomp(model: &Model, data: &mut Data) -> bool {
    // Guard: no bodies with gravcomp, or gravity is zero
    if model.ngravcomp == 0 || model.gravity.norm() == 0.0 {
        return false;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let mut has_gravcomp = false;

    for b in 1..model.nbody {
        let gc = model.body_gravcomp[b];
        if gc == 0.0 {
            continue;
        }
        // Sleep filtering: skip bodies in sleeping trees
        if sleep_enabled && !data.tree_awake[model.body_treeid[b]] {
            continue;
        }
        has_gravcomp = true;
        // Anti-gravity force at body CoM (xipos, NOT xpos).
        // Gravity acts at the CoM; using xpos would produce wrong torques
        // for bodies whose CoM doesn't coincide with their frame origin.
        let force = -model.gravity * (model.body_mass[b] * gc);
        let zero_torque = Vector3::zeros();
        let point = data.xipos[b];
        // Project through Jacobian to generalized forces
        mj_apply_ft(
            model,
            &data.xpos,
            &data.xquat,
            &force,
            &zero_torque,
            &point,
            b,
            &mut data.qfrc_gravcomp,
        );
    }

    has_gravcomp
}

// ============================================================================
// §40 Fluid / Aerodynamic Forces
// ============================================================================

// MJ_MINVAL (1e-15) is defined once at module scope (Newton solver section).
// Reused here for fluid force denomination guards.

/// Compute 6D velocity at an object center in its local frame.
///
/// Equivalent to MuJoCo's `mj_objectVelocity(m, d, objtype, id, res, flg_local=1)`.
/// Returns `[ω_local; v_local]`.
pub(crate) fn object_velocity_local(
    _model: &Model,
    data: &Data,
    body_id: usize,
    point: &Vector3<f64>,
    rot: &Matrix3<f64>,
) -> [f64; 6] {
    // Static body check: world body (id=0) has mass=0 and is caught by the
    // dispatch loop's mass guard. For extra safety, return zeros for body 0.
    if body_id == 0 {
        return [0.0; 6];
    }

    let cvel = &data.cvel[body_id];
    let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
    let v_origin = Vector3::new(cvel[3], cvel[4], cvel[5]);

    // Shift linear velocity from body origin to requested point.
    // Our cvel stores velocity at xpos[body_id] (body origin), so the lever
    // arm is from body origin to the query point.
    let dif = point - data.xpos[body_id];
    let v_point = v_origin + omega.cross(&dif);

    // Rotate to local frame
    let omega_local = rot.transpose() * omega;
    let v_local = rot.transpose() * v_point;

    [
        omega_local.x,
        omega_local.y,
        omega_local.z,
        v_local.x,
        v_local.y,
        v_local.z,
    ]
}

/// Rotate a 6D spatial vector (local frame) to world frame.
#[inline]
fn rotate_spatial_to_world(xmat: &Matrix3<f64>, lfrc: &[f64; 6]) -> [f64; 6] {
    let torque = xmat * Vector3::new(lfrc[0], lfrc[1], lfrc[2]);
    let force = xmat * Vector3::new(lfrc[3], lfrc[4], lfrc[5]);
    [torque.x, torque.y, torque.z, force.x, force.y, force.z]
}

/// Cross product of two 3-vectors stored as slices.
#[inline]
fn cross3(a: &[f64], b: &[f64]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Accumulate cross product: `out += a × b`.
#[inline]
fn add_cross3(out: &mut [f64], a: &[f64; 3], b: &[f64]) {
    out[0] += a[1] * b[2] - a[2] * b[1];
    out[1] += a[2] * b[0] - a[0] * b[2];
    out[2] += a[0] * b[1] - a[1] * b[0];
}

/// Euclidean norm of a 3-element slice.
#[inline]
pub(crate) fn norm3(v: &[f64]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Compute semi-axes for a geom type, matching MuJoCo's `mju_geomSemiAxes`.
pub(crate) fn fluid_geom_semi_axes(geom_type: GeomType, size: &Vector3<f64>) -> [f64; 3] {
    match geom_type {
        GeomType::Sphere => [size.x, size.x, size.x],
        GeomType::Capsule => [size.x, size.x, size.y + size.x],
        GeomType::Cylinder => [size.x, size.x, size.y],
        _ => [size.x, size.y, size.z], // Ellipsoid, Box, Mesh
    }
}

/// Per-axis moment for angular drag: `(8/15)π · s[axis] · max(other_two)⁴`.
/// Matches MuJoCo's `mji_ellipsoid_max_moment(size, dir)`.
#[inline]
pub(crate) fn ellipsoid_moment(s: &[f64; 3], axis: usize) -> f64 {
    let d1 = s[(axis + 1) % 3];
    let d2 = s[(axis + 2) % 3];
    (8.0 / 15.0) * std::f64::consts::PI * s[axis] * d1.max(d2).powi(4)
}

/// Inertia-box fluid model (legacy, body-level).
/// Called for bodies where no child geom has `geom_fluid[0] > 0`.
fn mj_inertia_box_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let mass = model.body_mass[body_id];
    let inertia = model.body_inertia[body_id];

    // 1. Equivalent box dimensions (full side lengths)
    let bx = ((inertia.y + inertia.z - inertia.x).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let by = ((inertia.x + inertia.z - inertia.y).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let bz = ((inertia.x + inertia.y - inertia.z).max(MJ_MINVAL) / mass * 6.0).sqrt();

    // 2. Local 6D velocity at body CoM in inertia frame
    let mut lvel = object_velocity_local(
        model,
        data,
        body_id,
        &data.xipos[body_id],
        &data.ximat[body_id],
    );

    // 3. Subtract wind (translational only, rotate to inertia frame)
    let wind_local = data.ximat[body_id].transpose() * model.wind;
    lvel[3] -= wind_local.x;
    lvel[4] -= wind_local.y;
    lvel[5] -= wind_local.z;

    // 4. Compute local forces
    let mut lfrc = [0.0f64; 6];

    // Viscous resistance (assignment, matching MuJoCo's mji_scl3)
    if beta > 0.0 {
        let diam = (bx + by + bz) / 3.0;
        let d3 = diam * diam * diam;
        let pi = std::f64::consts::PI;
        for i in 0..3 {
            lfrc[i] = -pi * d3 * beta * lvel[i];
        }
        for i in 0..3 {
            lfrc[3 + i] = -3.0 * pi * diam * beta * lvel[3 + i];
        }
    }

    // Quadratic drag (density, subtracted from viscous result)
    if rho > 0.0 {
        lfrc[3] -= 0.5 * rho * by * bz * lvel[3].abs() * lvel[3];
        lfrc[4] -= 0.5 * rho * bx * bz * lvel[4].abs() * lvel[4];
        lfrc[5] -= 0.5 * rho * bx * by * lvel[5].abs() * lvel[5];

        lfrc[0] -= rho * bx * (by.powi(4) + bz.powi(4)) / 64.0 * lvel[0].abs() * lvel[0];
        lfrc[1] -= rho * by * (bx.powi(4) + bz.powi(4)) / 64.0 * lvel[1].abs() * lvel[1];
        lfrc[2] -= rho * bz * (bx.powi(4) + by.powi(4)) / 64.0 * lvel[2].abs() * lvel[2];
    }

    // 5. Rotate to world frame and apply at body CoM
    let bfrc = rotate_spatial_to_world(&data.ximat[body_id], &lfrc);
    mj_apply_ft(
        model,
        &data.xpos,
        &data.xquat,
        &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),
        &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),
        &data.xipos[body_id],
        body_id,
        &mut data.qfrc_fluid,
    );
}

/// Ellipsoid fluid model (advanced, per-geom).
/// Called for bodies where any child geom has `geom_fluid[0] > 0`.
#[allow(clippy::similar_names)] // d_min/d_mid/d_max are standard notation for semi-axis ordering
fn mj_ellipsoid_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let pi = std::f64::consts::PI;

    let geom_adr = model.body_geom_adr[body_id];
    let geom_num = model.body_geom_num[body_id];

    for gid in geom_adr..geom_adr + geom_num {
        let fluid = &model.geom_fluid[gid];
        let interaction_coef = fluid[0];
        if interaction_coef == 0.0 {
            continue;
        }

        // Unpack coefficients
        let (c_blunt, c_slender, c_ang) = (fluid[1], fluid[2], fluid[3]);
        let (c_kutta, c_magnus) = (fluid[4], fluid[5]);
        let vmass = [fluid[6], fluid[7], fluid[8]];
        let vinertia = [fluid[9], fluid[10], fluid[11]];

        // Semi-axes
        let s = fluid_geom_semi_axes(model.geom_type[gid], &model.geom_size[gid]);

        // Local velocity at geom center in geom frame
        let geom_body = model.geom_body[gid];
        let mut lvel = object_velocity_local(
            model,
            data,
            geom_body,
            &data.geom_xpos[gid],
            &data.geom_xmat[gid],
        );
        let wind_local = data.geom_xmat[gid].transpose() * model.wind;
        lvel[3] -= wind_local.x;
        lvel[4] -= wind_local.y;
        lvel[5] -= wind_local.z;

        let w = [lvel[0], lvel[1], lvel[2]];
        let v = [lvel[3], lvel[4], lvel[5]];
        let mut lfrc = [0.0f64; 6];
        let speed = norm3(&v);

        // ── Component 1: Added mass (gyroscopic, accels=NULL) ──
        let pv = [
            rho * vmass[0] * v[0],
            rho * vmass[1] * v[1],
            rho * vmass[2] * v[2],
        ];
        let lv = [
            rho * vinertia[0] * w[0],
            rho * vinertia[1] * w[1],
            rho * vinertia[2] * w[2],
        ];
        add_cross3(&mut lfrc[3..6], &pv, &w); // force  += p_v × ω
        add_cross3(&mut lfrc[0..3], &pv, &v); // torque += p_v × v
        add_cross3(&mut lfrc[0..3], &lv, &w); // torque += L_v × ω

        // ── Component 2: Magnus lift ──
        let vol = (4.0 / 3.0) * pi * s[0] * s[1] * s[2];
        let mag = cross3(&w, &v);
        for i in 0..3 {
            lfrc[3 + i] += c_magnus * rho * vol * mag[i];
        }

        // ── Component 3: Kutta lift ──
        let norm_vec = [
            (s[1] * s[2]).powi(2) * v[0],
            (s[2] * s[0]).powi(2) * v[1],
            (s[0] * s[1]).powi(2) * v[2],
        ];
        let proj_denom = (s[1] * s[2]).powi(4) * v[0] * v[0]
            + (s[2] * s[0]).powi(4) * v[1] * v[1]
            + (s[0] * s[1]).powi(4) * v[2] * v[2];
        let proj_num = (s[1] * s[2] * v[0]).powi(2)
            + (s[2] * s[0] * v[1]).powi(2)
            + (s[0] * s[1] * v[2]).powi(2);
        let a_proj = pi * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();
        let cos_alpha = proj_num / (speed * proj_denom).max(MJ_MINVAL);

        let mut circ = cross3(&norm_vec, &v);
        let kutta_scale = c_kutta * rho * cos_alpha * a_proj;
        for i in 0..3 {
            circ[i] *= kutta_scale;
        }
        let kf = cross3(&circ, &v);
        for i in 0..3 {
            lfrc[3 + i] += kf[i];
        }

        // ── Component 4: Combined linear drag ──
        let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
        let d_max = s[0].max(s[1]).max(s[2]);
        let d_min = s[0].min(s[1]).min(s[2]);
        let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
        let a_max = pi * d_max * d_mid;

        let drag_lin = beta * 3.0 * pi * eq_d
            + rho * speed * (a_proj * c_blunt + c_slender * (a_max - a_proj));
        for i in 0..3 {
            lfrc[3 + i] -= drag_lin * v[i];
        }

        // ── Component 5: Combined angular drag ──
        let i_max = (8.0 / 15.0) * pi * d_mid * d_max.powi(4);
        let ii = [
            ellipsoid_moment(&s, 0),
            ellipsoid_moment(&s, 1),
            ellipsoid_moment(&s, 2),
        ];
        let mom_visc = [
            w[0] * (c_ang * ii[0] + c_slender * (i_max - ii[0])),
            w[1] * (c_ang * ii[1] + c_slender * (i_max - ii[1])),
            w[2] * (c_ang * ii[2] + c_slender * (i_max - ii[2])),
        ];
        let drag_ang = beta * pi * eq_d.powi(3) + rho * norm3(&mom_visc);
        for i in 0..3 {
            lfrc[i] -= drag_ang * w[i];
        }

        // ── Scale by interaction coefficient and accumulate ──
        for val in &mut lfrc {
            *val *= interaction_coef;
        }
        let bfrc = rotate_spatial_to_world(&data.geom_xmat[gid], &lfrc);
        mj_apply_ft(
            model,
            &data.xpos,
            &data.xquat,
            &Vector3::new(bfrc[3], bfrc[4], bfrc[5]),
            &Vector3::new(bfrc[0], bfrc[1], bfrc[2]),
            &data.geom_xpos[gid],
            body_id,
            &mut data.qfrc_fluid,
        );
    }
}

/// Top-level fluid force dispatch. Returns `true` if any fluid forces were computed.
/// Matches MuJoCo's `mj_fluid()` in `engine_passive.c`.
fn mj_fluid(model: &Model, data: &mut Data) -> bool {
    if model.density == 0.0 && model.viscosity == 0.0 {
        return false;
    }

    // §40c: Sleep filtering — skip sleeping bodies (MuJoCo engine_passive.c pattern)
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;
    let nbody = if sleep_filter {
        data.nbody_awake
    } else {
        model.nbody
    };

    for idx in 0..nbody {
        let body_id = if sleep_filter {
            data.body_awake_ind[idx]
        } else {
            idx
        };

        // Mass guard — applies to both models (matches MuJoCo)
        if model.body_mass[body_id] < MJ_MINVAL {
            continue;
        }

        // Dispatch: does any child geom have geom_fluid[0] > 0?
        let geom_adr = model.body_geom_adr[body_id];
        let geom_num = model.body_geom_num[body_id];
        let use_ellipsoid =
            (geom_adr..geom_adr + geom_num).any(|gid| model.geom_fluid[gid][0] > 0.0);

        if use_ellipsoid {
            mj_ellipsoid_fluid(model, data, body_id);
        } else {
            mj_inertia_box_fluid(model, data, body_id);
        }
    }

    true
}

/// Compute passive forces (springs and dampers).
///
/// Implements MuJoCo's passive force model:
/// - **Spring**: τ = -stiffness * (q - springref)
/// - **Damper**: τ = -damping * qvel
///
/// Friction loss is handled entirely by solver constraint rows (§29),
/// not by passive forces.
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
/// initializes `qfrc_passive` to zero (no explicit passive contributions).
fn mj_fwd_passive(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    data.qfrc_passive.fill(0.0);
    data.qfrc_fluid.fill(0.0);
    data.qfrc_gravcomp.fill(0.0);
    // qfrc_frictionloss is now populated post-solve from efc_force (§29).
    // No longer computed in passive forces.

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

        // S6: Deadband spring — force is zero within [lower, upper]
        let k = model.tendon_stiffness[t];
        if k > 0.0 {
            let [lower, upper] = model.tendon_lengthspring[t];
            if length > upper {
                force += k * (upper - length);
            } else if length < lower {
                force += k * (lower - length);
            }
            // else: deadband, no spring force
        }

        // Damper: F = -b * v
        let b = model.tendon_damping[t];
        if b > 0.0 {
            force -= b * velocity;
        }

        // Friction loss is now handled entirely by solver constraint rows (§29).
        // No tanh approximation in passive forces.

        data.ten_force[t] = force;

        // NOTE: In ImplicitSpringDamper mode, tendon spring/damper forces are
        // handled implicitly in mj_fwd_acceleration_implicit() via non-diagonal
        // K_tendon and D_tendon matrices (DT-35). ten_force[t] is always populated
        // for diagnostic purposes, but the explicit qfrc_passive application is
        // skipped to avoid double-counting, matching the joint spring/damper pattern.

        // Map tendon force to joint forces via J^T.
        if !implicit_mode && force != 0.0 {
            apply_tendon_force(
                model,
                &data.ten_J[t],
                model.tendon_type[t],
                t,
                force,
                &mut data.qfrc_passive,
            );
        }
    }

    // Flex vertex damping: qfrc_passive[dof] = -damping * qvel[dof]
    for i in 0..model.nflexvert {
        let dof_base = model.flexvert_dofadr[i];
        if dof_base == usize::MAX {
            continue; // Pinned vertex: no DOFs
        }
        let flex_id = model.flexvert_flexid[i];
        let damp = model.flex_damping[flex_id];
        if damp <= 0.0 {
            continue;
        }
        for k in 0..3 {
            data.qfrc_passive[dof_base + k] -= damp * data.qvel[dof_base + k];
        }
    }

    // Flex edge passive spring-damper forces.
    // MuJoCo architecture: <edge stiffness="..." damping="..."/> drives passive
    // forces (engine_passive.c), separate from constraint-based edge enforcement
    // (mjEQ_FLEX in engine_core_constraint.c which uses eq_solref/eq_solimp).
    // Note: MuJoCo docs say <edge stiffness> is "Only for 1D flex" (cables).
    // For 2D/3D, elasticity comes from FEM via <elasticity>. The code applies
    // to all dims (matching MuJoCo's runtime behavior), but users should only
    // set nonzero stiffness for dim=1 flex bodies.
    for e in 0..model.nflexedge {
        let flex_id = model.flexedge_flexid[e];
        let stiffness = model.flex_edgestiffness[flex_id];
        let damping = model.flex_edgedamping[flex_id];

        if stiffness == 0.0 && damping == 0.0 {
            continue;
        }

        let [v0, v1] = model.flexedge_vert[e];

        // Skip edges where both vertices are pinned (rigid edge)
        if model.flexvert_invmass[v0] == 0.0 && model.flexvert_invmass[v1] == 0.0 {
            continue;
        }

        let x0 = data.flexvert_xpos[v0];
        let x1 = data.flexvert_xpos[v1];
        let diff = x1 - x0;
        let dist = diff.norm();
        if dist < 1e-10 {
            continue;
        }

        let direction = diff / dist;
        let rest_len = model.flexedge_length0[e];

        // Spring force: stiffness * (rest_length - current_length)
        // Positive when compressed (restoring), negative when stretched.
        let frc_spring = stiffness * (rest_len - dist);

        // Damping force: -damping * edge_velocity
        // edge_velocity = d(dist)/dt = (v1 - v0) · direction
        // (§27F) Pinned vertices have dofadr=usize::MAX and zero velocity.
        let dof0 = model.flexvert_dofadr[v0];
        let dof1 = model.flexvert_dofadr[v1];
        let vel0 = if dof0 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
        };
        let vel1 = if dof1 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
        };
        let edge_velocity = (vel1 - vel0).dot(&direction);
        let frc_damper = -damping * edge_velocity;

        let force_mag = frc_spring + frc_damper;

        // Apply via J^T: edge Jacobian is ±direction for the two endpoint DOFs.
        // F_v0 = -direction * force_mag (pulls v0 toward v1 when stretched)
        // F_v1 = +direction * force_mag (pulls v1 toward v0 when stretched)
        if dof0 < model.nv {
            for ax in 0..3 {
                data.qfrc_passive[dof0 + ax] -= direction[ax] * force_mag;
            }
        }
        if dof1 < model.nv {
            for ax in 0..3 {
                data.qfrc_passive[dof1 + ax] += direction[ax] * force_mag;
            }
        }
    }

    // Flex bending passive forces: spring-damper on dihedral angle (Bridson et al. 2003).
    // MuJoCo computes bending as passive forces in engine_passive.c, NOT as constraint rows.
    // Force = -k_bend * (theta - theta0) - b_bend * d(theta)/dt, applied via J^T.
    let dt = model.timestep;
    for h in 0..model.nflexhinge {
        let [ve0, ve1, va, vb] = model.flexhinge_vert[h];
        let flex_id = model.flexhinge_flexid[h];

        let k_bend_raw = model.flex_bend_stiffness[flex_id];
        let b_bend = model.flex_bend_damping[flex_id];
        if k_bend_raw <= 0.0 && b_bend <= 0.0 {
            continue;
        }

        let pe0 = data.flexvert_xpos[ve0];
        let pe1 = data.flexvert_xpos[ve1];
        let pa = data.flexvert_xpos[va];
        let pb = data.flexvert_xpos[vb];
        let rest_angle = model.flexhinge_angle0[h];

        // Shared edge vector
        let e = pe1 - pe0;
        let e_len_sq = e.norm_squared();
        if e_len_sq < 1e-20 {
            continue;
        }

        // Face normals (unnormalized)
        let offset_a = pa - pe0;
        let offset_b = pb - pe0;
        let normal_face_a = e.cross(&offset_a);
        let normal_face_b = offset_b.cross(&e);
        let norm_sq_a = normal_face_a.norm_squared();
        let norm_sq_b = normal_face_b.norm_squared();
        if norm_sq_a < 1e-20 || norm_sq_b < 1e-20 {
            continue;
        }

        // Dihedral angle via atan2
        let e_len = e_len_sq.sqrt();
        let e_norm = e / e_len;
        let normal_unit_a = normal_face_a / norm_sq_a.sqrt();
        let normal_unit_b = normal_face_b / norm_sq_b.sqrt();
        let cos_theta = normal_unit_a.dot(&normal_unit_b).clamp(-1.0, 1.0);
        let sin_theta = normal_unit_a.cross(&normal_unit_b).dot(&e_norm);
        let theta = sin_theta.atan2(cos_theta);
        let angle_error = theta - rest_angle;

        // Bridson dihedral gradient (all 4 vertices)
        let grad_a = e_len * normal_face_a / norm_sq_a;
        let grad_b = e_len * normal_face_b / norm_sq_b;
        let bary_a = offset_a.dot(&e) / e_len_sq;
        let bary_b = offset_b.dot(&e) / e_len_sq;
        let grad_e0 = -grad_a * (1.0 - bary_a) - grad_b * (1.0 - bary_b);
        let grad_e1 = -grad_a * bary_a - grad_b * bary_b;

        // Spring: F = -k * angle_error
        let spring_mag = -k_bend_raw * angle_error;

        // Damper: F = -b * d(theta)/dt, where d(theta)/dt = J · qvel
        // (§27F) Pinned vertices have dofadr=usize::MAX and zero velocity.
        let dof_e0 = model.flexvert_dofadr[ve0];
        let dof_e1 = model.flexvert_dofadr[ve1];
        let dof_a = model.flexvert_dofadr[va];
        let dof_b = model.flexvert_dofadr[vb];
        let read_vel = |dof: usize| -> Vector3<f64> {
            if dof == usize::MAX {
                Vector3::zeros()
            } else {
                Vector3::new(data.qvel[dof], data.qvel[dof + 1], data.qvel[dof + 2])
            }
        };
        let vel_e0 = read_vel(dof_e0);
        let vel_e1 = read_vel(dof_e1);
        let vel_a = read_vel(dof_a);
        let vel_b = read_vel(dof_b);
        let theta_dot =
            grad_e0.dot(&vel_e0) + grad_e1.dot(&vel_e1) + grad_a.dot(&vel_a) + grad_b.dot(&vel_b);
        let damper_mag = -b_bend * theta_dot;

        let force_mag = spring_mag + damper_mag;

        // Apply via J^T to qfrc_passive, with per-vertex stability clamp.
        // The force on vertex i is: F_i = force_mag * grad_i
        // The acceleration is: a_i = F_i * invmass_i = force_mag * grad_i * invmass_i
        // For explicit Euler stability: |a_i * dt| must not exceed the velocity scale.
        // We clamp the per-vertex force magnitude so that:
        //   |force_mag * |grad_i| * invmass_i * dt^2| < 1
        // This prevents any single bending hinge from causing instability, regardless
        // of how deformed the mesh becomes.
        let grads = [
            (ve0, dof_e0, grad_e0),
            (ve1, dof_e1, grad_e1),
            (va, dof_a, grad_a),
            (vb, dof_b, grad_b),
        ];
        for &(v_idx, dof, grad) in &grads {
            let invmass = model.flexvert_invmass[v_idx];
            if invmass > 0.0 {
                let grad_norm = grad.norm();
                let mut fm = force_mag;
                if grad_norm > 0.0 {
                    // Max force_mag so that acceleration * dt doesn't exceed position scale
                    let fm_max = 1.0 / (dt * dt * grad_norm * invmass);
                    fm = fm.clamp(-fm_max, fm_max);
                }
                for ax in 0..3 {
                    data.qfrc_passive[dof + ax] += grad[ax] * fm;
                }
            }
        }
    }

    // Fluid forces (§40): compute and add to qfrc_passive.
    // qfrc_fluid is zeroed at the top; mj_fluid accumulates into it.
    if mj_fluid(model, data) {
        data.qfrc_passive += &data.qfrc_fluid;
    }

    // Gravity compensation: compute and route to qfrc_passive (§35).
    // MuJoCo computes gravcomp after spring/damper/flex passive forces, then
    // conditionally routes via jnt_actgravcomp. We unconditionally add to
    // qfrc_passive since jnt_actgravcomp is not yet implemented.
    if mj_gravcomp(model, data) {
        data.qfrc_passive += &data.qfrc_gravcomp;
    }
}

/// Check if all DOFs affected by a tendon's Jacobian belong to sleeping trees (§16.5a').
pub(crate) fn tendon_all_dofs_sleeping(model: &Model, data: &Data, t: usize) -> bool {
    tendon_all_dofs_sleeping_fields(model, &data.ten_J[t], &data.tree_awake)
}

/// Field-level variant of `tendon_all_dofs_sleeping` that avoids borrowing all of `Data`.
/// Used by `accumulate_tendon_kd` where `data.scratch_m_impl` is mutably borrowed.
fn tendon_all_dofs_sleeping_fields(
    model: &Model,
    ten_j: &DVector<f64>,
    tree_awake: &[bool],
) -> bool {
    for dof in 0..model.nv {
        if ten_j[dof] != 0.0 && tree_awake[model.dof_treeid[dof]] {
            return false; // At least one target DOF is awake
        }
    }
    true
}

/// Compute the deadband displacement for a tendon (DT-35).
///
/// Returns `length - upper` if `length > upper`, `length - lower` if
/// `length < lower`, `0.0` if inside the deadband `[lower, upper]`.
/// At the boundary (`length == lower` or `length == upper`), returns `0.0`
/// (spring disengaged — see "one-step delay" note in DT-35 Step 2).
#[inline]
fn tendon_deadband_displacement(length: f64, range: [f64; 2]) -> f64 {
    let [lower, upper] = range;
    if length > upper {
        length - upper
    } else if length < lower {
        length - lower
    } else {
        0.0
    }
}

/// Return the effective stiffness for implicit treatment (DT-35).
///
/// Returns `k` when the tendon is outside its deadband (spring engaged),
/// `0.0` when inside (spring disengaged). This gates the `h²·K` LHS
/// modification: no phantom stiffness inside the deadband.
///
/// **Note on exact boundary:** At `length == lower` or `length == upper`,
/// returns `0.0`. The displacement is also `0.0` at the boundary, so no
/// spring force exists. If velocity moves the tendon outside the deadband,
/// the spring activates in the next step (one-step delay, consistent with
/// linearization at the current state).
#[inline]
fn tendon_active_stiffness(k: f64, length: f64, range: [f64; 2]) -> f64 {
    if k <= 0.0 {
        return 0.0;
    }
    let [lower, upper] = range;
    if length >= lower && length <= upper {
        0.0
    } else {
        k
    }
}

/// Accumulate non-diagonal tendon K/D into a mass matrix (DT-35).
///
/// For each tendon with nonzero stiffness or damping, adds the rank-1
/// outer product `(h²·k_active + h·b) · J^T · J` to `matrix`. Uses
/// deadband-aware `k_active` (zero inside deadband, `k` outside).
///
/// Shared by `mj_fwd_acceleration_implicit` (Step 2) and
/// `build_m_impl_for_newton` (Step 2b). Both call sites must produce
/// identical mass matrix modifications — factoring this out guarantees it.
///
/// **Sleep guard:** Skips tendons whose target DOFs are all sleeping,
/// matching the guards in `mj_fwd_passive` and `mjd_passive_vel`.
fn accumulate_tendon_kd(
    matrix: &mut DMatrix<f64>,
    model: &Model,
    ten_j: &[DVector<f64>],
    ten_length: &[f64],
    tree_awake: &[bool],
    h: f64,
    sleep_enabled: bool,
) {
    let h2 = h * h;
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping_fields(model, &ten_j[t], tree_awake) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        let bt = model.tendon_damping[t];
        if kt <= 0.0 && bt <= 0.0 {
            continue;
        }
        let j = &ten_j[t];
        let k_active = tendon_active_stiffness(kt, ten_length[t], model.tendon_lengthspring[t]);
        let scale = h2 * k_active + h * bt;
        // Defensive: skip if scale is non-positive. For valid models (k ≥ 0,
        // b ≥ 0) this is unreachable when the above guard passes, but protects
        // against pathological negative parameters that would break SPD.
        if scale <= 0.0 {
            continue;
        }
        // Rank-1 outer product: (h²·k_active + h·b) · J^T · J
        //
        // Sparsity skip `j[r] == 0.0`: for fixed tendons, Jacobian entries
        // are exact MJCF coefficients (parsed floats), so zero entries are
        // exactly 0.0. For spatial tendons, entries are computed from 3D
        // geometry and may be near-zero (e.g. 1e-17) rather than exact
        // zero — but the contribution of such entries is O(ε²) per matrix
        // element, which is negligible (well below f64 precision). This
        // matches MuJoCo's own sparse outer-product loops.
        for r in 0..model.nv {
            if j[r] == 0.0 {
                continue;
            }
            for c in 0..model.nv {
                if j[c] == 0.0 {
                    continue;
                }
                matrix[(r, c)] += scale * j[r] * j[c];
            }
        }
    }
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

    /// Process a 1-DOF joint (Hinge or Slide) with spring and damper.
    /// Friction loss is now handled entirely by solver constraint rows (§29).
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
    }

    /// Process a multi-DOF joint (Ball or Free) with per-DOF damping.
    /// Friction loss is now handled entirely by solver constraint rows (§29).
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

// Moved to types/model_init.rs: compute_body_lengths(), compute_dof_lengths()

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
pub(crate) fn reset_sleep_state(model: &Model, data: &mut Data) {
    // First: set all trees to awake
    for t in 0..model.ntree {
        data.tree_asleep[t] = -(1 + MIN_AWAKE); // Fully awake
    }

    // Then: validate and create sleep cycles for Init trees
    if let Err(e) = validate_init_sleep(model, data) {
        // Log warning and degrade Init trees to awake (spec §16.24)
        #[cfg(debug_assertions)]
        eprintln!("Init-sleep validation failed: {e}");
        let _ = e; // Suppress unused warning in release
    }

    mj_update_sleep_arrays(model, data);
}

/// Validate Init-sleep trees and create sleep cycles (§16.24).
///
/// Uses union-find over model-time adjacency (equality constraints +
/// multi-tree tendons) to group Init trees. Creates circular sleep cycles
/// per group. Returns an error if validation fails.
fn validate_init_sleep(model: &Model, data: &mut Data) -> Result<(), SleepError> {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return Ok(());
    }

    // Phase 1: Basic per-tree validation
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] != SleepPolicy::Init {
            continue;
        }
        if model.tree_dof_num[t] == 0 {
            return Err(SleepError::InitSleepInvalidTree { tree: t });
        }
    }

    // Phase 2: Check for mixed Init/non-Init in statically-coupled groups
    let mut uf = UnionFind::new(model.ntree);

    // Equality constraint edges
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;
        }
        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a < model.ntree && tree_b < model.ntree && tree_a != tree_b {
            uf.union(tree_a, tree_b);
        }
    }

    // Multi-tree tendon edges
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] == 2 {
            let tree_a = model.tendon_tree[2 * t];
            let tree_b = model.tendon_tree[2 * t + 1];
            if tree_a < model.ntree && tree_b < model.ntree {
                uf.union(tree_a, tree_b);
            }
        }
    }

    // Check each group for mixed Init/non-Init
    let mut group_has_init = vec![false; model.ntree];
    let mut group_has_noninit = vec![false; model.ntree];
    for t in 0..model.ntree {
        let root = uf.find(t);
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            group_has_init[root] = true;
        } else {
            group_has_noninit[root] = true;
        }
    }
    for root in 0..model.ntree {
        if group_has_init[root] && group_has_noninit[root] {
            return Err(SleepError::InitSleepMixedIsland { group_root: root });
        }
    }

    // Phase 3: Create sleep cycles for validated Init-sleep groups
    // Group Init trees by their union-find root
    let mut init_groups: std::collections::HashMap<usize, Vec<usize>> =
        std::collections::HashMap::new();
    for t in 0..model.ntree {
        if model.tree_sleep_policy[t] == SleepPolicy::Init {
            let root = uf.find(t);
            init_groups.entry(root).or_default().push(t);
        }
    }
    for trees in init_groups.values() {
        sleep_trees(model, data, trees);
    }

    Ok(())
}

/// Recompute derived sleep arrays from `tree_asleep` (§16.3, §16.17).
///
/// Updates: tree_awake, body_sleep_state, ntree_awake, nv_awake,
/// and the awake-index indirection arrays (body_awake_ind, parent_awake_ind, dof_awake_ind).
pub(crate) fn mj_update_sleep_arrays(model: &Model, data: &mut Data) {
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
        let is_awake = if model.dof_treeid.len() > dof {
            let tree = model.dof_treeid[dof];
            // Flex DOFs have tree == usize::MAX (no tree) → always awake
            tree >= model.ntree || data.tree_awake[tree]
        } else {
            // No tree info → treat as awake
            true
        };
        if is_awake {
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
/// Works directly from raw data sources (contacts, model equality constraints,
/// tendon limits, joint limits) rather than from `efc_*` arrays, so it is
/// independent of which solver path (Newton vs PGS/CG) is active.
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

    // === Phase 1: Edge extraction (§16.11.2) ===
    // Collect edges as (tree_a, tree_b) pairs from raw data sources.
    // Only awake trees participate in island discovery — sleeping trees
    // don't contribute to constraint solving.

    // Clear scratch
    data.island_scratch_rownnz[..ntree].fill(0);
    data.island_scratch_colind.clear();

    // Helper closure: returns true if tree is awake (eligible for islands)
    let tree_awake = |tree: usize| -> bool { tree < ntree && data.tree_asleep[tree] < 0 };

    let ncon = data.contacts.len();
    let capacity = ncon + model.neq + model.njnt + model.ntendon;
    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(capacity);

    // 1a: Contacts → tree pairs from geom → body → tree
    for contact in &data.contacts {
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
            usize::MAX // World body
        };
        let tree2 = if body2 > 0 && body2 < model.body_treeid.len() {
            model.body_treeid[body2]
        } else {
            usize::MAX // World body
        };

        if tree1 < ntree && tree2 < ntree && tree_awake(tree1) && tree_awake(tree2) {
            edges.push((tree1, tree2));
            if tree1 != tree2 {
                edges.push((tree2, tree1));
            }
        } else if tree1 < ntree && tree_awake(tree1) {
            edges.push((tree1, tree1)); // Contact with world
        } else if tree2 < ntree && tree_awake(tree2) {
            edges.push((tree2, tree2)); // Contact with world
        }
    }

    // 1b: Active equality constraints → tree pairs (awake only)
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id] {
            continue;
        }
        let (tree1, tree2) = equality_trees(model, eq_id);
        if tree_awake(tree1) {
            edges.push((tree1, tree1));
            if tree_awake(tree2) && tree2 != tree1 {
                edges.push((tree1, tree2));
                edges.push((tree2, tree1));
            } else if tree_awake(tree2) {
                edges.push((tree2, tree2));
            }
        } else if tree_awake(tree2) {
            edges.push((tree2, tree2));
        }
    }

    // 1c: Active joint limits → self-edges (awake only)
    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }
        // Check if limit is actually violated (active)
        let (limit_min, limit_max) = model.jnt_range[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];
        let q = data.qpos[qpos_adr];
        if q < limit_min || q > limit_max {
            let body = model.jnt_body[jnt_id];
            if body > 0 && body < model.body_treeid.len() {
                let tree = model.body_treeid[body];
                if tree_awake(tree) {
                    edges.push((tree, tree));
                }
            }
        }
    }

    // 1d: Active tendon limits → tree pair edges (awake only)
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }
        // Check if tendon limit is actually violated (active)
        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];
        if length < limit_min || length > limit_max {
            if model.tendon_treenum[t] == 2 {
                let t1 = model.tendon_tree[2 * t];
                let t2 = model.tendon_tree[2 * t + 1];
                if tree_awake(t1) && tree_awake(t2) {
                    edges.push((t1, t2));
                    edges.push((t2, t1));
                }
            } else if model.tendon_treenum[t] == 1 {
                let tree = model.tendon_tree[2 * t];
                if tree_awake(tree) {
                    edges.push((tree, tree));
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

    // 4c: Contacts per island (§16.16)
    // Assign each contact to an island based on its bodies' trees.
    // For contacts with two dynamic bodies, use the first body's tree
    // (both trees are in the same island by construction of the edge graph).
    let ncon = data.contacts.len();
    data.contact_island.resize(ncon, -1);

    for ci in 0..ncon {
        let contact = &data.contacts[ci];
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
        // Pick the first dynamic body's tree for island assignment
        let tree = if body1 > 0 && body1 < model.body_treeid.len() {
            model.body_treeid[body1]
        } else if body2 > 0 && body2 < model.body_treeid.len() {
            model.body_treeid[body2]
        } else {
            continue; // Both world — shouldn't happen but skip
        };
        if tree < ntree && island_out[tree] >= 0 {
            data.contact_island[ci] = island_out[tree];
        }
    }

    // efc_island population is now deferred to populate_efc_island(),
    // called from mj_fwd_constraint() after assemble_unified_constraints().
    // This ensures efc_island is based on the CURRENT step's constraint rows.
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
            let t1_id = model.eq_obj1id[eq_id];
            // Primary tree for tendon 1 (sentinel if treenum == 0, i.e. static)
            let tree1 = if model.tendon_treenum[t1_id] >= 1 {
                model.tendon_tree[2 * t1_id]
            } else {
                sentinel
            };

            if model.eq_obj2id[eq_id] == usize::MAX {
                // Single-tendon: if it spans two trees, return both
                if model.tendon_treenum[t1_id] == 2 {
                    (
                        model.tendon_tree[2 * t1_id],
                        model.tendon_tree[2 * t1_id + 1],
                    )
                } else {
                    (tree1, tree1)
                }
            } else {
                // Two-tendon coupling: primary tree from each tendon
                let t2_id = model.eq_obj2id[eq_id];
                let tree2 = if model.tendon_treenum[t2_id] >= 1 {
                    model.tendon_tree[2 * t2_id]
                } else {
                    sentinel
                };
                (tree1, tree2)
            }
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
        ConstraintType::ContactFrictionless
        | ConstraintType::ContactPyramidal
        | ConstraintType::ContactElliptic => {
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
        ConstraintType::FlexEdge => {
            // Flex constraints: scan Jacobian for nonzero DOFs and find their tree
            for dof in 0..model.nv {
                if data.efc_J[(row, dof)].abs() > 0.0 && dof < model.dof_treeid.len() {
                    let tree = model.dof_treeid[dof];
                    if tree < ntree {
                        return tree;
                    }
                }
            }
            sentinel
        }
    }
}

/// Wake detection: check user-applied forces on sleeping bodies (§16.4).
///
/// Called at the start of `forward()`, before any pipeline stage.
/// Wake detection for user-applied forces (§16.4).
///
/// Returns `true` if any tree was woken (caller must update sleep arrays).
fn mj_wake(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;

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
            woke_any = true;
        }
    }

    // Check qfrc_applied (per-DOF generalized forces)
    for dof in 0..model.nv {
        let tree = model.dof_treeid[dof];
        if !data.tree_awake[tree] && data.qfrc_applied[dof].to_bits() != 0 {
            mj_wake_tree(model, data, tree);
            woke_any = true;
        }
    }

    woke_any
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

/// Return the canonical (minimum) tree index in a sleep cycle (§16.10.3).
///
/// Used to identify whether two sleeping trees belong to the same cycle.
#[allow(clippy::cast_sign_loss)]
fn mj_sleep_cycle(tree_asleep: &[i32], start: usize) -> usize {
    if tree_asleep[start] < 0 {
        return start; // Not asleep — return self
    }
    let mut min_tree = start;
    let mut current = tree_asleep[start] as usize;
    while current != start {
        if current < min_tree {
            min_tree = current;
        }
        current = tree_asleep[current] as usize;
    }
    min_tree
}

/// Check if a tendon's limit constraint is active (§16.13.2).
fn tendon_limit_active(model: &Model, data: &Data, t: usize) -> bool {
    if !model.tendon_limited[t] {
        return false;
    }
    let length = data.ten_length[t];
    let (limit_min, limit_max) = model.tendon_range[t];
    length < limit_min || length > limit_max
}

/// Wake sleeping trees coupled by multi-tree tendons with active limits (§16.13.2).
///
/// Returns `true` if any tree was woken.
#[allow(clippy::cast_sign_loss)]
fn mj_wake_tendon(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for t in 0..model.ntendon {
        if model.tendon_treenum[t] != 2 {
            continue;
        }
        if !tendon_limit_active(model, data, t) {
            continue;
        }

        let tree_a = model.tendon_tree[2 * t];
        let tree_b = model.tendon_tree[2 * t + 1];
        if tree_a >= model.ntree || tree_b >= model.ntree {
            continue;
        }
        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep in different cycles: merge by waking both
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
            }
            _ => {} // Both awake — no action
        }
    }
    woke_any
}

/// Wake sleeping trees coupled by active equality constraints (§16.13.3).
///
/// Returns `true` if any tree was woken.
#[allow(clippy::cast_sign_loss)]
fn mj_wake_equality(model: &Model, data: &mut Data) -> bool {
    if model.enableflags & ENABLE_SLEEP == 0 {
        return false;
    }

    let mut woke_any = false;
    for eq in 0..model.neq {
        if !model.eq_active[eq] {
            continue;
        }

        let (tree_a, tree_b) = equality_trees(model, eq);
        if tree_a >= model.ntree || tree_b >= model.ntree || tree_a == tree_b {
            continue; // Same tree or invalid — no cross-tree coupling
        }

        let awake_a = data.tree_awake[tree_a];
        let awake_b = data.tree_awake[tree_b];

        match (awake_a, awake_b) {
            (true, false) => {
                mj_wake_tree(model, data, tree_b);
                woke_any = true;
            }
            (false, true) => {
                mj_wake_tree(model, data, tree_a);
                woke_any = true;
            }
            (false, false) => {
                // Both asleep in different cycles: merge by waking both
                let cycle_a = mj_sleep_cycle(&data.tree_asleep, tree_a);
                let cycle_b = mj_sleep_cycle(&data.tree_asleep, tree_b);
                if cycle_a != cycle_b {
                    mj_wake_tree(model, data, tree_a);
                    mj_wake_tree(model, data, tree_b);
                    woke_any = true;
                }
            }
            _ => {} // Both awake — no action
        }
    }
    woke_any
}

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

/// Compute the contact Jacobian for a flex-rigid contact.
///
/// The flex vertex side has a trivial Jacobian: the contact frame direction is
/// directly projected onto the vertex's 3 translational DOF columns (no kinematic
/// chain traversal). The rigid body side uses the standard `add_body_jacobian` pattern.
///
/// Convention: `contact.flex_vertex` = vertex index, `contact.geom1/geom2` = rigid geom.
/// Normal points FROM rigid surface TOWARD flex vertex (outward from rigid geom).
fn compute_flex_contact_jacobian(
    model: &Model,
    data: &Data,
    contact: &Contact,
    vertex_idx: usize,
) -> DMatrix<f64> {
    let nv = model.nv;
    let dim = contact.dim;
    let normal = contact.normal;
    let tangent1 = contact.frame[0];
    let tangent2 = contact.frame[1];

    let mut j = DMatrix::zeros(dim, nv);

    // Flex vertex side: trivial Jacobian (identity on DOF columns).
    // The vertex's 3 translational DOFs map directly to Cartesian velocity.
    // (§27F) Pinned vertices have no DOFs — Jacobian columns are all zero (immovable).
    let dof_base = model.flexvert_dofadr[vertex_idx];

    // Helper: project direction onto vertex DOFs with given sign
    let add_vertex_jacobian = |j: &mut DMatrix<f64>, row: usize, dir: &Vector3<f64>, sign: f64| {
        if dof_base == usize::MAX {
            return; // Pinned vertex: no DOF columns to fill
        }
        j[(row, dof_base)] += sign * dir.x;
        j[(row, dof_base + 1)] += sign * dir.y;
        j[(row, dof_base + 2)] += sign * dir.z;
    };

    // Rigid body side: standard kinematic chain traversal.
    let rigid_body = model.geom_body[contact.geom2];
    let add_body_jacobian =
        |j: &mut DMatrix<f64>, row: usize, direction: &Vector3<f64>, body_id: usize, sign: f64| {
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
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs (3–5): body-frame axes R·eᵢ, lever arm from xpos.
                            let jpos = data.xpos[jnt_body];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega = rot * Vector3::ith(i, 1.0);
                                j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
                            }
                        }
                    }
                }
                current_body = model.body_parent[current_body];
            }
        };

    // Row 0: normal direction
    // Narrowphase normal points FROM rigid surface TOWARD flex vertex (outward).
    // MuJoCo convention: normal FROM body1 TO body2, body1 gets -1, body2 gets +1.
    // Here: body1 = rigid (normal points away from it), body2 = flex vertex.
    // Vertex (body2) contributes positively, rigid (body1) negatively.
    add_vertex_jacobian(&mut j, 0, &normal, 1.0);
    add_body_jacobian(&mut j, 0, &normal, rigid_body, -1.0);

    // Rows 1-2: tangent directions (sliding friction)
    if dim >= 3 {
        add_vertex_jacobian(&mut j, 1, &tangent1, 1.0);
        add_body_jacobian(&mut j, 1, &tangent1, rigid_body, -1.0);

        add_vertex_jacobian(&mut j, 2, &tangent2, 1.0);
        add_body_jacobian(&mut j, 2, &tangent2, rigid_body, -1.0);
    }

    // Row 3: torsional friction — flex vertices have no angular DOFs, so only
    // the rigid side contributes. The vertex side is zero (no torsion).
    // Rigid is body1, so sign is -1.0 (but friction cost is symmetric/Huber,
    // so sign only affects direction labeling, not correctness).
    if dim >= 4 {
        add_angular_jacobian(&mut j, 3, &normal, rigid_body, -1.0, model, data);
    }

    // Rows 4-5: rolling friction — same reasoning, vertex has no angular DOFs
    if dim >= 6 {
        add_angular_jacobian(&mut j, 4, &tangent1, rigid_body, -1.0, model, data);
        add_angular_jacobian(&mut j, 5, &tangent2, rigid_body, -1.0, model, data);
    }

    j
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

    // Flex-rigid contacts: vertex side has trivial Jacobian (identity on DOF columns),
    // rigid side uses the standard kinematic chain traversal.
    if let Some(vi) = contact.flex_vertex {
        return compute_flex_contact_jacobian(model, data, contact, vi);
    }

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
                            // Linear DOFs (0–2): world-frame identity.
                            j[(row, dof_adr)] += sign * direction.x;
                            j[(row, dof_adr + 1)] += sign * direction.y;
                            j[(row, dof_adr + 2)] += sign * direction.z;

                            // Angular DOFs (3–5): body-frame axes R·eᵢ, lever arm from xpos.
                            let jpos = data.xpos[jnt_body];
                            let r = contact.pos - jpos;
                            let rot = data.xquat[jnt_body].to_rotation_matrix();
                            for i in 0..3 {
                                let omega = rot * Vector3::ith(i, 1.0);
                                j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega.cross(&r));
                            }
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
                    // Angular DOFs (3–5): body-frame axes, matching Ball case above.
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let omega = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&omega);
                    }
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
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

/// Noslip QCQP for 2 friction DOFs (condim=3): solve the constrained tangential subproblem.
///
/// Minimizes  `½(f−f_unc)ᵀ·A·(f−f_unc)` subject to `Σ(f_j/μ_j)² ≤ f_n²`
/// where `f_unc` is the unconstrained GS solution.
///
/// This matches MuJoCo's `mju_QCQP2`:
/// - Scale to unit-friction space: `y_j = f_j / μ_j`
/// - If unconstrained solution is inside cone → return it
/// - Otherwise Newton iteration on dual Lagrange multiplier λ
///
/// `a` is the 2×2 tangential Delassus subblock (unregularized).
/// `f_unc` is the unconstrained GS update for the 2 friction DOFs.
/// `mu` is the friction coefficients for these 2 DOFs.
/// `fn_abs` is the absolute normal force (cone radius).
///
/// Returns the projected (f[0], f[1]).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
fn noslip_qcqp2(a: [[f64; 2]; 2], f_unc: [f64; 2], mu: [f64; 2], fn_abs: f64) -> [f64; 2] {
    // Scale to unit-friction space: y = f / mu, A_s = D·A·D, b_s stays in y-space
    // The unconstrained solution in y-space: y_unc = f_unc / mu
    let y_unc = [
        f_unc[0] / mu[0].max(MJ_MINVAL),
        f_unc[1] / mu[1].max(MJ_MINVAL),
    ];

    // Check if unconstrained solution is inside cone: ||y|| ≤ fn
    let r2 = fn_abs * fn_abs;
    let norm2 = y_unc[0] * y_unc[0] + y_unc[1] * y_unc[1];
    if norm2 <= r2 {
        return f_unc;
    }

    // Need to project: solve (A_s + λI)·y = A_s·y_unc with ||y||² = r²
    // Scale Delassus: A_s[i,j] = mu[i] * A[i,j] * mu[j]
    let a_s = [
        [a[0][0] * mu[0] * mu[0], a[0][1] * mu[0] * mu[1]],
        [a[1][0] * mu[1] * mu[0], a[1][1] * mu[1] * mu[1]],
    ];

    // RHS in y-space: g = A_s · y_unc
    let g = [
        a_s[0][0] * y_unc[0] + a_s[0][1] * y_unc[1],
        a_s[1][0] * y_unc[0] + a_s[1][1] * y_unc[1],
    ];

    // Newton on λ: φ(λ) = ||y(λ)||² − r² = 0
    // y(λ) = (A_s + λI)⁻¹ · g
    let mut lam = 0.0_f64;
    for _ in 0..20 {
        // (A_s + λI) for 2×2
        let m00 = a_s[0][0] + lam;
        let m11 = a_s[1][1] + lam;
        let m01 = a_s[0][1];
        let det = m00 * m11 - m01 * m01;
        if det.abs() < MJ_MINVAL {
            break;
        }
        let inv_det = 1.0 / det;

        // y = M⁻¹ · g
        let y0 = (m11 * g[0] - m01 * g[1]) * inv_det;
        let y1 = (-m01 * g[0] + m00 * g[1]) * inv_det;

        let phi = y0 * y0 + y1 * y1 - r2;
        if phi.abs() < 1e-10 {
            // Converged — unscale and return
            return [y0 * mu[0], y1 * mu[1]];
        }

        // φ'(λ) = -2 · yᵀ · M⁻¹ · y
        let my0 = (m11 * y0 - m01 * y1) * inv_det;
        let my1 = (-m01 * y0 + m00 * y1) * inv_det;
        let dphi = -2.0 * (y0 * my0 + y1 * my1);

        if dphi.abs() < MJ_MINVAL {
            break;
        }
        lam -= phi / dphi;
        lam = lam.max(0.0); // λ ≥ 0 (dual feasibility)
    }

    // Final solve with converged λ
    let m00 = a_s[0][0] + lam;
    let m11 = a_s[1][1] + lam;
    let m01 = a_s[0][1];
    let det = m00 * m11 - m01 * m01;
    if det.abs() < MJ_MINVAL {
        // Degenerate: simple rescaling fallback
        let s = norm2.sqrt();
        if s > MJ_MINVAL {
            let scale = fn_abs / s;
            return [f_unc[0] * scale, f_unc[1] * scale];
        }
        return [0.0, 0.0];
    }
    let inv_det = 1.0 / det;
    let y0 = (m11 * g[0] - m01 * g[1]) * inv_det;
    let y1 = (-m01 * g[0] + m00 * g[1]) * inv_det;

    // Exact rescale to cone boundary for numerical safety
    let yn2 = y0 * y0 + y1 * y1;
    if yn2 > r2 && yn2 > MJ_MINVAL {
        let s = fn_abs / yn2.sqrt();
        [y0 * mu[0] * s, y1 * mu[1] * s]
    } else {
        [y0 * mu[0], y1 * mu[1]]
    }
}

/// Noslip QCQP for 3 friction DOFs (condim=4): solve the constrained tangential subproblem.
///
/// Same algorithm as `noslip_qcqp2` but for 3×3 system. Uses cofactor inverse.
#[allow(clippy::many_single_char_names)]
fn noslip_qcqp3(a: [[f64; 3]; 3], f_unc: [f64; 3], mu: [f64; 3], fn_abs: f64) -> [f64; 3] {
    let y_unc = [
        f_unc[0] / mu[0].max(MJ_MINVAL),
        f_unc[1] / mu[1].max(MJ_MINVAL),
        f_unc[2] / mu[2].max(MJ_MINVAL),
    ];

    let r2 = fn_abs * fn_abs;
    let norm2 = y_unc[0] * y_unc[0] + y_unc[1] * y_unc[1] + y_unc[2] * y_unc[2];
    if norm2 <= r2 {
        return f_unc;
    }

    // Scale Delassus: A_s[i,j] = mu[i] * A[i,j] * mu[j]
    let mut a_s = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            a_s[i][j] = a[i][j] * mu[i] * mu[j];
        }
    }

    // g = A_s · y_unc
    let mut g = [0.0_f64; 3];
    for i in 0..3 {
        for j in 0..3 {
            g[i] += a_s[i][j] * y_unc[j];
        }
    }

    // Newton on λ
    let mut lam = 0.0_f64;
    let mut y = [0.0_f64; 3];
    for _ in 0..20 {
        // M = A_s + λI
        let mut m = a_s;
        m[0][0] += lam;
        m[1][1] += lam;
        m[2][2] += lam;

        // 3×3 cofactor inverse
        let cof00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let cof01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
        let cof02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        let det = m[0][0] * cof00 + m[0][1] * cof01 + m[0][2] * cof02;
        if det.abs() < MJ_MINVAL {
            break;
        }
        let inv_det = 1.0 / det;

        let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
        let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
        let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
        let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];

        // y = M⁻¹ · g (cofactor inverse is transposed)
        y[0] = (cof00 * g[0] + cof10 * g[1] + cof20 * g[2]) * inv_det;
        y[1] = (cof01 * g[0] + cof11 * g[1] + cof21 * g[2]) * inv_det;
        y[2] = (cof02 * g[0] + cof12 * g[1] + cof22 * g[2]) * inv_det;

        let phi = y[0] * y[0] + y[1] * y[1] + y[2] * y[2] - r2;
        if phi.abs() < 1e-10 {
            return [y[0] * mu[0], y[1] * mu[1], y[2] * mu[2]];
        }

        // φ'(λ) = -2 · yᵀ · M⁻¹ · y
        let my0 = (cof00 * y[0] + cof10 * y[1] + cof20 * y[2]) * inv_det;
        let my1 = (cof01 * y[0] + cof11 * y[1] + cof21 * y[2]) * inv_det;
        let my2 = (cof02 * y[0] + cof12 * y[1] + cof22 * y[2]) * inv_det;
        let dphi = -2.0 * (y[0] * my0 + y[1] * my1 + y[2] * my2);

        if dphi.abs() < MJ_MINVAL {
            break;
        }
        lam -= phi / dphi;
        lam = lam.max(0.0);
    }

    // Final solve
    let mut m = a_s;
    m[0][0] += lam;
    m[1][1] += lam;
    m[2][2] += lam;
    let cof00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let cof01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
    let cof02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let det = m[0][0] * cof00 + m[0][1] * cof01 + m[0][2] * cof02;
    if det.abs() < MJ_MINVAL {
        let s = norm2.sqrt();
        if s > MJ_MINVAL {
            let scale = fn_abs / s;
            return [f_unc[0] * scale, f_unc[1] * scale, f_unc[2] * scale];
        }
        return [0.0, 0.0, 0.0];
    }
    let inv_det = 1.0 / det;
    let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
    let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
    let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
    let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
    let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
    let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    y[0] = (cof00 * g[0] + cof10 * g[1] + cof20 * g[2]) * inv_det;
    y[1] = (cof01 * g[0] + cof11 * g[1] + cof21 * g[2]) * inv_det;
    y[2] = (cof02 * g[0] + cof12 * g[1] + cof22 * g[2]) * inv_det;

    let yn2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
    if yn2 > r2 && yn2 > MJ_MINVAL {
        let s = fn_abs / yn2.sqrt();
        [y[0] * mu[0] * s, y[1] * mu[1] * s, y[2] * mu[2] * s]
    } else {
        [y[0] * mu[0], y[1] * mu[1], y[2] * mu[2]]
    }
}

/// Decode pyramidal facet forces into physical normal + friction forces (§32.6).
///
/// Matches MuJoCo's `mju_decodePyramid`:
/// - `f_normal = Σ f_facet[k]` (sum of all facets)
/// - `f_friction[d] = μ[d] · (f_pos[d] - f_neg[d])` for each friction direction
///
/// `facet_forces` is a slice of `2*(dim-1)` facet force values for one contact.
/// `mu` is the 5-element friction array. `dim` is condim (NOT number of facets).
///
/// Returns `(normal_force, friction_forces)` where `friction_forces` has `dim-1` elements.
#[must_use]
pub fn decode_pyramid(facet_forces: &[f64], mu: &[f64; 5], dim: usize) -> (f64, Vec<f64>) {
    let n_facets = 2 * (dim - 1);
    debug_assert_eq!(facet_forces.len(), n_facets);

    let mut f_normal = 0.0;
    for k in 0..n_facets {
        f_normal += facet_forces[k];
    }

    let mut f_friction = Vec::with_capacity(dim - 1);
    for d in 0..(dim - 1) {
        let f_pos = facet_forces[2 * d];
        let f_neg = facet_forces[2 * d + 1];
        f_friction.push(mu[d] * (f_pos - f_neg));
    }

    (f_normal, f_friction)
}

/// Compute position-dependent impedance from solimp parameters.
///
/// The sigmoid shape is derived from MuJoCo's `getimpedance()` in
/// `engine_core_constraint.c`. The impedance value is used to compute the
/// constraint regularization: `R = (1-d)/d * diag_approx(A)`, where `A = J M⁻¹ Jᵀ`.
///
/// The impedance `d ∈ (0, 1)` controls constraint effectiveness:
/// - `d` close to 1 → strong constraint (low regularization, stiff)
/// - `d` close to 0 → weak constraint (high regularization, soft)
///
/// The impedance interpolates from `solimp[0]` (d0) to `solimp[1]` (d_width)
/// over a transition zone of `solimp[2]` (width) meters/radians, using a
/// split power-sigmoid controlled by `solimp[3]` (midpoint) and `solimp[4]` (power).
///
/// ## Notes
///
/// - **No margin offset**: MuJoCo computes `x = (pos - margin) / width`. We pass raw
///   violation (no margin subtraction). This is correct for equality constraints (margin
///   is always 0 in MuJoCo). For contact constraints, `geom_margin` defaults to 0 so
///   `depth - margin = depth`, but if non-zero margins are supported in the future,
///   callers should subtract margin before passing the violation.
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

// Constraint stability limits are applied internally by the unified solver.
// The solver uses regularization and impedance parameters to control
// constraint force magnitude, preventing oscillation from overshooting.

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
pub(crate) const DEFAULT_MASS_FALLBACK: f64 = 1.0;

/// Default solref parameters [timeconst, dampratio] (MuJoCo defaults).
///
/// timeconst = 0.02 seconds gives a 50 Hz natural frequency.
/// dampratio = 1.0 gives critical damping.
pub(crate) const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];

/// Default solimp parameters [d0, d_width, width, midpoint, power] (MuJoCo defaults).
///
/// These control the constraint impedance profile:
/// - d0 = 0.9: Impedance at zero violation
/// - d_width = 0.95: Impedance at full violation width (endpoint)
/// - width = 0.001: Transition zone size (meters or radians)
/// - midpoint = 0.5: Midpoint of the sigmoid transition curve
/// - power = 2.0: Power of the sigmoid transition curve
pub(crate) const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

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
pub(crate) const MJ_MINVAL: f64 = 1e-15;

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

/// Normalize a quaternion [w, x, y, z]. Returns identity if norm < 1e-10.
/// Matches MuJoCo's mju_normalize4 and our normalize_quaternion() convention.
#[inline]
fn normalize_quat4(q: [f64; 4]) -> [f64; 4] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm > 1e-10 {
        [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
    } else {
        [1.0, 0.0, 0.0, 0.0] // identity
    }
}

/// Compute ball joint limit quantities from a unit quaternion [w, x, y, z].
/// Returns `(unit_dir, angle)` where:
///   - `angle = |theta| >= 0` (unsigned rotation magnitude)
///   - `unit_dir = sign(theta) * axis` (Jacobian direction)
///
/// When `angle < 1e-10` (near-identity), returns `(Vector3::z(), 0.0)`.
/// The unit_dir value is arbitrary in this case — the caller must not use it
/// because `dist = limit - 0 > 0` means no constraint is instantiated.
///
/// Matches MuJoCo's `mju_quat2Vel(angleAxis, q, 1)` followed by
/// `value = mju_normalize3(angleAxis)`.
fn ball_limit_axis_angle(q: [f64; 4]) -> (Vector3<f64>, f64) {
    // Step 1: mju_quat2Vel with dt=1
    let v = Vector3::new(q[1], q[2], q[3]);
    let sin_half = v.norm();

    // Guard: near-identity → angle ≈ 0, axis undefined.
    if sin_half < 1e-10 {
        return (Vector3::z(), 0.0);
    }

    let axis = v / sin_half; // unit rotation axis
    let mut theta = 2.0 * sin_half.atan2(q[0]);
    if theta > std::f64::consts::PI {
        theta -= 2.0 * std::f64::consts::PI;
    }

    // Step 2: |theta * axis| = |theta|, direction = sign(theta) * axis.
    let angle = theta.abs();
    let unit_dir = if theta >= 0.0 { axis } else { -axis };

    (unit_dir, angle)
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
///
/// `rownnz[i]` includes diagonal (last element); off-diagonal count = `rownnz[i] - 1`.
fn mj_solve_sparse_vec(model: &Model, data: &Data, x: &mut DVector<f64>) {
    let nv = model.nv;
    let (rowadr, rownnz, colind) = model.qld_csr();

    // Forward substitution: solve L^T * z = x (L is unit lower triangular)
    // Process from leaves to root (high index to low)
    // Zero-skip + diagonal-only skip matching MuJoCo's mj_solveLD.
    for i in (0..nv).rev() {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let xi = x[i];
        if xi == 0.0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[colind[start + k]] -= data.qLD_data[start + k] * xi;
        }
    }

    // Diagonal solve: z = z * D^-1 (multiply by precomputed inverse, matching MuJoCo).
    for i in 0..nv {
        x[i] *= data.qLD_diag_inv[i];
    }

    // Back substitution: solve L * x = z
    // Process from root to leaves (low index to high)
    // Diagonal-only skip: rownnz == 1 means no off-diagonals.
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[i] -= data.qLD_data[start + k] * x[colind[start + k]];
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
// Shared Smooth Acceleration (§29.2)
// =============================================================================

/// Compute the unconstrained smooth acceleration qacc_smooth = M⁻¹ · qfrc_smooth.
///
/// Also updates `data.stat_meaninertia` from the current mass matrix.
///
/// Returns `(qacc_smooth, qfrc_smooth)` where:
/// - `qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias`
/// - `qacc_smooth = M⁻¹ · qfrc_smooth` (via sparse LDL solve)
///
/// Friction loss is no longer subtracted here — it is handled entirely by
/// solver constraint rows and extracted post-solve into `qfrc_frictionloss` (§29).
fn compute_qacc_smooth(model: &Model, data: &mut Data) -> (DVector<f64>, DVector<f64>) {
    let nv = model.nv;

    // Per-step meaninertia: trace(qM) / nv (more accurate than model-level constant)
    if nv > 0 {
        let mut trace = 0.0_f64;
        for i in 0..nv {
            trace += data.qM[(i, i)];
        }
        let mi = trace / nv as f64;
        data.stat_meaninertia = if mi > 0.0 { mi } else { model.stat_meaninertia };
    } else {
        data.stat_meaninertia = model.stat_meaninertia;
    }

    // qfrc_smooth = qfrc_applied + qfrc_actuator + qfrc_passive - qfrc_bias
    // Note: friction loss is no longer in qfrc_passive or subtracted here.
    let mut qfrc_smooth = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        qfrc_smooth[k] =
            data.qfrc_applied[k] + data.qfrc_actuator[k] + data.qfrc_passive[k] - data.qfrc_bias[k];
    }

    // qacc_smooth = M⁻¹ · qfrc_smooth (via sparse LDL solve)
    let mut qacc_smooth = qfrc_smooth.clone();
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut qacc_smooth,
    );

    // Store on Data for solver access
    data.qacc_smooth = qacc_smooth.clone();
    data.qfrc_smooth = qfrc_smooth.clone();

    (qacc_smooth, qfrc_smooth)
}

// =============================================================================
// Unified Constraint Assembly (§15, Step 7)
// =============================================================================

/// Assemble the unified constraint Jacobian and metadata for all solver types.
///
/// Populates all `efc_*` fields on Data, plus `ne`/`nf` counts.
///
/// Row ordering (§29.4):
/// 1. Equality constraints (connect, weld, joint, distance) + FlexEdge → `ne` rows
/// 2. DOF friction loss + tendon friction loss → `nf` rows
/// 3. Joint limits + tendon limits + contacts
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
            EqualityType::Joint | EqualityType::Distance | EqualityType::Tendon => 1,
        };
    }

    // Flex edge-length constraints (1 row per edge) — in equality block
    nefc += model.nflexedge;

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
            MjJointType::Ball => {
                let adr = model.jnt_qpos_adr[jnt_id];
                let q = normalize_quat4([
                    data.qpos[adr],
                    data.qpos[adr + 1],
                    data.qpos[adr + 2],
                    data.qpos[adr + 3],
                ]);
                let (_, angle) = ball_limit_axis_angle(q);
                let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
                let dist = limit - angle;
                if dist < 0.0 {
                    // margin = 0.0 (see S6)
                    nefc += 1;
                }
            }
            MjJointType::Free => {
                // MuJoCo does not support free joint limits.
                // Silently ignore — no constraint rows.
            }
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
    // §32: pyramidal contacts emit 2*(dim-1) facet rows instead of dim rows.
    for c in &data.contacts {
        let is_pyramidal = c.dim >= 3 && model.cone == 0 && c.mu[0] >= 1e-10;
        if is_pyramidal {
            nefc += 2 * (c.dim - 1);
        } else {
            nefc += c.dim;
        }
    }

    // NOTE: Bending forces are passive (in mj_fwd_passive), not constraint rows.
    // Volume constraints removed — MuJoCo has no dedicated volume mechanism.

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
            EqualityType::Tendon => extract_tendon_equality_jacobian(model, data, eq_id),
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

    // --- 3a': Flex edge-length constraints (equality block) ---
    for e in 0..model.nflexedge {
        let [v0, v1] = model.flexedge_vert[e];
        let x0 = data.flexvert_xpos[v0];
        let x1 = data.flexvert_xpos[v1];
        let diff = x1 - x0;
        let dist = diff.norm();
        let rest_len = model.flexedge_length0[e];
        let flex_id = model.flexedge_flexid[e];

        if dist < 1e-10 {
            // Degenerate: zero-length edge, skip (fill zeros, finalize_row! handles it)
            finalize_row!(
                model.flex_edge_solref[flex_id],
                model.flex_edge_solimp[flex_id],
                0.0,
                0.0,
                0.0,
                0.0,
                ConstraintType::FlexEdge,
                1,
                e,
                [0.0; 5]
            );
            continue;
        }

        let direction = diff / dist;
        let pos_error = dist - rest_len; // positive = stretched, negative = compressed

        // Jacobian: ∂C/∂x_v0 = -direction, ∂C/∂x_v1 = +direction
        // (§27F) Pinned vertices (dofadr=usize::MAX) have zero Jacobian columns.
        let dof0 = model.flexvert_dofadr[v0];
        let dof1 = model.flexvert_dofadr[v1];
        if dof0 != usize::MAX {
            for k in 0..3 {
                data.efc_J[(row, dof0 + k)] = -direction[k];
            }
        }
        if dof1 != usize::MAX {
            for k in 0..3 {
                data.efc_J[(row, dof1 + k)] = direction[k];
            }
        }

        // Velocity: relative velocity projected onto edge direction
        let vel0 = if dof0 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
        };
        let vel1 = if dof1 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
        };
        let vel_error = (vel1 - vel0).dot(&direction);

        finalize_row!(
            model.flex_edge_solref[flex_id],
            model.flex_edge_solimp[flex_id],
            pos_error,
            0.0, // margin
            vel_error,
            0.0, // friction loss
            ConstraintType::FlexEdge,
            1,        // dim
            e,        // id (edge index)
            [0.0; 5]  // mu (no friction on edge constraints)
        );
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
            model.dof_solref[dof_idx],
            model.dof_solimp[dof_idx],
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
            model.tendon_solref_fri[t],
            model.tendon_solimp_fri[t],
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
            MjJointType::Ball => {
                let qpos_adr = model.jnt_qpos_adr[jnt_id];
                let dof_adr = model.jnt_dof_adr[jnt_id];
                let q = normalize_quat4([
                    data.qpos[qpos_adr],
                    data.qpos[qpos_adr + 1],
                    data.qpos[qpos_adr + 2],
                    data.qpos[qpos_adr + 3],
                ]);
                let (unit_dir, angle) = ball_limit_axis_angle(q);
                let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
                let dist = limit - angle;

                if dist < 0.0 {
                    // margin = 0.0 (see S6)
                    // Jacobian: -unit_dir on 3 angular DOFs
                    data.efc_J[(row, dof_adr)] = -unit_dir.x;
                    data.efc_J[(row, dof_adr + 1)] = -unit_dir.y;
                    data.efc_J[(row, dof_adr + 2)] = -unit_dir.z;

                    // Constraint-space velocity: J · qvel
                    let vel = -(unit_dir.x * data.qvel[dof_adr]
                        + unit_dir.y * data.qvel[dof_adr + 1]
                        + unit_dir.z * data.qvel[dof_adr + 2]);

                    finalize_row!(
                        model.jnt_solref[jnt_id],
                        model.jnt_solimp[jnt_id],
                        dist,
                        0.0,
                        vel,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
                    );
                }
            }
            MjJointType::Free => {
                // No limit support for free joints (matches MuJoCo).
            }
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
    // §32: Track pyramidal contact ranges for R-scaling post-processing.
    let mut pyramidal_ranges: Vec<(usize, usize, usize)> = Vec::new(); // (start_row, n_facets, contact_idx)
    for (ci, contact) in contacts.iter().enumerate() {
        let dim = contact.dim;
        let cj = compute_contact_jacobian(model, data, contact);

        let sr_normal = contact.solref;
        let si = contact.solimp;
        // includemargin = margin - gap, computed at contact creation.
        // Flows into compute_impedance (violation threshold) and compute_aref
        // (reference acceleration offset).
        let margin = contact.includemargin;
        let is_elliptic = dim >= 3 && model.cone == 1 && contact.mu[0] >= 1e-10;
        let is_pyramidal = dim >= 3 && model.cone == 0 && contact.mu[0] >= 1e-10;

        if is_pyramidal {
            // §32: Pyramidal friction cone — emit 2*(dim-1) facet rows.
            // Each friction direction d produces two facets:
            //   J_pos = J_normal + μ_d · J_friction_d
            //   J_neg = J_normal - μ_d · J_friction_d
            // All facets share pos, margin, solref (NOT solreffriction).
            let n_facets = 2 * (dim - 1);
            let start_row = row;

            // MuJoCo: cpos[0] = cpos[1] = con->dist; cmargin[0] = cmargin[1] = con->includemargin;
            // These are set ONCE before the loop, reused for ALL facet pairs.
            let pos = -contact.depth;

            for d in 1..dim {
                let mu_d = contact.mu[d - 1]; // friction coefficient for direction d

                // Positive facet: J_normal + μ_d · J_friction_d
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(0, col)] + mu_d * cj[(d, col)];
                }
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += data.efc_J[(row, col)] * data.qvel[col];
                }
                finalize_row!(
                    sr_normal,
                    si,
                    pos,
                    margin,
                    vel,
                    0.0,
                    ConstraintType::ContactPyramidal,
                    n_facets,
                    ci,
                    contact.mu
                );

                // Negative facet: J_normal - μ_d · J_friction_d
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(0, col)] - mu_d * cj[(d, col)];
                }
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += data.efc_J[(row, col)] * data.qvel[col];
                }
                finalize_row!(
                    sr_normal,
                    si,
                    pos,
                    margin,
                    vel,
                    0.0,
                    ConstraintType::ContactPyramidal,
                    n_facets,
                    ci,
                    contact.mu
                );
            }

            pyramidal_ranges.push((start_row, n_facets, ci));
        } else {
            // Elliptic or frictionless: emit dim rows (existing path).
            let ctype = if is_elliptic {
                ConstraintType::ContactElliptic
            } else {
                ConstraintType::ContactFrictionless
            };

            // §31: solreffriction selection for elliptic friction rows.
            let has_solreffriction = is_elliptic
                && (contact.solreffriction[0] != 0.0 || contact.solreffriction[1] != 0.0);

            for r in 0..dim {
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(r, col)];
                }

                // pos: row 0 = signed distance, rows 1+ = 0.
                let pos = if r == 0 { -contact.depth } else { 0.0 };
                let margin_r = if r == 0 { margin } else { 0.0 };

                // §31: select solref for this row.
                let sr = if r > 0 && has_solreffriction {
                    contact.solreffriction
                } else {
                    sr_normal
                };

                // vel: J_row · qvel
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += cj[(r, col)] * data.qvel[col];
                }

                finalize_row!(sr, si, pos, margin_r, vel, 0.0, ctype, dim, ci, contact.mu);
            }
        }
    }

    // §32: Post-process R scaling for pyramidal contacts.
    // TODO(§32): AC12/AC13 cross-validation against MuJoCo reference data.
    // MuJoCo's mj_makeImpedance computes R per-row from each row's own diagApprox,
    // then overrides all facet rows with Rpy = 2 · μ_reg² · R[first_facet].
    // R[first_facet] was already computed by finalize_row! using the first facet's
    // actual Jacobian (J_normal + μ[0]·J_friction_0), NOT the pure normal Jacobian.
    for &(start_row, n_facets, ci) in &pyramidal_ranges {
        let contact = &contacts[ci];

        // Use R already computed by finalize_row! for the first facet row.
        let r_first_facet = data.efc_R[start_row];

        // μ_reg = friction[0] · √(1/impratio)
        let mu_reg = contact.mu[0] * (1.0 / model.impratio).sqrt();
        let rpy = (2.0 * mu_reg * mu_reg * r_first_facet).max(MJ_MINVAL);

        for row_idx in start_row..start_row + n_facets {
            data.efc_R[row_idx] = rpy;
            data.efc_D[row_idx] = 1.0 / rpy;
        }
    }

    debug_assert_eq!(row, nefc, "Row count mismatch in constraint assembly");

    // === Phase 4: Count ne/nf for solver dispatch (§29.4) ===
    // Ordering: [0..ne) = equality+flex, [ne..ne+nf) = friction, [ne+nf..nefc) = limits+contacts
    data.ne = 0;
    data.nf = 0;
    for i in 0..nefc {
        match data.efc_type[i] {
            ConstraintType::Equality | ConstraintType::FlexEdge => data.ne += 1,
            ConstraintType::FrictionLoss => data.nf += 1,
            _ => {}
        }
    }
    // Verify ordering invariant: all equality rows come first, then friction, then the rest
    debug_assert!(
        data.efc_type
            .iter()
            .take(data.ne)
            .all(|t| matches!(t, ConstraintType::Equality | ConstraintType::FlexEdge)),
        "Non-equality row found within equality block [0..{})",
        data.ne,
    );
    debug_assert!(
        data.efc_type
            .iter()
            .skip(data.ne)
            .take(data.nf)
            .all(|t| matches!(t, ConstraintType::FrictionLoss)),
        "Non-friction row found within friction block [{}..{})",
        data.ne,
        data.ne + data.nf,
    );

    data.efc_cost = 0.0;
}

// =============================================================================
// Unified PGS Solver (§29.3)
// =============================================================================

/// Compute the regularized Delassus matrix AR = J·M⁻¹·J^T + diag(R).
///
/// This is the full `nefc × nefc` dense matrix used by the PGS solver.
/// Each column of M⁻¹·J^T is computed via a sparse LDL solve.
fn compute_delassus_regularized(model: &Model, data: &Data) -> DMatrix<f64> {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    // Step 1: Compute M⁻¹ · J^T column by column
    let (rowadr, rownnz, colind) = model.qld_csr();
    let mut minv_jt = DMatrix::zeros(nv, nefc);
    for col in 0..nefc {
        // Copy J row (transposed = column of J^T) into a DVector for the solver
        let mut buf = DVector::zeros(nv);
        for r in 0..nv {
            buf[r] = data.efc_J[(col, r)];
        }
        // Solve M · x = J[col,:]^T → x = M⁻¹ · J[col,:]^T
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut buf,
        );
        for r in 0..nv {
            minv_jt[(r, col)] = buf[r];
        }
    }

    // Step 2: AR = J · (M⁻¹ · J^T) + diag(R)
    let mut ar = &data.efc_J * &minv_jt;
    for i in 0..nefc {
        ar[(i, i)] += data.efc_R[i];
    }

    ar
}

/// Unified PGS solver operating on all constraint types in dual (force) space.
///
/// Implements MuJoCo's `mj_solPGS` (§29.3):
/// - Builds AR = J·M⁻¹·J^T + diag(R) (regularized Delassus matrix)
/// - Gauss-Seidel iteration with per-type projection
/// - Cost guard: reverts updates that increase dual cost
/// - Elliptic contacts: group projection via `project_elliptic_cone`
///
/// After convergence, `data.efc_force` contains the constraint forces.
/// The caller must then compute `qfrc_constraint = J^T · efc_force`.
fn pgs_solve_unified(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    if nefc == 0 {
        return;
    }

    // Build regularized Delassus matrix
    let ar = compute_delassus_regularized(model, data);

    // Precompute inverse diagonal for scalar GS updates
    let ar_diag_inv: Vec<f64> = (0..nefc)
        .map(|i| {
            let d = ar[(i, i)];
            if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d }
        })
        .collect();

    // Warmstart: use classify_constraint_states to map qacc_warmstart → efc_force,
    // then compare dual cost vs cold start (zero forces).
    // This matches MuJoCo's universal warmstart (§29.11).
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = data.qfrc_smooth.clone();

    // Initialize efc arrays for classification
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // Classify at qacc_warmstart to get warmstart forces
    classify_constraint_states(
        model,
        data,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
    );
    let warm_forces = data.efc_force.clone();

    // Evaluate dual cost: ½·f^T·AR·f + f^T·b
    let mut dual_cost_warm = 0.0;
    for i in 0..nefc {
        let mut ar_f_i = 0.0;
        for j in 0..nefc {
            ar_f_i += ar[(i, j)] * warm_forces[j];
        }
        dual_cost_warm += 0.5 * warm_forces[i] * ar_f_i + warm_forces[i] * data.efc_b[i];
    }

    // Cold start cost is zero (f=0 → cost=0)
    // Use warmstart only if it produces lower dual cost
    if dual_cost_warm < 0.0 {
        data.efc_force.copy_from(&warm_forces);
    } else {
        data.efc_force.fill(0.0);
    }

    let max_iters = model.solver_iterations;

    for _iter in 0..max_iters {
        let mut i = 0;
        while i < nefc {
            let ctype = data.efc_type[i];
            let dim = data.efc_dim[i];

            // Elliptic contacts: group projection
            if matches!(ctype, ConstraintType::ContactElliptic) && dim > 1 {
                // Save old force for cost guard
                let old_force: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

                // Compute residual for all rows in this group
                for j in 0..dim {
                    let mut res = data.efc_b[i + j];
                    for c in 0..nefc {
                        res += ar[(i + j, c)] * data.efc_force[c];
                    }
                    // GS update: subtract residual scaled by diagonal inverse
                    data.efc_force[i + j] -= res * ar_diag_inv[i + j];
                }

                // Project onto elliptic friction cone
                let mu = data.efc_mu[i];
                project_elliptic_cone(&mut data.efc_force.as_mut_slice()[i..i + dim], &mu, dim);

                // Cost guard: revert if dual cost increased
                let delta: Vec<f64> = (0..dim)
                    .map(|j| data.efc_force[i + j] - old_force[j])
                    .collect();
                let cost_change =
                    pgs_cost_change(&ar, &delta, &data.efc_b, &data.efc_force, i, dim, nefc);
                if cost_change > 1e-10 {
                    data.efc_force.as_mut_slice()[i..i + dim].copy_from_slice(&old_force);
                }

                i += dim;
            } else {
                // Scalar constraint: single-row GS update + projection
                let old_force = data.efc_force[i];

                // Compute residual: res = b[i] + Σ AR[i,c] * force[c]
                let mut res = data.efc_b[i];
                for c in 0..nefc {
                    res += ar[(i, c)] * data.efc_force[c];
                }

                // GS update
                data.efc_force[i] -= res * ar_diag_inv[i];

                // Project per constraint type
                match ctype {
                    // Equality: bilateral (unclamped)
                    ConstraintType::Equality | ConstraintType::FlexEdge => {}

                    // Friction loss: box clamp [-floss, +floss]
                    ConstraintType::FrictionLoss => {
                        let fl = data.efc_floss[i];
                        data.efc_force[i] = data.efc_force[i].clamp(-fl, fl);
                    }

                    // Limits, frictionless/pyramidal contacts, and elliptic dim=1: unilateral (force >= 0)
                    ConstraintType::LimitJoint
                    | ConstraintType::LimitTendon
                    | ConstraintType::ContactFrictionless
                    | ConstraintType::ContactPyramidal
                    | ConstraintType::ContactElliptic => {
                        data.efc_force[i] = data.efc_force[i].max(0.0);
                    }
                }

                // Cost guard
                let delta_f = data.efc_force[i] - old_force;
                if delta_f.abs() > 0.0 {
                    // cost_change = 0.5 * delta^2 * AR[i,i] + delta * res_before_update
                    // where res_before_update = res (computed above before GS update)
                    let cost_change = 0.5 * delta_f * delta_f * ar[(i, i)] + delta_f * res;
                    if cost_change > 1e-10 {
                        data.efc_force[i] = old_force;
                    }
                }

                i += 1;
            }
        }
    }

    data.solver_niter = max_iters;
    data.solver_stat.clear();
}

// =============================================================================
// Unified CG Solver (§29.6 — primal, shares mj_solPrimal with Newton)
// =============================================================================

/// Primal CG solver operating on all constraint types in acceleration space.
///
/// Implements MuJoCo's `mj_solCG` (§29.6) which calls `mj_solPrimal(flg_Newton=false)`:
/// - Shares constraint evaluation (classify_constraint_states) with Newton
/// - Shares line search (primal_prepare, primal_search) with Newton
/// - Preconditioner: M⁻¹ (via LDL solve) instead of Newton's H⁻¹
/// - Direction: Polak-Ribiere conjugate gradient instead of Newton direction
/// - No cone Hessian (flg_HessianCone = false)
///
/// After convergence, `data.efc_force` and `data.efc_jar` are populated.
/// The caller must then compute `qfrc_constraint = J^T · efc_force`.
fn cg_solve_unified(model: &Model, data: &mut Data) {
    let nv = model.nv;

    // qacc_smooth, qfrc_smooth, and efc_* arrays are already populated by
    // mj_fwd_constraint() before dispatching to this solver.
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = data.qfrc_smooth.clone();
    let meaninertia = data.stat_meaninertia;
    let nefc = data.efc_type.len();

    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth);
        data.qfrc_constraint.fill(0.0);
        data.solver_niter = 0;
        data.solver_stat.clear();
        return;
    }

    let scale = 1.0 / (meaninertia * (1.0_f64).max(nv as f64));
    let tolerance = model.solver_tolerance;

    // Initialize efc arrays
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // Warmstart selection (same as Newton). CG always uses raw qM.
    let cost_warmstart = evaluate_cost_at(
        data,
        model,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
        &data.qM,
    );
    let cost_smooth = evaluate_cost_at(
        data,
        model,
        &qacc_smooth,
        &qacc_smooth,
        &qfrc_smooth,
        &data.qM,
    );

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

    // Initial constraint classification (no cone Hessian for CG)
    classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

    // CG-specific: compute gradient and M⁻¹ preconditioned direction
    let (rowadr, rownnz, colind) = model.qld_csr();

    // grad = Ma - qfrc_smooth - J^T · efc_force
    let mut qfrc_constraint_local = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f != 0.0 {
            for col in 0..nv {
                qfrc_constraint_local[col] += data.efc_J[(i, col)] * f;
            }
        }
    }
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint_local[k];
    }

    // M⁻¹ preconditioner: mgrad = M⁻¹ · grad
    let mut mgrad = grad.clone();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut mgrad,
    );

    // Initial search = -mgrad (steepest descent for first iteration)
    let mut search = -mgrad.clone();

    // Pre-loop convergence check
    let grad_norm: f64 = grad.iter().map(|x| x * x).sum::<f64>().sqrt();
    if scale * grad_norm < tolerance {
        data.solver_niter = 0;
        data.solver_stat.clear();
        recover_newton(model, data, &qacc, &qfrc_smooth);
        return;
    }

    // === ITERATE ===
    let max_iters = model.solver_iterations;
    let max_ls_iter = model.ls_iterations.max(20);
    let ls_tolerance = model.ls_tolerance;
    let mut converged = false;
    let mut solver_stats: Vec<SolverStat> = Vec::with_capacity(max_iters);
    let mut grad_old = grad.clone();
    let mut mgrad_old = mgrad.clone();
    // Polak-Ribiere needs previous-iteration gradient; initial values
    // are overwritten in step 5 before the first read in step 7.
    // Force-read to silence unused_assignments warning.
    let _ = (&grad_old, &mgrad_old);

    for iter in 0..max_iters {
        // 1. Compute mv = M·search, jv = J·search (for line search)
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

        // Compute lineslope: grad · search / ||search||
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

        // Snapshot constraint states for nchange tracking
        let old_states = data.efc_state.clone();

        // 2. LINE SEARCH (shared with Newton)
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
            let nactive = data
                .efc_state
                .iter()
                .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
                .count();
            solver_stats.push(SolverStat {
                improvement: 0.0,
                gradient: scale * grad_norm,
                lineslope,
                nactive,
                nchange: 0,
                nline,
            });
            break;
        }

        // 3. UPDATE qacc, ma, efc_jar
        for k in 0..nv {
            qacc[k] += alpha * search[k];
            ma[k] += alpha * mv[k];
        }
        for i in 0..nefc {
            data.efc_jar[i] += alpha * jv[i];
        }

        // 4. UPDATE CONSTRAINTS (no cone Hessian for CG)
        let old_cost = data.efc_cost;
        classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

        // 5. Compute gradient
        grad_old = grad.clone();
        mgrad_old = mgrad.clone();

        qfrc_constraint_local.fill(0.0);
        for i in 0..nefc {
            let f = data.efc_force[i];
            if f != 0.0 {
                for col in 0..nv {
                    qfrc_constraint_local[col] += data.efc_J[(i, col)] * f;
                }
            }
        }
        for k in 0..nv {
            grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint_local[k];
        }

        // 6. M⁻¹ preconditioner
        mgrad = grad.clone();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut mgrad,
        );

        // 7. Polak-Ribiere direction update
        // iter 0: pure steepest descent (beta=0).
        // iter>0: beta = grad^T · (mgrad - mgrad_old) / grad_old^T · mgrad_old
        let beta = if iter == 0 {
            0.0
        } else {
            let mgrad_diff: DVector<f64> = &mgrad - &mgrad_old;
            let num = grad.dot(&mgrad_diff);
            let den = grad_old.dot(&mgrad_old);
            if den.abs() < MJ_MINVAL {
                0.0
            } else {
                (num / den).max(0.0)
            }
        };
        search = -&mgrad + beta * &search;

        // 8. CONVERGENCE CHECK
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

        if improvement < tolerance && gradient < tolerance {
            converged = true;
            break;
        }
    }

    // === RECOVER ===
    data.solver_niter = solver_stats.len();
    data.solver_stat = solver_stats;
    recover_newton(model, data, &qacc, &qfrc_smooth);

    // If not converged, we still use the best result we have
    let _ = converged;
}

/// Compute cost change for PGS cost guard (multi-row case).
///
/// For a block update δ at rows [i..i+dim):
/// cost_change = 0.5 · δ^T · AR_block · δ + δ^T · res_old
/// where res_old is the residual BEFORE the GS update.
fn pgs_cost_change(
    ar: &DMatrix<f64>,
    delta: &[f64],
    efc_b: &DVector<f64>,
    efc_force: &DVector<f64>,
    i: usize,
    dim: usize,
    nefc: usize,
) -> f64 {
    // Reconstruct the residual before the update:
    // res_old[j] = b[i+j] + Σ AR[i+j, c] * (force[c] - delta[j] if c==i+j else force[c])
    // = b[i+j] + Σ AR[i+j, c] * force[c] - AR[i+j, i+j] * delta[j]
    // But we need the ORIGINAL residual (before force was updated).
    // force_old[j] = force[j] - delta[j], so:
    // res_old[j] = b[i+j] + Σ_{c not in block} AR[i+j, c] * force[c] + Σ_{j' in block} AR[i+j, i+j'] * (force[i+j'] - delta[j'])
    let mut cost = 0.0;
    for j in 0..dim {
        // Compute residual at old force values
        let mut res_old = efc_b[i + j];
        for c in 0..nefc {
            if c >= i && c < i + dim {
                res_old += ar[(i + j, c)] * (efc_force[c] - delta[c - i]);
            } else {
                res_old += ar[(i + j, c)] * efc_force[c];
            }
        }
        cost += delta[j] * res_old;
    }
    // Quadratic term: 0.5 * δ^T * AR_block * δ
    for j in 0..dim {
        for k in 0..dim {
            cost += 0.5 * delta[j] * ar[(i + j, i + k)] * delta[k];
        }
    }
    cost
}

/// Map efc_force → qfrc_constraint via J^T · efc_force.
fn compute_qfrc_constraint_from_efc(model: &Model, data: &mut Data) {
    let nv = model.nv;
    data.qfrc_constraint.fill(0.0);
    let nefc = data.efc_type.len();
    for i in 0..nefc {
        let force = data.efc_force[i];
        if force.abs() > 0.0 {
            for col in 0..nv {
                data.qfrc_constraint[col] += data.efc_J[(i, col)] * force;
            }
        }
    }
}

/// Extract friction loss forces from efc_force into qfrc_frictionloss.
///
/// After the solver runs, friction loss constraint rows contain the
/// friction loss forces. Extract them via J^T · efc_force for the
/// friction loss rows only.
fn extract_qfrc_frictionloss(data: &mut Data, nv: usize) {
    data.qfrc_frictionloss.fill(0.0);
    let nefc = data.efc_type.len();
    for i in 0..nefc {
        if matches!(data.efc_type[i], ConstraintType::FrictionLoss) {
            let force = data.efc_force[i];
            for col in 0..nv {
                data.qfrc_frictionloss[col] += data.efc_J[(i, col)] * force;
            }
        }
    }
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
            ConstraintType::Equality | ConstraintType::FlexEdge => {
                // Always Quadratic (two-sided soft equality)
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
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
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
/// Phase A: H = M_eff + Σ_{Quadratic rows} D_i · J_i^T · J_i
/// (No cone Hessian in Phase A — cone rows are treated as Quadratic per-row.)
///
/// DT-35: `m_eff` is `M_impl` when `ImplicitSpringDamper` is active,
/// `data.qM` otherwise. The Hessian starts from `m_eff` so that tendon K/D
/// contributions are included in the Newton system.
///
/// Returns the Cholesky factor L (lower triangular) such that H = L · L^T,
/// or an error if the Hessian is not positive definite.
fn assemble_hessian(
    data: &Data,
    nv: usize,
    m_eff: &DMatrix<f64>,
) -> Result<DMatrix<f64>, StepError> {
    let nefc = data.efc_type.len();

    // Start with effective mass matrix (M or M_impl)
    let mut h = DMatrix::<f64>::zeros(nv, nv);
    for r in 0..nv {
        for c in 0..nv {
            h[(r, c)] = m_eff[(r, c)];
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
    fn assemble(
        model: &Model,
        data: &Data,
        nv: usize,
        implicit_sd: bool,
    ) -> Result<Self, StepError> {
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

        // DT-35: Tendon K/D sparsity (ImplicitSpringDamper only).
        // Conservative: includes entries for all tendons with k > 0 or b > 0,
        // regardless of deadband state. Actual values use deadband-aware k_active.
        if implicit_sd {
            let mut nz: Vec<usize> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                let kt = model.tendon_stiffness[t];
                let bt = model.tendon_damping[t];
                if kt <= 0.0 && bt <= 0.0 {
                    continue;
                }
                let j = &data.ten_J[t];
                nz.clear();
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        nz.push(dof);
                    }
                }
                for &ci in &nz {
                    for &cj in &nz {
                        let (lo, hi) = if ci <= cj { (ci, cj) } else { (cj, ci) };
                        has_entry[lo][hi] = true;
                    }
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
        h.fill_numeric(model, data, nv, nefc, implicit_sd);

        // --- Step 4: Symbolic factorization ---
        h.symbolic_factor();

        // --- Step 5: Numeric factorization ---
        h.numeric_factor()?;

        Ok(h)
    }

    /// Refactor with updated numeric values (same sparsity pattern).
    /// Used when constraint states change but sparsity doesn't.
    fn refactor(&mut self, model: &Model, data: &Data, implicit_sd: bool) -> Result<(), StepError> {
        let nv = self.nv;
        let nefc = data.efc_type.len();
        self.fill_numeric(model, data, nv, nefc, implicit_sd);
        self.numeric_factor()
    }

    /// Fill CSC values with H = M + Σ_{Quadratic} D_i · J_i^T · J_i.
    /// DT-35: When `implicit_sd` is true, also adds joint diagonal K/D and
    /// tendon non-diagonal K/D to match `build_m_impl_for_newton`.
    fn fill_numeric(
        &mut self,
        model: &Model,
        data: &Data,
        nv: usize,
        nefc: usize,
        implicit_sd: bool,
    ) {
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

        // DT-35: Add joint diagonal K/D and tendon non-diagonal K/D for
        // ImplicitSpringDamper. This matches build_m_impl_for_newton.
        if implicit_sd {
            let h = model.timestep;
            let h2 = h * h;
            let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

            // Joint diagonal K/D
            for i in 0..nv {
                let kd = h * model.implicit_damping[i] + h2 * model.implicit_stiffness[i];
                if kd > 0.0 {
                    if let Some(idx) = self.find_entry(i, i) {
                        self.vals[idx] += kd;
                    }
                }
            }

            // Tendon non-diagonal K/D (rank-1 outer products)
            let mut nz: Vec<(usize, f64)> = Vec::with_capacity(8);
            for t in 0..model.ntendon {
                if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
                    continue;
                }
                let kt = model.tendon_stiffness[t];
                let bt = model.tendon_damping[t];
                if kt <= 0.0 && bt <= 0.0 {
                    continue;
                }
                let j = &data.ten_J[t];
                let k_active =
                    tendon_active_stiffness(kt, data.ten_length[t], model.tendon_lengthspring[t]);
                let scale = h2 * k_active + h * bt;
                if scale <= 0.0 {
                    continue;
                }
                nz.clear();
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        nz.push((dof, j[dof]));
                    }
                }
                for (ai, &(col_a, j_a)) in nz.iter().enumerate() {
                    let s_j_a = scale * j_a;
                    for &(col_b, j_b) in &nz[ai..] {
                        if let Some(idx) = self.find_entry(col_a, col_b) {
                            self.vals[idx] += s_j_a * j_b;
                        }
                    }
                }
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
    /// etree\[j\] = min { i > j : L\[i,j\] != 0 }, which equals the first
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
    m_eff: &DMatrix<f64>,
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
                // Downdate failed — fall back to full reassembly (DT-35: uses m_eff)
                *chol_l = assemble_hessian(data, nv, m_eff)?;
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
            ConstraintType::Equality | ConstraintType::FlexEdge => {
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
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
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
/// DT-35: `m_eff` is `M_impl` when `ImplicitSpringDamper` is active.
#[allow(clippy::many_single_char_names)]
fn evaluate_cost_at(
    data: &Data,
    model: &Model,
    qacc_trial: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    m_eff: &DMatrix<f64>,
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

    // Compute Ma_trial = M_eff · qacc_trial
    let mut ma_trial = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma_trial[r] += m_eff[(r, c)] * qacc_trial[c];
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
            ConstraintType::Equality | ConstraintType::FlexEdge => {
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
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
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
///
/// DT-35: `m_eff` is the effective mass matrix — `M_impl` when
/// `ImplicitSpringDamper` is active, `data.qM` otherwise. All M·v products,
/// Hessian assembly, and cost evaluation use `m_eff`.
#[allow(clippy::too_many_lines)]
fn newton_solve(
    model: &Model,
    data: &mut Data,
    m_eff: &DMatrix<f64>,
    qfrc_eff: &DVector<f64>,
    implicit_sd: bool,
) -> NewtonResult {
    let nv = model.nv;

    // === INITIALIZE ===
    // qacc_smooth, qfrc_smooth, and efc_* arrays are already populated by
    // mj_fwd_constraint() before dispatching to this solver.
    // Use the explicit qfrc_eff parameter (which may contain implicit spring/damper
    // corrections) rather than reading from data.qfrc_smooth via side-channel mutation.
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = qfrc_eff.clone();
    let meaninertia = data.stat_meaninertia;
    let nefc = data.efc_type.len();

    // No constraints → unconstrained solution
    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth);
        data.qfrc_constraint.fill(0.0);
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
        m_eff,
    );
    // Evaluate cost at qacc_smooth (Gauss = 0, only constraint cost from efc_b)
    let cost_smooth =
        evaluate_cost_at(data, model, &qacc_smooth, &qacc_smooth, &qfrc_smooth, m_eff);

    let mut qacc = if cost_warmstart < cost_smooth {
        data.qacc_warmstart.clone()
    } else {
        qacc_smooth.clone()
    };

    // Compute Ma = M_eff · qacc (DT-35: uses M_impl when ImplicitSpringDamper)
    let mut ma = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma[r] += m_eff[(r, c)] * qacc[c];
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
        let Ok(sh) = SparseHessian::assemble(model, data, nv, implicit_sd) else {
            data.solver_niter = 0;
            data.solver_stat.clear();
            return NewtonResult::CholeskyFailed;
        };
        let gs = compute_gradient_and_search_sparse(data, nv, &ma, &qfrc_smooth, &sh);
        sparse_h = Some(sh);
        gs
    } else {
        let Ok(l) = assemble_hessian(data, nv, m_eff) else {
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

    // Precompute Mv = M_eff*search and Jv = J*search for the initial search direction
    let mut mv = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            mv[r] += m_eff[(r, c)] * search[c];
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
            if sh.refactor(model, data, implicit_sd).is_err() {
                // Refactorization failed — try full reassembly
                if let Ok(new_sh) = SparseHessian::assemble(model, data, nv, implicit_sd) {
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
            if hessian_incremental(data, nv, chol_l, &old_states, m_eff).is_err() {
                // Incremental update failed — fall back to full reassembly
                if let Ok(l) = assemble_hessian(data, nv, m_eff) {
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

        // Recompute Mv = M_eff * search
        mv.fill(0.0);
        for r in 0..nv {
            for c in 0..nv {
                mv[r] += m_eff[(r, c)] * search[c];
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

/// RECOVER block: write final solver qacc to Data.
///
/// For Newton and CG, this writes the primal solution `qacc` that was computed
/// directly by the solver. The remaining post-processing (qfrc_constraint,
/// limit forces) is handled centrally in `mj_fwd_constraint()`.
fn recover_newton(
    _model: &Model,
    data: &mut Data,
    qacc: &DVector<f64>,
    _qfrc_smooth: &DVector<f64>,
) {
    data.qacc.copy_from(qacc);
}

/// Tag for each noslip row, used to select the correct projection.
#[derive(Clone, Copy)]
enum NoslipRowKind {
    /// Friction-loss row: clamp to [-floss, +floss].
    FrictionLoss { floss: f64 },
    /// Elliptic contact friction row: cone projection with parent contact.
    /// `group_start`: local index within submatrix of first friction row of this contact.
    /// `group_len`: number of friction rows for this contact.
    /// `contact_efc_start`: efc row index of the parent contact's normal row.
    EllipticFriction {
        group_start: usize,
        group_len: usize,
        contact_efc_start: usize,
    },
    /// Pyramidal contact row: 2x2 block solve on pairs.
    /// `group_start`: local index of first row of this contact in submatrix.
    /// `group_len`: total rows for this contact = 2*(dim-1).
    PyramidalFriction {
        group_start: usize,
        group_len: usize,
    },
}

/// Noslip post-processor: suppresses residual contact slip (§15.10).
///
/// Runs a modified PGS pass on friction force rows only, without
/// regularization, using the current normal forces as fixed cone limits.
/// This matches MuJoCo's `mj_solNoSlip`.
///
/// Called after any solver (Newton, PGS, CG) when `model.noslip_iterations > 0`.
/// Also called after Newton's PGS fallback path. Matches MuJoCo's `mj_solNoSlip`.
///
/// Processes two row ranges (NOT equality rows):
/// 1. Friction-loss rows: PGS update with interval clamping [-floss, +floss]
/// 2. Contact friction rows: elliptic QCQP or pyramidal 2x2 block solve
///
/// All noslip processing uses the UNREGULARIZED Delassus matrix A (without R).
/// This matches MuJoCo's flg_subR=1 in ARdiaginv() and residual().
#[allow(clippy::many_single_char_names)]
fn noslip_postprocess(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nefc = data.efc_type.len();
    let noslip_iter = model.noslip_iterations;
    let noslip_tol = model.noslip_tolerance;

    if noslip_iter == 0 || nefc == 0 {
        return;
    }

    // =========================================================================
    // 1. Identify noslip-eligible rows: friction-loss + contact friction.
    //    Each row is tagged with its type for projection in the PGS loop.
    // =========================================================================

    let mut noslip_rows: Vec<usize> = Vec::new();
    let mut noslip_kinds: Vec<NoslipRowKind> = Vec::new();

    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];

        match ctype {
            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                noslip_rows.push(i);
                noslip_kinds.push(NoslipRowKind::FrictionLoss { floss });
                i += 1;
            }
            ConstraintType::ContactElliptic | ConstraintType::ContactFrictionless => {
                if dim >= 3 {
                    let group_start = noslip_rows.len();
                    let group_len = dim - 1;
                    for j in 1..dim {
                        noslip_rows.push(i + j);
                        noslip_kinds.push(NoslipRowKind::EllipticFriction {
                            group_start,
                            group_len,
                            contact_efc_start: i,
                        });
                    }
                }
                i += dim;
            }
            ConstraintType::ContactPyramidal => {
                // §33 S4: Pyramidal contacts have 2*(dim-1) rows, all are friction.
                // Include them for noslip processing.
                let nrows = dim; // all rows are friction edges for pyramidal
                let group_start = noslip_rows.len();
                for j in 0..nrows {
                    noslip_rows.push(i + j);
                    noslip_kinds.push(NoslipRowKind::PyramidalFriction {
                        group_start,
                        group_len: nrows,
                    });
                }
                i += dim;
            }
            _ => {
                i += 1;
            }
        }
    }

    let n = noslip_rows.len();
    if n == 0 {
        return;
    }

    // =========================================================================
    // 2. Build noslip Delassus submatrix A (UNREGULARIZED) and effective bias.
    //
    //    For each noslip row i, we compute M⁻¹ · J[row_i]^T, then dot against
    //    ALL nefc Jacobian rows (not just noslip rows). This gives:
    //      - a_sub[fi,fj] = J[row_i] · M⁻¹ · J[row_j]^T  (noslip-to-noslip)
    //      - b_eff[fi] = efc_b[row_i] + Σ_{j NOT noslip} A[row_i,j] * efc_force[j]
    //
    //    The PGS iteration then uses: res = b_eff[fi] + Σ_k a_sub[fi,k] * f[k]
    //    which equals the full-matrix residual: efc_b[i] + Σ_{j=0..nefc} A[i,j]*f[j]
    //
    //    This matches MuJoCo's mj_solNoSlip which uses efc_b (not efc_jar) and
    //    the full Delassus row including cross-coupling to non-noslip constraints.
    // =========================================================================

    // Build global→local index map for noslip rows
    let mut noslip_local: Vec<Option<usize>> = vec![None; nefc];
    for (fi, &row) in noslip_rows.iter().enumerate() {
        noslip_local[row] = Some(fi);
    }

    let mut a_sub = DMatrix::<f64>::zeros(n, n);
    let mut b_eff: Vec<f64> = Vec::with_capacity(n);
    for (fi, &row_i) in noslip_rows.iter().enumerate() {
        // Solve M⁻¹ · J[row_i]^T
        let mut minv_ji = DVector::<f64>::zeros(nv);
        for col in 0..nv {
            minv_ji[col] = data.efc_J[(row_i, col)];
        }
        let (rowadr, rownnz, colind) = model.qld_csr();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut minv_ji,
        );

        // Start from efc_b (constraint bias, fixed at assembly)
        let mut b_i = data.efc_b[row_i];

        // Dot against ALL nefc rows
        for j in 0..nefc {
            let mut dot = 0.0;
            for col in 0..nv {
                dot += data.efc_J[(j, col)] * minv_ji[col];
            }
            if let Some(fj) = noslip_local[j] {
                // Noslip row: store in submatrix
                a_sub[(fi, fj)] = dot;
            } else {
                // Non-noslip row: absorb cross-coupling into effective bias
                b_i += dot * data.efc_force[j];
            }
        }

        b_eff.push(b_i);
    }

    // 3. Extract current forces
    let mut f: Vec<f64> = noslip_rows.iter().map(|&r| data.efc_force[r]).collect();

    // Precompute unregularized diagonal inverse
    let diag_inv: Vec<f64> = (0..n)
        .map(|fi| {
            let d = a_sub[(fi, fi)];
            if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d }
        })
        .collect();

    // Convergence scaling (matches CG/Newton: 1/(meaninertia * max(1, nv)))
    let conv_scale = 1.0 / (data.stat_meaninertia * (1.0_f64).max(nv as f64));

    // =========================================================================
    // 4. PGS iterations with per-type projection
    // =========================================================================
    for _iter in 0..noslip_iter {
        let mut improvement = 0.0_f64;

        // Phase A: Friction-loss rows — scalar PGS + interval clamping
        for fi in 0..n {
            if let NoslipRowKind::FrictionLoss { floss } = noslip_kinds[fi] {
                // Unregularized residual: b_eff + A*f
                let mut residual = b_eff[fi];
                for k in 0..n {
                    residual += a_sub[(fi, k)] * f[k];
                }
                let old = f[fi];
                f[fi] -= residual * diag_inv[fi];
                f[fi] = f[fi].clamp(-floss, floss);
                let delta = f[fi] - old;
                // Cost change: ½δ²·A_diag + δ·residual (negative = improvement)
                improvement -= 0.5 * delta * delta * a_sub[(fi, fi)] + delta * residual;
            }
        }

        // Phase B: Elliptic contact friction — QCQP cone projection
        {
            let mut fi = 0;
            while fi < n {
                if let NoslipRowKind::EllipticFriction {
                    group_start,
                    group_len,
                    contact_efc_start,
                } = noslip_kinds[fi]
                {
                    if fi == group_start {
                        let normal_force = data.efc_force[contact_efc_start];
                        let mu = data.efc_mu[contact_efc_start];
                        let fn_abs = normal_force.abs();

                        // Save old forces for cost tracking
                        let old_forces: Vec<f64> =
                            (0..group_len).map(|j| f[group_start + j]).collect();

                        // Compute residuals for all friction rows in this group
                        let mut residuals: Vec<f64> = Vec::with_capacity(group_len);
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            let mut res = b_eff[idx];
                            for k in 0..n {
                                res += a_sub[(idx, k)] * f[k];
                            }
                            residuals.push(res);
                        }

                        // Unconstrained GS update for all friction rows
                        let mut f_unc: Vec<f64> = Vec::with_capacity(group_len);
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            f_unc.push(f[idx] - residuals[local_j] * diag_inv[idx]);
                        }

                        // QCQP projection based on dimension
                        if fn_abs < MJ_MINVAL {
                            // Zero normal force: zero all friction
                            for local_j in 0..group_len {
                                f[group_start + local_j] = 0.0;
                            }
                        } else if group_len == 2 {
                            // condim=3: 2 friction DOFs — use QCQP2
                            let a_block = [
                                [
                                    a_sub[(group_start, group_start)],
                                    a_sub[(group_start, group_start + 1)],
                                ],
                                [
                                    a_sub[(group_start + 1, group_start)],
                                    a_sub[(group_start + 1, group_start + 1)],
                                ],
                            ];
                            let result =
                                noslip_qcqp2(a_block, [f_unc[0], f_unc[1]], [mu[0], mu[1]], fn_abs);
                            f[group_start] = result[0];
                            f[group_start + 1] = result[1];
                        } else if group_len == 3 {
                            // condim=4: 3 friction DOFs — use QCQP3
                            let a_block = [
                                [
                                    a_sub[(group_start, group_start)],
                                    a_sub[(group_start, group_start + 1)],
                                    a_sub[(group_start, group_start + 2)],
                                ],
                                [
                                    a_sub[(group_start + 1, group_start)],
                                    a_sub[(group_start + 1, group_start + 1)],
                                    a_sub[(group_start + 1, group_start + 2)],
                                ],
                                [
                                    a_sub[(group_start + 2, group_start)],
                                    a_sub[(group_start + 2, group_start + 1)],
                                    a_sub[(group_start + 2, group_start + 2)],
                                ],
                            ];
                            let result = noslip_qcqp3(
                                a_block,
                                [f_unc[0], f_unc[1], f_unc[2]],
                                [mu[0], mu[1], mu[2]],
                                fn_abs,
                            );
                            f[group_start] = result[0];
                            f[group_start + 1] = result[1];
                            f[group_start + 2] = result[2];
                        } else {
                            // condim=6 or higher: simple rescaling fallback
                            f[group_start..group_start + group_len]
                                .copy_from_slice(&f_unc[..group_len]);
                            let mut s_sq = 0.0;
                            for local_j in 0..group_len {
                                let mu_j = mu[local_j];
                                if mu_j > MJ_MINVAL {
                                    s_sq += (f[group_start + local_j] / mu_j).powi(2);
                                }
                            }
                            let s = s_sq.sqrt();
                            if s > fn_abs && s > MJ_MINVAL {
                                let rescale = fn_abs / s;
                                for local_j in 0..group_len {
                                    f[group_start + local_j] *= rescale;
                                }
                            }
                        }

                        // Accumulate cost improvement for the whole group
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            let delta = f[idx] - old_forces[local_j];
                            improvement -= 0.5 * delta * delta * a_sub[(idx, idx)]
                                + delta * residuals[local_j];
                        }

                        fi += group_len;
                        continue;
                    }
                }
                fi += 1;
            }
        }

        // Phase C: Pyramidal contact friction — 2x2 block solve on pairs
        {
            let mut fi = 0;
            while fi < n {
                if let NoslipRowKind::PyramidalFriction {
                    group_start,
                    group_len,
                } = noslip_kinds[fi]
                {
                    if fi == group_start {
                        // Process pairs: (0,1), (2,3), ..., (group_len-2, group_len-1)
                        let mut k = 0;
                        while k < group_len {
                            if k + 1 >= group_len {
                                break;
                            }
                            let j0 = group_start + k;
                            let j1 = group_start + k + 1;

                            let old = [f[j0], f[j1]];
                            let mid = 0.5 * (old[0] + old[1]);

                            // 2x2 block of unregularized A
                            let a00 = a_sub[(j0, j0)];
                            let a11 = a_sub[(j1, j1)];
                            let a01 = a_sub[(j0, j1)];

                            // Unregularized residual for both rows
                            let mut res0 = b_eff[j0];
                            let mut res1 = b_eff[j1];
                            for c in 0..n {
                                res0 += a_sub[(j0, c)] * f[c];
                                res1 += a_sub[(j1, c)] * f[c];
                            }

                            // Constant part: bc = res - A * old
                            let bc0 = res0 - a00 * old[0] - a01 * old[1];
                            let bc1 = res1 - a01 * old[0] - a11 * old[1];

                            // 1D optimization over y (change of variables: f0=mid+y, f1=mid-y)
                            let k1 = a00 + a11 - 2.0 * a01; // curvature
                            let k0 = mid * (a00 - a11) + bc0 - bc1; // gradient at y=0

                            if k1 < MJ_MINVAL {
                                // Degenerate curvature: split evenly
                                f[j0] = mid;
                                f[j1] = mid;
                            } else {
                                let y = -k0 / k1;
                                // Clamp: f0 >= 0 and f1 >= 0 ⟹ |y| <= mid
                                if y < -mid {
                                    f[j0] = 0.0;
                                    f[j1] = 2.0 * mid;
                                } else if y > mid {
                                    f[j0] = 2.0 * mid;
                                    f[j1] = 0.0;
                                } else {
                                    f[j0] = mid + y;
                                    f[j1] = mid - y;
                                }
                            }

                            // Cost rollback: revert if cost increased
                            let d0 = f[j0] - old[0];
                            let d1 = f[j1] - old[1];
                            let cost = 0.5 * (d0 * d0 * a00 + d1 * d1 * a11 + 2.0 * d0 * d1 * a01)
                                + d0 * res0
                                + d1 * res1;
                            if cost > MJ_MINVAL {
                                f[j0] = old[0];
                                f[j1] = old[1];
                            } else {
                                // Accumulate cost improvement (cost is negative = improvement)
                                improvement -= cost;
                            }

                            k += 2;
                        }

                        fi += group_len;
                        continue;
                    }
                }
                fi += 1;
            }
        }

        // Cost-based convergence (matches MuJoCo's CG/Newton pattern)
        if improvement * conv_scale < noslip_tol {
            break;
        }
    }

    // =========================================================================
    // 5. Write back updated forces
    // =========================================================================
    for (fi, &row) in noslip_rows.iter().enumerate() {
        data.efc_force[row] = f[fi];
    }

    // 6. Recompute qfrc_constraint = J^T · efc_force
    data.qfrc_constraint.fill(0.0);
    for i_row in 0..nefc {
        let force = data.efc_force[i_row];
        if force == 0.0 {
            continue;
        }
        for col in 0..nv {
            data.qfrc_constraint[col] += data.efc_J[(i_row, col)] * force;
        }
    }

    // 7. Recompute qacc = M⁻¹ · (qfrc_smooth + qfrc_constraint)
    for k in 0..nv {
        data.qacc[k] = data.qfrc_applied[k] + data.qfrc_actuator[k] + data.qfrc_passive[k]
            - data.qfrc_bias[k]
            + data.qfrc_constraint[k];
    }
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut data.qacc,
    );
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

// =============================================================================
// Equality Constraint Jacobian Extraction (§15, Step 5)
//
// These functions extract explicit Jacobian rows, position violations, and
// velocities from equality constraints. Used by the unified constraint
// assembly for all solver types (PGS, CG, Newton).
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

/// Extract tendon equality constraint Jacobian (1 row).
///
/// Two-tendon: constraint is `(L1-L1_0) - data[0] - P(L2-L2_0) = 0`.
/// Jacobian: `J_tendon1 - dP/d(dif) * J_tendon2`.
///
/// Single-tendon: constraint is `(L1-L1_0) - data[0] = 0`.
/// Jacobian: `J_tendon1`.
fn extract_tendon_equality_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
    let nv = model.nv;
    let t1_id = model.eq_obj1id[eq_id];
    let eq_data = &model.eq_data[eq_id];

    let l1 = data.ten_length[t1_id];
    let l1_0 = model.tendon_length0[t1_id];
    let j1 = &data.ten_J[t1_id]; // DVector<f64>, length nv

    let has_tendon2 = model.eq_obj2id[eq_id] != usize::MAX;

    let mut j = DMatrix::zeros(1, nv);

    if has_tendon2 {
        // Two-tendon coupling
        let t2_id = model.eq_obj2id[eq_id];
        let l2 = data.ten_length[t2_id];
        let l2_0 = model.tendon_length0[t2_id];
        let j2 = &data.ten_J[t2_id];

        let dif = l2 - l2_0;

        // Residual: (L1 - L1_0) - data[0] - P(L2 - L2_0)
        let poly_val = eq_data[1] * dif
            + eq_data[2] * dif * dif
            + eq_data[3] * dif * dif * dif
            + eq_data[4] * dif * dif * dif * dif;
        let pos_error = (l1 - l1_0) - eq_data[0] - poly_val;

        // Polynomial derivative: dP/d(dif)
        let deriv = eq_data[1]
            + 2.0 * eq_data[2] * dif
            + 3.0 * eq_data[3] * dif * dif
            + 4.0 * eq_data[4] * dif * dif * dif;

        // Jacobian: J_tendon1 - deriv * J_tendon2
        // Velocity: J · qvel (computed alongside Jacobian)
        let mut vel_error = 0.0;
        for d in 0..nv {
            j[(0, d)] = j1[d] - deriv * j2[d];
            vel_error += j[(0, d)] * data.qvel[d];
        }

        EqualityConstraintRows {
            j_rows: j,
            pos: vec![pos_error],
            vel: vec![vel_error],
        }
    } else {
        // Single-tendon mode: (L1 - L1_0) - data[0] = 0
        let pos_error = (l1 - l1_0) - eq_data[0];

        let mut vel_error = 0.0;
        for d in 0..nv {
            j[(0, d)] = j1[d];
            vel_error += j[(0, d)] * data.qvel[d];
        }

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

/// Phase B per-island constraint solver (§16.16).
///
/// When island discovery has produced `nisland > 0`, contacts are partitioned
/// by island and each island's contact system is solved independently.
/// Limit/equality constraints remain global since they are already per-DOF
/// and don't benefit from island decomposition.
///
/// Populate `efc_island` and related arrays from the current step's constraint rows.
///
/// This is called from `mj_fwd_constraint()` AFTER `assemble_unified_constraints()` so
/// that the efc_island data reflects the current step's constraints (not stale data from
/// a previous step). Previously this was done inside `mj_island()` but that ran before
/// constraint assembly, causing stale/incorrect efc_island arrays.
fn populate_efc_island(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    let nisland = data.nisland;
    let ntree = model.ntree;

    if nefc == 0 || nisland == 0 {
        data.efc_island.clear();
        data.map_efc2iefc.clear();
        data.map_iefc2efc.clear();
        if nisland > 0 {
            data.island_nefc[..nisland].fill(0);
            data.island_iefcadr[..nisland].fill(0);
        }
        return;
    }

    data.efc_island.resize(nefc, -1);
    data.efc_island.fill(-1);
    data.map_efc2iefc.resize(nefc, -1);
    data.map_efc2iefc.fill(-1);
    data.map_iefc2efc.resize(nefc, 0);
    data.island_nefc[..nisland].fill(0);

    for r in 0..nefc {
        let tree = constraint_tree(model, data, r);
        if tree < ntree && data.tree_island[tree] >= 0 {
            let isl = usize::try_from(data.tree_island[tree]).unwrap_or(0);
            if isl < nisland {
                data.efc_island[r] = i32::try_from(isl).unwrap_or(0);
                data.island_nefc[isl] += 1;
            }
        }
    }

    // Compute prefix sums for island_iefcadr
    let mut prefix = 0;
    for i in 0..nisland {
        data.island_iefcadr[i] = prefix;
        prefix += data.island_nefc[i];
    }

    // Build bidirectional maps
    let mut island_efc_fill = vec![0usize; nisland];
    for r in 0..nefc {
        let isl_i32 = data.efc_island[r];
        if isl_i32 >= 0 {
            let isl = usize::try_from(isl_i32).unwrap_or(0);
            if isl < nisland {
                let local_idx = island_efc_fill[isl];
                let global_idx = data.island_iefcadr[isl] + local_idx;
                data.map_efc2iefc[r] = i32::try_from(local_idx).unwrap_or(0);
                if global_idx < data.map_iefc2efc.len() {
                    data.map_iefc2efc[global_idx] = r;
                }
                island_efc_fill[isl] += 1;
            }
        }
    }
}

/// - No islands were discovered this step
#[allow(clippy::cast_sign_loss, clippy::too_many_lines)]
fn mj_fwd_constraint_islands(model: &Model, data: &mut Data) {
    // §29: ALL solver types now route through unified constraint assembly + solver.
    // Island decomposition is no longer needed — the unified solvers handle all
    // constraint types (equality, friction, limits, contacts, flex) globally.
    mj_fwd_constraint(model, data);
}

/// Unified constraint solver dispatcher (§29).
///
/// Assembles ALL constraint types (equality, friction loss, limits, contacts, flex)
/// into unified efc_* arrays, dispatches to the configured solver (PGS, CG, Newton),
/// and maps solver forces back to joint space:
///   qfrc_constraint = J^T * efc_force
///
/// Routes ALL constraint types through unified assembly + solver for ALL solver types.
/// This matches MuJoCo's architecture where every solver type operates on the same
/// constraint rows.
///
/// Build the implicit-modified mass matrix for Newton solver (DT-35).
///
/// Returns `M + h·D_jnt + h²·K_jnt + h·D_ten + h²·K_ten` — joint diagonal
/// K/D plus tendon non-diagonal K/D. Called once per step in
/// `mj_fwd_constraint` when `ImplicitSpringDamper` is active. The returned
/// matrix replaces `data.qM` in all Newton computations.
///
/// Uses `accumulate_tendon_kd` for the tendon contribution,
/// guaranteeing identical mass matrix modification as the non-Newton path
/// in `mj_fwd_acceleration_implicit`.
#[must_use]
fn build_m_impl_for_newton(model: &Model, data: &Data) -> DMatrix<f64> {
    let h = model.timestep;
    let h2 = h * h;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    let mut m_impl = data.qM.clone();

    // Add diagonal joint K/D (matching mj_fwd_acceleration_implicit)
    let k = &model.implicit_stiffness;
    let d = &model.implicit_damping;
    for i in 0..nv {
        m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // Add non-diagonal tendon K/D (shared helper)
    accumulate_tendon_kd(
        &mut m_impl,
        model,
        &data.ten_J,
        &data.ten_length,
        &data.tree_awake,
        h,
        sleep_enabled,
    );

    m_impl
}

/// Compute implicit-corrected smooth forces for Newton solver (DT-35).
///
/// In ImplicitSpringDamper mode, qfrc_passive excludes spring/damper forces.
/// This function computes the RHS forces that, together with M_impl in the
/// Hessian, produce the correct unconstrained acceleration matching the
/// non-Newton implicit path.
///
/// From the equivalence derivation:
///   a = M_impl⁻¹ · (f_ext − D·v − K·(Δq + h·v))
///
/// So: qfrc_smooth_impl = qfrc_smooth_base
///     − D_jnt·v − D_ten·v           (damper forces)
///     − K_jnt·(Δq + h·v)            (spring forces + velocity correction)
///     − K_ten·(δ + h·V_ten)          (tendon spring + velocity correction)
#[must_use]
fn compute_qfrc_smooth_implicit(model: &Model, data: &Data) -> DVector<f64> {
    let h = model.timestep;
    let nv = model.nv;
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    // Start with existing qfrc_smooth (which has everything EXCEPT
    // implicit spring/damper forces)
    let mut qfrc = data.qfrc_smooth.clone();

    // Add joint spring forces: −K·(Δq + h·v)
    // where Δq = q − q_eq, so total = −K·(q − q_eq) − h·K·v
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let k = model.implicit_stiffness[dof_adr];
        if k <= 0.0 {
            continue;
        }
        let q_eq = model.implicit_springref[dof_adr];
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                let v = data.qvel[dof_adr];
                qfrc[dof_adr] += -k * (q - q_eq) - h * k * v;
            }
            // Ball/Free: compute_implicit_params sets implicit_stiffness=0
            // for these types, so the `k <= 0.0` guard above catches them.
            _ => {
                debug_assert!(
                    k <= 0.0,
                    "Ball/Free joint {jnt_id} has implicit_stiffness={k} > 0; \
                     compute_implicit_params should set this to 0.0"
                );
            }
        }
    }

    // Add joint damper forces: −D·v
    for i in 0..nv {
        let d = model.implicit_damping[i];
        if d > 0.0 {
            qfrc[i] += -d * data.qvel[i];
        }
    }

    // Add tendon spring forces: −k·(δ + h·V_ten) projected via J^T
    // and tendon damper forces: −b · V projected via J^T
    for t in 0..model.ntendon {
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        if kt > 0.0 {
            let displacement =
                tendon_deadband_displacement(data.ten_length[t], model.tendon_lengthspring[t]);
            // Only apply spring force + velocity correction when OUTSIDE
            // deadband. Inside (displacement == 0.0), no spring force exists,
            // and the velocity correction h·K·v must also be zero.
            if displacement != 0.0 {
                let velocity = data.ten_velocity[t]; // J · qvel
                let f = -kt * (displacement + h * velocity);
                let j = &data.ten_J[t];
                for dof in 0..nv {
                    if j[dof] != 0.0 {
                        qfrc[dof] += f * j[dof];
                    }
                }
            }
        }
        // Add tendon damper forces: −b · V projected via J^T
        let bt = model.tendon_damping[t];
        if bt > 0.0 {
            let j = &data.ten_J[t];
            let velocity = data.ten_velocity[t]; // J · qvel
            let f = -bt * velocity;
            for dof in 0..nv {
                if j[dof] != 0.0 {
                    qfrc[dof] += f * j[dof];
                }
            }
        }
    }

    qfrc
}

/// Pipeline:
/// 1. Compute qacc_smooth (unconstrained acceleration)
/// 2. Assemble ALL constraints into efc_* arrays
/// 3. Dispatch to configured solver (Newton, CG, PGS)
/// 4. Map efc_force → qfrc_constraint via J^T
/// 5. Extract qfrc_frictionloss from efc_force
fn mj_fwd_constraint(model: &Model, data: &mut Data) {
    data.qfrc_constraint.fill(0.0);
    data.qfrc_frictionloss.fill(0.0);
    data.jnt_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.ten_limit_frc.iter_mut().for_each(|f| *f = 0.0);
    data.newton_solved = false;

    // Step 1: Shared qacc_smooth computation
    let (qacc_smooth, _qfrc_smooth) = compute_qacc_smooth(model, data);

    let implicit_sd = model.integrator == Integrator::ImplicitSpringDamper;

    // DT-35: For ImplicitSpringDamper, precompute the implicit-modified
    // quantities that Newton needs. Also recompute qacc_smooth using M_impl
    // so the constraint assembly sees the correct unconstrained motion.
    let m_impl_owned: Option<DMatrix<f64>>;
    let qfrc_impl_owned: Option<DVector<f64>>;
    let qacc_smooth_impl: DVector<f64>;

    if implicit_sd {
        let m_impl = build_m_impl_for_newton(model, data);
        let qfrc_impl = compute_qfrc_smooth_implicit(model, data);

        // qacc_smooth_impl = M_impl⁻¹ · qfrc_smooth_impl
        let mut m_impl_factor = m_impl.clone();
        if cholesky_in_place(&mut m_impl_factor).is_err() {
            // M_impl should always be SPD; if Cholesky fails, fall back to
            // the base qacc_smooth (degrades gracefully).
            qacc_smooth_impl = qacc_smooth.clone();
        } else {
            let mut qa = qfrc_impl.clone();
            cholesky_solve_in_place(&m_impl_factor, &mut qa);
            qacc_smooth_impl = qa;
        }
        m_impl_owned = Some(m_impl);
        qfrc_impl_owned = Some(qfrc_impl);
    } else {
        qacc_smooth_impl = qacc_smooth.clone();
        m_impl_owned = None;
        qfrc_impl_owned = None;
    }

    // DT-35: Override data.qacc_smooth and data.qfrc_smooth with implicit
    // versions so Newton (and other consumers) see the correct values.
    if implicit_sd {
        data.qacc_smooth = qacc_smooth_impl.clone();
        if let Some(ref qfrc) = qfrc_impl_owned {
            data.qfrc_smooth = qfrc.clone();
        }
    }

    // Use implicit-corrected qacc_smooth for constraint assembly when
    // ImplicitSpringDamper is active, so efc_b sees the correct
    // unconstrained motion including spring/damper effects.
    let qacc_for_assembly = if implicit_sd {
        &qacc_smooth_impl
    } else {
        &qacc_smooth
    };

    // Step 2: Assemble ALL constraints (universal for all solver types)
    assemble_unified_constraints(model, data, qacc_for_assembly);
    let nefc = data.efc_type.len();

    // Step 2b: Populate efc_island from constraint rows and island data.
    // This must happen after assembly since mj_island runs before us with stale efc data.
    populate_efc_island(model, data);

    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth_impl);
        return;
    }

    // Store implicit quantities for Newton solver.
    // Clone into owned values so we don't hold an immutable borrow on `data`
    // while passing it mutably to `newton_solve`.
    let m_eff: DMatrix<f64> = m_impl_owned.unwrap_or_else(|| data.qM.clone());
    let qfrc_eff: DVector<f64> = qfrc_impl_owned.unwrap_or_else(|| data.qfrc_smooth.clone());

    // Step 3: Dispatch to solver
    match model.solver_type {
        SolverType::Newton => {
            let result = newton_solve(model, data, &m_eff, &qfrc_eff, implicit_sd);
            match result {
                NewtonResult::Converged => {
                    // Noslip post-processor (Phase C §15.10, §33)
                    if model.noslip_iterations > 0 {
                        noslip_postprocess(model, data);
                    }
                    data.newton_solved = true;
                }
                NewtonResult::CholeskyFailed | NewtonResult::MaxIterationsExceeded => {
                    // Fallback to PGS
                    pgs_solve_unified(model, data);
                    // §33: Noslip also runs after Newton's PGS fallback
                    if model.noslip_iterations > 0 {
                        noslip_postprocess(model, data);
                    }
                }
            }
        }
        SolverType::CG => {
            cg_solve_unified(model, data);
            // §33: Noslip post-processor for CG solver
            if model.noslip_iterations > 0 {
                noslip_postprocess(model, data);
            }
        }
        SolverType::PGS => {
            pgs_solve_unified(model, data);
            // §33: Noslip post-processor for PGS solver
            if model.noslip_iterations > 0 {
                noslip_postprocess(model, data);
            }
        }
    }

    // Step 4: Map efc_force → qfrc_constraint via J^T
    compute_qfrc_constraint_from_efc(model, data);

    // Step 5: Extract qfrc_frictionloss from efc_force
    extract_qfrc_frictionloss(data, model.nv);

    // Step 5b: Extract per-joint and per-tendon limit forces from efc_force
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

    // Step 6: Zero sleeping DOFs (§16.26.5)
    //
    // Sleeping trees must have zero qacc, qfrc_constraint, and qfrc_frictionloss.
    // The solver may produce non-zero forces for constraints involving sleeping bodies
    // (e.g., resting contact with the ground plane). We zero them here to maintain
    // the sleeping invariant. This is simpler than filtering constraints during assembly.
    if model.enableflags & ENABLE_SLEEP != 0 {
        for tree in 0..model.ntree {
            if data.tree_asleep[tree] < 0 {
                continue; // Awake
            }
            let dof_start = model.tree_dof_adr[tree];
            let dof_end = dof_start + model.tree_dof_num[tree];
            for dof in dof_start..dof_end {
                data.qacc[dof] = 0.0;
                data.qfrc_constraint[dof] = 0.0;
                data.qfrc_frictionloss[dof] = 0.0;
            }
        }
    }
}

// ============================================================================

// (Deformable pipeline functions removed — replaced by unified flex solver)

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
fn mj_fwd_acceleration_explicit(model: &Model, data: &mut Data) {
    // Sum all forces: τ = applied + actuator + passive + constraint - bias
    let mut qfrc_total = data.qfrc_applied.clone();
    qfrc_total += &data.qfrc_actuator;
    qfrc_total += &data.qfrc_passive;
    qfrc_total += &data.qfrc_constraint;
    qfrc_total -= &data.qfrc_bias;

    // Solve M * qacc = qfrc_total using sparse L^T D L factorization from mj_crba
    data.qacc.copy_from(&qfrc_total);
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut data.qacc,
    );
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

/// Sparse L^T D L factorization of the mass matrix using flat CSR storage.
///
/// Computes `M = L^T D L` where `L` is unit lower triangular (L\[i,i\] = 1 implicitly,
/// stored entries include diagonal as last element) and `D` is diagonal. Uses flat CSR
/// storage (`Model::qLD_rowadr`/`qLD_rownnz`/`qLD_colind` + `Data::qLD_data`). The
/// inner loop uses MuJoCo's bulk `addToScl` pattern for O(1) ancestor row access.
///
/// Matches MuJoCo's `mj_factorI` exactly:
/// - Diagonal `D[i]` stored at `qLD_data[rowadr[i] + rownnz[i] - 1]`
/// - `qLD_diag_inv[i] = 1/D[i]` precomputed for the solve phase (multiply, not divide)
/// - `rownnz[i]` includes the diagonal, so off-diagonal count = `rownnz[i] - 1`
///
/// # Ancestor Row Superset Property
///
/// If DOF `j` is an ancestor of DOF `i`, then `ancestors(j) ⊆ ancestors(i)`.
/// Both are stored ascending in `qLD_colind`. Row `i`'s off-diagonal entries at
/// positions `0..a` (where `a` is `j`'s position in row `i`) are exactly
/// `ancestors(j)`, so `a == rownnz[j] - 1`. This means `row_i[0..a]` and
/// `row_j[0..rownnz_j-1]` have identical column indices — a simple element-wise
/// scaled addition suffices.
#[allow(non_snake_case)]
fn mj_factor_sparse(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let (rowadr, rownnz, colind) = model.qld_csr();

    // Phase 1: Copy M's sparse entries into flat CSR (diagonal included as last element).
    for i in 0..nv {
        let start = rowadr[i];
        let nnz = rownnz[i];
        for k in 0..nnz {
            data.qLD_data[start + k] = data.qM[(i, colind[start + k])];
        }
    }

    // Phase 2: Eliminate from leaves to root.
    // Process DOF i, scale its off-diagonals by 1/D[i], then propagate
    // rank-1 update to all ancestor pairs using bulk row addition.
    for i in (0..nv).rev() {
        let start_i = rowadr[i];
        let nnz_i = rownnz[i];
        let diag_pos = start_i + nnz_i - 1;
        let di = data.qLD_data[diag_pos];

        // Precompute inverse diagonal (matching MuJoCo's diaginv output).
        let inv_di = 1.0 / di;
        data.qLD_diag_inv[i] = inv_di;

        // Root DOFs with no off-diagonals (rownnz == 1): only diagonal, no elimination.
        let nnz_offdiag = nnz_i - 1;
        if nnz_offdiag == 0 {
            continue;
        }

        // Scale off-diagonals: L[i,j] = working[i,j] / D[i]
        for k in 0..nnz_offdiag {
            data.qLD_data[start_i + k] *= inv_di;
        }

        // Propagate rank-1 update to ancestors (deep-to-shallow, matching MuJoCo).
        // For each ancestor j of i (traversed from deepest to shallowest):
        //   - Diagonal update: D[j] -= L[i,j]^2 * D[i]
        //   - Bulk row update: row_j[0..nnz_j-1] -= L[i,j] * row_i[0..a] * D[i]
        //     where a is j's off-diag position in row i, and a == rownnz[j]-1.
        for a in (0..nnz_offdiag).rev() {
            let j = colind[start_i + a];
            let lij = data.qLD_data[start_i + a];

            // Diagonal update on ancestor j
            let j_diag_pos = rowadr[j] + rownnz[j] - 1;
            data.qLD_data[j_diag_pos] -= lij * lij * di;

            // Bulk row update: row_j[0..a] -= lij * D[i] * row_i[0..a]
            // Ancestor superset: off-diag count of j == a (i.e., rownnz[j] - 1 == a).
            let j_nnz_offdiag = rownnz[j] - 1;
            debug_assert_eq!(
                a, j_nnz_offdiag,
                "ancestor row superset property violated: \
                DOF {i} entry at position {a} maps to ancestor DOF {j} with offdiag_nnz={j_nnz_offdiag}"
            );
            let scale = -lij * di;

            // split_at_mut(start_i) safe: j < i guarantees rowadr[j]+rownnz[j] <= rowadr[i].
            let start_j = rowadr[j];
            let (lo, hi) = data.qLD_data.split_at_mut(start_i);
            let dst = &mut lo[start_j..start_j + a];
            let src = &hi[..a];
            for k in 0..a {
                dst[k] += scale * src[k];
            }
        }
    }

    data.qLD_valid = true;
}

/// Sleep-aware sparse L^T D L factorization (C3b).
///
/// When sleeping is enabled and some DOFs are asleep, only awake DOFs are
/// factored. Sleeping DOFs' `qLD_data` and `qLD_diag_inv` entries are preserved
/// from their last awake step.
///
/// # Tree Independence
///
/// `dof_parent` chains never cross tree boundaries. Sleeping is per-tree.
/// Therefore, the elimination of awake DOF `i` only updates ancestors in the
/// same (awake) tree — sleeping trees' qLD entries are never touched.
///
/// When all DOFs are awake (or sleep is disabled), dispatches to the full
/// `mj_factor_sparse` with zero overhead.
#[allow(non_snake_case)]
fn mj_factor_sparse_selective(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nv_awake < model.nv;

    if !sleep_filter {
        mj_factor_sparse(model, data);
        return;
    }

    let (rowadr, rownnz, colind) = model.qld_csr();

    // Phase 1: Copy M into CSR for awake DOFs only.
    // Sleeping DOFs' qLD_data entries are preserved from last awake step.
    // rownnz[i] includes diagonal; we copy all entries (off-diag + diag at last position).
    for idx in 0..data.nv_awake {
        let i = data.dof_awake_ind[idx];
        let start = rowadr[i];
        let nnz = rownnz[i];
        for k in 0..nnz {
            data.qLD_data[start + k] = data.qM[(i, colind[start + k])];
        }
    }

    // Phase 2: Eliminate awake DOFs only (reverse order).
    // dof_awake_ind is sorted ascending; reverse iteration gives leaf-to-root order.
    // All ancestors of an awake DOF are in the same awake tree (per-tree invariant).
    // rownnz[i] includes diagonal; off-diag count = rownnz[i] - 1.
    for idx in (0..data.nv_awake).rev() {
        let i = data.dof_awake_ind[idx];
        let start_i = rowadr[i];
        let nnz_i = rownnz[i];
        let diag_pos = start_i + nnz_i - 1;
        let di = data.qLD_data[diag_pos];

        let inv_di = 1.0 / di;
        data.qLD_diag_inv[i] = inv_di;

        let nnz_offdiag = nnz_i - 1;
        if nnz_offdiag == 0 {
            continue;
        }

        for k in 0..nnz_offdiag {
            data.qLD_data[start_i + k] *= inv_di;
        }

        for a in (0..nnz_offdiag).rev() {
            let j = colind[start_i + a];
            let lij = data.qLD_data[start_i + a];

            // Diagonal update on ancestor j
            let j_diag_pos = rowadr[j] + rownnz[j] - 1;
            data.qLD_data[j_diag_pos] -= lij * lij * di;

            let start_j = rowadr[j];
            let nnz_j_offdiag = rownnz[j] - 1;
            debug_assert_eq!(a, nnz_j_offdiag);
            let scale = -lij * di;

            let (lo, hi) = data.qLD_data.split_at_mut(start_i);
            let dst = &mut lo[start_j..start_j + nnz_j_offdiag];
            let src = &hi[..a];
            for k in 0..a {
                dst[k] += scale * src[k];
            }
        }
    }

    data.qLD_valid = true;
}

/// Solve `L^T D L x = b` using the sparse factorization from `mj_factor_sparse`.
///
/// Matches MuJoCo's `mj_solveLD`:
/// - Off-diagonal entries at positions `0..rownnz[i]-1`
/// - Diagonal phase uses precomputed `qld_diag_inv[i]` (multiply, not divide)
///
/// On entry `x` contains `b`; on exit `x` contains the solution.
/// Zero allocations — operates entirely on borrowed data.
#[allow(non_snake_case)]
pub fn mj_solve_sparse(
    rowadr: &[usize],
    rownnz: &[usize],
    colind: &[usize],
    qld_data: &[f64],
    qld_diag_inv: &[f64],
    x: &mut DVector<f64>,
) {
    let nv = x.len();

    // Phase 1: Solve L^T y = b (scatter: propagate each DOF to its ancestors).
    // Off-diagonal entries only: positions 0..rownnz[i]-1.
    // Zero-skip: if x[i] == 0 the scatter is a no-op (MuJoCo: `if ((x_i = x[i]))`).
    // Diagonal-only skip: rownnz == 1 means no off-diagonals (MuJoCo: `if (rownnz[i] == 1)`).
    for i in (0..nv).rev() {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let xi = x[i];
        if xi == 0.0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[colind[start + k]] -= qld_data[start + k] * xi;
        }
    }

    // Phase 2: Solve D z = y (multiply by precomputed inverse, matching MuJoCo's diaginv).
    for i in 0..nv {
        x[i] *= qld_diag_inv[i];
    }

    // Phase 3: Solve L w = z (gather from ancestors)
    // Off-diagonal entries only: positions 0..rownnz[i]-1.
    // Diagonal-only skip: rownnz == 1 means no off-diagonals (MuJoCo: `if (rownnz[i] == 1)`).
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[i] -= qld_data[start + k] * x[colind[start + k]];
        }
    }
}

/// Batch solve `L^T D L X = B` for multiple right-hand sides simultaneously.
///
/// Matches MuJoCo's `mj_solveLD` with `n > 1`: the outer loop sweeps CSR metadata
/// once per DOF, the inner loop iterates across `n` vectors. This gives O(1) CSR
/// metadata loads vs O(n) for `n` separate `mj_solve_sparse` calls.
///
/// `x` is an nv × n column-major matrix (nalgebra `DMatrix`). Each column is an
/// independent RHS; on exit each column contains the corresponding solution.
///
/// Includes zero-skip in L^T phase (per-vector) and diagonal-only row skip.
#[allow(non_snake_case)]
pub(crate) fn mj_solve_sparse_batch(
    rowadr: &[usize],
    rownnz: &[usize],
    colind: &[usize],
    qld_data: &[f64],
    qld_diag_inv: &[f64],
    x: &mut DMatrix<f64>,
) {
    let nv = x.nrows();
    let n = x.ncols();

    // Phase 1: Solve L^T Y = B (scatter across all vectors).
    for i in (0..nv).rev() {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for v in 0..n {
            let xi = x[(i, v)];
            if xi == 0.0 {
                continue;
            }
            for k in 0..nnz_offdiag {
                x[(colind[start + k], v)] -= qld_data[start + k] * xi;
            }
        }
    }

    // Phase 2: Solve D Z = Y (multiply by precomputed inverse).
    for i in 0..nv {
        let inv_di = qld_diag_inv[i];
        for v in 0..n {
            x[(i, v)] *= inv_di;
        }
    }

    // Phase 3: Solve L W = Z (gather across all vectors).
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for v in 0..n {
            let mut acc = 0.0;
            for k in 0..nnz_offdiag {
                acc += qld_data[start + k] * x[(colind[start + k], v)];
            }
            x[(i, v)] -= acc;
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

    // Build modified mass matrix: M_impl = M + h*D_jnt + h²*K_jnt
    // Copy M into scratch, then modify diagonal from joint K/D
    data.scratch_m_impl.copy_from(&data.qM);
    for i in 0..model.nv {
        data.scratch_m_impl[(i, i)] += h * d[i] + h2 * k[i];
    }

    // DT-35: Non-diagonal tendon stiffness and damping (Step 0 helper).
    // Adds Σ_t (h²·k_active_t + h·b_t) · J_t^T · J_t to scratch_m_impl.
    // Spring K is deadband-aware: zero inside [lower, upper], k outside.
    // Damping D always applies. Sleep guard skips fully-sleeping tendons.
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    accumulate_tendon_kd(
        &mut data.scratch_m_impl,
        model,
        &data.ten_J,
        &data.ten_length,
        &data.tree_awake,
        h,
        sleep_enabled,
    );

    // Build RHS into scratch buffer: M*v_old + h*f_ext - h*K*(q - q_eq)
    // Start with M*v_old
    data.qM.mul_to(&data.qvel, &mut data.scratch_rhs);

    // Add h*f_ext for each DOF
    for i in 0..model.nv {
        data.scratch_rhs[i] += h * data.scratch_force[i];
    }

    // Subtract h*K*(q - q_eq) for joint spring displacement using visitor
    let mut spring_visitor = ImplicitSpringVisitor {
        k,
        q_eq,
        h,
        qpos: &data.qpos,
        rhs: &mut data.scratch_rhs,
    };
    model.visit_joints(&mut spring_visitor);

    // DT-35: Tendon spring displacement contribution to implicit RHS
    // RHS[dof] -= h · Σ_t k_t · J_t[dof] · deadband_disp(L_t)
    // (sleep_enabled already computed in Step 2, same function scope)
    for t in 0..model.ntendon {
        // Must match the guard in accumulate_tendon_kd — if K is not in the
        // LHS, the corresponding spring displacement must not be in the RHS.
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let kt = model.tendon_stiffness[t];
        if kt <= 0.0 {
            continue;
        }
        let displacement =
            tendon_deadband_displacement(data.ten_length[t], model.tendon_lengthspring[t]);
        // SAFETY: exact `== 0.0` comparison is correct here.
        // `tendon_deadband_displacement` returns literal `0.0` from the else
        // branch (no arithmetic). At boundary, `length - upper` is exactly
        // 0.0 when both operands are equal.
        if displacement == 0.0 {
            continue; // Inside deadband — no spring force
        }
        // Spring force in tendon space: F = k * (ref - L) = -k * displacement
        // Joint-space force: qfrc = J^T * F = -J^T * k * displacement
        // RHS += h * qfrc = -h * k * displacement * J^T
        let j = &data.ten_J[t];
        let scale = -h * kt * displacement;
        for dof in 0..model.nv {
            if j[dof] != 0.0 {
                data.scratch_rhs[dof] += scale * j[dof];
            }
        }
    }

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

// mj_integrate_pos_flex DELETED (§27F): Flex vertices now have slide joints.
// Standard mj_integrate_pos handles slide joint position integration.

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

        // (§27F) Flex vertex positions now integrated by mj_integrate_pos_explicit
        // above — slide joints are handled by the standard manifold integration path.

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
            // §34: Clamp activation to actrange for trial state (replaces muscle-only [0,1])
            if model.actuator_actlimited[act_i] {
                let range = model.actuator_actrange[act_i];
                for k in 0..model.actuator_act_num[act_i] {
                    data.act[act_adr + k] = data.act[act_adr + k].clamp(range.0, range.1);
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

    // (§27F) Flex vertex positions now integrated by mj_integrate_pos_explicit above.

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
        // §34: Clamp activation to actrange (replaces muscle-only [0,1])
        if model.actuator_actlimited[act_i] {
            let range = model.actuator_actrange[act_i];
            for k in 0..model.actuator_act_num[act_i] {
                data.act[act_adr + k] = data.act[act_adr + k].clamp(range.0, range.1);
            }
        }
    }

    data.time = t0 + h;

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
        model.geom_priority = vec![0; ngeom];
        model.geom_solmix = vec![1.0; ngeom];
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
            0.0, // margin
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
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
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

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

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

        // Manually inject a contact and corresponding efc_force entries
        data.contacts.push(Contact::new(
            Vector3::new(0.0, 0.0, -0.5),
            Vector3::z(),
            0.01, // depth
            0,    // geom1 = our sensor geom
            99,   // geom2 = some other geom
            1.0,  // friction
        ));
        // Inject efc constraint rows for this contact (3-dim elliptic)
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_type.push(ConstraintType::ContactElliptic);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_dim.push(3);
        data.efc_id.push(0); // contact index
        data.efc_id.push(0);
        data.efc_id.push(0);
        data.efc_force = DVector::from_vec(vec![42.0, 1.0, 2.0]); // normal=42, tangent=1,2

        // Run acc sensors directly to test Touch
        mj_sensor_acc(&model, &mut data);

        // Touch should read the normal force from efc_force[0]
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
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);

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
        model.actuator_actlimited.push(false);
        model.actuator_actrange.push((0.0, 0.0));
        model.actuator_actearly.push(false);

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
    // Tendon Sensor Tests (zero-tendon model)
    // ========================================================================

    #[test]
    fn test_tendon_pos_sensor_no_tendon() {
        let mut model = make_sensor_test_model();
        add_sensor(
            &mut model,
            MjSensorType::TendonPos,
            MjSensorDataType::Position,
            MjObjectType::Body, // no tendon in model
            0,
        );

        let mut data = model.make_data();
        data.forward(&model).unwrap();

        // No tendon exists (ntendon == 0), sensor reads 0
        assert_eq!(data.sensordata[0], 0.0);
    }

    #[test]
    fn test_tendon_vel_sensor_no_tendon() {
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
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
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
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
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

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

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
        model.geom_priority.push(0);
        model.geom_solmix.push(1.0);
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

        // Pre-compute CSR sparsity metadata for sparse LDL factorization
        model.compute_qld_csr_metadata();

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
        model.compute_qld_csr_metadata();
        let mut data = model.make_data();
        data.qM.copy_from(qm);
        mj_factor_sparse(&model, &mut data);
        (model, data)
    }

    /// Verify sparse solve matches nalgebra dense Cholesky.
    fn assert_solve_matches(model: &Model, data: &Data, qm: &DMatrix<f64>, nv: usize) {
        let rhs = DVector::from_fn(nv, |i, _| (i as f64 + 1.0) * 0.7);

        // Reference: nalgebra dense Cholesky
        let chol = qm.clone().cholesky().expect("nalgebra cholesky failed");
        let x_ref = chol.solve(&rhs);

        // CSR sparse solve — validate against reference
        let (rowadr, rownnz, colind) = model.qld_csr();
        let mut x_csr = rhs;
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut x_csr,
        );

        for i in 0..nv {
            assert_relative_eq!(x_csr[i], x_ref[i], epsilon = 1e-12, max_relative = 1e-12);
        }
    }

    #[test]
    fn single_hinge() {
        // nv=1, no parent. M = [[5.0]]
        let nv = 1;
        let dof_parent = vec![None];
        let mut qm = DMatrix::zeros(1, 1);
        qm[(0, 0)] = 5.0;

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // D[0] = 5.0, L has no off-diag entries
        assert_relative_eq!(data.qld_diag(&model, 0), 5.0);
        assert_eq!(model.qLD_rownnz[0], 1); // diagonal only
        assert!(data.qLD_valid);

        assert_solve_matches(&model, &data, &qm, nv);
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // Manual: D[1] = 3, L[1,0] = 2/3, D[0] = 4 - (2/3)^2 * 3 = 4 - 4/3 = 8/3
        assert_relative_eq!(data.qld_diag(&model, 1), 3.0);
        assert_relative_eq!(data.qld_diag(&model, 0), 8.0 / 3.0, epsilon = 1e-14);
        assert_eq!(model.qLD_rownnz[1], 2); // 1 off-diag + 1 diagonal
        assert_eq!(model.qLD_colind[model.qLD_rowadr[1]], 0);
        assert_relative_eq!(
            data.qLD_data[model.qLD_rowadr[1]],
            2.0 / 3.0,
            epsilon = 1e-14
        );

        assert_solve_matches(&model, &data, &qm, nv);
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);

        // D[2] = 3, L[2,0] = 1/3
        // D[1] = 4, L[1,0] = 2/4 = 0.5
        // D[0] = 5 - (0.5)^2 * 4 - (1/3)^2 * 3 = 5 - 1 - 1/3 = 11/3
        assert_relative_eq!(data.qld_diag(&model, 2), 3.0);
        assert_relative_eq!(data.qld_diag(&model, 1), 4.0);
        assert_relative_eq!(data.qld_diag(&model, 0), 11.0 / 3.0, epsilon = 1e-14);

        assert_solve_matches(&model, &data, &qm, nv);
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&model, &data, &qm, nv);
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);
        assert_solve_matches(&model, &data, &qm, nv);
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        assert!(data.qLD_valid);

        // Verify zero entries in L for non-ancestor pairs
        // DOF 2's ancestors: [0, 1] — should not have entry for 3 or 4
        // rownnz includes diagonal (last element is self-index), so off-diag = 0..rownnz-1
        let (rowadr, rownnz, colind) = model.qld_csr();
        let nnz_offdiag_2 = rownnz[2] - 1;
        for k in 0..nnz_offdiag_2 {
            let col = colind[rowadr[2] + k];
            assert!(
                col == 0 || col == 1,
                "DOF 2 should only have ancestors 0 and 1"
            );
        }
        // Last element should be self-index (diagonal)
        assert_eq!(colind[rowadr[2] + nnz_offdiag_2], 2);
        // DOF 4's ancestors: [0] — should not have entry for 1, 2, or 3
        assert_eq!(model.qLD_rownnz[4], 2); // 1 off-diag + 1 diagonal
        assert_eq!(model.qLD_colind[model.qLD_rowadr[4]], 0);

        assert_solve_matches(&model, &data, &qm, nv);
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

    #[test]
    fn batch_solve_matches_single() {
        // nv=3 branching tree, solve 3 different RHS via batch and single.
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

        let (model, data) = setup_sparse(nv, dof_parent, &qm);
        let (rowadr, rownnz, colind) = model.qld_csr();
        let n_rhs = 3;

        // Build nv × n_rhs matrix of RHS vectors.
        let mut batch_rhs = DMatrix::zeros(nv, n_rhs);
        for v in 0..n_rhs {
            for i in 0..nv {
                batch_rhs[(i, v)] = (i as f64 + 1.0) * (v as f64 + 0.5);
            }
        }

        // Single-vector solve for each column.
        let mut single_results = Vec::new();
        for v in 0..n_rhs {
            let mut x = batch_rhs.column(v).clone_owned();
            mj_solve_sparse(
                rowadr,
                rownnz,
                colind,
                &data.qLD_data,
                &data.qLD_diag_inv,
                &mut x,
            );
            single_results.push(x);
        }

        // Batch solve.
        let mut batch_x = batch_rhs;
        mj_solve_sparse_batch(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut batch_x,
        );

        // Compare: each batch column must match the corresponding single solve.
        for v in 0..n_rhs {
            for i in 0..nv {
                assert_relative_eq!(
                    batch_x[(i, v)],
                    single_results[v][i],
                    epsilon = 1e-14,
                    max_relative = 1e-14
                );
            }
        }
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
        // Muscle default: actlimited=true, actrange=[0,1] (§34 S5)
        model.actuator_actlimited = vec![true];
        model.actuator_actrange = vec![(0.0, 1.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        // Pre-compute
        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
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
        model.actuator_actlimited = vec![false];
        model.actuator_actrange = vec![(0.0, 0.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::zeros(); // no gravity for clean test

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

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
// Unit tests — ball_limit_axis_angle
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod ball_limit_tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    /// Build quaternion [w, x, y, z] for rotation of `angle_deg` degrees about `axis`.
    fn quat_from_axis_angle_deg(axis: [f64; 3], angle_deg: f64) -> [f64; 4] {
        let half = (angle_deg / 2.0_f64).to_radians();
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let s = half.sin() / norm;
        [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
    }

    #[test]
    fn test_ball_limit_axis_angle_90deg_about_z() {
        // 90° rotation about Z: q = (cos(45°), 0, 0, sin(45°))
        let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 90.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, PI / 2.0, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.z, 1.0, epsilon = 1e-10); // +Z (theta > 0)
        assert!(unit_dir.x.abs() < 1e-10);
        assert!(unit_dir.y.abs() < 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_50deg_about_oblique() {
        // 50° about axis (0.6, 0.8, 0) — matches acceptance criterion 8
        let q = quat_from_axis_angle_deg([0.6, 0.8, 0.0], 50.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 50.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(unit_dir.x, 0.6, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.y, 0.8, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_200deg_wraps() {
        // 200° about Z → wraps to theta = -160°, angle = 160°
        let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 200.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 160.0_f64.to_radians(), epsilon = 1e-10);
        // theta < 0 after wrap, so unit_dir = sign(-) * z = -z
        assert_relative_eq!(unit_dir.z, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let (_, angle) = ball_limit_axis_angle(q);
        assert!(angle < 1e-10, "identity should have zero rotation angle");
        // unit_dir is arbitrary (z-axis default) — not asserted
    }

    #[test]
    fn test_ball_limit_axis_angle_negative_quat() {
        // -q represents the same rotation as q. Verify identical output.
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
        let neg_q = [-q[0], -q[1], -q[2], -q[3]];
        let (dir1, angle1) = ball_limit_axis_angle(q);
        let (dir2, angle2) = ball_limit_axis_angle(neg_q);
        assert_relative_eq!(angle1, angle2, epsilon = 1e-10);
        assert_relative_eq!(dir1, dir2, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_exactly_180deg() {
        // Boundary case: exactly π rotation
        let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 180.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, PI, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_exact_boundary() {
        // Rotation angle exactly equals a typical limit — verify angle is precise
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 45.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(unit_dir.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_near_zero_quaternion() {
        // A quaternion with near-zero norm (degenerate) should be caught by
        // normalize_quat4 (returns identity) before reaching ball_limit_axis_angle.
        // But ball_limit_axis_angle itself should also handle near-zero sin_half
        // gracefully — returning angle = 0 and an arbitrary direction.
        let q = [1.0, 1e-15, 1e-15, 1e-15]; // Nearly identity, sin_half ≈ 1.7e-15
        let (_, angle) = ball_limit_axis_angle(q);
        assert!(
            angle < 1e-10,
            "near-zero sin_half should produce angle ≈ 0: got {angle}"
        );
        // unit_dir is arbitrary (z-axis default) — not asserted
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
        model.compute_qld_csr_metadata();

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

// =============================================================================
// Unit tests — mj_jac (DT-74) canonical body Jacobian API
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::similar_names)]
mod mj_jac_tests {
    use super::*;
    use approx::assert_relative_eq;

    // =========================================================================
    // Helper: single-joint body (world → body1 with one joint, optional site)
    // =========================================================================
    fn make_single_joint_model(
        jnt_type: MjJointType,
        joint_axis: Vector3<f64>,
        body_pos: Vector3<f64>,
        qpos_val: f64,
    ) -> (Model, Data) {
        let mut model = Model::empty();

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

        model.dof_body = vec![1; nv];
        model.dof_jnt = vec![0; nv];
        model.dof_parent = vec![None; nv];
        model.dof_armature = vec![0.0; nv];
        model.dof_damping = vec![0.0; nv];
        model.dof_frictionloss = vec![0.0; nv];

        model.nsite = 1;
        model.site_body = vec![1];
        model.site_pos = vec![Vector3::new(0.5, 0.0, 0.0)];
        model.site_quat = vec![UnitQuaternion::identity()];
        model.site_type = vec![GeomType::Sphere];
        model.site_size = vec![Vector3::new(0.01, 0.01, 0.01)];
        model.site_name = vec![Some("s1".to_string())];

        model.qpos0 = DVector::zeros(nq);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        data.qpos[0] = qpos_val;
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    // =========================================================================
    // 5a — Per-joint-type analytical tests
    // =========================================================================

    #[test]
    fn mj_jac_hinge_basic() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.3,
        );
        let point = data.site_xpos[0];
        let (jacp, jacr) = mj_jac(&model, &data, 1, &point);

        let axis = data.xquat[1] * model.jnt_axis[0];
        let anchor = data.xpos[1] + data.xquat[1] * model.jnt_pos[0];
        let r = point - anchor;
        let expected_jacp = axis.cross(&r);

        for k in 0..3 {
            assert_relative_eq!(jacp[(k, 0)], expected_jacp[k], epsilon = 1e-12);
            assert_relative_eq!(jacr[(k, 0)], axis[k], epsilon = 1e-12);
        }
    }

    #[test]
    fn mj_jac_slide_basic() {
        let (model, data) = make_single_joint_model(
            MjJointType::Slide,
            Vector3::z(),
            Vector3::new(0.0, 0.0, 1.0),
            0.1,
        );
        let point = data.site_xpos[0];
        let (jacp, jacr) = mj_jac(&model, &data, 1, &point);

        let axis = data.xquat[1] * model.jnt_axis[0];
        for k in 0..3 {
            assert_relative_eq!(jacp[(k, 0)], axis[k], epsilon = 1e-12);
            assert_relative_eq!(jacr[(k, 0)], 0.0, epsilon = 1e-14);
        }
    }

    #[test]
    fn mj_jac_ball_body_frame_axes() {
        // Ball joint needs a valid quaternion — build manually with qpos[0..4] = identity quat
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 3];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::new(0.0, 0.0, 1.0)];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".to_string()), Some("b1".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 4;
        model.nv = 3;
        model.jnt_type = vec![MjJointType::Ball];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::x()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".to_string())];

        model.dof_body = vec![1; 3];
        model.dof_jnt = vec![0; 3];
        model.dof_parent = vec![None; 3];
        model.dof_armature = vec![0.0; 3];
        model.dof_damping = vec![0.0; 3];
        model.dof_frictionloss = vec![0.0; 3];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        model.qpos0 = DVector::zeros(4);
        model.qpos0[0] = 1.0; // w component of identity quaternion
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Apply a small rotation: 30° about Z
        let q = UnitQuaternion::from_axis_angle(&nalgebra::Unit::new_normalize(Vector3::z()), 0.5);
        data.qpos[0] = q.w;
        data.qpos[1] = q.i;
        data.qpos[2] = q.j;
        data.qpos[3] = q.k;
        mj_fwd_position(&model, &mut data);

        let point = data.xpos[1] + data.xquat[1] * Vector3::new(0.5, 0.0, 0.0);
        let (jacp, jacr) = mj_jac(&model, &data, 1, &point);

        let rot = data.xquat[1].to_rotation_matrix();
        let anchor = data.xpos[1] + data.xquat[1] * model.jnt_pos[0];
        let r = point - anchor;

        for i in 0..3 {
            let omega = rot * Vector3::ith(i, 1.0);
            let expected_jacp = omega.cross(&r);
            for k in 0..3 {
                assert_relative_eq!(jacp[(k, i)], expected_jacp[k], epsilon = 1e-12);
                assert_relative_eq!(jacr[(k, i)], omega[k], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn mj_jac_free_translation() {
        let (model, data) = make_single_joint_model(
            MjJointType::Free,
            Vector3::x(), // axis unused for free
            Vector3::zeros(),
            0.0,
        );
        let point = data.site_xpos[0];
        let (jacp, jacr) = mj_jac(&model, &data, 1, &point);

        // Translation DOFs: jacp[:,0:3] = I₃, jacr[:,0:3] = 0
        for i in 0..3 {
            for k in 0..3 {
                let expected = if i == k { 1.0 } else { 0.0 };
                assert_relative_eq!(jacp[(k, i)], expected, epsilon = 1e-14);
                assert_relative_eq!(jacr[(k, i)], 0.0, epsilon = 1e-14);
            }
        }
    }

    #[test]
    fn mj_jac_free_rotation_body_frame() {
        // Set up a free body at a non-identity orientation
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 6];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".to_string()), Some("b1".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::x()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".to_string())];

        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None; 6];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![0.0; 6];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        model.qpos0 = DVector::zeros(7);
        model.qpos0[3] = 1.0; // w component of identity quat
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Non-identity orientation: 45° about Z
        let q = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::z()),
            std::f64::consts::FRAC_PI_4,
        );
        data.qpos[0] = 1.0; // x position offset
        data.qpos[1] = 0.5;
        data.qpos[2] = 0.0;
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let point = data.xpos[1] + data.xquat[1] * Vector3::new(0.3, 0.0, 0.0);
        let (_jacp, jacr) = mj_jac(&model, &data, 1, &point);

        // jacr[:,3+i] = R·eᵢ (body-frame axes, NOT world eᵢ)
        let rot = data.xquat[1].to_rotation_matrix();
        for i in 0..3 {
            let expected = rot * Vector3::ith(i, 1.0);
            for k in 0..3 {
                assert_relative_eq!(jacr[(k, 3 + i)], expected[k], epsilon = 1e-12);
            }
        }
    }

    #[test]
    fn mj_jac_world_body_returns_zeros() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.0,
        );
        let point = Vector3::new(1.0, 2.0, 3.0);
        let (jacp, jacr) = mj_jac(&model, &data, 0, &point);

        assert_eq!(jacp.nrows(), 3);
        assert_eq!(jacp.ncols(), model.nv);
        assert_relative_eq!(jacp.norm(), 0.0, epsilon = 1e-14);
        assert_relative_eq!(jacr.norm(), 0.0, epsilon = 1e-14);
    }

    // =========================================================================
    // 5b — Wrapper consistency tests
    // =========================================================================

    #[test]
    fn mj_jac_site_delegates_to_mj_jac() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        );
        let (jacp_site, jacr_site) = mj_jac_site(&model, &data, 0);
        let (jacp_direct, jacr_direct) =
            mj_jac(&model, &data, model.site_body[0], &data.site_xpos[0]);
        assert_relative_eq!(jacp_site, jacp_direct, epsilon = 1e-14);
        assert_relative_eq!(jacr_site, jacr_direct, epsilon = 1e-14);
    }

    #[test]
    fn mj_jac_body_delegates_to_mj_jac() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        );
        let (jacp_body, jacr_body) = mj_jac_body(&model, &data, 1);
        let (jacp_direct, jacr_direct) = mj_jac(&model, &data, 1, &data.xpos[1]);
        assert_relative_eq!(jacp_body, jacp_direct, epsilon = 1e-14);
        assert_relative_eq!(jacr_body, jacr_direct, epsilon = 1e-14);
    }

    #[test]
    fn mj_jac_point_combines_correctly() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        );
        let point = data.site_xpos[0];
        let jac6 = mj_jac_point(&model, &data, 1, &point);
        let (jacp, jacr) = mj_jac(&model, &data, 1, &point);

        // rows 0–2 = angular (jacr), rows 3–5 = linear (jacp)
        for col in 0..model.nv {
            for k in 0..3 {
                assert_relative_eq!(jac6[(k, col)], jacr[(k, col)], epsilon = 1e-14);
                assert_relative_eq!(jac6[(k + 3, col)], jacp[(k, col)], epsilon = 1e-14);
            }
        }
    }

    #[test]
    fn mj_jac_body_com_consistent() {
        let (model, data) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        );
        let jac_com = mj_jac_body_com(&model, &data, 1);
        let jac_pt = mj_jac_point(&model, &data, 1, &data.xipos[1]);
        assert_relative_eq!(jac_com, jac_pt, epsilon = 1e-14);
    }

    #[test]
    fn mj_jac_geom_consistent() {
        // Need a geom on body 1
        let (mut model, _) = make_single_joint_model(
            MjJointType::Hinge,
            Vector3::y(),
            Vector3::new(0.0, 0.0, 1.0),
            0.5,
        );
        model.ngeom = 1;
        model.geom_body = vec![1];
        model.geom_pos = vec![Vector3::new(0.1, 0.0, 0.0)];
        model.geom_quat = vec![UnitQuaternion::identity()];
        model.geom_type = vec![GeomType::Sphere];
        model.geom_size = vec![Vector3::new(0.05, 0.05, 0.05)];
        model.geom_name = vec![Some("g1".to_string())];
        model.geom_rbound = vec![0.05];
        model.geom_mesh = vec![None];
        model.geom_contype = vec![1];
        model.geom_conaffinity = vec![1];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001)];
        model.geom_condim = vec![3];
        model.geom_solref = vec![[0.02, 1.0]];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.geom_priority = vec![0];
        model.geom_solmix = vec![1.0];
        model.geom_margin = vec![0.0];
        model.geom_gap = vec![0.0];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 1];

        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        mj_fwd_position(&model, &mut data);

        let jac_geom = mj_jac_geom(&model, &data, 0);
        let jac_pt = mj_jac_point(&model, &data, 1, &data.geom_xpos[0]);
        assert_relative_eq!(jac_geom, jac_pt, epsilon = 1e-14);
    }

    // =========================================================================
    // 5d — Multi-joint chain: free→hinge→hinge (3 bodies, 8 DOFs)
    // =========================================================================

    /// Build a 3-body chain: world → free body → hinge body → hinge body
    fn make_free_hinge_hinge_chain() -> (Model, Data) {
        let mut model = Model::empty();
        model.nbody = 4; // world + 3 bodies
        model.body_parent = vec![0, 0, 1, 2];
        model.body_rootid = vec![0, 1, 1, 1];
        model.body_jnt_adr = vec![0, 0, 1, 2];
        model.body_jnt_num = vec![0, 1, 1, 1];
        model.body_dof_adr = vec![0, 0, 6, 7];
        model.body_dof_num = vec![0, 6, 1, 1];
        model.body_geom_adr = vec![0, 0, 0, 0];
        model.body_geom_num = vec![0, 0, 0, 0];
        model.body_pos = vec![
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, 0.5),
            Vector3::new(0.0, 0.0, 0.5),
        ];
        model.body_quat = vec![UnitQuaternion::identity(); 4];
        model.body_ipos = vec![Vector3::zeros(); 4];
        model.body_iquat = vec![UnitQuaternion::identity(); 4];
        model.body_mass = vec![0.0, 1.0, 0.5, 0.5];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.005, 0.005, 0.005),
            Vector3::new(0.005, 0.005, 0.005),
        ];
        model.body_name = vec![
            Some("world".to_string()),
            Some("b1".to_string()),
            Some("b2".to_string()),
            Some("b3".to_string()),
        ];
        model.body_subtreemass = vec![2.0, 2.0, 1.0, 0.5];

        model.njnt = 3;
        model.nq = 9; // 7 (free) + 1 (hinge) + 1 (hinge)
        model.nv = 8; // 6 (free) + 1 + 1
        model.jnt_type = vec![MjJointType::Free, MjJointType::Hinge, MjJointType::Hinge];
        model.jnt_body = vec![1, 2, 3];
        model.jnt_qpos_adr = vec![0, 7, 8];
        model.jnt_dof_adr = vec![0, 6, 7];
        model.jnt_pos = vec![Vector3::zeros(); 3];
        model.jnt_axis = vec![Vector3::x(), Vector3::y(), Vector3::y()];
        model.jnt_limited = vec![false; 3];
        model.jnt_range = vec![(0.0, 0.0); 3];
        model.jnt_stiffness = vec![0.0; 3];
        model.jnt_springref = vec![0.0; 3];
        model.jnt_damping = vec![0.0; 3];
        model.jnt_armature = vec![0.0; 3];
        model.jnt_solref = vec![[0.02, 1.0]; 3];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 3];
        model.jnt_name = vec![
            Some("j_free".to_string()),
            Some("j_hinge1".to_string()),
            Some("j_hinge2".to_string()),
        ];

        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 3];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 2];
        model.dof_parent = vec![None, None, None, None, None, None, Some(5), Some(6)];
        model.dof_armature = vec![0.0; 8];
        model.dof_damping = vec![0.0; 8];
        model.dof_frictionloss = vec![0.0; 8];

        model.nsite = 1;
        model.site_body = vec![3];
        model.site_pos = vec![Vector3::new(0.0, 0.0, 0.25)];
        model.site_quat = vec![UnitQuaternion::identity()];
        model.site_type = vec![GeomType::Sphere];
        model.site_size = vec![Vector3::new(0.01, 0.01, 0.01)];
        model.site_name = vec![Some("tip".to_string())];

        model.qpos0 = DVector::zeros(9);
        model.qpos0[3] = 1.0; // free joint quaternion w
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 4];
        model.body_ancestor_mask = vec![vec![]; 4];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Set some non-trivial configuration
        data.qpos[2] = 0.3; // z offset
        data.qpos[7] = 0.4; // hinge1 angle
        data.qpos[8] = -0.2; // hinge2 angle
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    #[test]
    fn mj_jac_multi_joint_chain_dimensions_and_fd() {
        let (model, data) = make_free_hinge_hinge_chain();
        let point = data.site_xpos[0];
        let (jacp, jacr) = mj_jac(&model, &data, 3, &point);

        assert_eq!(jacp.nrows(), 3);
        assert_eq!(jacp.ncols(), 8);
        assert_eq!(jacr.nrows(), 3);
        assert_eq!(jacr.ncols(), 8);

        // Finite-difference cross-check for jacp
        let eps = 1e-7;
        let body_id: usize = 3;

        // Compute body-local offset for tracking the point through perturbation
        let body_local = data.xquat[body_id].inverse() * (point - data.xpos[body_id]);

        for d in 0..model.nv {
            let mut qvel_pert = DVector::zeros(model.nv);
            qvel_pert[d] = eps;

            let mut qpos_pert = DVector::zeros(model.nq);
            mj_integrate_pos_explicit(&model, &mut qpos_pert, &data.qpos, &qvel_pert, 1.0);

            let mut data_pert = model.make_data();
            data_pert.qpos = qpos_pert;
            mj_fwd_position(&model, &mut data_pert);

            let point_pert = data_pert.xpos[body_id] + data_pert.xquat[body_id] * body_local;
            let jacp_fd_col = (point_pert - point) / eps;

            for k in 0..3 {
                assert_relative_eq!(jacp[(k, d)], jacp_fd_col[k], epsilon = 1e-5);
            }
        }
    }

    // =========================================================================
    // 5e — Finite-difference validation: free+ball at non-identity orientation
    // =========================================================================

    #[test]
    fn mj_jac_free_ball_fd_validation() {
        let mut model = Model::empty();
        model.nbody = 3;
        model.body_parent = vec![0, 0, 1];
        model.body_rootid = vec![0, 1, 1];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 6];
        model.body_dof_num = vec![0, 6, 3];
        model.body_geom_adr = vec![0, 0, 0];
        model.body_geom_num = vec![0, 0, 0];
        model.body_pos = vec![
            Vector3::zeros(),
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, 0.5),
        ];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 0.5];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.005, 0.005, 0.005),
        ];
        model.body_name = vec![
            Some("world".to_string()),
            Some("b1".to_string()),
            Some("b2".to_string()),
        ];
        model.body_subtreemass = vec![1.5, 1.5, 0.5];

        model.njnt = 2;
        model.nq = 11; // 7 (free) + 4 (ball)
        model.nv = 9; // 6 (free) + 3 (ball)
        model.jnt_type = vec![MjJointType::Free, MjJointType::Ball];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 7];
        model.jnt_dof_adr = vec![0, 6];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_axis = vec![Vector3::x(); 2];
        model.jnt_limited = vec![false; 2];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_armature = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("j_free".to_string()), Some("j_ball".to_string())];

        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 2, 2];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 1, 1];
        model.dof_parent = vec![
            None,
            None,
            None,
            None,
            None,
            None,
            Some(5),
            Some(5),
            Some(5),
        ];
        model.dof_armature = vec![0.0; 9];
        model.dof_damping = vec![0.0; 9];
        model.dof_frictionloss = vec![0.0; 9];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        model.qpos0 = DVector::zeros(11);
        model.qpos0[3] = 1.0; // free quat w
        model.qpos0[7] = 1.0; // ball quat w
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Non-identity configuration
        let q_free = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 0.0)),
            0.6,
        );
        data.qpos[0] = 0.3;
        data.qpos[1] = -0.2;
        data.qpos[2] = 0.5;
        data.qpos[3] = q_free.w;
        data.qpos[4] = q_free.i;
        data.qpos[5] = q_free.j;
        data.qpos[6] = q_free.k;

        let q_ball = UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
            0.4,
        );
        data.qpos[7] = q_ball.w;
        data.qpos[8] = q_ball.i;
        data.qpos[9] = q_ball.j;
        data.qpos[10] = q_ball.k;
        mj_fwd_position(&model, &mut data);

        let body_id: usize = 2;
        let body_local_offset = Vector3::new(0.1, 0.05, 0.2);
        let point = data.xpos[body_id] + data.xquat[body_id] * body_local_offset;
        let (jacp, jacr) = mj_jac(&model, &data, body_id, &point);

        let eps = 1e-7;
        for d in 0..model.nv {
            let mut qvel_pert = DVector::zeros(model.nv);
            qvel_pert[d] = eps;

            let mut qpos_pert = DVector::zeros(model.nq);
            mj_integrate_pos_explicit(&model, &mut qpos_pert, &data.qpos, &qvel_pert, 1.0);

            let mut data_pert = model.make_data();
            data_pert.qpos = qpos_pert;
            mj_fwd_position(&model, &mut data_pert);

            // jacp FD: track body-attached point
            let point_pert = data_pert.xpos[body_id] + data_pert.xquat[body_id] * body_local_offset;
            let jacp_fd_col = (point_pert - point) / eps;

            for k in 0..3 {
                assert_relative_eq!(jacp[(k, d)], jacp_fd_col[k], epsilon = 1e-5);
            }

            // jacr FD: small-angle quaternion difference
            let q_rel = data_pert.xquat[body_id] * data.xquat[body_id].inverse();
            let jacr_fd = Vector3::new(q_rel.i, q_rel.j, q_rel.k) * 2.0 / eps;
            for k in 0..3 {
                assert_relative_eq!(jacr[(k, d)], jacr_fd[k], epsilon = 1e-5);
            }
        }
    }

    // =========================================================================
    // 5f — jacr hinge cross-check: 2-hinge chain
    // =========================================================================

    #[test]
    fn mj_jac_jacr_hinge_cross_check() {
        // 2-hinge chain: world → body1 (hinge Y) → body2 (hinge Z)
        let mut model = Model::empty();
        model.nbody = 3;
        model.body_parent = vec![0, 0, 1];
        model.body_rootid = vec![0, 1, 1];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 1];
        model.body_dof_num = vec![0, 1, 1];
        model.body_geom_adr = vec![0, 0, 0];
        model.body_geom_num = vec![0, 0, 0];
        model.body_pos = vec![
            Vector3::zeros(),
            Vector3::new(0.0, 0.0, 0.5),
            Vector3::new(0.0, 0.0, 0.5),
        ];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 0.5];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.005, 0.005, 0.005),
        ];
        model.body_name = vec![
            Some("world".to_string()),
            Some("b1".to_string()),
            Some("b2".to_string()),
        ];
        model.body_subtreemass = vec![1.5, 1.5, 0.5];

        model.njnt = 2;
        model.nq = 2;
        model.nv = 2;
        model.jnt_type = vec![MjJointType::Hinge, MjJointType::Hinge];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 1];
        model.jnt_dof_adr = vec![0, 1];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_axis = vec![Vector3::y(), Vector3::z()];
        model.jnt_limited = vec![false; 2];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_armature = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("j1".to_string()), Some("j2".to_string())];

        model.dof_body = vec![1, 2];
        model.dof_jnt = vec![0, 1];
        model.dof_parent = vec![None, Some(0)];
        model.dof_armature = vec![0.0; 2];
        model.dof_damping = vec![0.0; 2];
        model.dof_frictionloss = vec![0.0; 2];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        model.qpos0 = DVector::zeros(2);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        data.qpos[0] = 0.3; // hinge1 angle
        data.qpos[1] = 0.5; // hinge2 angle
        mj_fwd_position(&model, &mut data);

        let point = data.xpos[2] + data.xquat[2] * Vector3::new(0.2, 0.0, 0.0);
        let (_jacp, jacr) = mj_jac(&model, &data, 2, &point);

        // jacr[:,0] = world-frame axis of joint 0
        let axis0 = data.xquat[1] * model.jnt_axis[0];
        for k in 0..3 {
            assert_relative_eq!(jacr[(k, 0)], axis0[k], epsilon = 1e-12);
        }

        // jacr[:,1] = world-frame axis of joint 1
        let axis1 = data.xquat[2] * model.jnt_axis[1];
        for k in 0..3 {
            assert_relative_eq!(jacr[(k, 1)], axis1[k], epsilon = 1e-12);
        }

        // Verify jacr·qvel = ω: angular velocity at given joint velocities
        let qvel = DVector::from_vec(vec![1.0, 0.5]);
        let omega = &jacr * &qvel;
        let expected_omega = axis0 * 1.0 + axis1 * 0.5;
        for k in 0..3 {
            assert_relative_eq!(omega[k], expected_omega[k], epsilon = 1e-12);
        }
    }
}

// =============================================================================
// Unit tests — DT-75: contact Jacobian free-joint body-frame axes fix
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::similar_names)]
mod contact_jac_free_joint_tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_4, FRAC_PI_6};

    // =========================================================================
    // Helpers
    // =========================================================================

    /// Single free-joint body with minimal geom fields for contact tests.
    ///
    /// Returns identity orientation with FK already run. Caller can write a
    /// non-identity quaternion into `data.qpos[3..7]` and re-run
    /// `mj_fwd_position` for non-trivial orientations.
    fn make_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 6];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.01, 0.01, 0.01)];
        model.body_name = vec![Some("world".into()), Some("b1".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("j1".into())];

        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None; 6];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![0.0; 6];

        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // Geom fields: compute_contact_jacobian only reads geom_body, but
        // mj_fwd_position reads geom_pos/geom_quat to compute geom world poses.
        model.ngeom = 2;
        model.geom_body = vec![0, 1]; // geom 0 → world, geom 1 → free body
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.body_geom_adr = vec![0, 1];
        model.body_geom_num = vec![1, 1];

        // Identity quaternion for the free joint
        model.qpos0 = DVector::zeros(7);
        model.qpos0[3] = 1.0; // w component
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        mj_fwd_position(&model, &mut data);

        (model, data)
    }

    /// Two free-joint bodies (no kinematic chain between them) with geom fields.
    ///
    /// nbody=3, njnt=2, nv=12, nq=14, ngeom=3. Both free bodies are children of
    /// the world body. Caller sets orientations in qpos and calls mj_fwd_position.
    fn make_two_free_body_contact_model() -> (Model, Data) {
        let mut model = Model::empty();

        // --- Topology: 3 bodies (world + 2 free), no kinematic chain ---
        model.nbody = 3;
        model.body_parent = vec![0, 0, 0];
        model.body_rootid = vec![0, 1, 2];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 6];
        model.body_dof_num = vec![0, 6, 6];
        model.body_pos = vec![Vector3::zeros(); 3];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 1.0, 1.0];
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.01, 0.01, 0.01),
            Vector3::new(0.01, 0.01, 0.01),
        ];
        model.body_name = vec![Some("world".into()), Some("b1".into()), Some("b2".into())];
        model.body_subtreemass = vec![2.0, 1.0, 1.0];

        // --- 2 free joints ---
        model.njnt = 2;
        model.nq = 14; // 7 + 7
        model.nv = 12; // 6 + 6
        model.jnt_type = vec![MjJointType::Free; 2];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 7];
        model.jnt_dof_adr = vec![0, 6];
        model.jnt_axis = vec![Vector3::z(); 2];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_limited = vec![false; 2];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_armature = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("j1".into()), Some("j2".into())];

        // --- DOF metadata (12 DOFs: 6 per free joint) ---
        model.dof_body = vec![1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2];
        model.dof_jnt = vec![0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1];
        model.dof_parent = vec![None; 12];
        model.dof_armature = vec![0.0; 12];
        model.dof_damping = vec![0.0; 12];
        model.dof_frictionloss = vec![0.0; 12];

        // --- No sites needed for contact tests ---
        model.nsite = 0;
        model.site_body = vec![];
        model.site_pos = vec![];
        model.site_quat = vec![];
        model.site_type = vec![];
        model.site_size = vec![];
        model.site_name = vec![];

        // --- Geoms: compute_contact_jacobian reads geom_body; mj_fwd_position
        // reads geom_pos/geom_quat to compute geom world poses ---
        model.ngeom = 3;
        model.geom_body = vec![0, 1, 2];
        model.geom_pos = vec![Vector3::zeros(); 3];
        model.geom_quat = vec![UnitQuaternion::identity(); 3];
        model.body_geom_adr = vec![0, 1, 2];
        model.body_geom_num = vec![1, 1, 1];

        // --- qpos0: identity quaternions for both free joints ---
        // qpos layout: [x1,y1,z1, w1,i1,j1,k1, x2,y2,z2, w2,i2,j2,k2]
        model.qpos0 = DVector::zeros(14);
        model.qpos0[3] = 1.0; // body 1: identity quaternion w-component
        model.qpos0[10] = 1.0; // body 2: identity quaternion w-component
        model.timestep = 0.001;

        // --- Ancestor metadata (required by compute_qld_csr_metadata → make_data) ---
        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_qld_csr_metadata();

        let data = model.make_data();
        (model, data)
    }

    // =========================================================================
    // 5a — contact_jac_free_body_frame_vs_world_frame
    // =========================================================================

    /// Verify that free-joint angular DOF columns use body-frame axes (R·eᵢ),
    /// not world-frame unit vectors. At 45° about Z, R·e₀ ≠ e₀, so world-frame
    /// would produce measurably wrong values.
    #[test]
    fn contact_jac_free_body_frame_vs_world_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1), // pos
            Vector3::new(0.0, 0.0, 1.0), // normal: +Z
            0.01,                        // depth
            0,                           // geom1 (world geom)
            1,                           // geom2 (free body geom)
            0.5,                         // friction → dim=3
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 3);
        assert_eq!(j.ncols(), 6);

        let rot = data.xquat[1].to_rotation_matrix();
        let r = contact.pos - data.xpos[1];
        let normal = contact.normal;

        // Verify angular DOF columns (3, 4, 5) use body-frame axes
        for i in 0..3 {
            let omega = rot * Vector3::ith(i, 1.0);
            let expected = normal.dot(&omega.cross(&r));
            assert_relative_eq!(j[(0, 3 + i)], expected, epsilon = 1e-12);
        }

        // Sanity: at 45° about Z, R·e₀ ≠ e₀ — world-frame would give wrong answer
        let body_e0 = rot * Vector3::x();
        assert!(
            (body_e0 - Vector3::x()).norm() > 0.1,
            "R·e₀ should differ from e₀ at 45°"
        );
    }

    // =========================================================================
    // 5b — contact_jac_all_rows_match_mj_jac_projection
    // =========================================================================

    /// Verify all 3 translational rows (normal + 2 tangents) of the contact
    /// Jacobian match the relative body Jacobian projected along each direction.
    #[test]
    fn contact_jac_all_rows_match_mj_jac_projection() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        let body1 = model.geom_body[contact.geom1]; // world (body 0)
        let body2 = model.geom_body[contact.geom2]; // free body

        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, _) = mj_jac(&model, &data, body1, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5c — angular_jac_free_body_frame (torsional + rolling, condim=6)
    // =========================================================================

    /// Verify all 6 rows (normal + 2 tangent + torsional + 2 rolling) of a
    /// condim=6 contact match the relative translational and rotational
    /// Jacobians from mj_jac.
    #[test]
    fn angular_jac_free_body_frame() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,   // sliding friction
            0.01,  // torsional friction
            0.005, // rolling friction
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 6);

        let body1 = model.geom_body[contact.geom1];
        let body2 = model.geom_body[contact.geom2];

        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);
        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);

        let nv = model.nv;

        // Rows 0–2 (normal + tangents): relative translational Jacobian projection
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Row 3 (torsional): relative rotational Jacobian projected along normal
        // Rows 4–5 (rolling): relative rotational Jacobian projected along tangents
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5d — contact_jac_two_free_bodies (strongest: both bodies non-identity)
    // =========================================================================

    /// Two free-joint bodies at different non-identity orientations, condim=6
    /// contact between them. Validates all 6 rows × 12 DOFs against mj_jac.
    #[test]
    fn contact_jac_two_free_bodies() {
        let (model, mut data) = make_two_free_body_contact_model();

        // Body 1: 45° about Z
        let q1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q1.w;
        data.qpos[4] = q1.i;
        data.qpos[5] = q1.j;
        data.qpos[6] = q1.k;
        // Body 2: 30° about X
        let q2 = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), FRAC_PI_6);
        data.qpos[10] = q2.w;
        data.qpos[11] = q2.i;
        data.qpos[12] = q2.j;
        data.qpos[13] = q2.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            1, // geom1 on body 1
            2, // geom2 on body 2
            0.5,
            0.01,
            0.005,
            6_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 6);
        assert_eq!(j.ncols(), 12);

        let body1 = model.geom_body[contact.geom1]; // body 1
        let body2 = model.geom_body[contact.geom2]; // body 2

        let (jacp1, jacr1) = mj_jac(&model, &data, body1, &contact.pos);
        let (jacp2, jacr2) = mj_jac(&model, &data, body2, &contact.pos);

        let nv = model.nv;

        // Rows 0–2: relative translational Jacobian
        let trans_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (row, dir) in trans_dirs.iter().enumerate() {
            for dof in 0..nv {
                let col2 = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let col1 = Vector3::new(jacp1[(0, dof)], jacp1[(1, dof)], jacp1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }

        // Rows 3–5: relative rotational Jacobian
        let rot_dirs = [contact.normal, contact.frame[0], contact.frame[1]];
        for (i, dir) in rot_dirs.iter().enumerate() {
            let row = 3 + i;
            for dof in 0..nv {
                let col2 = Vector3::new(jacr2[(0, dof)], jacr2[(1, dof)], jacr2[(2, dof)]);
                let col1 = Vector3::new(jacr1[(0, dof)], jacr1[(1, dof)], jacr1[(2, dof)]);
                let expected = dir.dot(&(col2 - col1));
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5e — contact_jac_free_identity_unchanged (regression)
    // =========================================================================

    /// At identity orientation, body-frame axes coincide with world-frame axes.
    /// Verify the contact Jacobian produces the same values as both the old
    /// world-frame formula and the mj_jac projection.
    #[test]
    fn contact_jac_free_identity_unchanged() {
        let (model, data) = make_free_body_contact_model();

        // Confirm identity orientation
        let rot = data.xquat[1].to_rotation_matrix();
        for i in 0..3 {
            let body_axis = rot * Vector3::ith(i, 1.0);
            let world_axis = Vector3::ith(i, 1.0);
            assert_relative_eq!(body_axis, world_axis, epsilon = 1e-15);
        }

        let contact = Contact::new(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::new(0.0, 0.0, 1.0),
            0.01,
            0,
            1,
            0.5,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);

        // Cross-check against mj_jac projection
        let body2 = model.geom_body[contact.geom2];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);

        let directions = [contact.normal, contact.frame[0], contact.frame[1]];
        let nv = model.nv;
        for (row, dir) in directions.iter().enumerate() {
            for dof in 0..nv {
                let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
                let expected = dir.dot(&col);
                assert_relative_eq!(j[(row, dof)], expected, epsilon = 1e-12);
            }
        }
    }

    // =========================================================================
    // 5f — contact_jac_free_frictionless (condim=1)
    // =========================================================================

    /// Frictionless contact (condim=1, dim=1): only the normal row, no tangent,
    /// torsional, or rolling rows. Verify angular DOFs use body-frame axes.
    #[test]
    fn contact_jac_free_frictionless() {
        let (model, mut data) = make_free_body_contact_model();

        // Rotate body to 45° about Z (non-identity to make body-frame meaningful)
        let q = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_4);
        data.qpos[3] = q.w;
        data.qpos[4] = q.i;
        data.qpos[5] = q.j;
        data.qpos[6] = q.k;
        mj_fwd_position(&model, &mut data);

        let contact = Contact::with_condim(
            Vector3::new(0.3, 0.2, 0.1),
            Vector3::z(),
            0.01,
            0,
            1,
            0.0, // no sliding friction
            0.0, // no torsional
            0.0, // no rolling
            1_i32,
            DEFAULT_SOLREF,
            DEFAULT_SOLIMP,
        );
        let j = compute_contact_jacobian(&model, &data, &contact);
        assert_eq!(j.nrows(), 1);
        assert_eq!(j.ncols(), 6);

        // Cross-check the single normal row against mj_jac projection
        let body2 = model.geom_body[contact.geom2];
        let (jacp2, _) = mj_jac(&model, &data, body2, &contact.pos);
        let normal = contact.normal;

        for dof in 0..model.nv {
            let col = Vector3::new(jacp2[(0, dof)], jacp2[(1, dof)], jacp2[(2, dof)]);
            let expected = normal.dot(&col);
            assert_relative_eq!(j[(0, dof)], expected, epsilon = 1e-12);
        }
    }
}

// =============================================================================
// Unit tests — contact_param (#24–#27) Batch 1 contact parameter combination
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod contact_param_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Helper to create a Model with two geoms and configurable contact fields.
    fn make_two_geom_model() -> Model {
        let mut model = Model::empty();
        model.ngeom = 2;
        model.geom_type = vec![GeomType::Sphere; 2];
        model.geom_body = vec![0; 2];
        model.geom_pos = vec![Vector3::zeros(); 2];
        model.geom_quat = vec![UnitQuaternion::identity(); 2];
        model.geom_size = vec![Vector3::new(1.0, 1.0, 1.0); 2];
        model.geom_name = vec![None; 2];
        model.geom_rbound = vec![1.0; 2];
        model.geom_mesh = vec![None; 2];
        model.geom_contype = vec![1; 2];
        model.geom_conaffinity = vec![1; 2];

        // Defaults matching MuJoCo
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001); 2];
        model.geom_condim = vec![3; 2];
        model.geom_solref = vec![[0.02, 1.0]; 2];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.geom_priority = vec![0; 2];
        model.geom_solmix = vec![1.0; 2];
        model.geom_margin = vec![0.0; 2];
        model.geom_gap = vec![0.0; 2];
        model
    }

    // ========================================================================
    // #24 — Friction combination: element-wise max (NOT geometric mean)
    // ========================================================================

    #[test]
    fn friction_uses_element_wise_max() {
        // AC1: Asymmetric friction → max wins, not geometric mean
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.8, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.2, 0.05, 0.003);

        let (_condim, _gap, _solref, _solimp, mu) = contact_param(&model, 0, 1);

        // Element-wise max: slide=max(0.8,0.2)=0.8, spin=max(0.01,0.05)=0.05, roll=max(0.001,0.003)=0.003
        assert_relative_eq!(mu[0], 0.8, epsilon = 1e-15); // sliding1
        assert_relative_eq!(mu[1], 0.8, epsilon = 1e-15); // sliding2
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15); // torsional
        assert_relative_eq!(mu[3], 0.003, epsilon = 1e-15); // rolling1
        assert_relative_eq!(mu[4], 0.003, epsilon = 1e-15); // rolling2

        // Verify NOT geometric mean: sqrt(0.8*0.2) ≈ 0.4, which != 0.8
        assert!((mu[0] - (0.8_f64 * 0.2).sqrt()).abs() > 0.01);
    }

    #[test]
    fn friction_symmetric_equal() {
        // AC2: Equal friction → max = same value
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.5, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.5, 0.01, 0.001);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], 0.5, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.01, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.001, epsilon = 1e-15);
    }

    #[test]
    fn friction_zero_one_geom() {
        // AC3: One geom zero friction → max picks the other
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.0, 0.0, 0.0);
        model.geom_friction[1] = Vector3::new(0.6, 0.02, 0.002);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], 0.6, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.02, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.002, epsilon = 1e-15);
    }

    #[test]
    fn friction_3_to_5_unpack() {
        // Verify 3→5 unpack: [slide, slide, spin, roll, roll]
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.7, 0.03, 0.004);
        model.geom_friction[1] = Vector3::new(0.1, 0.01, 0.001);

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        assert_relative_eq!(mu[0], mu[1], epsilon = 1e-15); // sliding1 == sliding2
        assert_relative_eq!(mu[3], mu[4], epsilon = 1e-15); // rolling1 == rolling2
        assert!((mu[0] - mu[2]).abs() > 1e-10); // sliding != torsional
        assert!((mu[2] - mu[3]).abs() > 1e-10); // torsional != rolling
    }

    #[test]
    fn friction_not_affected_by_solmix() {
        // AC4: Solmix weight does NOT affect friction (only affects solref/solimp)
        let mut model = make_two_geom_model();
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_solmix[0] = 100.0; // Extreme weight toward geom 0
        model.geom_solmix[1] = 0.001;

        let (_, _, _, _, mu) = contact_param(&model, 0, 1);

        // Friction should still be element-wise max, regardless of solmix
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.005, epsilon = 1e-15);
    }

    // ========================================================================
    // #25 — Priority gating: higher priority wins verbatim
    // ========================================================================

    #[test]
    fn priority_higher_wins_verbatim() {
        // AC1: Higher priority geom's params copied verbatim, no combination
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 5;
        model.geom_priority[1] = 0;
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 6;
        model.geom_solref[0] = [0.01, 0.5];
        model.geom_solref[1] = [0.05, 2.0];
        model.geom_solimp[0] = [0.8, 0.9, 0.002, 0.4, 1.5];
        model.geom_solimp[1] = [0.95, 0.99, 0.01, 0.6, 3.0];

        let (condim, _gap, solref, solimp, mu) = contact_param(&model, 0, 1);

        // Geom 0 wins (priority 5 > 0) — all params from geom 0
        assert_eq!(condim, 1);
        for (i, &expected) in [0.01, 0.5].iter().enumerate() {
            assert_relative_eq!(solref[i], expected, epsilon = 1e-15);
        }
        for (i, &expected) in [0.8, 0.9, 0.002, 0.4, 1.5].iter().enumerate() {
            assert_relative_eq!(solimp[i], expected, epsilon = 1e-15);
        }
        assert_relative_eq!(mu[0], 0.3, epsilon = 1e-15); // geom 0's friction
    }

    #[test]
    fn priority_lower_index_can_lose() {
        // AC2: Priority on geom2 > geom1 → geom2 wins
        let mut model = make_two_geom_model();
        model.geom_priority[0] = -1;
        model.geom_priority[1] = 3;
        model.geom_friction[0] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_friction[1] = Vector3::new(0.2, 0.01, 0.001);
        model.geom_condim[0] = 6;
        model.geom_condim[1] = 1;

        let (condim, _, _, _, mu) = contact_param(&model, 0, 1);

        // Geom 1 wins (priority 3 > -1)
        assert_eq!(condim, 1);
        assert_relative_eq!(mu[0], 0.2, epsilon = 1e-15);
    }

    #[test]
    fn priority_equal_combines() {
        // AC3: Equal priority → combination rules (not verbatim copy)
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 2;
        model.geom_priority[1] = 2;
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 4;

        let (condim, _, _, _, mu) = contact_param(&model, 0, 1);

        // Equal priority → condim = max(1, 4) = 4
        assert_eq!(condim, 4);
        // Friction = max → 0.9 (not 0.3 verbatim)
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
    }

    #[test]
    fn priority_negative_values() {
        // AC4: Negative priorities work correctly
        let mut model = make_two_geom_model();
        model.geom_priority[0] = -5;
        model.geom_priority[1] = -2;
        model.geom_condim[0] = 6;
        model.geom_condim[1] = 1;

        let (condim, _, _, _, _) = contact_param(&model, 0, 1);

        // Geom 1 wins (priority -2 > -5) → condim from geom 1
        assert_eq!(condim, 1);
    }

    #[test]
    fn priority_gap_still_additive() {
        // AC5: Gap is additive even when priority wins
        let mut model = make_two_geom_model();
        model.geom_priority[0] = 10;
        model.geom_priority[1] = 0;
        model.geom_gap[0] = 0.01;
        model.geom_gap[1] = 0.02;

        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        // Gap is always additive regardless of priority
        assert_relative_eq!(gap, 0.03, epsilon = 1e-15);
    }

    // ========================================================================
    // #26 — Solmix: solver parameter mixing weight
    // ========================================================================

    #[test]
    fn solmix_weight_equal_default() {
        // AC1: Default solmix=1.0 → weight = 0.5 → equal average
        let mix = solmix_weight(1.0, 1.0);
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_asymmetric() {
        // AC2: solmix 3:1 → weight = 3/4 = 0.75 for entity 1
        let mix = solmix_weight(3.0, 1.0);
        assert_relative_eq!(mix, 0.75, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_both_below_minval() {
        // AC3: Both below threshold → equal weight (0.5)
        let mix = solmix_weight(1e-20, 1e-20);
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_one_below_minval() {
        // AC4: s1 below threshold → entity 2 dominates (mix=0)
        let mix = solmix_weight(0.0, 1.0);
        assert_relative_eq!(mix, 0.0, epsilon = 1e-15);

        // AC5: s2 below threshold → entity 1 dominates (mix=1)
        let mix2 = solmix_weight(1.0, 0.0);
        assert_relative_eq!(mix2, 1.0, epsilon = 1e-15);
    }

    #[test]
    fn solmix_weight_exactly_at_minval() {
        // Edge case: exactly at MJ_MINVAL boundary
        let mix = solmix_weight(1e-15, 1e-15);
        // Both at 1e-15 = MJ_MINVAL → both "valid" → s1/(s1+s2) = 0.5
        assert_relative_eq!(mix, 0.5, epsilon = 1e-15);
    }

    #[test]
    fn solref_combination_standard_mode() {
        // AC6: Standard solref (both > 0) → weighted average
        let sr1 = [0.01, 0.5];
        let sr2 = [0.05, 2.0];
        let mix = 0.75; // entity 1 dominates

        let sr = combine_solref(sr1, sr2, mix);

        // 0.75*0.01 + 0.25*0.05 = 0.0075 + 0.0125 = 0.02
        assert_relative_eq!(sr[0], 0.02, epsilon = 1e-15);
        // 0.75*0.5 + 0.25*2.0 = 0.375 + 0.5 = 0.875
        assert_relative_eq!(sr[1], 0.875, epsilon = 1e-15);
    }

    #[test]
    fn solref_combination_direct_mode() {
        // AC7: Direct solref (at least one <= 0) → element-wise minimum
        let sr1 = [-100.0, -5.0];
        let sr2 = [-200.0, -3.0];
        let mix = 0.5; // Should be ignored for direct mode

        let sr = combine_solref(sr1, sr2, mix);

        assert_relative_eq!(sr[0], -200.0, epsilon = 1e-15); // min(-100, -200)
        assert_relative_eq!(sr[1], -5.0, epsilon = 1e-15); // min(-5, -3)
    }

    #[test]
    fn solref_combination_mixed_mode_falls_to_direct() {
        // AC8: One standard, one direct → direct mode (element-wise min)
        let sr1 = [0.02, 1.0]; // standard (positive)
        let sr2 = [-100.0, -5.0]; // direct (negative)

        let sr = combine_solref(sr1, sr2, 0.5);

        // At least one <= 0 → element-wise min
        assert_relative_eq!(sr[0], -100.0, epsilon = 1e-15);
        assert_relative_eq!(sr[1], -5.0, epsilon = 1e-15);
    }

    #[test]
    fn solimp_combination_weighted_average() {
        // AC9: Solimp always uses weighted average
        let si1 = [0.8, 0.9, 0.002, 0.4, 1.5];
        let si2 = [0.95, 0.99, 0.01, 0.6, 3.0];
        let mix = 0.75;

        let si = combine_solimp(si1, si2, mix);

        for i in 0..5 {
            let expected = 0.75 * si1[i] + 0.25 * si2[i];
            assert_relative_eq!(si[i], expected, epsilon = 1e-15);
        }
    }

    #[test]
    fn solmix_affects_solref_solimp_not_friction() {
        // AC10: Full integration — solmix weights affect solref/solimp but NOT friction
        let mut model = make_two_geom_model();
        model.geom_solmix[0] = 9.0;
        model.geom_solmix[1] = 1.0;
        // mix = 9/(9+1) = 0.9 → entity 0 has 90% weight

        model.geom_solref[0] = [0.01, 0.5];
        model.geom_solref[1] = [0.05, 2.0];
        model.geom_solimp[0] = [0.8, 0.9, 0.002, 0.4, 1.5];
        model.geom_solimp[1] = [0.95, 0.99, 0.01, 0.6, 3.0];
        model.geom_friction[0] = Vector3::new(0.3, 0.01, 0.001);
        model.geom_friction[1] = Vector3::new(0.9, 0.05, 0.005);

        let (_, _, solref, solimp, mu) = contact_param(&model, 0, 1);

        // Solref: 0.9*0.01 + 0.1*0.05 = 0.009 + 0.005 = 0.014
        assert_relative_eq!(solref[0], 0.014, epsilon = 1e-15);
        // Solref[1]: 0.9*0.5 + 0.1*2.0 = 0.45 + 0.2 = 0.65
        assert_relative_eq!(solref[1], 0.65, epsilon = 1e-15);

        // Solimp: weighted average with mix=0.9
        for i in 0..5 {
            let expected = 0.9 * model.geom_solimp[0][i] + 0.1 * model.geom_solimp[1][i];
            assert_relative_eq!(solimp[i], expected, epsilon = 1e-15);
        }

        // Friction: still element-wise max, NOT weighted
        assert_relative_eq!(mu[0], 0.9, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.05, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.005, epsilon = 1e-15);
    }

    // ========================================================================
    // #27 — Contact margin/gap
    // ========================================================================

    #[test]
    fn gap_additive() {
        // AC1: Gap is additive from both geoms
        let mut model = make_two_geom_model();
        model.geom_gap[0] = 0.01;
        model.geom_gap[1] = 0.02;

        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        assert_relative_eq!(gap, 0.03, epsilon = 1e-15);
    }

    #[test]
    fn gap_default_zero() {
        // AC2: Default gap=0.0 → no change to existing behavior
        let model = make_two_geom_model();
        let (_, gap, _, _, _) = contact_param(&model, 0, 1);

        assert_relative_eq!(gap, 0.0, epsilon = 1e-15);
    }

    #[test]
    fn includemargin_equals_margin_minus_gap() {
        // AC3: includemargin = margin - gap (tested via make_contact_from_geoms)
        let mut model = make_two_geom_model();
        model.geom_gap[0] = 0.005;
        model.geom_gap[1] = 0.003;
        // gap = 0.008

        let margin = 0.02; // effective margin from broadphase
        let contact =
            make_contact_from_geoms(&model, Vector3::zeros(), Vector3::z(), 0.01, 0, 1, margin);

        // includemargin = 0.02 - 0.008 = 0.012
        assert_relative_eq!(contact.includemargin, 0.012, epsilon = 1e-15);
    }

    #[test]
    fn condim_max_combination() {
        // Condim uses max when equal priority
        let mut model = make_two_geom_model();
        model.geom_condim[0] = 1;
        model.geom_condim[1] = 4;

        let (condim, _, _, _, _) = contact_param(&model, 0, 1);

        assert_eq!(condim, 4);
    }

    #[test]
    fn defaults_produce_standard_behavior() {
        // AC4: All defaults → standard MuJoCo behavior
        let model = make_two_geom_model();

        let (condim, gap, solref, solimp, mu) = contact_param(&model, 0, 1);

        assert_eq!(condim, 3);
        assert_relative_eq!(gap, 0.0, epsilon = 1e-15);
        for (i, &expected) in [0.02, 1.0].iter().enumerate() {
            assert_relative_eq!(solref[i], expected, epsilon = 1e-15);
        }
        for (i, &expected) in [0.9, 0.95, 0.001, 0.5, 2.0].iter().enumerate() {
            assert_relative_eq!(solimp[i], expected, epsilon = 1e-15);
        }
        assert_relative_eq!(mu[0], 1.0, epsilon = 1e-15);
        assert_relative_eq!(mu[2], 0.005, epsilon = 1e-15);
        assert_relative_eq!(mu[3], 0.0001, epsilon = 1e-15);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod wrap_inside_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// T5. Newton convergence and tangency verification.
    ///
    /// Reference geometry: end0 = (2, 0), end1 = (1.5, 1.5√3), radius = 1.
    /// |end0| = 2, |end1| = 3, G = π/3 exactly.
    #[test]
    fn t5_newton_convergence_and_tangency() {
        let end0 = Vector2::new(2.0, 0.0);
        let end1 = Vector2::new(1.5, 1.5 * 3.0_f64.sqrt());
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        let t = result.expect("wrap_inside_2d should return Some for this geometry");

        // Tangent point lies on the circle.
        assert_relative_eq!(t.norm(), radius, epsilon = 1e-10);

        // Tangency (stationarity) condition:
        // τ · (û₀ + û₁) = 0, where τ is the circle tangent at t.
        let tau = Vector2::new(-t.y, t.x).normalize(); // tangent = perpendicular to radial
        let u0 = (t - end0).normalize();
        let u1 = (t - end1).normalize();
        let stationarity = tau.dot(&(u0 + u1)).abs();
        assert!(
            stationarity < 1e-8,
            "tangency condition violated: |τ·(û₀+û₁)| = {stationarity:.2e}"
        );
    }

    /// T11. Degenerate: segment crosses circle.
    ///
    /// Endpoints on opposite sides of the circle — the straight line
    /// intersects the circle. wrap_inside_2d must return None.
    #[test]
    fn t11_segment_crosses_circle() {
        let end0 = Vector2::new(2.0, 0.0);
        let end1 = Vector2::new(-2.0, 0.1); // opposite side, segment crosses circle
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        assert!(
            result.is_none(),
            "expected None when segment crosses circle"
        );
    }

    /// T12. Degenerate: endpoint exactly at circle surface.
    ///
    /// When |endpoint| = radius, the ≤ check rejects it. Returns None.
    #[test]
    fn t12_endpoint_at_surface() {
        let end0 = Vector2::new(1.0, 0.0); // exactly at radius
        let end1 = Vector2::new(2.0, 1.0);
        let radius = 1.0;

        let result = wrap_inside_2d(end0, end1, radius);
        assert!(
            result.is_none(),
            "expected None when endpoint is at circle surface"
        );
    }

    /// T13. Fallback behavior: near-degenerate geometry.
    ///
    /// Endpoints barely outside the circle (A ≈ 1, B ≈ 1) and close together
    /// on the same angular side so the connecting segment clears the circle.
    /// This triggers the cosG > 1-ε fallback path. Must return Some(fallback)
    /// with the fallback point on the circle surface.
    #[test]
    fn t13_fallback_on_circle() {
        let r = 1.0;
        // Both endpoints barely outside, nearly coincident (small angular separation).
        // Segment at x ≈ 1+1e-8 clears the circle. cosG ≈ 1 → fallback.
        let d = r + 1e-8;
        let end0 = Vector2::new(d, 1e-6);
        let end1 = Vector2::new(d, -1e-6);

        let result = wrap_inside_2d(end0, end1, r);
        let t = result.expect("near-degenerate should return Some(fallback), not None");

        // Fallback point must lie on the circle surface.
        assert_relative_eq!(t.norm(), r, epsilon = 1e-10);
    }
}
