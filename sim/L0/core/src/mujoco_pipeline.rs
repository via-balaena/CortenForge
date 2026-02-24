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
// - dead_code/unused_imports: forward/ module now owns these functions; monolith copies
//   are retained temporarily for inline tests until Phase 8b/8c deletion
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
    clippy::imprecise_flops,
    dead_code,
    unused_imports
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

use crate::collision_shape::Aabb;
// CollisionShape moved to collision/narrow.rs (Phase 3)
// heightfield imports moved to collision/hfield.rs (Phase 3)
// mesh imports moved to collision/mesh_collide.rs (Phase 3)
// raycast_shape moved to sensor/position.rs (Phase 4)
// sdf imports moved to collision/sdf_collide.rs (Phase 3)
use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, Point3, UnitQuaternion, Vector3};
// Pose moved to collision/narrow.rs (Phase 3)
// Arc moved to sensor/position.rs (Phase 4)

// Re-imports from types module (accumulated during structural refactor phases 1–8c,
// removed in Phase 12 when the monolith shim is deleted).
pub(crate) use crate::types::ActuatorDynamics;
pub(crate) use crate::types::ActuatorTransmission;
pub(crate) use crate::types::BiasType;
pub(crate) use crate::types::ConstraintType;
pub(crate) use crate::types::DISABLE_ISLAND;
pub(crate) use crate::types::ENABLE_SLEEP;
pub(crate) use crate::types::EqualityType;
pub(crate) use crate::types::GainType;
pub(crate) use crate::types::GeomType;
pub(crate) use crate::types::Integrator;
pub(crate) use crate::types::MIN_AWAKE;
pub(crate) use crate::types::MjJointType;
pub(crate) use crate::types::Model;
pub(crate) use crate::types::SleepError;
pub(crate) use crate::types::SleepPolicy;
pub(crate) use crate::types::SleepState;
// SolverType: no longer re-imported — only used by constraint/mod.rs (imports directly from types)
pub(crate) use crate::types::StepError;
pub(crate) use crate::types::TendonType;

// Re-imports from linalg module (Phase 2 extraction, removed in Phase 12).
pub(crate) use crate::linalg::UnionFind;
pub(crate) use crate::linalg::cholesky_in_place;
pub(crate) use crate::linalg::cholesky_solve_in_place;
pub(crate) use crate::linalg::lu_factor_in_place;
pub(crate) use crate::linalg::lu_solve_factored;
pub(crate) use crate::linalg::mj_solve_sparse;
#[cfg(test)]
pub(crate) use crate::linalg::mj_solve_sparse_batch;

// Re-imports from dynamics module (Phase 2 extraction, removed in Phase 12).
pub use crate::dynamics::SpatialVector;
pub(crate) use crate::dynamics::compute_body_spatial_inertia;
// shift_spatial_inertia, spatial_cross_force, spatial_cross_motion: no longer re-imported —
// only called from dynamics/crba.rs, dynamics/rne.rs, derivatives.rs (all import directly).

// Re-imports from dynamics module (Phase 7 extraction, removed in Phase 12).
// DEFAULT_MASS_FALLBACK: no longer re-imported — only used by constraint/equality.rs (imports directly)
pub(crate) use crate::dynamics::crba::mj_crba;
#[cfg(test)]
pub(crate) use crate::dynamics::factor::mj_factor_sparse; // monolith: removed in Phase 12
pub(crate) use crate::dynamics::flex::mj_flex;
pub(crate) use crate::dynamics::rne::{mj_gravcomp, mj_rne};

// Re-imports from joint_visitor module (Phase 2 extraction, removed in Phase 12).
pub(crate) use crate::joint_visitor::JointContext;
pub(crate) use crate::joint_visitor::JointVisitor;
// joint_motion_subspace: no longer re-imported — only called from dynamics/ and derivatives.rs (import directly).

// Re-imports from collision module (Phase 3 extraction, removed in Phase 12).
use crate::collision::mj_collision;
use crate::collision::narrow::GEOM_EPSILON;
// check_collision_affinity, collide_geoms, apply_pair_overrides, narrowphase_sphere_geom,
// make_contact_flex_rigid: no longer re-imported — only called from collision/ internally.
// geom_to_collision_shape: no longer re-imported — sensor/position.rs imports from collision::narrow directly.
// contact_param, make_contact_from_geoms, collide_with_plane, combine_solref, combine_solimp,
// solmix_weight: no longer re-imported — tests moved to collision/ modules (Phase 3).

// Re-imports from tendon module (Phase 5 extraction, removed in Phase 12).
pub(crate) use crate::tendon::accumulate_point_jacobian;
pub(crate) use crate::tendon::apply_tendon_force;
pub(crate) use crate::tendon::mj_fwd_tendon;
pub(crate) use crate::tendon::subquat;

// Re-imports from sensor/energy modules (Phase 4 extraction, removed in Phase 12).
// Only pipeline-callers are re-imported; internal helpers resolved within their modules.
use crate::energy::mj_energy_pos; // monolith: removed in Phase 12
use crate::energy::mj_energy_vel; // monolith: removed in Phase 12
use crate::sensor::mj_sensor_acc; // monolith: removed in Phase 12
use crate::sensor::mj_sensor_pos; // monolith: removed in Phase 12
use crate::sensor::mj_sensor_postprocess; // monolith: removed in Phase 12
use crate::sensor::mj_sensor_vel; // monolith: removed in Phase 12

// Re-imports from constraint module (Phase 6 extraction, removed in Phase 12).
// Phase 6 step 5: assemble_unified_constraints moved to constraint/assembly.rs, removing
// the last monolith callers of: ball_limit_axis_angle, compute_aref, compute_diag_approx_exact,
// compute_impedance, compute_kbip, compute_regularization, normalize_quat4,
// extract_{connect,weld,joint_equality,tendon_equality,distance}_jacobian,
// compute_contact_jacobian, compute_point_velocity, quaternion_to_axis_angle.
use crate::constraint::assembly::tendon_deadband_displacement; // monolith: removed in Phase 8a
// Re-imports still needed by inline tests (removed when tests move in Phase 12).
pub(crate) use crate::constraint::impedance::DEFAULT_SOLIMP; // monolith: removed in Phase 12
pub(crate) use crate::constraint::impedance::DEFAULT_SOLREF; // monolith: removed in Phase 12
pub(crate) use crate::constraint::impedance::MJ_MINVAL; // monolith: removed in Phase 12
#[cfg(test)]
use crate::constraint::impedance::ball_limit_axis_angle; // test-only: removed in Phase 12
#[cfg(test)]
use crate::constraint::impedance::compute_impedance; // test-only: removed in Phase 12
#[cfg(test)]
use crate::constraint::jacobian::compute_contact_jacobian; // test-only: removed in Phase 12
use crate::constraint::mj_fwd_constraint_islands; // monolith: removed in Phase 12

// Re-imports from forward module (Phase 8a extraction, removed in Phase 12).
// mj_next_activation: no longer re-imported — integrate() moved to integrate/mod.rs (Phase 8b).
use crate::forward::mj_fwd_position; // monolith: removed in Phase 12

// Test-only re-imports from forward/muscle.rs: inline tests reference these.
// Removed when monolith inline tests are migrated in Phase 12.
#[cfg(test)]
use crate::forward::muscle_activation_dynamics;
#[cfg(test)]
use crate::forward::{muscle_gain_length, muscle_gain_velocity, muscle_passive_force, sigmoid};

// mj_integrate_pos, mj_normalize_quat: no longer re-imported — integrate() and
// integrate_without_velocity() moved to integrate/mod.rs (Phase 8b).
// tendon_all_dofs_sleeping, tendon_active_stiffness, accumulate_tendon_kd: no longer re-imported —
// all callers import directly from integrate::implicit.

// Re-imports from jacobian module (Phase 8a extraction, removed in Phase 12).
use crate::jacobian::mj_jac_site; // called by compute_muscle_params
// mj_differentiate_pos: used by inline tests. mj_integrate_pos_explicit: used by inline tests.
// (integrate()/mj_runge_kutta() moved to integrate/ module — these stay for test-only usage.)
#[cfg(test)]
use crate::jacobian::mj_differentiate_pos;
#[cfg(test)]
use crate::jacobian::mj_integrate_pos_explicit;
#[cfg(test)]
use crate::jacobian::{mj_jac, mj_jac_body, mj_jac_body_com, mj_jac_geom, mj_jac_point}; // used in inline tests

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

// Moved to joint_visitor.rs: JointContext, JointVisitor trait

// Moved to linalg.rs: UnionFind

// Moved to types/keyframe.rs: Keyframe struct
// (no re-export needed — all consumers import directly from types::keyframe)

// Moved to types/contact_types.rs: ContactPair, Contact, impl Contact, compute_tangent_frame
pub(crate) use crate::types::Contact;
// ContactPair, compute_tangent_frame: no longer re-imported — only used by collision/narrow.rs

// Moved to types/data.rs: Data struct, Clone impl
pub(crate) use crate::types::Data;

impl Model {
    // Moved to types/model_init.rs: empty()
    // Moved to joint_visitor.rs: visit_joints()

    // Moved to dynamics/factor.rs: compute_qld_csr_metadata()

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

    // Moved to tendon/mod.rs: compute_spatial_tendon_length0()

    // Moved to types/model_init.rs: compute_stat_meaninertia()

    // Moved to types/model_factories.rs: n_link_pendulum(), double_pendulum(),
    // spherical_pendulum(), free_body()
}

impl Data {
    // Moved to types/data.rs: qld_diag(), reset(), reset_to_keyframe()

    // Moved to energy.rs: total_energy()

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

    // Moved to forward/mod.rs: step(), forward(), forward_skip_sensors(), forward_core()

    // Moved to integrate/mod.rs: integrate(), integrate_without_velocity()
}

// ============================================================================
// MuJoCo Pipeline Functions (Phase 2)
// ============================================================================

// Moved to forward/check.rs: mj_check_pos, mj_check_vel, mj_check_acc

// Moved to forward/position.rs: mj_fwd_position

// Moved to forward/position.rs: aabb_from_geom, SweepAndPrune

// Moved to collision/mod.rs: check_collision_affinity, mj_collision, mj_collision_flex

// Moved to collision/flex_collide.rs: narrowphase_sphere_geom, make_contact_flex_rigid

// Moved to collision/narrow.rs: collide_geoms

// Moved to collision/narrow.rs: geom_to_collision_shape

// Moved to collision/narrow.rs: apply_pair_overrides

// Moved to collision/mod.rs: contact_param, contact_param_flex_rigid,
//                            solmix_weight, combine_solref, combine_solimp

// Moved to collision/narrow.rs: make_contact_from_geoms

// Moved to collision/narrow.rs: GEOM_EPSILON, AXIS_VERTICAL_THRESHOLD,
//                               AXIS_HORIZONTAL_THRESHOLD, CAP_COLLISION_THRESHOLD

// Moved to collision/hfield.rs: collide_with_hfield

// Moved to collision/sdf_collide.rs: collide_with_sdf

// Moved to collision/mesh_collide.rs: collide_with_mesh, collide_mesh_plane

// Moved to collision/plane.rs: collide_with_plane, collide_cylinder_plane_impl,
//                              collide_ellipsoid_plane_impl

// Moved to collision/pair_convex.rs: collide_sphere_sphere, collide_capsule_capsule,
//                                   collide_sphere_capsule, collide_sphere_box

// Moved to collision/pair_cylinder.rs: collide_cylinder_sphere, collide_cylinder_capsule,
//                                     collide_capsule_box, collide_box_box, test_sat_axis

// Moved to forward/position.rs: closest_point_segment, closest_points_segments

// Moved to energy.rs: mj_energy_pos, mj_energy_vel

// Moved to sensor/position.rs: mj_sensor_pos

// Moved to sensor/velocity.rs: mj_sensor_vel

// Moved to sensor/acceleration.rs: mj_sensor_acc

// Moved to sensor/postprocess.rs: sensor_write, sensor_write3, sensor_write4, mj_sensor_postprocess

// Moved to sensor/derived.rs: compute_subtree_com, compute_subtree_momentum,
// compute_body_acceleration, compute_body_angular_acceleration, compute_site_force_torque,
// is_body_in_subtree, compute_subtree_angmom

// Moved to dynamics/flex.rs: mj_flex

// Moved to forward/velocity.rs: mj_fwd_velocity

// Moved to tendon/mod.rs: mj_fwd_tendon (dispatch)
// Moved to tendon/fixed.rs: mj_fwd_tendon_fixed

// Moved to tendon/spatial.rs: mj_fwd_tendon_spatial, accumulate_point_jacobian

// Moved to jacobian.rs: mj_jac, mj_jac_site, mj_jac_body, mj_jac_point,
//                        mj_jac_body_com, mj_jac_geom, mj_apply_ft

// Moved to tendon/spatial.rs: apply_tendon_force, subquat

// Moved to tendon/wrap_math.rs: WrapResult, segments_intersect_2d, directional_wrap_angle,
// sphere_tangent_point, compute_tangent_pair, circle_tangent_2d, compute_tangent_pair_2d,
// sphere_wrapping_plane, wrap_inside_2d, sphere_wrap, cylinder_wrap

// Moved to forward/actuation.rs: mj_transmission_site, compute_contact_normal_jacobian,
//                                 mj_transmission_body, mj_transmission_body_dispatch,
//                                 mj_actuator_length

// Moved to forward/muscle.rs: muscle_gain_length, muscle_gain_velocity,
//                              muscle_passive_force, sigmoid, muscle_activation_dynamics,
//                              mj_next_activation

// Moved to forward/actuation.rs: mj_fwd_actuation

// Moved to dynamics/crba.rs: mj_crba, cache_body_effective_mass, MIN_INERTIA_THRESHOLD
// MIN_INERTIA_THRESHOLD now lives in constraint/equality.rs (moved in Phase 6 step 4).
// (DEFAULT_MASS_FALLBACK re-exported above from dynamics module)

// Moved to dynamics/rne.rs: mj_rne, mj_gravcomp

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

// Moved to forward/passive.rs: rotate_spatial_to_world, cross3, add_cross3, norm3,
//                               fluid_geom_semi_axes, ellipsoid_moment

// Moved to forward/passive.rs: mj_inertia_box_fluid, mj_ellipsoid_fluid, mj_fluid,
//                               mj_fwd_passive

// Moved to integrate/implicit.rs: tendon_all_dofs_sleeping, tendon_all_dofs_sleeping_fields,
//                                 tendon_active_stiffness, accumulate_tendon_kd

// Moved to constraint/assembly.rs: tendon_deadband_displacement

// Moved to forward/passive.rs: PassiveForceVisitor

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
pub(crate) fn mj_sleep(model: &Model, data: &mut Data) -> usize {
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
pub(crate) fn mj_check_qpos_changed(model: &Model, data: &mut Data) -> bool {
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
pub(crate) fn mj_island(model: &Model, data: &mut Data) {
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
pub(crate) fn mj_wake(model: &Model, data: &mut Data) -> bool {
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
pub(crate) fn mj_wake_collision(model: &Model, data: &mut Data) -> bool {
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
pub(crate) fn mj_wake_tendon(model: &Model, data: &mut Data) -> bool {
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
pub(crate) fn mj_wake_equality(model: &Model, data: &mut Data) -> bool {
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

// Moved to sensor/mod.rs: sensor_body_id

// ============================================================================
// Contact Jacobian Computation
// ============================================================================

// Moved to constraint/jacobian.rs: compute_flex_contact_jacobian,
// compute_contact_jacobian, add_angular_jacobian

// Moved to constraint/solver/noslip.rs: project_elliptic_cone, noslip_qcqp2,
// noslip_qcqp3, NoslipRowKind, noslip_postprocess

// Moved to constraint/equality.rs: DofKind, get_min_diagonal_mass,
// get_min_translational_mass, get_min_rotational_inertia, EqualityConstraintRows,
// extract_connect_jacobian, extract_weld_jacobian, extract_joint_equality_jacobian,
// extract_tendon_equality_jacobian, extract_distance_jacobian,
// add_body_point_jacobian_row, add_body_angular_jacobian_row

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
pub(crate) fn populate_efc_island(model: &Model, data: &mut Data) {
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

// Moved to constraint/mod.rs (Phase 6):
// mj_fwd_constraint_islands, build_m_impl_for_newton

// Moved to constraint/mod.rs (Phase 6):
// compute_qfrc_smooth_implicit, mj_fwd_constraint, compute_point_velocity

// (Deformable pipeline functions removed — replaced by unified flex solver)

// Moved to forward/acceleration.rs: mj_fwd_acceleration, mj_fwd_acceleration_explicit,
//                                    mj_fwd_acceleration_implicit, ImplicitSpringVisitor,
//                                    mj_fwd_acceleration_implicitfast,
//                                    mj_fwd_acceleration_implicit_full

// Moved to linalg.rs: lu_factor_in_place, lu_solve_factored

// Moved to integrate/euler.rs: mj_integrate_pos, PositionIntegrateVisitor,
//                              mj_normalize_quat, QuaternionNormalizeVisitor

// Moved to jacobian.rs: mj_differentiate_pos, mj_integrate_pos_explicit

// Moved to integrate/rk4.rs: mj_runge_kutta

// Moved to collision/plane.rs: primitive_collision_tests

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

// Moved to sensor/mod.rs: sensor_tests
// (test body removed — see sensor/mod.rs for all sensor_tests)
// ============================================================================
// In-place Cholesky tests
// ============================================================================

// Moved to linalg.rs: cholesky_tests

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
        let mut act_up: f64 = 0.0;
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
        let mut act_down: f64 = 1.0;
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

// Moved to tendon/spatial.rs: subquat_tests

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

// Moved to collision/mod.rs: contact_param_tests

// Moved to tendon/wrap_math.rs: wrap_inside_tests
