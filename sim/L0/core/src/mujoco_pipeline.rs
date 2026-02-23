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
pub(crate) use crate::types::Model;
pub(crate) use crate::types::SleepError;
pub(crate) use crate::types::SleepPolicy;
pub(crate) use crate::types::SleepState;
pub(crate) use crate::types::SolverStat;
// SolverType: no longer re-imported — only used by constraint/mod.rs (imports directly from types)
pub(crate) use crate::types::StepError;
pub(crate) use crate::types::TendonType;

// Re-imports from linalg module (Phase 2 extraction, removed in Phase 12).
pub(crate) use crate::linalg::UnionFind;
pub(crate) use crate::linalg::cholesky_in_place;
pub(crate) use crate::linalg::cholesky_rank1_downdate;
pub(crate) use crate::linalg::cholesky_rank1_update;
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
use crate::constraint::solver::primal::compute_gradient_and_search; // monolith: removed in Phase 6 step 9/11
use crate::constraint::solver::primal::compute_gradient_and_search_sparse; // monolith: removed in Phase 6 step 9/11
use crate::constraint::solver::primal::evaluate_cost_at; // monolith: removed in Phase 6 step 8/11
use crate::constraint::solver::primal::primal_prepare; // monolith: removed in Phase 6 step 8/11
use crate::constraint::solver::primal::primal_search; // monolith: removed in Phase 6 step 8/11

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
pub(crate) fn aabb_from_geom(
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
pub(crate) struct SweepAndPrune {
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
    pub(crate) fn new(aabbs: Vec<Aabb>) -> Self {
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
    pub(crate) fn query_pairs(&self) -> Vec<(usize, usize)> {
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

/// Find closest point on line segment AB to point P.
#[inline]
pub(crate) fn closest_point_segment(
    a: Vector3<f64>,
    b: Vector3<f64>,
    p: Vector3<f64>,
) -> Vector3<f64> {
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
pub(crate) fn closest_points_segments(
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

// Moved to energy.rs: mj_energy_pos, mj_energy_vel

// Moved to sensor/position.rs: mj_sensor_pos

// Moved to sensor/velocity.rs: mj_sensor_vel

// Moved to sensor/acceleration.rs: mj_sensor_acc

// Moved to sensor/postprocess.rs: sensor_write, sensor_write3, sensor_write4, mj_sensor_postprocess

// Moved to sensor/derived.rs: compute_subtree_com, compute_subtree_momentum,
// compute_body_acceleration, compute_body_angular_acceleration, compute_site_force_torque,
// is_body_in_subtree, compute_subtree_angmom

// Moved to dynamics/flex.rs: mj_flex

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

// Moved to tendon/mod.rs: mj_fwd_tendon (dispatch)
// Moved to tendon/fixed.rs: mj_fwd_tendon_fixed

// Moved to tendon/spatial.rs: mj_fwd_tendon_spatial, accumulate_point_jacobian

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
pub(crate) fn mj_apply_ft(
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

// Moved to tendon/spatial.rs: apply_tendon_force, subquat

// Moved to tendon/wrap_math.rs: WrapResult, segments_intersect_2d, directional_wrap_angle,
// sphere_tangent_point, compute_tangent_pair, circle_tangent_2d, compute_tangent_pair_2d,
// sphere_wrapping_plane, wrap_inside_2d, sphere_wrap, cylinder_wrap

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

// Moved to constraint/assembly.rs: tendon_deadband_displacement

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
pub(crate) fn accumulate_tendon_kd(
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

// Moved to sensor/mod.rs: sensor_body_id

// ============================================================================
// Contact Jacobian Computation
// ============================================================================

// Moved to constraint/jacobian.rs: compute_flex_contact_jacobian,
// compute_contact_jacobian, add_angular_jacobian

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

// Moved to constraint/solver/mod.rs: decode_pyramid

// Moved to constraint/impedance.rs (Phase 6):
// compute_impedance, quaternion_to_axis_angle, compute_kbip, compute_aref,
// normalize_quat4, ball_limit_axis_angle, compute_diag_approx_exact,
// mj_solve_sparse_vec, compute_regularization, MIN_INERTIA_THRESHOLD (local copy),
// DEFAULT_SOLREF, DEFAULT_SOLIMP, MJ_MINVAL
//
// (function bodies removed — see constraint/impedance.rs)
// =============================================================================
// Shared Smooth Acceleration (§29.2)
// =============================================================================

// Moved to constraint/mod.rs (Phase 6): compute_qacc_smooth

// =============================================================================
// Unified Constraint Assembly (§15, Step 7)
// Moved to constraint/assembly.rs: assemble_unified_constraints

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
pub(crate) fn pgs_solve_unified(model: &Model, data: &mut Data) {
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
pub(crate) fn cg_solve_unified(model: &Model, data: &mut Data) {
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

// Moved to constraint/solver/mod.rs: compute_qfrc_constraint_from_efc

// Moved to constraint/solver/mod.rs: extract_qfrc_frictionloss

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
pub(crate) struct SparseHessian {
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
    pub(crate) fn solve(&self, x: &mut DVector<f64>) {
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

// Moved to constraint/solver/primal.rs: compute_gradient_and_search,
// compute_gradient_and_search_sparse, PrimalQuad, PrimalPoint, primal_prepare,
// primal_eval, primal_search, evaluate_cost_at

/// Result of the Newton solver. Used by Step 14 (PGS fallback).
#[derive(Debug)]
pub(crate) enum NewtonResult {
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
pub(crate) fn newton_solve(
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
pub(crate) fn noslip_postprocess(model: &Model, data: &mut Data) {
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

// Moved to linalg.rs: cholesky_in_place, cholesky_solve_in_place,
// cholesky_rank1_update, cholesky_rank1_downdate

// Moved to dynamics/factor.rs: mj_factor_sparse, mj_factor_sparse_selective

// Moved to linalg.rs: mj_solve_sparse, mj_solve_sparse_batch

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

// Moved to linalg.rs: lu_factor_in_place, lu_solve_factored

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
