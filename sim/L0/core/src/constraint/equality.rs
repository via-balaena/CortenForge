//! Equality constraint Jacobian extraction.
//!
//! Computes Jacobian rows for connect, weld, joint, distance, and tendon equality
//! constraints. Each `extract_*_jacobian` function fills one or more rows
//! of the unified constraint arrays.
//! Corresponds to equality machinery in MuJoCo's `engine_core_constraint.c` (§15).

use nalgebra::{DMatrix, UnitQuaternion, Vector3};

use crate::constraint::compute_point_velocity;
use crate::constraint::impedance::quaternion_to_axis_angle;
use crate::dynamics::DEFAULT_MASS_FALLBACK;
use crate::types::{Data, MjJointType, Model};

/// Minimum inertia threshold for mass-matrix diagonal queries.
const MIN_INERTIA_THRESHOLD: f64 = 1e-10;

/// DOF type for mass matrix extraction.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum DofKind {
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
pub fn get_min_diagonal_mass(model: &Model, data: &Data, body_id: usize, kind: DofKind) -> f64 {
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
pub fn get_min_translational_mass(model: &Model, data: &Data, body_id: usize) -> f64 {
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
pub fn get_min_rotational_inertia(model: &Model, data: &Data, body_id: usize) -> f64 {
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
pub struct EqualityConstraintRows {
    /// Jacobian rows (nrows × nv). Connect: 3×nv, Weld: 6×nv, Joint: 1×nv, Distance: 1×nv.
    pub j_rows: DMatrix<f64>,
    /// Per-row constraint violation (signed).
    pub pos: Vec<f64>,
    /// Per-row constraint velocity (J·qvel).
    pub vel: Vec<f64>,
}

/// Extract connect constraint Jacobian (3 translational rows).
///
/// Constraint: anchor_on_body1 = body2_origin (3D position match).
/// Jacobian structure: 3×nv with sparse columns at both bodies' DOFs.
pub fn extract_connect_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
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
pub fn extract_weld_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
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
pub fn extract_joint_equality_jacobian(
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
pub fn extract_tendon_equality_jacobian(
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
pub fn extract_distance_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
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
pub fn add_body_point_jacobian_row(
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
pub fn add_body_angular_jacobian_row(
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
