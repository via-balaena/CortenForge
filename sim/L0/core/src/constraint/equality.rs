//! Equality constraint Jacobian extraction.
//!
//! Computes Jacobian rows for connect, weld, joint, distance, and tendon equality
//! constraints. Each `extract_*_jacobian` function fills one or more rows
//! of the unified constraint arrays.
//! Corresponds to equality machinery in MuJoCo's `engine_core_constraint.c` (§15).

use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};

use crate::constraint::compute_point_velocity;
use crate::types::{Data, MjJointType, Model};

/// Compute constraint velocity as J * qvel (matching MuJoCo's `mj_referenceConstraint`).
///
/// This guarantees consistency between the Jacobian and the velocity used in
/// Baumgarte stabilization, unlike computing velocity from cvel which can diverge.
fn compute_jqvel(j: &DMatrix<f64>, qvel: &DVector<f64>, nrows: usize, nv: usize) -> Vec<f64> {
    let mut vel = vec![0.0; nrows];
    for r in 0..nrows {
        for c in 0..nv {
            vel[r] += j[(r, c)] * qvel[c];
        }
    }
    vel
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
/// MuJoCo convention: each body has its own anchor point. eq_data layout:
///   [0..3] = anchor in body1's local frame (user-specified)
///   [3..6] = anchor in body2's local frame (auto-computed at model build)
///
/// Constraint: body1_anchor_world = body2_anchor_world (3D position match).
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

    let anchor1 = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);
    let anchor2 = Vector3::new(eq_data[3], eq_data[4], eq_data[5]);

    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let (p2, r2) = if body2 != 0 {
        (data.xpos[body2], data.xquat[body2])
    } else {
        (Vector3::zeros(), UnitQuaternion::identity())
    };

    // Each body's anchor in world frame
    let pos1 = p1 + r1 * anchor1;
    let pos2 = p2 + r2 * anchor2;
    let pos_error = pos1 - pos2;

    let mut j = DMatrix::zeros(3, nv);

    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        // Body1 Jacobian at its anchor point
        add_body_point_jacobian_row(&mut j, row, &direction, body1, pos1, 1.0, model, data);
        // Body2 Jacobian at its anchor point
        add_body_point_jacobian_row(&mut j, row, &direction, body2, pos2, -1.0, model, data);
    }

    // Velocity: J * qvel (consistent with Jacobian, matching MuJoCo)
    let vel = compute_jqvel(&j, &data.qvel, 3, nv);

    EqualityConstraintRows {
        j_rows: j,
        pos: vec![pos_error.x, pos_error.y, pos_error.z],
        vel,
    }
}

/// Extract weld constraint Jacobian (6 rows: 3 translational + 3 rotational).
///
/// MuJoCo convention: separate anchor points per body. eq_data layout:
///   [0..3] = anchor in body2's local frame
///   [3..6] = anchor in body1's local frame (auto-computed at model build)
///   [6..10] = relpose quaternion [w,x,y,z] = inv(q1_ref) * q2_ref
///
/// Constraint: pos_body1 = pos_body2 AND orientation match.
pub fn extract_weld_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
    let body1 = model.eq_obj1id[eq_id];
    let body2 = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];
    let nv = model.nv;

    let anchor_body2 = Vector3::new(eq_data[0], eq_data[1], eq_data[2]);
    let anchor_body1 = Vector3::new(eq_data[3], eq_data[4], eq_data[5]);
    let relpose_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
        eq_data[6], eq_data[7], eq_data[8], eq_data[9],
    ));

    let p1 = data.xpos[body1];
    let r1 = data.xquat[body1];
    let (p2, r2) = if body2 != 0 {
        (data.xpos[body2], data.xquat[body2])
    } else {
        (Vector3::zeros(), UnitQuaternion::identity())
    };

    // Each body's anchor in world frame
    let cpos1 = p1 + r1 * anchor_body1;
    let cpos2 = p2 + r2 * anchor_body2;

    // Position error (zero at reference configuration)
    let pos_error = cpos1 - cpos2;

    // Orientation error: imag(q2^{-1} * q1 * relpose).
    //
    // MuJoCo convention: the error quaternion is q2^{-1} * q1 * relpose.
    // At reference (q1*relpose = q2), this equals identity → error = 0.
    // The imaginary part approximates 0.5*axis_angle for small deviations.
    let quat_target = r1 * relpose_quat; // q1 * relpose
    let q_err = r2.inverse() * quat_target; // q2^{-1} * q1 * relpose
    let rot_error = Vector3::new(
        q_err.quaternion().i,
        q_err.quaternion().j,
        q_err.quaternion().k,
    );

    let mut j = DMatrix::zeros(6, nv);

    // Rows 0-2: translational Jacobian at each body's anchor point.
    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        add_body_point_jacobian_row(&mut j, row, &direction, body1, cpos1, 1.0, model, data);
        add_body_point_jacobian_row(&mut j, row, &direction, body2, cpos2, -1.0, model, data);
    }

    // Rows 3-5: rotational Jacobian with per-column quaternion correction.
    //
    // MuJoCo corrects each angular Jacobian column via:
    //   corrected = 0.5 * imag(q2^{-1} * [0, raw_col] * q1 * relpose)
    //
    // This rotates the Jacobian into the error frame (body2-local), ensuring
    // consistency with the orientation error. At reference pose this reduces to
    // 0.5 * R2^T * col. At identity it reduces to 0.5 * col (simple scaling).

    // Step 1: build uncorrected angular Jacobian difference (sign ±1, no 0.5 scale)
    for row in 0..3 {
        let direction = Vector3::ith(row, 1.0);
        add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body1, 1.0, model, data);
        add_body_angular_jacobian_row(&mut j, 3 + row, &direction, body2, -1.0, model, data);
    }

    // Step 2: per-column quaternion correction
    let q2_inv_raw = *r2.inverse().quaternion();
    let qt_raw = *quat_target.quaternion();
    for col in 0..nv {
        let ax = j[(3, col)];
        let ay = j[(4, col)];
        let az = j[(5, col)];
        if ax * ax + ay * ay + az * az < 1e-30 {
            continue;
        }
        // pure_q = [0, axis]
        let pure_q = nalgebra::Quaternion::new(0.0, ax, ay, az);
        // step1 = q2^{-1} * [0, axis]
        let step1 = q2_inv_raw * pure_q;
        // step2 = step1 * (q1 * relpose)
        let step2 = step1 * qt_raw;
        // corrected = 0.5 * imaginary part
        j[(3, col)] = 0.5 * step2.i;
        j[(4, col)] = 0.5 * step2.j;
        j[(5, col)] = 0.5 * step2.k;
    }

    // Velocity: J * qvel (consistent with Jacobian, matching MuJoCo)
    let vel = compute_jqvel(&j, &data.qvel, 6, nv);

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
        vel,
    }
}

/// Extract joint equality constraint Jacobian (1 row).
///
/// MuJoCo convention: `joint1 = poly(joint2)`.
/// Two-joint: constraint is `q1 − poly(q2) = 0`, Jacobian has +1 at dof1 and -poly'(q2) at dof2.
/// Single-joint: constraint is `q1 − c0 = 0`, Jacobian has +1 at dof1.
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

    let mut j = DMatrix::zeros(1, nv);

    if joint2_id < model.njnt {
        // Two-joint equality: constraint = q1 - poly(q2) = 0
        // MuJoCo convention: joint1 is the dependent joint, joint2 is independent.
        let qpos2_adr = model.jnt_qpos_adr[joint2_id];
        let dof2_adr = model.jnt_dof_adr[joint2_id];
        let q2 = data.qpos[qpos2_adr];
        let qd2 = data.qvel[dof2_adr];

        // poly(q2) = c0 + c1*q2 + c2*q2² + c3*q2³ + c4*q2⁴
        let q1_target = c0 + c1 * q2 + c2 * q2 * q2 + c3 * q2 * q2 * q2 + c4 * q2 * q2 * q2 * q2;

        // poly'(q2) = c1 + 2*c2*q2 + 3*c3*q2² + 4*c4*q2³
        let dpoly_dq2 = c1 + 2.0 * c2 * q2 + 3.0 * c3 * q2 * q2 + 4.0 * c4 * q2 * q2 * q2;
        let qd1_target = dpoly_dq2 * qd2;

        let pos_error = q1 - q1_target;
        let vel_error = qd1 - qd1_target;

        // Jacobian: d(q1 - poly(q2))/d_qvel = +1 at dof1, -poly'(q2) at dof2
        j[(0, dof1_adr)] = 1.0;
        j[(0, dof2_adr)] = -dpoly_dq2;

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
// Equality assembly takes the full per-constraint context (model, data, J/M, equality records, output buffers).
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
                    // Linear DOFs (world frame)
                    j[(row, dof_adr)] += sign * direction.x;
                    j[(row, dof_adr + 1)] += sign * direction.y;
                    j[(row, dof_adr + 2)] += sign * direction.z;
                    // Angular DOFs (body-local frame): use body rotation columns R·eᵢ,
                    // matching the Ball joint pattern and contact Jacobian (DT-75).
                    // Free joint qvel[3..6] is body-local angular velocity.
                    let jpos = Vector3::new(
                        data.qpos[model.jnt_qpos_adr[jnt_id]],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                        data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                    );
                    let r = point - jpos;
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let body_axis = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&body_axis.cross(&r));
                    }
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
                    // Angular DOFs (body-local frame): use body rotation columns R·eᵢ,
                    // matching the Ball joint pattern and contact Jacobian (DT-75).
                    let rot = data.xquat[jnt_body].to_rotation_matrix();
                    for i in 0..3 {
                        let body_axis = rot * Vector3::ith(i, 1.0);
                        j[(row, dof_adr + 3 + i)] += sign * direction.dot(&body_axis);
                    }
                }
            }
        }
        current_body = model.body_parent[current_body];
    }
}
