//! Equality constraint Jacobian extraction.
//!
//! Computes Jacobian rows for connect, weld, joint, distance, and tendon equality
//! constraints. Each `extract_*_jacobian` function fills one or more rows
//! of the unified constraint arrays.
//! Corresponds to equality machinery in MuJoCo's `engine_core_constraint.c` (§15).

use nalgebra::{DMatrix, UnitQuaternion, Vector3};

use crate::jacobian::mj_jac;
use crate::quat::{mul_mat_vec3, mul_quat, mul_quat_axis, neg_quat};
use crate::types::{Data, MjObjectType, Model};

// =============================================================================
// Equality Constraint Jacobian Extraction (§15, Step 5)
//
// These functions extract explicit Jacobian rows and position violations
// from equality constraints; the rows' velocities are `J·qvel`
// (`crate::constraint::mj_reference_constraint`). Used by the unified constraint
// assembly for all solver types (PGS, CG, Newton).
// =============================================================================

/// Extracted equality constraint data for Newton solver assembly.
pub struct EqualityConstraintRows {
    /// Jacobian rows (nrows × nv). Connect: 3×nv, Weld: 6×nv, Joint: 1×nv, Distance: 1×nv.
    pub j_rows: DMatrix<f64>,
    /// Per-row constraint violation (signed).
    pub pos: Vec<f64>,
}

/// The world point of each body's anchor, as MuJoCo's equalities place it
/// (`mju_mulMatVec3(xmat, anchor)` then `+ xpos`).
fn anchor_point(data: &Data, body: usize, anchor: &[f64]) -> Vector3<f64> {
    let r = mul_mat_vec3(&data.xmat[body], &[anchor[0], anchor[1], anchor[2]]);
    Vector3::new(r[0], r[1], r[2]) + data.xpos[body]
}

fn quat_array(q: &UnitQuaternion<f64>) -> [f64; 4] {
    let q = q.quaternion();
    [q.w, q.i, q.j, q.k]
}

/// Connect or weld `eq_id`'s two points in the world and the bodies they sit
/// on, as MuJoCo 3.5.0's `mj_instantiateEquality` finds them: under the site
/// semantic (`eq_objtype` `Site`) the sites' positions and bodies
/// (`engine_core_constraint.c:438-444`, `:470-476`); else each body's anchor,
/// at `eq_data[anchors[k]..+3]` in its frame, placed in the world.
fn equality_points(
    model: &Model,
    data: &Data,
    eq_id: usize,
    anchors: [usize; 2],
) -> ([Vector3<f64>; 2], [usize; 2]) {
    let objects = [model.eq_obj1id[eq_id], model.eq_obj2id[eq_id]];
    if model.eq_objtype[eq_id] == MjObjectType::Site {
        return (
            objects.map(|s| data.site_xpos[s]),
            objects.map(|s| model.site_body[s]),
        );
    }
    let eq_data = &model.eq_data[eq_id];
    let point = |k: usize| anchor_point(data, objects[k], &eq_data[anchors[k]..anchors[k] + 3]);
    ([point(0), point(1)], objects)
}

/// A connect constraint's three rows, as MuJoCo 3.5.0's
/// `mj_instantiateEquality` builds them (`engine_core_constraint.c`,
/// `mjEQ_CONNECT`): its two points ([`equality_points`]; under the body
/// semantic `eq_data[0..3]` in body 1's frame, `[3..6]` in body 2's), the
/// error the first point minus the second, the rows the difference of the
/// two points' Jacobians (`mj_jacDifPair`, first minus second).
pub fn extract_connect_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
    let ([pos1, pos2], [body1, body2]) = equality_points(model, data, eq_id, [0, 3]);
    let pos_error = pos1 - pos2;
    let j = mj_jac(model, data, body1, &pos1).0 - mj_jac(model, data, body2, &pos2).0;
    EqualityConstraintRows {
        j_rows: j,
        pos: vec![pos_error.x, pos_error.y, pos_error.z],
    }
}

/// A weld constraint's six rows, as MuJoCo 3.5.0's `mj_instantiateEquality`
/// builds them (`mjEQ_WELD`): its two points ([`equality_points`]; under the
/// body semantic `eq_data[3..6]` in body 1's frame, `[0..3]` in body 2's) and
/// the translational rows as a connect's; the orientation error the axis
/// part of `neg(q2) · q1`, where under the body semantic `q1` is body 1's
/// orientation times `relpose` (`eq_data[6..10]`) and `q2` body 2's, and
/// under the site semantic each the site's body's orientation times the
/// site's (`:494-507`); each column of the rotational Jacobians' difference
/// corrected to `0.5 · neg(q2) · [0, column] · q1` (`mju_mulQuatAxis`,
/// `mju_mulQuat`). The rotational error and rows are then scaled by the
/// weld's `torquescale`, `eq_data[10]` (`engine_core_constraint.c:482`,
/// `:510`, `:530`).
pub fn extract_weld_jacobian(model: &Model, data: &Data, eq_id: usize) -> EqualityConstraintRows {
    let eq_data = &model.eq_data[eq_id];
    let nv = model.nv;
    let ([cpos1, cpos2], [body1, body2]) = equality_points(model, data, eq_id, [3, 0]);
    let pos_error = cpos1 - cpos2;
    let (trans1, rot1) = mj_jac(model, data, body1, &cpos1);
    let (trans2, rot2) = mj_jac(model, data, body2, &cpos2);
    let rot_dif = rot1 - rot2;

    let torquescale = eq_data[10];
    let (quat, quat1) = if model.eq_objtype[eq_id] == MjObjectType::Site {
        let turn = |site: usize, body: usize| {
            mul_quat(
                &quat_array(&data.xquat[body]),
                &quat_array(&model.site_quat[site]),
            )
        };
        (
            turn(model.eq_obj1id[eq_id], body1),
            neg_quat(&turn(model.eq_obj2id[eq_id], body2)),
        )
    } else {
        let relpose = [eq_data[6], eq_data[7], eq_data[8], eq_data[9]];
        (
            mul_quat(&quat_array(&data.xquat[body1]), &relpose),
            neg_quat(&quat_array(&data.xquat[body2])),
        )
    };
    let err = mul_quat(&quat1, &quat);

    let mut j = DMatrix::zeros(6, nv);
    j.rows_mut(0, 3).copy_from(&(trans1 - trans2));
    for col in 0..nv {
        let axis = [rot_dif[(0, col)], rot_dif[(1, col)], rot_dif[(2, col)]];
        let q3 = mul_quat(&mul_quat_axis(&quat1, &axis), &quat);
        for k in 0..3 {
            j[(3 + k, col)] = 0.5 * q3[1 + k] * torquescale;
        }
    }
    EqualityConstraintRows {
        j_rows: j,
        pos: vec![
            pos_error.x,
            pos_error.y,
            pos_error.z,
            err[1] * torquescale,
            err[2] * torquescale,
            err[3] * torquescale,
        ],
    }
}

/// Evaluate `poly(q)` and `poly'(q)` for a joint-equality polynomial in one Horner
/// pass. `coef[k]` is the coefficient of `qᵏ`, so the full degree-`N` polynomial is
/// `Σ coef[k]·qᵏ` over all slots. Used for a polynomial past MuJoCo's quartic
/// ([`joint_error`]): gait2392's coupled knee needs degree-8 to track the
/// tibiofemoral roll-glide SimmSpline to ≤0.17 mm (G3 spike); `eq_data` carries up
/// to 11 coefficients (degree-10 ceiling).
fn eval_poly_and_deriv(coef: &[f64], q: f64) -> (f64, f64) {
    // Horner with synthetic derivative: walk coefficients high → low.
    let n = coef.len();
    let mut p = coef[n - 1]; // poly value
    let mut d = 0.0; // poly derivative
    for k in (0..n - 1).rev() {
        d = d * q + p;
        p = p * q + coef[k];
    }
    (p, d)
}

/// A two-joint equality's error and its coupling's derivative in `dif`, from
/// each joint's offset from its `qpos0`: `x1` the first's, `dif` the
/// second's. MuJoCo 3.5.0's coupling is a quartic, and the error
/// `x1 − data[0] − (data[1]·dif + … + data[4]·dif⁴)`, in this arithmetic
/// (`engine_core_constraint.c:589-594`); where a coefficient past the quartic
/// is not 0 (registry `D-POLYCOEF-DEGREE`), `x1 −` the whole polynomial by
/// [`eval_poly_and_deriv`].
fn joint_error(coef: &[f64; 11], x1: f64, dif: f64) -> (f64, f64) {
    if coef[5..].iter().any(|&c| c != 0.0) {
        let (poly, deriv) = eval_poly_and_deriv(coef, dif);
        return (x1 - poly, deriv);
    }
    let poly = coef[1] * dif
        + coef[2] * dif * dif
        + coef[3] * dif * dif * dif
        + coef[4] * dif * dif * dif * dif;
    let deriv =
        coef[1] + 2.0 * coef[2] * dif + 3.0 * coef[3] * dif * dif + 4.0 * coef[4] * dif * dif * dif;
    (x1 - coef[0] - poly, deriv)
}

/// A joint equality's row, as MuJoCo 3.5.0's `mj_instantiateEquality` builds
/// it (`engine_core_constraint.c:535-614`): each joint's position measured
/// from its `qpos0` (a hinge's or a slide's `ref`), the error
/// [`joint_error`], the row `+1` at the first joint's dof and the coupling's
/// derivative, negated, at the second's; with one joint, `q1 − qpos0₁ −
/// data[0]` and `+1`.
pub fn extract_joint_equality_jacobian(
    model: &Model,
    data: &Data,
    eq_id: usize,
) -> EqualityConstraintRows {
    let joint1_id = model.eq_obj1id[eq_id];
    let joint2_id = model.eq_obj2id[eq_id];
    let eq_data = &model.eq_data[eq_id];
    let qpos1_adr = model.jnt_qpos_adr[joint1_id];
    let x1 = data.qpos[qpos1_adr] - model.qpos0[qpos1_adr];

    let mut j = DMatrix::zeros(1, model.nv);
    j[(0, model.jnt_dof_adr[joint1_id])] = 1.0;
    let pos_error = if joint2_id < model.njnt {
        let qpos2_adr = model.jnt_qpos_adr[joint2_id];
        let dif = data.qpos[qpos2_adr] - model.qpos0[qpos2_adr];
        let (error, deriv) = joint_error(eq_data, x1, dif);
        j[(0, model.jnt_dof_adr[joint2_id])] = -deriv;
        error
    } else {
        x1 - eq_data[0]
    };
    EqualityConstraintRows {
        j_rows: j,
        pos: vec![pos_error],
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
        for d in 0..nv {
            j[(0, d)] = j1[d] - deriv * j2[d];
        }

        EqualityConstraintRows {
            j_rows: j,
            pos: vec![pos_error],
        }
    } else {
        // Single-tendon mode: (L1 - L1_0) - data[0] = 0
        let pos_error = (l1 - l1_0) - eq_data[0];

        for d in 0..nv {
            j[(0, d)] = j1[d];
        }

        EqualityConstraintRows {
            j_rows: j,
            pos: vec![pos_error],
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
        };
    };

    let mut j = DMatrix::zeros(1, nv);

    // The scalar constraint is d(|p1-p2|)/dt, Jacobian row = direction·(J_body1 - J_body2)
    add_body_point_jacobian_row(&mut j, 0, &direction, body1, p1, 1.0, model, data);
    add_body_point_jacobian_row(&mut j, 0, &direction, body2, p2, -1.0, model, data);

    EqualityConstraintRows {
        j_rows: j,
        pos: vec![scalar_error],
    }
}

/// Add `sign · direction · Jp` to row `row` of `j`, `Jp` the translational
/// Jacobian of `point` on `body_id` ([`mj_jac`]).
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
    let jacp = mj_jac(model, data, body_id, &point).0;
    for c in 0..model.nv {
        let v =
            direction.x * jacp[(0, c)] + direction.y * jacp[(1, c)] + direction.z * jacp[(2, c)];
        j[(row, c)] += sign * v;
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]
    use super::add_body_point_jacobian_row;
    use crate::forward::mj_fwd_position;
    use crate::jacobian::mj_jac;
    use crate::test_fixtures::builders::{add_body, add_hinge_joint, finalize};
    use crate::types::Model;
    use nalgebra::{DMatrix, Vector3};

    /// The equality-constraint point-Jacobian row must agree with `mj_jac`
    /// (FD-verified) on a MULTI-JOINT body: the row is `direction·jacp[:,k]`.
    /// Guards the constraint subsystem reads the partial-frame axis/anchor.
    #[test]
    fn constraint_point_jacobian_row_matches_mj_jac_multi_joint() {
        let ax0 = Vector3::new(1.0, 0.3, 0.0).normalize();
        let ax1 = Vector3::new(0.2, 1.0, 0.1).normalize();
        let mut m = Model::empty();
        let b = add_body(
            &mut m,
            0,
            "l0",
            Vector3::zeros(),
            1.0,
            Vector3::new(0.02, 0.03, 0.04),
            Vector3::new(0.01, -0.02, -0.15),
        );
        add_hinge_joint(&mut m, b, "h0", ax0, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        add_hinge_joint(&mut m, b, "h1", ax1, 0.0, 0.0, 0.0, false, (-3.0, 3.0));
        finalize(&mut m);

        let mut data = m.make_data();
        data.qpos.as_mut_slice().copy_from_slice(&[0.3, -0.35]);
        mj_fwd_position(&m, &mut data);

        let point = data.xpos[b] + data.xquat[b] * Vector3::new(0.1, -0.05, -0.2);
        let direction = Vector3::new(0.4, -0.7, 0.6).normalize();
        let (jacp, _) = mj_jac(&m, &data, b, &point);

        let mut j = DMatrix::zeros(1, m.nv);
        add_body_point_jacobian_row(&mut j, 0, &direction, b, point, 1.0, &m, &data);
        for k in 0..2 {
            let expected = jacp[(0, k)] * direction.x
                + jacp[(1, k)] * direction.y
                + jacp[(2, k)] * direction.z;
            approx::assert_relative_eq!(j[(0, k)], expected, epsilon = 1e-12);
        }
    }
}
