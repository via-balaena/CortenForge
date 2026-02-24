//! Jacobian computation and position differentiation/integration.
//!
//! Provides the body/site/point Jacobian infrastructure (`mj_jac`, `mj_jac_site`,
//! etc.), force projection (`mj_apply_ft`), and SO(3)-aware position
//! differentiation/integration (`mj_differentiate_pos`, `mj_integrate_pos_explicit`).
//!
//! Corresponds to the Jacobian sections of MuJoCo's `engine_core_smooth.c`
//! and `engine_util_spatial.c`.

use crate::types::{Data, MjJointType, Model};
use nalgebra::{DMatrix, DVector, UnitQuaternion, Vector3};

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
