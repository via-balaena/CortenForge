//! SO(3)-aware position differentiation and integration.
//!
//! Provides `mj_differentiate_pos` (velocity from position difference) and
//! `mj_integrate_pos_explicit` (position from velocity integration), both
//! handling quaternion joints correctly on SO(3).
//!
//! Corresponds to the position integration sections of MuJoCo's
//! `engine_util_spatial.c`.

use crate::types::{MjJointType, Model};
use nalgebra::{DVector, UnitQuaternion, Vector3};

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
                // Right-multiply convention: q2 = q1 * q_delta  =>  q_delta = q1^-1 * q2
                // angular velocity = log(q_delta) / dt  (body-frame, consistent with integrate)
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

                // q_delta = q1.inverse() * q2  (right convention)
                let q_delta = q1.inverse() * q2;

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

                // Angular velocity from quaternion difference (right convention)
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

                let q_delta = q1.inverse() * q2;
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
