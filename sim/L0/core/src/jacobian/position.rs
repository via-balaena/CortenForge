//! SO(3)-aware position differentiation and integration.
//!
//! Provides `mj_differentiate_pos` (velocity from position difference) and
//! `mj_integrate_pos_explicit` (position from velocity integration), both
//! handling quaternion joints correctly on SO(3).
//!
//! Corresponds to the position integration sections of MuJoCo's
//! `engine_util_spatial.c`.

use crate::quat::{quat_integrate, sub_quat};
use crate::types::{MjJointType, Model};
use nalgebra::DVector;

/// The velocity that moves `qpos1` to `qpos2` in time `dt`, as MuJoCo 3.5.0's
/// `mj_differentiatePos` computes it (`engine_support.c:608-638`).
///
/// A hinge's,
/// a slide's and a free joint's position `(qpos2 − qpos1)/dt`, and a ball's
/// and a free joint's rotation `mju_subQuat(q2, q1)` (the rotation vector of
/// `q1⁻¹·q2`, its angle taken in (−π, π]) times `1/dt`, the quaternions as
/// given. The tangent is at `qpos1`, in the body frame: for unit quaternions,
/// `qpos1` integrated by the result over `dt` is `qpos2`.
///
/// `dt` is not checked: 0 gives non-finite velocities, as MuJoCo's does.
pub fn mj_differentiate_pos(
    model: &Model,
    qvel: &mut DVector<f64>,
    qpos1: &DVector<f64>,
    qpos2: &DVector<f64>,
    dt: f64,
) {
    let quat = |q: &DVector<f64>, adr: usize| [q[adr], q[adr + 1], q[adr + 2], q[adr + 3]];
    for jnt_id in 0..model.njnt {
        let mut padr = model.jnt_qpos_adr[jnt_id];
        let mut vadr = model.jnt_dof_adr[jnt_id];
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                qvel[vadr] = (qpos2[padr] - qpos1[padr]) / dt;
            }
            kind @ (MjJointType::Free | MjJointType::Ball) => {
                if kind == MjJointType::Free {
                    for i in 0..3 {
                        qvel[vadr + i] = (qpos2[padr + i] - qpos1[padr + i]) / dt;
                    }
                    padr += 3;
                    vadr += 3;
                }
                let rotation = sub_quat(&quat(qpos2, padr), &quat(qpos1, padr));
                let inv_dt = 1.0 / dt;
                for i in 0..3 {
                    qvel[vadr + i] = rotation[i] * inv_dt;
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
                let mut quat = [
                    qpos[qpos_adr],
                    qpos[qpos_adr + 1],
                    qpos[qpos_adr + 2],
                    qpos[qpos_adr + 3],
                ];
                quat_integrate(
                    &mut quat,
                    [qvel[dof_adr], qvel[dof_adr + 1], qvel[dof_adr + 2]],
                    dt,
                );
                qpos_out.as_mut_slice()[qpos_adr..qpos_adr + 4].copy_from_slice(&quat);
            }

            MjJointType::Free => {
                // Linear position
                qpos_out[qpos_adr] = qpos[qpos_adr] + qvel[dof_adr] * dt;
                qpos_out[qpos_adr + 1] = qpos[qpos_adr + 1] + qvel[dof_adr + 1] * dt;
                qpos_out[qpos_adr + 2] = qpos[qpos_adr + 2] + qvel[dof_adr + 2] * dt;

                let mut quat = [
                    qpos[qpos_adr + 3],
                    qpos[qpos_adr + 4],
                    qpos[qpos_adr + 5],
                    qpos[qpos_adr + 6],
                ];
                quat_integrate(
                    &mut quat,
                    [qvel[dof_adr + 3], qvel[dof_adr + 4], qvel[dof_adr + 5]],
                    dt,
                );
                qpos_out.as_mut_slice()[qpos_adr + 3..qpos_adr + 7].copy_from_slice(&quat);
            }
        }
    }
}
