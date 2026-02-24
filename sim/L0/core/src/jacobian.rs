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

// =============================================================================
// Unit tests — mj_jac_site (relocated from monolith, Phase 12)
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod jac_site_tests {
    use super::*;
    use crate::forward::mj_fwd_position;
    use crate::tendon::accumulate_point_jacobian;
    use crate::types::{GeomType, Model};
    use approx::assert_relative_eq;
    use nalgebra::DVector;

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
// (relocated from monolith, Phase 12)
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::similar_names)]
mod mj_jac_tests {
    use super::*;
    use crate::forward::mj_fwd_position;
    use crate::types::{GeomType, Model};
    use approx::assert_relative_eq;
    use nalgebra::DVector;

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
