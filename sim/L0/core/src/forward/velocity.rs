//! Velocity-stage forward kinematics.
//!
//! Computes body velocities and tendon velocities from joint velocities.
//! Corresponds to MuJoCo's velocity-stage of `engine_core_smooth.c`.

use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::spatial::spatial_cross_motion;
use crate::dynamics::{SpatialVector, mju_mul_dof_vec, object_velocity};
use crate::linalg::{dot_sparse, mju_dot};
use crate::types::{Data, ENABLE_SLEEP, MjJointType, Model};
use nalgebra::Vector3;

/// Velocity kinematics: each body's spatial velocity `cvel[b]`
/// `[angular; linear]` about its tree's subtree centre of mass, and each
/// dof's axis rate `cdof_dot`, as MuJoCo's `mj_comVel`
/// (`engine_core_smooth.c:2276-2343`); then the tendon velocities.
///
/// A body's velocity is its parent's plus each of its joints' `cdof·qvel`,
/// in the joints' order; a dof's `cdof_dot` is the velocity before its joint
/// crossed with its axis (`cvel ×` the axis, motion vectors), a free joint's
/// rotation taking the velocity after the joint's translation, whose own
/// rates are 0. Only the awake bodies are computed; a sleeping body keeps
/// its velocity and rates.
pub fn mj_fwd_velocity(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    // §16.27: Use indirection array for cache-friendly iteration over awake bodies.
    let use_body_ind = sleep_enabled && data.nbody_awake < model.nbody;

    // Invalidate subtree velocity fields from previous step
    data.flg_subtreevel = false;

    // World body has zero velocity
    data.cvel[0] = SpatialVector::zeros();

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

        let mut cvel = data.cvel[model.body_parent[body_id]];
        let bda = model.body_dof_adr[body_id];
        let end = bda + model.body_dof_num[body_id];
        let mut dof = bda;
        while dof < end {
            // A ball's three axes share the velocity before the ball; a free
            // joint's rotation, the velocity after its translation.
            let rotation = |data: &mut Data, cvel: &mut SpatialVector, first: usize| {
                for k in first..first + 3 {
                    data.cdof_dot[k] = spatial_cross_motion(*cvel, data.cdof[k]);
                }
                *cvel += mju_mul_dof_vec(
                    &data.cdof[first..first + 3],
                    &data.qvel.as_slice()[first..first + 3],
                );
            };
            match model.jnt_type[model.dof_jnt[dof]] {
                MjJointType::Free => {
                    for k in dof..dof + 3 {
                        data.cdof_dot[k] = SpatialVector::zeros();
                    }
                    cvel += mju_mul_dof_vec(
                        &data.cdof[dof..dof + 3],
                        &data.qvel.as_slice()[dof..dof + 3],
                    );
                    rotation(data, &mut cvel, dof + 3);
                    dof += 6;
                }
                MjJointType::Ball => {
                    rotation(data, &mut cvel, dof);
                    dof += 3;
                }
                MjJointType::Hinge | MjJointType::Slide => {
                    data.cdof_dot[dof] = spatial_cross_motion(cvel, data.cdof[dof]);
                    cvel +=
                        mju_mul_dof_vec(&data.cdof[dof..=dof], &data.qvel.as_slice()[dof..=dof]);
                    dof += 1;
                }
            }
        }

        data.cvel[body_id] = cvel;
    }

    // Tendon velocities, `ten_J · qvel`: at the sparse size over each
    // tendon's chain (`mju_mulMatVecSparse`, `mju_dotSparse`), at the dense
    // one over every dof (`mju_mulMatVec`, `mju_dot`), as MuJoCo's
    // `mj_fwdVelocity` (`engine_forward.c:232-238`).
    let nv = model.nv;
    let mut row = Vec::with_capacity(nv);
    for t in 0..model.ntendon {
        data.ten_velocity[t] = if model.is_sparse() {
            let colind = &data.ten_J_colind[t * nv..t * nv + data.ten_J_rownnz[t]];
            row.clear();
            row.extend(colind.iter().map(|&dof| data.ten_J[t][dof]));
            dot_sparse(&row, |k| data.qvel[colind[k]])
        } else {
            mju_dot(data.ten_J[t].as_slice(), data.qvel.as_slice())
        };
    }
}

/// Compute subtree linear velocity and angular momentum (§56).
///
/// O(n) three-phase backward accumulation:
/// 1. Initialize per-body linear momentum and spin angular momentum
/// 2. Backward accumulate linear momentum → convert to velocity
/// 3. Backward accumulate angular momentum with reference-point corrections
///
/// Requires `subtree_com` from the position stage, `cvel` from the velocity
/// stage, and `Model::body_subtreemass`.
///
/// # MuJoCo Equivalence
///
/// MuJoCo's `mj_subtreeVel()` (`engine_core_smooth.c:2346-2420`): when some
/// trees sleep, each phase walks the awake bodies only, and a sleeping body
/// adds nothing to its parent.
#[allow(clippy::similar_names)] // dx/dv/dp are physics convention (displacement/velocity/momentum)
pub fn mj_subtree_vel(model: &Model, data: &mut Data) {
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.nbody_awake < model.nbody;
    let nbody_awake = if sleep_filter {
        data.nbody_awake
    } else {
        model.nbody
    };
    let awake: Vec<usize> = if sleep_filter {
        data.body_awake_ind[..nbody_awake].to_vec()
    } else {
        (0..model.nbody).collect()
    };

    // Temporary: per-body 6D velocity at COM in world frame
    let mut body_vel: Vec<[f64; 6]> = vec![[0.0; 6]; model.nbody];

    // Phase 1: Per-body quantities
    for &b in &awake {
        // The body's velocity at its centre of mass (`mj_objectVelocity`).
        let (omega, v_com) = object_velocity(model, data, b, &data.xipos[b], None);

        body_vel[b] = [omega.x, omega.y, omega.z, v_com.x, v_com.y, v_com.z];

        // Linear momentum
        let mass = model.body_mass[b];
        data.subtree_linvel[b] = mass * v_com;

        // Spin angular momentum, `ximat · diag(I) · ximatᵀ · ω`, each product
        // summed left to right (`mju_mulMatTVec3`, `mji_mulMatVec3`).
        let inertia = model.body_inertia[b];
        let r = &data.ximat[b];
        let dv = Vector3::from_fn(|k, _| {
            (r[(0, k)] * omega.x + r[(1, k)] * omega.y + r[(2, k)] * omega.z) * inertia[k]
        });
        data.subtree_angmom[b] =
            Vector3::from_fn(|k, _| r[(k, 0)] * dv.x + r[(k, 1)] * dv.y + r[(k, 2)] * dv.z);
    }

    // Phase 2: Backward accumulate linear momentum, then convert to velocity
    for &b in awake.iter().rev() {
        if b > 0 {
            let parent = model.body_parent[b];
            let linvel_b = data.subtree_linvel[b];
            data.subtree_linvel[parent] += linvel_b;
        }
        // Momentum to velocity, times the reciprocal of the subtree's mass,
        // at least mjMINVAL (`engine_core_smooth.c:2383-2384`).
        data.subtree_linvel[b] *= 1.0 / model.body_subtreemass[b].max(MJ_MINVAL);
    }

    // Phase 3: Backward accumulate angular momentum
    for &b in awake[1..].iter().rev() {
        let parent = model.body_parent[b];
        let mass = model.body_mass[b];
        let smass = model.body_subtreemass[b];
        let v_body = Vector3::new(body_vel[b][3], body_vel[b][4], body_vel[b][5]);

        // Part A: Correct angmom from "about xipos[b]" to "about subtree_com[b]"
        let dx_a = data.xipos[b] - data.subtree_com[b];
        let dv_a = v_body - data.subtree_linvel[b];
        let dp_a = mass * dv_a;
        data.subtree_angmom[b] += dx_a.cross(&dp_a);

        // Part B: Transfer subtree b's angmom to parent
        let angmom_b = data.subtree_angmom[b];
        data.subtree_angmom[parent] += angmom_b;

        // Part C: Reference point correction (subtree_com[b] → subtree_com[parent])
        let dx_c = data.subtree_com[b] - data.subtree_com[parent];
        let linvel_b = data.subtree_linvel[b];
        let linvel_p = data.subtree_linvel[parent];
        let dv_c = linvel_b - linvel_p;
        let dp_c = smass * dv_c;
        data.subtree_angmom[parent] += dx_c.cross(&dp_c);
    }

    data.flg_subtreevel = true;
}
