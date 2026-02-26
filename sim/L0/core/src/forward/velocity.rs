//! Velocity-stage forward kinematics.
//!
//! Computes body velocities and tendon velocities from joint velocities.
//! Corresponds to MuJoCo's velocity-stage of `engine_core_smooth.c`.

use crate::dynamics::SpatialVector;
use crate::types::{Data, ENABLE_SLEEP, MjJointType, Model};
use nalgebra::Vector3;

/// Velocity kinematics: compute body velocities from qvel.
pub fn mj_fwd_velocity(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    // §16.27: Use indirection array for cache-friendly iteration over awake bodies.
    let use_body_ind = sleep_enabled && data.nbody_awake < model.nbody;

    // Invalidate subtree velocity fields from previous step
    data.flg_subtreevel = false;

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

/// Compute subtree linear velocity and angular momentum (§56).
///
/// O(n) three-phase backward accumulation:
/// 1. Initialize per-body linear momentum and spin angular momentum
/// 2. Backward accumulate linear momentum → convert to velocity
/// 3. Backward accumulate angular momentum with reference-point corrections
///
/// Requires `subtree_com` and `subtree_mass` from position stage,
/// and `cvel` from velocity stage.
///
/// # MuJoCo Equivalence
///
/// Matches `mj_subtreeVel()` in `engine_core_smooth.c`.
#[allow(clippy::similar_names)] // dx/dv/dp are physics convention (displacement/velocity/momentum)
pub fn mj_subtree_vel(model: &Model, data: &mut Data) {
    // Temporary: per-body 6D velocity at COM in world frame
    let mut body_vel: Vec<[f64; 6]> = vec![[0.0; 6]; model.nbody];

    // Phase 1: Per-body quantities
    #[allow(clippy::needless_range_loop)] // b indexes data.*[b] and body_vel[b] together
    for b in 0..model.nbody {
        let cvel = &data.cvel[b];
        let omega = Vector3::new(cvel[0], cvel[1], cvel[2]);
        let v_origin = Vector3::new(cvel[3], cvel[4], cvel[5]);

        // Shift linear velocity from xpos[b] to xipos[b] (body COM)
        let dif = data.xipos[b] - data.xpos[b];
        let v_com = v_origin + omega.cross(&dif);

        body_vel[b] = [omega.x, omega.y, omega.z, v_com.x, v_com.y, v_com.z];

        // Linear momentum
        let mass = model.body_mass[b];
        data.subtree_linvel[b] = mass * v_com;

        // Spin angular momentum: ximat * diag(I) * ximat^T * omega
        let inertia = model.body_inertia[b];
        let ximat = &data.ximat[b];
        let omega_local = ximat.transpose() * omega;
        let i_omega_local = Vector3::new(
            inertia.x * omega_local.x,
            inertia.y * omega_local.y,
            inertia.z * omega_local.z,
        );
        data.subtree_angmom[b] = ximat * i_omega_local;
    }

    // Phase 2: Backward accumulate linear momentum, then convert to velocity
    for b in (0..model.nbody).rev() {
        if b > 0 {
            let parent = model.body_parent[b];
            let linvel_b = data.subtree_linvel[b];
            data.subtree_linvel[parent] += linvel_b;
        }
        let smass = data.subtree_mass[b];
        // MuJoCo uses MJ_MINVAL = 1e-15 as the zero-mass guard
        if smass > 1e-15 {
            data.subtree_linvel[b] /= smass;
        } else {
            data.subtree_linvel[b] = Vector3::zeros();
        }
    }

    // Phase 3: Backward accumulate angular momentum
    for b in (1..model.nbody).rev() {
        let parent = model.body_parent[b];
        let mass = model.body_mass[b];
        let smass = data.subtree_mass[b];
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
