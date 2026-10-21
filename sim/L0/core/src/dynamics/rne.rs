//! Recursive Newton-Euler algorithm for computing bias forces.
//!
//! Computes the bias force vector `c(q, qdot)` containing gravity, Coriolis,
//! and centrifugal forces for the equation of motion `M * qacc + c = τ`.
//! Also includes gravity compensation for bodies with non-zero `gravcomp`.

use nalgebra::Vector3;

use crate::dynamics::spatial::{
    SpatialVector, mju_mul_dof_vec, mju_mul_inert_vec, spatial_cross_force,
};
use crate::jacobian::mj_apply_ft;
use crate::linalg::mji_dot6;
use crate::types::{DISABLE_GRAVITY, Data, ENABLE_SLEEP, MIN_VAL, Model};

/// The bias force `qfrc_bias`, gravity's and the velocities' (Coriolis,
/// centrifugal, gyroscopic) generalized force, as MuJoCo's `mj_rne` with
/// `flg_acc` 0 (`engine_core_smooth.c:2425-2488`).
///
/// Each body's acceleration `cacc_bias`, about its tree's subtree centre of
/// mass: the world's `[0; −gravity]` (0 with gravity disabled), a body's its
/// parent's plus `cdof_dot · qvel`. Its force, `cfrc_bias =
/// cinert · cacc_bias + cvel ×* (cinert · cvel)`, is summed into its
/// ancestors' (not the world's). Each dof's bias force is `cdof · cfrc_bias`
/// of its body. Only the awake bodies and dofs are computed: a sleeping dof
/// keeps the bias force of its last awake pass.
pub fn mj_rne(model: &Model, data: &mut Data) {
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.nbody_awake < model.nbody;
    let (nbody, nparent, nv) = if sleep_filter {
        (data.nbody_awake, data.nparent_awake, data.nv_awake)
    } else {
        (model.nbody, model.nbody, model.nv)
    };

    data.cacc_bias[0] = SpatialVector::zeros();
    if model.disableflags & DISABLE_GRAVITY == 0 {
        for k in 0..3 {
            data.cacc_bias[0][3 + k] = -model.gravity[k];
        }
    }

    for b in 1..nbody {
        let i = if sleep_filter {
            data.body_awake_ind[b]
        } else {
            b
        };
        let dofs = model.body_dof_adr[i]..model.body_dof_adr[i] + model.body_dof_num[i];
        let tmp = mju_mul_dof_vec(&data.cdof_dot[dofs.clone()], &data.qvel.as_slice()[dofs]);
        data.cacc_bias[i] = data.cacc_bias[model.body_parent[i]] + tmp;

        let inertia = data.cinert[i];
        let momentum = mju_mul_inert_vec(&inertia, &data.cvel[i]);
        data.cfrc_bias[i] = mju_mul_inert_vec(&inertia, &data.cacc_bias[i])
            + spatial_cross_force(data.cvel[i], momentum);
    }

    data.cfrc_bias[0] = SpatialVector::zeros();
    for b in (1..nparent).rev() {
        let i = if sleep_filter {
            data.parent_awake_ind[b]
        } else {
            b
        };
        let parent = model.body_parent[i];
        if parent != 0 {
            let force = data.cfrc_bias[i];
            data.cfrc_bias[parent] += force;
        }
    }

    for v in 0..nv {
        let i = if sleep_filter {
            data.dof_awake_ind[v]
        } else {
            v
        };
        data.qfrc_bias[i] = mji_dot6(
            data.cdof[i].as_slice(),
            data.cfrc_bias[model.dof_body[i]].as_slice(),
        );
    }
}

/// Compute gravity compensation forces for bodies with non-zero `gravcomp`.
///
/// Matches MuJoCo's `mj_gravcomp()` in `engine_passive.c`. For each body with
/// `gravcomp != 0`, applies an anti-gravity force `F = -gravity * mass * gravcomp`
/// at the body's center of mass (`xipos`), projecting through the kinematic chain
/// via `mj_apply_ft()` into `qfrc_gravcomp`.
///
/// Returns `true` if any gravcomp was applied (for conditional routing).
pub fn mj_gravcomp(model: &Model, data: &mut Data) -> bool {
    // S4.2: Guard — no gravcomp bodies, gravity disabled, or near-zero gravity.
    if model.ngravcomp == 0
        || model.disableflags & DISABLE_GRAVITY != 0
        || model.gravity.norm() < MIN_VAL
    {
        return false;
    }

    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let mut has_gravcomp = false;

    for b in 1..model.nbody {
        let gc = model.body_gravcomp[b];
        if gc == 0.0 {
            continue;
        }
        // Sleep filtering: skip bodies in sleeping trees (a static body is in
        // none, and MuJoCo computes it with the awake ones)
        let tree = model.body_treeid[b];
        if sleep_enabled && tree < model.ntree && !data.tree_awake[tree] {
            continue;
        }
        has_gravcomp = true;
        // Anti-gravity force at body CoM (xipos, NOT xpos).
        // Gravity acts at the CoM; using xpos would produce wrong torques
        // for bodies whose CoM doesn't coincide with their frame origin.
        let force = -model.gravity * (model.body_mass[b] * gc);
        let zero_torque = Vector3::zeros();
        let point = data.xipos[b];
        // Project through Jacobian to generalized forces
        let mut qfrc = std::mem::take(&mut data.qfrc_gravcomp);
        mj_apply_ft(model, data, &force, &zero_torque, &point, b, &mut qfrc);
        data.qfrc_gravcomp = qfrc;
    }

    has_gravcomp
}
