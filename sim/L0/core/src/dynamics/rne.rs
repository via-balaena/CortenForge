//! Recursive Newton-Euler algorithm for computing bias forces.
//!
//! Computes the bias force vector `c(q, qdot)` containing gravity, Coriolis,
//! and centrifugal forces for the equation of motion `M * qacc + c = τ`.
//! Also includes gravity compensation for bodies with non-zero `gravcomp`.

use nalgebra::Vector3;

use crate::dynamics::spatial::{SpatialVector, spatial_cross_force, spatial_cross_motion};
use crate::jacobian::mj_apply_ft;
use crate::joint_visitor::joint_motion_subspace;
use crate::types::{DISABLE_GRAVITY, Data, ENABLE_SLEEP, MIN_VAL, MjJointType, Model, SleepState};

/// Recursive Newton-Euler: compute bias forces (Coriolis + centrifugal + gravity).
///
/// The bias force vector `c(q, qdot)` contains:
/// - Gravity forces projected to joint space
/// - Coriolis/centrifugal forces: velocity-dependent terms
/// - Gyroscopic forces (ω × Iω): captured by the Featherstone backward pass
///
/// For the equation of motion: `M * qacc + c(q, qdot) = τ`
///
/// ## Implementation Notes
///
/// Two-phase approach:
/// 1. **Gravity**: O(n) computation using precomputed subtree mass and COM
/// 2. **Coriolis/Centrifugal/Gyroscopic**: O(n) Featherstone RNE forward-backward
///    pass with spatial algebra. The backward pass term `v ×* (I @ v)` naturally
///    captures gyroscopic torques in the angular part.
///
/// Free joint correction: the forward pass subtracts `[0; ω × v]` from the bias
/// acceleration to account for the convention difference between spatial acceleration
/// (`a_O - ω × v_O`) and the world-frame translational acceleration (`a_O`) used
/// by free joint DOFs.
///
/// Reference: Featherstone, "Rigid Body Dynamics Algorithms", Chapter 5
///
/// Reference: MuJoCo Computation docs - mj_rne section
// RNE inlined as a single function so the forward/backward Featherstone passes read end-to-end; paired body/parent identifiers are intentionally similar; indexed loops mutate parallel per-body force/acceleration buffers.
#[allow(
    clippy::too_many_lines,
    clippy::similar_names,
    clippy::needless_range_loop
)]
pub fn mj_rne(model: &Model, data: &mut Data) {
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;

    data.qfrc_bias.fill(0.0);

    if model.nv == 0 {
        return;
    }

    // ========== Gravity contribution (O(n) using precomputed subtree mass/COM) ==========
    // The bias force is what we need to SUBTRACT from applied forces.
    // For joint i on body b: τ_g[i] = J_i^T * (M_subtree * g)
    // where M_subtree is total mass below body b, and the torque acts at subtree COM

    // S4.2: Effective gravity — zero when DISABLE_GRAVITY is set.
    let grav = if model.disableflags & DISABLE_GRAVITY != 0 {
        Vector3::zeros()
    } else {
        model.gravity
    };

    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let jnt_body = model.jnt_body[jnt_id];

        // §16.5a: Skip RNE for sleeping bodies — bias forces are zeroed
        if sleep_enabled && data.body_sleep_state[jnt_body] == SleepState::Asleep {
            continue;
        }

        // Use precomputed subtree mass and COM for O(n) gravity
        let subtree_mass = data.subtree_mass[jnt_body];
        let subtree_com = data.subtree_com[jnt_body];

        // Negative because qfrc_bias opposes motion
        let gravity_force = -subtree_mass * grav;

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr] += torque.dot(&axis);
            }
            MjJointType::Slide => {
                let axis = data.xquat[jnt_body] * model.jnt_axis[jnt_id];
                data.qfrc_bias[dof_adr] += gravity_force.dot(&axis);
            }
            MjJointType::Ball => {
                let jpos = data.xpos[jnt_body] + data.xquat[jnt_body] * model.jnt_pos[jnt_id];
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                let body_torque = data.xquat[jnt_body].inverse() * torque;
                data.qfrc_bias[dof_adr] += body_torque.x;
                data.qfrc_bias[dof_adr + 1] += body_torque.y;
                data.qfrc_bias[dof_adr + 2] += body_torque.z;
            }
            MjJointType::Free => {
                // Linear gravity
                data.qfrc_bias[dof_adr] += gravity_force.x;
                data.qfrc_bias[dof_adr + 1] += gravity_force.y;
                data.qfrc_bias[dof_adr + 2] += gravity_force.z;
                // Angular: gravity torque from subtree COM relative to free joint position
                let jpos = Vector3::new(
                    data.qpos[model.jnt_qpos_adr[jnt_id]],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 1],
                    data.qpos[model.jnt_qpos_adr[jnt_id] + 2],
                );
                let r = subtree_com - jpos;
                let torque = r.cross(&gravity_force);
                data.qfrc_bias[dof_adr + 3] += torque.x;
                data.qfrc_bias[dof_adr + 4] += torque.y;
                data.qfrc_bias[dof_adr + 5] += torque.z;
            }
        }
    }

    // (§27F) Flex vertex gravity now handled by the joint loop above — each vertex
    // has a body with 3 slide joints, so the standard gravity path applies automatically.

    // NOTE: Gyroscopic torques (ω × Iω) are NOT computed directly here.
    // They are handled by the Featherstone RNE backward pass below, which
    // computes v ×* (I @ v) — this naturally includes the gyroscopic term
    // in the angular part of the spatial force. Computing it directly here
    // would double-count the gyroscopic effect.

    // ========== Coriolis/Centrifugal via Analytical Featherstone RNE ==========
    //
    // This is O(n) and replaces the O(n³) Christoffel symbol computation.
    // The algorithm:
    //   1. Forward pass: compute bias accelerations (velocity-dependent accelerations)
    //   2. Backward pass: compute bias forces and project to joint space
    //
    // Skip if all velocities are small (optimization for static/quasi-static cases)
    let max_qdot = data.qvel.iter().map(|v| v.abs()).fold(0.0, f64::max);
    if max_qdot < 1e-6 {
        return;
    }

    // Initialize bias accelerations and forces
    for i in 0..model.nbody {
        data.cacc_bias[i] = SpatialVector::zeros();
        data.cfrc_bias[i] = SpatialVector::zeros();
    }

    // ========== Forward Pass: Compute Bias Accelerations ==========
    // For each body, accumulate velocity-dependent accelerations.
    //
    // Featherstone's algorithm for bias acceleration:
    //   a_bias[i] = a_bias[parent] + c[i]
    //   c[i] = v[i] ×_m (S[i] @ qdot[i])  (velocity-product acceleration)
    //
    // The key insight: the bias acceleration arises from two sources:
    // 1. The acceleration of the parent frame (a_bias[parent])
    // 2. The velocity-product c[i] = v[i] ×_m v_joint, which captures
    //    centripetal and Coriolis accelerations
    //
    // Note: v[i] is the FULL body velocity (already computed in mj_fwd_velocity)

    // Pre-compute all joint motion subspaces once for RNE (same optimization as CRBA)
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    for body_id in 1..model.nbody {
        let parent_id = model.body_parent[body_id];

        // Start with parent's bias acceleration, transported from xpos[parent] to xpos[child].
        // Spatial motion transport: angular unchanged, linear += alpha × r.
        let parent_bias = data.cacc_bias[parent_id];
        let alpha_p = nalgebra::Vector3::new(parent_bias[0], parent_bias[1], parent_bias[2]);
        let a_p = nalgebra::Vector3::new(parent_bias[3], parent_bias[4], parent_bias[5]);
        let r = data.xpos[body_id] - data.xpos[parent_id];
        let a_transported = a_p + alpha_p.cross(&r);
        let mut a_bias = SpatialVector::zeros();
        a_bias[0] = alpha_p.x;
        a_bias[1] = alpha_p.y;
        a_bias[2] = alpha_p.z;
        a_bias[3] = a_transported.x;
        a_bias[4] = a_transported.y;
        a_bias[5] = a_transported.z;

        // Add velocity-product acceleration from joints on this body
        // c[i] = v[i] ×_m (S[i] @ qdot[i])
        // This uses the BODY velocity (not parent), as per Featherstone

        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let nv = model.jnt_type[jnt_id].nv();

            // Compute joint velocity contribution: v_joint = S @ qdot
            let mut v_joint = SpatialVector::zeros();
            for d in 0..nv {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }

            // Velocity-product acceleration: v[parent] ×_m v_joint
            // This is the centripetal/Coriolis acceleration contribution
            // Note: using parent velocity because v[i] = v[parent] + S@qdot,
            // and (S@qdot) ×_m (S@qdot) = 0, so v[i] ×_m (S@qdot) = v[parent] ×_m (S@qdot)
            let v_parent = data.cvel[parent_id];
            a_bias += spatial_cross_motion(v_parent, v_joint);

            // Free joint correction: the spatial acceleration convention uses
            // a_spatial = [α; a_O - ω × v_O], but the free joint's translational
            // DOFs use world-frame acceleration a_O directly. At qacc=0:
            //   a_spatial = S @ 0 + c = c (from parent velocity-product)
            // but the CORRECT spatial bias acceleration also includes the
            // -[0; ω × v] correction from the velocity convention difference.
            // Without this, the backward pass produces a spurious m*(ω × v)
            // force on the translational DOFs.
            if model.jnt_type[jnt_id] == MjJointType::Free {
                let omega_world = nalgebra::Vector3::new(
                    data.cvel[body_id][0],
                    data.cvel[body_id][1],
                    data.cvel[body_id][2],
                );
                let v_world = nalgebra::Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let correction = omega_world.cross(&v_world);
                a_bias[3] -= correction.x;
                a_bias[4] -= correction.y;
                a_bias[5] -= correction.z;
            }
        }

        data.cacc_bias[body_id] = a_bias;
    }

    // ========== Backward Pass: Compute Bias Forces ==========
    // For each body from leaves to root:
    //   f_bias[i] = I[i] @ a_bias[i] + v[i] ×* (I[i] @ v[i])
    //   f_bias[parent] += f_bias[i]
    //
    // Then project to joint space:
    //   τ_bias[dof] = S[i]^T @ f_bias[i]

    // First compute f_bias for each body using cinert (computed once in FK)
    for body_id in 1..model.nbody {
        let i = &data.cinert[body_id];
        let v = data.cvel[body_id];
        let a_bias = data.cacc_bias[body_id];

        // I @ v (momentum)
        let i_v = i * v;

        // I @ a_bias (inertial force from bias acceleration)
        let i_a = i * a_bias;

        // v ×* (I @ v) (gyroscopic/Coriolis force)
        let gyro = spatial_cross_force(v, i_v);

        // f_bias = I @ a_bias + v ×* (I @ v)
        data.cfrc_bias[body_id] = i_a + gyro;
    }

    // Propagate forces from leaves to root
    for body_id in (1..model.nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            // Add child's force to parent (already in world frame, no transform needed)
            let child_force = data.cfrc_bias[body_id];
            data.cfrc_bias[parent_id] += child_force;
        }
    }

    // ========== Project to Joint Space ==========
    // τ_bias[dof] = S^T @ f_bias[body]
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id]; // Use cached subspace
        let nv = model.jnt_type[jnt_id].nv();

        let f = data.cfrc_bias[body_id];

        // τ = S^T @ f
        for d in 0..nv {
            let mut tau = 0.0;
            for row in 0..6 {
                tau += s[(row, d)] * f[row];
            }
            data.qfrc_bias[dof_adr + d] += tau;
        }
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
        // Sleep filtering: skip bodies in sleeping trees
        if sleep_enabled && !data.tree_awake[model.body_treeid[b]] {
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
        mj_apply_ft(
            model,
            &data.xpos,
            &data.xquat,
            &force,
            &zero_torque,
            &point,
            b,
            &mut data.qfrc_gravcomp,
        );
    }

    has_gravcomp
}
