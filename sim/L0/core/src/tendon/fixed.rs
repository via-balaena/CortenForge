//! Fixed tendon kinematics — linear combination of joint positions.
//!
//! MuJoCo ref: `mj_tendon()` in `engine_core_smooth.c`.
//! Length uses `coef * qpos[jnt_qposadr]` (position space).
//! Jacobian uses `coef` at `jnt_dofadr` (velocity space).
//!
//! For hinge/slide joints, `jnt_qposadr == jnt_dofadr` so both are equivalent.
//! For ball/free joints, `jnt_qposadr != jnt_dofadr` — the length couples to
//! the first qpos component (quaternion w or position x), while the Jacobian
//! couples to the first DOF (angular velocity or linear velocity).

use crate::types::{Data, Model, WrapType};

/// Fixed tendon kinematics for a single tendon.
///
/// Fixed tendon length is a linear combination of joint positions:
///   L_t = Σ_w coef_w * qpos[jnt_qposadr_w]
///
/// The Jacobian is constant (configuration-independent):
///   J_t[dof_adr_w] = coef_w
///
/// `wrap_objid[w]` stores the DOF address (`jnt_dof_adr`) for the joint.
/// For length, we look up the joint's qpos address via `dof_jnt` → `jnt_qpos_adr`.
#[inline]
pub fn mj_fwd_tendon_fixed(model: &Model, data: &mut Data, t: usize) {
    let adr = model.tendon_adr[t];
    let num = model.tendon_num[t];

    // Zero the Jacobian row. For fixed tendons, only a few entries are non-zero.
    data.ten_J[t].fill(0.0);

    let mut length = 0.0;
    for w in adr..(adr + num) {
        debug_assert_eq!(model.wrap_type[w], WrapType::Joint);
        let dof_adr = model.wrap_objid[w];
        let coef = model.wrap_prm[w];

        // Length: use qpos address (position space), not dof address.
        // For hinge/slide: qpos_adr == dof_adr.
        // For ball/free: qpos_adr != dof_adr (quaternion/position vs angular/linear velocity).
        if dof_adr < model.nv {
            let jnt_id = model.dof_jnt[dof_adr];
            let qpos_adr = model.jnt_qpos_adr[jnt_id];
            if qpos_adr < data.qpos.len() {
                length += coef * data.qpos[qpos_adr];
            }
        }

        // Jacobian: dL/dqvel at dof_adr = coef (velocity space).
        if dof_adr < model.nv {
            data.ten_J[t][dof_adr] += coef;
        }
    }

    data.ten_length[t] = length;
}
