//! Fixed tendon kinematics — linear combination of joint positions.

use crate::types::{Data, Model, WrapType};

/// Fixed tendon kinematics for a single tendon.
///
/// Fixed tendon length is a linear combination of joint positions:
///   L_t = Σ_w coef_w * qpos[dof_adr_w]
///
/// The Jacobian is constant (configuration-independent):
///   J_t[dof_adr_w] = coef_w
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

        if dof_adr < data.qpos.len() {
            length += coef * data.qpos[dof_adr];
        }

        if dof_adr < model.nv {
            data.ten_J[t][dof_adr] += coef;
        }
    }

    data.ten_length[t] = length;
}
