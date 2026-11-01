//! Tendon armature, the inertia a tendon adds along itself, as MuJoCo 3.5.0
//! adds it (`engine_core_smooth.c`): to the mass matrix (`mj_tendonArmature`)
//! and as a bias force (`mj_tendonBias`), which takes the time derivative of
//! a spatial tendon's Jacobian (`mj_tendonDot`). The arithmetic is MuJoCo's,
//! in its order, so the results are its bit for bit.

use nalgebra::{DMatrix, DVector, Vector3};

use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::object_velocity;
use crate::jacobian::{mj_jac, mj_jac_dot};
use crate::linalg::mju_dot;
use crate::types::{Data, ENABLE_SLEEP, Model, WrapType};

/// Each tendon's `armature · Jᵀ·J` added to the mass matrix on the matrix's
/// pattern, as MuJoCo's `mj_tendonArmature` (`engine_core_smooth.c:1745-1803`)
/// adds it after `mj_crb`: a dof's row holds the dof and the dofs it descends
/// from, or the diagonal alone for a simple dof, so a term between dofs of
/// different branches is dropped. A tendon without armature adds nothing; a
/// sleeping one is skipped while some dof sleeps.
pub fn mj_tendon_armature(model: &Model, data: &mut Data) {
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.nv_awake < model.nv;
    for t in 0..model.ntendon {
        if sleep_filter && crate::island::tendon_asleep(model, data, t) {
            continue;
        }
        let armature = model.tendon_armature[t];
        if armature == 0.0 {
            continue;
        }
        for i in 0..model.nv {
            let ten_j_i = data.ten_J[t][i];
            if ten_j_i == 0.0 {
                continue;
            }
            let row = model.M_rowadr[i]..model.M_rowadr[i] + model.M_rownnz[i];
            for &c in &model.M_colind[row] {
                data.qM[(i, c)] += armature * ten_j_i * data.ten_J[t][c];
                data.qM[(c, i)] = data.qM[(i, c)];
            }
        }
    }
}

/// `v` made unit, its length returned, as MuJoCo's `mju_normalize3`
/// (`engine_util_blas.c:120-133`): times `1/length`; below mjMINVAL, x.
fn normalize3(v: &mut Vector3<f64>) -> f64 {
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if norm < MJ_MINVAL {
        *v = Vector3::x();
    } else {
        let inv = 1.0 / norm;
        *v *= inv;
    }
    norm
}

/// `res += scale · (jac2 − jac1)ᵀ·v` for 3×nv Jacobians, as MuJoCo's
/// `mju_sub`, `mju_mulMatTVec` (`engine_util_blas.c:793-801`: row by row, a
/// zero entry of `v` skipped) and `mju_addToScl` sum it.
fn add_scaled_difference_t_vec(
    res: &mut DVector<f64>,
    jac2: &DMatrix<f64>,
    jac1: &DMatrix<f64>,
    v: &Vector3<f64>,
    scale: f64,
) {
    let mut tmp = DVector::<f64>::zeros(res.len());
    for r in 0..3 {
        if v[r] != 0.0 {
            for c in 0..res.len() {
                tmp[c] += (jac2[(r, c)] - jac1[(r, c)]) * v[r];
            }
        }
    }
    for c in 0..res.len() {
        res[c] += tmp[c] * scale;
    }
}

/// The time derivative of tendon `t`'s Jacobian, as MuJoCo's `mj_tendonDot`
/// (`engine_core_smooth.c:1128-1244`): 0 for a fixed tendon; for a spatial
/// one, each straight segment between sites on different bodies adds, divided
/// by the pulley divisor before it, `(J̇₂ − J̇₁)ᵀ·d + (J₂ − J₁)ᵀ·ḋ`, `d` the
/// unit direction from the first site to the second and `ḋ` its rate.
///
/// The caller passes no tendon that wraps a geom, which MuJoCo does not
/// support here ([`Model::try_make_data`] refuses one with armature).
fn mj_tendon_dot(model: &Model, data: &Data, t: usize) -> DVector<f64> {
    let mut jdot = DVector::zeros(model.nv);
    let adr = model.tendon_adr[t];
    if model.wrap_type[adr] == WrapType::Joint {
        return jdot;
    }
    let num = model.tendon_num[t];
    let mut divisor = 1.0;
    let mut j = 0;
    while j + 1 < num {
        let (type0, type1) = (model.wrap_type[adr + j], model.wrap_type[adr + j + 1]);
        if type0 == WrapType::Pulley || type1 == WrapType::Pulley {
            if type0 == WrapType::Pulley {
                divisor = model.wrap_prm[adr + j];
            }
            j += 1;
            continue;
        }
        let (site0, site1) = (model.wrap_objid[adr + j], model.wrap_objid[adr + j + 1]);
        let (body0, body1) = (model.site_body[site0], model.site_body[site1]);
        if body0 != body1 {
            let (p0, p1) = (data.site_xpos[site0], data.site_xpos[site1]);
            let (_, v0) = object_velocity(model, data, body0, &p0, None);
            let (_, v1) = object_velocity(model, data, body1, &p1, None);

            let mut dpnt = p1 - p0;
            let norm = normalize3(&mut dpnt);
            let mut dvel = v1 - v0;
            let dot = dpnt[0] * dvel[0] + dpnt[1] * dvel[1] + dpnt[2] * dvel[2];
            for k in 0..3 {
                dvel[k] += dpnt[k] * -dot;
            }
            let scale = if norm > MJ_MINVAL { 1.0 / norm } else { 0.0 };
            for k in 0..3 {
                dvel[k] *= scale;
            }

            let (jac1, _) = mj_jac_dot(model, data, body0, &p0);
            let (jac2, _) = mj_jac_dot(model, data, body1, &p1);
            add_scaled_difference_t_vec(&mut jdot, &jac2, &jac1, &dpnt, 1.0 / divisor);
            let (jac1, _) = mj_jac(model, data, body0, &p0);
            let (jac2, _) = mj_jac(model, data, body1, &p1);
            add_scaled_difference_t_vec(&mut jdot, &jac2, &jac1, &dvel, 1.0 / divisor);
        }
        j += 1;
    }
    jdot
}

/// Each tendon's bias force `armature · (J̇ · qvel) · J` added to
/// `qfrc_bias`, as MuJoCo's `mj_tendonBias` (`engine_core_smooth.c:2686-2738`)
/// adds it after `mj_rne`: a tendon without armature adds nothing; a sleeping
/// one is skipped while some tree sleeps.
pub fn mj_tendon_bias(model: &Model, data: &mut Data) {
    let sleep_filter = model.enableflags & ENABLE_SLEEP != 0 && data.ntree_awake < model.ntree;
    for t in 0..model.ntendon {
        if sleep_filter && crate::island::tendon_asleep(model, data, t) {
            continue;
        }
        let armature = model.tendon_armature[t];
        if armature == 0.0 {
            continue;
        }
        let jdot = mj_tendon_dot(model, data, t);
        let coef = armature * mju_dot(jdot.as_slice(), data.qvel.as_slice());
        if coef != 0.0 {
            for i in 0..model.nv {
                data.qfrc_bias[i] += data.ten_J[t][i] * coef;
            }
        }
    }
}
