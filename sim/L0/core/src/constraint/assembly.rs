//! Constraint row assembly for all constraint types.
//!
//! Populates the unified `efc_*` arrays on [`Data`] with constraint Jacobians,
//! parameters, and metadata for equality, friction, limit, and contact
//! constraints. Corresponds to MuJoCo's `engine_core_constraint.c`.
//!
//! Equality and contact row assembly are delegated to
//! [`super::equality_assembly`] and [`super::contact_assembly`] respectively.

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::{
    MJ_MINVAL, ball_limit_axis_angle, compute_aref, compute_diag_approx_exact, compute_impedance,
    compute_kbip, compute_regularization, normalize_quat4,
};
use crate::types::flags::disabled;
use crate::types::{
    ConstraintState, ConstraintType, DISABLE_CONTACT, DISABLE_EQUALITY, DISABLE_FRICTIONLOSS,
    DISABLE_LIMIT, Data, EqualityType, MjJointType, Model,
};

use super::contact_assembly::{assemble_contact_rows, postprocess_pyramidal_r_scaling};
use super::equality_assembly::assemble_equality_rows;

/// Compute the deadband displacement for a tendon (DT-35).
///
/// Returns `length - upper` if `length > upper`, `length - lower` if
/// `length < lower`, `0.0` if inside the deadband `[lower, upper]`.
/// At the boundary (`length == lower` or `length == upper`), returns `0.0`
/// (spring disengaged — see "one-step delay" note in DT-35 Step 2).
#[inline]
pub fn tendon_deadband_displacement(length: f64, range: [f64; 2]) -> f64 {
    let [lower, upper] = range;
    if length > upper {
        length - upper
    } else if length < lower {
        length - lower
    } else {
        0.0
    }
}

/// Populate per-row metadata and compute impedance, KBIP, aref, diagApprox, R, D.
///
/// This is the shared finalizer called for every constraint row after the
/// Jacobian, position error, and velocity error have been written into the
/// `efc_*` arrays. It pushes type/pos/margin/vel/solref/solimp/floss/mu/dim/id,
/// computes impedance and regularization, then writes `efc_aref` and `efc_b`.
///
/// Increments `*row` by 1 on completion.
#[allow(clippy::too_many_arguments)]
pub fn finalize_constraint_row(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
    solref: [f64; 2],
    solimp: [f64; 5],
    pos: f64,
    margin: f64,
    vel: f64,
    floss: f64,
    ctype: ConstraintType,
    dim_val: usize,
    id_val: usize,
    mu_val: [f64; 5],
    bw_diag: f64,
) {
    data.efc_type.push(ctype);
    data.efc_pos.push(pos);
    data.efc_margin.push(margin);
    data.efc_vel[*row] = vel;
    data.efc_solref.push(solref);
    data.efc_solimp.push(solimp);
    data.efc_floss.push(floss);
    data.efc_mu.push(mu_val);
    data.efc_dim.push(dim_val);
    data.efc_id.push(id_val);

    // Impedance
    let violation = (pos - margin).abs();
    let imp = compute_impedance(solimp, violation);
    data.efc_imp.push(imp);

    // diagApprox: exact M⁻¹ solve or O(1) bodyweight approximation (DT-39)
    let diag = if model.diagapprox_bodyweight {
        bw_diag.max(MJ_MINVAL)
    } else {
        let j_row_slice: Vec<f64> = (0..nv).map(|col| data.efc_J[(*row, col)]).collect();
        compute_diag_approx_exact(&j_row_slice, nv, model, data)
    };
    data.efc_diagApprox.push(diag);

    // Regularization
    let (r_val, d_val) = compute_regularization(imp, diag);
    data.efc_R.push(r_val);
    data.efc_D.push(d_val);

    // KBIP + aref
    let (k, b) = compute_kbip(model, solref, solimp);
    data.efc_aref[*row] = compute_aref(k, b, imp, pos, margin, vel);

    // efc_b = J_row · qacc_smooth - aref
    let mut j_dot_qacc = 0.0;
    for col in 0..nv {
        j_dot_qacc += data.efc_J[(*row, col)] * qacc_smooth[col];
    }
    data.efc_b[*row] = j_dot_qacc - data.efc_aref[*row];

    *row += 1;
}

/// Assemble the unified constraint Jacobian and metadata for all solver types.
///
/// Populates all `efc_*` fields on Data, plus `ne`/`nf` counts.
///
/// Row ordering (§29.4):
/// 1. Equality constraints (connect, weld, joint, distance) + FlexEdge → `ne` rows
/// 2. DOF friction loss + tendon friction loss → `nf` rows
/// 3. Joint limits + tendon limits + contacts
/// 4. Joint limits (potentially active only)
/// 5. Tendon limits (potentially active only)
/// 6. Contacts
///
/// `qacc_smooth` is the unconstrained acceleration (M⁻¹ · qfrc_smooth), needed for efc_b.
pub fn assemble_unified_constraints(model: &Model, data: &mut Data, qacc_smooth: &DVector<f64>) {
    let nv = model.nv;

    // === Phase 1: Count rows ===
    let mut nefc = 0usize;
    let equality_disabled = disabled(model, DISABLE_EQUALITY);
    let frictionloss_disabled = disabled(model, DISABLE_FRICTIONLOSS);
    let limit_disabled = disabled(model, DISABLE_LIMIT);

    // Equality constraints (S4.4: gated on DISABLE_EQUALITY)
    if !equality_disabled {
        for eq_id in 0..model.neq {
            if !model.eq_active[eq_id] {
                continue;
            }
            nefc += match model.eq_type[eq_id] {
                EqualityType::Connect => 3,
                EqualityType::Weld => 6,
                EqualityType::Joint | EqualityType::Distance | EqualityType::Tendon => 1,
            };
        }

        // Flex edge-length constraints (1 row per edge) — in equality block
        nefc += model.nflexedge;
    }

    // DOF friction loss (S4.5: gated on DISABLE_FRICTIONLOSS)
    if !frictionloss_disabled {
        for dof_idx in 0..nv {
            if model.dof_frictionloss[dof_idx] > 0.0 {
                nefc += 1;
            }
        }

        // Tendon friction loss
        for t in 0..model.ntendon {
            if model.tendon_frictionloss[t] > 0.0 {
                nefc += 1;
            }
        }
    }

    // Joint limits (S4.6: gated on DISABLE_LIMIT)
    // MuJoCo convention: dist < 0 means violated.
    if !limit_disabled {
        for jnt_id in 0..model.njnt {
            if !model.jnt_limited[jnt_id] {
                continue;
            }
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let (limit_min, limit_max) = model.jnt_range[jnt_id];
                    let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                    let margin = model.jnt_margin[jnt_id];
                    // Lower limit: dist = q - limit_min (negative when violated)
                    if q - limit_min < margin {
                        nefc += 1;
                    }
                    // Upper limit: dist = limit_max - q (negative when violated)
                    if limit_max - q < margin {
                        nefc += 1;
                    }
                }
                MjJointType::Ball => {
                    let adr = model.jnt_qpos_adr[jnt_id];
                    let q = normalize_quat4([
                        data.qpos[adr],
                        data.qpos[adr + 1],
                        data.qpos[adr + 2],
                        data.qpos[adr + 3],
                    ]);
                    let (_, angle) = ball_limit_axis_angle(q);
                    let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
                    let dist = limit - angle;
                    let margin = model.jnt_margin[jnt_id];
                    if dist < margin {
                        nefc += 1;
                    }
                }
                MjJointType::Free => {
                    // MuJoCo does not support free joint limits.
                    // Silently ignore — no constraint rows.
                }
            }
        }

        // Tendon limits (MuJoCo convention: dist < margin means active)
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            let margin = model.tendon_margin[t];
            // Lower tendon limit: dist = length - limit_min
            if length - limit_min < margin {
                nefc += 1;
            }
            // Upper tendon limit: dist = limit_max - length
            if limit_max - length < margin {
                nefc += 1;
            }
        }
    } // end if !limit_disabled

    // Contacts (S4.1 Site 2: defense-in-depth guard on DISABLE_CONTACT)
    // §32: pyramidal contacts emit 2*(dim-1) facet rows instead of dim rows.
    let contact_disabled = disabled(model, DISABLE_CONTACT);
    if !contact_disabled {
        for c in &data.contacts {
            let is_pyramidal = c.dim >= 3 && model.cone == 0 && c.mu[0] >= 1e-10;
            if is_pyramidal {
                nefc += 2 * (c.dim - 1);
            } else {
                nefc += c.dim;
            }
        }
    }

    // NOTE: Bending forces are passive (in mj_fwd_passive), not constraint rows.
    // Volume constraints removed — MuJoCo has no dedicated volume mechanism.

    // === Phase 2: Allocate ===
    data.efc_J = DMatrix::zeros(nefc, nv);
    data.efc_type = Vec::with_capacity(nefc);
    data.efc_pos = Vec::with_capacity(nefc);
    data.efc_margin = Vec::with_capacity(nefc);
    data.efc_vel = DVector::zeros(nefc);
    data.efc_solref = Vec::with_capacity(nefc);
    data.efc_solimp = Vec::with_capacity(nefc);
    data.efc_diagApprox = Vec::with_capacity(nefc);
    data.efc_R = Vec::with_capacity(nefc);
    data.efc_D = Vec::with_capacity(nefc);
    data.efc_imp = Vec::with_capacity(nefc);
    data.efc_aref = DVector::zeros(nefc);
    data.efc_floss = Vec::with_capacity(nefc);
    data.efc_mu = Vec::with_capacity(nefc);
    data.efc_dim = Vec::with_capacity(nefc);
    data.efc_id = Vec::with_capacity(nefc);
    data.efc_state = vec![ConstraintState::Quadratic; nefc];
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);
    data.efc_b = DVector::zeros(nefc);
    data.efc_cone_hessian = vec![None; data.contacts.len()];
    data.ncone = 0;

    let mut row = 0usize;

    // === Phase 3: Populate rows ===

    // --- 3a: Equality constraints + flex edges (S4.4: gated on DISABLE_EQUALITY) ---
    if !equality_disabled {
        assemble_equality_rows(model, data, nv, &mut row, qacc_smooth);
    }

    // --- 3b: DOF friction loss (S4.5: gated on DISABLE_FRICTIONLOSS) ---
    if !frictionloss_disabled {
        for dof_idx in 0..nv {
            let fl = model.dof_frictionloss[dof_idx];
            if fl <= 0.0 {
                continue;
            }
            // Jacobian: 1×nv with 1.0 at dof_idx
            data.efc_J[(row, dof_idx)] = 1.0;
            let vel = data.qvel[dof_idx];
            let bw = model.dof_invweight0.get(dof_idx).copied().unwrap_or(0.0);
            finalize_constraint_row(
                model,
                data,
                nv,
                &mut row,
                qacc_smooth,
                model.dof_solref[dof_idx],
                model.dof_solimp[dof_idx],
                0.0,
                0.0,
                vel,
                fl,
                ConstraintType::FrictionLoss,
                1,
                dof_idx,
                [0.0; 5],
                bw,
            );
        }

        // --- 3c: Tendon friction loss ---
        for t in 0..model.ntendon {
            let fl = model.tendon_frictionloss[t];
            if fl <= 0.0 {
                continue;
            }
            // Jacobian: tendon Jacobian row
            for col in 0..nv {
                data.efc_J[(row, col)] = data.ten_J[t][col];
            }
            let vel = data.ten_velocity[t];
            let bw = model.tendon_invweight0.get(t).copied().unwrap_or(0.0);
            finalize_constraint_row(
                model,
                data,
                nv,
                &mut row,
                qacc_smooth,
                model.tendon_solref_fri[t],
                model.tendon_solimp_fri[t],
                0.0,
                0.0,
                vel,
                fl,
                ConstraintType::FrictionLoss,
                1,
                t,
                [0.0; 5],
                bw,
            );
        }
    } // end if !frictionloss_disabled (3b+3c)

    // --- 3d: Joint limits (S4.6: gated on DISABLE_LIMIT) ---
    if !limit_disabled {
        for jnt_id in 0..model.njnt {
            if !model.jnt_limited[jnt_id] {
                continue;
            }
            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    let (limit_min, limit_max) = model.jnt_range[jnt_id];
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                    let qdot = data.qvel[dof_adr];
                    let sr = model.jnt_solref[jnt_id];
                    let si = model.jnt_solimp[jnt_id];
                    let margin = model.jnt_margin[jnt_id];

                    let bw = model.dof_invweight0.get(dof_adr).copied().unwrap_or(0.0);

                    // Lower limit: dist = q - limit_min (negative when q < limit_min)
                    let dist_lower = q - limit_min;
                    if dist_lower < margin {
                        data.efc_J[(row, dof_adr)] = 1.0;
                        finalize_constraint_row(
                            model,
                            data,
                            nv,
                            &mut row,
                            qacc_smooth,
                            sr,
                            si,
                            dist_lower,
                            margin,
                            qdot,
                            0.0,
                            ConstraintType::LimitJoint,
                            1,
                            jnt_id,
                            [0.0; 5],
                            bw,
                        );
                    }

                    // Upper limit: dist = limit_max - q (negative when q > limit_max)
                    let dist_upper = limit_max - q;
                    if dist_upper < margin {
                        data.efc_J[(row, dof_adr)] = -1.0;
                        finalize_constraint_row(
                            model,
                            data,
                            nv,
                            &mut row,
                            qacc_smooth,
                            sr,
                            si,
                            dist_upper,
                            margin,
                            -qdot,
                            0.0,
                            ConstraintType::LimitJoint,
                            1,
                            jnt_id,
                            [0.0; 5],
                            bw,
                        );
                    }
                }
                MjJointType::Ball => {
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    let dof_adr = model.jnt_dof_adr[jnt_id];
                    let q = normalize_quat4([
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ]);
                    let (unit_dir, angle) = ball_limit_axis_angle(q);
                    let limit = model.jnt_range[jnt_id].0.max(model.jnt_range[jnt_id].1);
                    let dist = limit - angle;
                    let margin = model.jnt_margin[jnt_id];

                    if dist < margin {
                        data.efc_J[(row, dof_adr)] = -unit_dir.x;
                        data.efc_J[(row, dof_adr + 1)] = -unit_dir.y;
                        data.efc_J[(row, dof_adr + 2)] = -unit_dir.z;

                        let vel = -(unit_dir.x * data.qvel[dof_adr]
                            + unit_dir.y * data.qvel[dof_adr + 1]
                            + unit_dir.z * data.qvel[dof_adr + 2]);

                        let bw = model.dof_invweight0.get(dof_adr).copied().unwrap_or(0.0);
                        finalize_constraint_row(
                            model,
                            data,
                            nv,
                            &mut row,
                            qacc_smooth,
                            model.jnt_solref[jnt_id],
                            model.jnt_solimp[jnt_id],
                            dist,
                            margin,
                            vel,
                            0.0,
                            ConstraintType::LimitJoint,
                            1,
                            jnt_id,
                            [0.0; 5],
                            bw,
                        );
                    }
                }
                MjJointType::Free => {
                    // No limit support for free joints (matches MuJoCo).
                }
            }
        }

        // --- 3e: Tendon limits ---
        for t in 0..model.ntendon {
            if !model.tendon_limited[t] {
                continue;
            }
            let (limit_min, limit_max) = model.tendon_range[t];
            let length = data.ten_length[t];
            let vel = data.ten_velocity[t];
            let sr = model.tendon_solref_lim[t];
            let si = model.tendon_solimp_lim[t];
            let margin = model.tendon_margin[t];
            let bw = model.tendon_invweight0.get(t).copied().unwrap_or(0.0);

            // Lower tendon limit: dist = length - limit_min
            let dist_lower = length - limit_min;
            if dist_lower < margin {
                for col in 0..nv {
                    data.efc_J[(row, col)] = data.ten_J[t][col];
                }
                finalize_constraint_row(
                    model,
                    data,
                    nv,
                    &mut row,
                    qacc_smooth,
                    sr,
                    si,
                    dist_lower,
                    margin,
                    vel,
                    0.0,
                    ConstraintType::LimitTendon,
                    1,
                    t,
                    [0.0; 5],
                    bw,
                );
            }

            // Upper tendon limit: dist = limit_max - length
            let dist_upper = limit_max - length;
            if dist_upper < margin {
                for col in 0..nv {
                    data.efc_J[(row, col)] = -data.ten_J[t][col];
                }
                finalize_constraint_row(
                    model,
                    data,
                    nv,
                    &mut row,
                    qacc_smooth,
                    sr,
                    si,
                    dist_upper,
                    margin,
                    -vel,
                    0.0,
                    ConstraintType::LimitTendon,
                    1,
                    t,
                    [0.0; 5],
                    bw,
                );
            }
        }
    } // end if !limit_disabled (3d+3e)

    // --- 3f: Contacts (S4.1 Site 2: defense-in-depth guard on DISABLE_CONTACT) ---
    let contacts = data.contacts.clone(); // Clone to avoid borrow conflict
    let pyramidal_ranges = if contact_disabled {
        Vec::new()
    } else {
        let ranges = assemble_contact_rows(model, data, nv, &mut row, qacc_smooth, &contacts);
        postprocess_pyramidal_r_scaling(model, data, &contacts, &ranges);
        ranges
    };
    let _ = pyramidal_ranges; // suppress unused warning when contacts disabled

    debug_assert_eq!(row, nefc, "Row count mismatch in constraint assembly");

    // === Phase 4: Count ne/nf for solver dispatch (§29.4) ===
    // Ordering: [0..ne) = equality+flex, [ne..ne+nf) = friction, [ne+nf..nefc) = limits+contacts
    data.ne = 0;
    data.nf = 0;
    for i in 0..nefc {
        match data.efc_type[i] {
            ConstraintType::Equality | ConstraintType::FlexEdge => data.ne += 1,
            ConstraintType::FrictionLoss => data.nf += 1,
            _ => {}
        }
    }
    // Verify ordering invariant: all equality rows come first, then friction, then the rest
    debug_assert!(
        data.efc_type
            .iter()
            .take(data.ne)
            .all(|t| matches!(t, ConstraintType::Equality | ConstraintType::FlexEdge)),
        "Non-equality row found within equality block [0..{})",
        data.ne,
    );
    debug_assert!(
        data.efc_type
            .iter()
            .skip(data.ne)
            .take(data.nf)
            .all(|t| matches!(t, ConstraintType::FrictionLoss)),
        "Non-friction row found within friction block [{}..{})",
        data.ne,
        data.ne + data.nf,
    );

    data.efc_cost = 0.0;
}

#[cfg(test)]
mod spec_b_margin_tests {
    //! Spec B — Joint margin tests (T8-T11).

    use crate::constraint::assembly::assemble_unified_constraints;
    use crate::types::{DISABLE_LIMIT, MjJointType, Model};
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    /// Build a hinge model with limits for constraint testing.
    fn build_hinge_limit_model(limit_min: f64, limit_max: f64, margin: f64) -> Model {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(limit_min, limit_max)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".into())];
        model.jnt_margin = vec![margin];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];
        model.dof_solref = vec![[0.02, 1.0]];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    // ---- T8: margin=0 regression — identical behavior to current code ----
    #[test]
    fn t8_margin_zero_regression() {
        let model = build_hinge_limit_model(-1.0, 1.0, 0.0);
        let mut data = model.make_data();
        data.qpos[0] = -1.5; // violates lower limit

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let nefc = data.efc_margin.len();
        // Should have at least 1 constraint (lower limit violated)
        assert!(nefc >= 1, "nefc = {nefc}, expected >= 1");
        // efc_margin should be 0.0
        assert!(
            (data.efc_margin[0] - 0.0).abs() < 1e-15,
            "efc_margin = {}, expected 0.0",
            data.efc_margin[0]
        );
        // efc_pos should be negative (dist = -1.5 - (-1.0) = -0.5)
        assert!(
            (data.efc_pos[0] - (-0.5)).abs() < 1e-10,
            "efc_pos = {}, expected -0.5",
            data.efc_pos[0]
        );
    }

    // ---- T9: margin > 0 pre-activation — hinge ----
    #[test]
    fn t9_margin_pre_activation() {
        let model = build_hinge_limit_model(-1.0, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = -0.95; // inside limit but within margin: dist = -0.95 - (-1.0) = 0.05 < 0.1

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let nefc = data.efc_margin.len();
        assert!(nefc >= 1, "nefc = {nefc}, expected >= 1 (pre-activated)");
        assert!(
            (data.efc_margin[0] - 0.1).abs() < 1e-15,
            "efc_margin = {}, expected 0.1",
            data.efc_margin[0]
        );
        assert!(
            (data.efc_pos[0] - 0.05).abs() < 1e-10,
            "efc_pos = {}, expected 0.05",
            data.efc_pos[0]
        );
    }

    // ---- T10: Ball joint with margin ----
    #[test]
    fn t10_ball_margin() {
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
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 4;
        model.nv = 3;
        model.jnt_type = vec![MjJointType::Ball];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(0.0, 1.0)]; // max angle 1.0 rad
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![1.0, 0.0, 0.0, 0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("ball".into())];
        model.jnt_margin = vec![0.05];

        model.dof_body = vec![1, 1, 1];
        model.dof_jnt = vec![0, 0, 0];
        model.dof_parent = vec![None, Some(0), Some(1)];
        model.dof_armature = vec![0.0; 3];
        model.dof_damping = vec![0.0; 3];
        model.dof_frictionloss = vec![0.0; 3];
        model.dof_solref = vec![[0.02, 1.0]; 3];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 3];

        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // Set angle to 0.97 rad about Z: dist = 1.0 - 0.97 = 0.03 < 0.05
        let half = 0.97_f64 / 2.0;
        data.qpos[0] = half.cos(); // w
        data.qpos[1] = 0.0;
        data.qpos[2] = 0.0;
        data.qpos[3] = half.sin(); // z

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let nefc = data.efc_margin.len();
        assert!(
            nefc >= 1,
            "nefc = {nefc}, expected >= 1 (ball pre-activated)",
        );
        assert!(
            (data.efc_margin[0] - 0.05).abs() < 1e-15,
            "efc_margin = {}, expected 0.05",
            data.efc_margin[0]
        );
        // dist = limit - angle ≈ 1.0 - 0.97 = 0.03
        assert!(
            (data.efc_pos[0] - 0.03).abs() < 1e-3,
            "efc_pos = {}, expected ~0.03",
            data.efc_pos[0]
        );
    }

    // ---- T11: DISABLE_LIMIT ignores margin ----
    #[test]
    fn t11_disable_limit_ignores_margin() {
        let mut model = build_hinge_limit_model(-1.0, 1.0, 0.5);
        model.disableflags |= DISABLE_LIMIT;
        let mut data = model.make_data();
        data.qpos[0] = 0.0; // within margin of both limits

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let nefc = data.efc_margin.len();
        assert_eq!(nefc, 0, "DISABLE_LIMIT: nefc should be 0, got {nefc}");
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod dt23_friction_solref_tests {
    //! DT-23 — Per-DOF friction loss solver param verification (T7–T11).
    //! Verifies multi-DOF fan-out, defaults cascade, and end-to-end wiring.

    use crate::constraint::assembly::assemble_unified_constraints;
    use crate::types::{ConstraintType, MjJointType, Model};
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    /// Build a minimal model with a single hinge joint and frictionloss.
    fn build_hinge_friction_model(
        frictionloss: f64,
        solreffriction: [f64; 2],
        solimpfriction: [f64; 5],
    ) -> Model {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-1.0, 1.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".into())];
        model.jnt_margin = vec![0.0];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![frictionloss];
        model.dof_solref = vec![solreffriction];
        model.dof_solimp = vec![solimpfriction];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    /// Build a ball joint model with frictionloss and custom solreffriction.
    fn build_ball_friction_model(frictionloss: f64, solreffriction: [f64; 2]) -> Model {
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
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 4;
        model.nv = 3;
        model.jnt_type = vec![MjJointType::Ball];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, std::f64::consts::PI)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![1.0, 0.0, 0.0, 0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("ball".into())];
        model.jnt_margin = vec![0.0];

        // Ball joint: 3 DOFs, all get same solreffriction
        model.dof_body = vec![1, 1, 1];
        model.dof_jnt = vec![0, 0, 0];
        model.dof_parent = vec![None, Some(0), Some(1)];
        model.dof_armature = vec![0.0; 3];
        model.dof_damping = vec![0.0; 3];
        model.dof_frictionloss = vec![frictionloss; 3];
        model.dof_solref = vec![solreffriction; 3];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 3];

        model.qpos0 = DVector::from_vec(vec![1.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    /// Build a free joint model with frictionloss and custom solreffriction.
    fn build_free_friction_model(frictionloss: f64, solreffriction: [f64; 2]) -> Model {
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
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 7;
        model.nv = 6;
        model.jnt_type = vec![MjJointType::Free];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("free".into())];
        model.jnt_margin = vec![0.0];

        // Free joint: 6 DOFs, all get same solreffriction
        model.dof_body = vec![1; 6];
        model.dof_jnt = vec![0; 6];
        model.dof_parent = vec![None, Some(0), Some(1), Some(2), Some(3), Some(4)];
        model.dof_armature = vec![0.0; 6];
        model.dof_damping = vec![0.0; 6];
        model.dof_frictionloss = vec![frictionloss; 6];
        model.dof_solref = vec![solreffriction; 6];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 6];

        model.qpos0 = DVector::from_vec(vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        model
    }

    // ---- T7: Ball joint 3 DOFs identical dof_solref → AC8 ----
    #[test]
    fn t7_ball_3dofs_identical_solref() {
        let solref = [0.05, 0.8];
        let model = build_ball_friction_model(1.0, solref);

        // Verify model fields: all 3 DOFs have identical solref
        assert_eq!(model.dof_solref.len(), 3);
        for i in 0..3 {
            assert_eq!(
                model.dof_solref[i], solref,
                "dof_solref[{i}] = {:?}, expected {solref:?}",
                model.dof_solref[i]
            );
        }

        // Also verify they flow through assembly
        let mut data = model.make_data();
        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        // Ball joint with frictionloss=1.0 on each DOF → 3 friction rows
        let friction_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::FrictionLoss))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(
            friction_rows.len(),
            3,
            "Expected 3 friction rows, got {}",
            friction_rows.len()
        );
        for &r in &friction_rows {
            assert_eq!(
                data.efc_solref[r], solref,
                "efc_solref[{r}] = {:?}, expected {solref:?}",
                data.efc_solref[r]
            );
        }
    }

    // ---- T8: Free joint 6 DOFs identical dof_solref → AC9 ----
    #[test]
    fn t8_free_6dofs_identical_solref() {
        let solref = [0.05, 0.8];
        let model = build_free_friction_model(1.0, solref);

        // Verify model fields: all 6 DOFs have identical solref
        assert_eq!(model.dof_solref.len(), 6);
        for i in 0..6 {
            assert_eq!(
                model.dof_solref[i], solref,
                "dof_solref[{i}] = {:?}, expected {solref:?}",
                model.dof_solref[i]
            );
        }

        // Verify through assembly
        let mut data = model.make_data();
        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let friction_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::FrictionLoss))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(
            friction_rows.len(),
            6,
            "Expected 6 friction rows, got {}",
            friction_rows.len()
        );
        for &r in &friction_rows {
            assert_eq!(
                data.efc_solref[r], solref,
                "efc_solref[{r}] = {:?}, expected {solref:?}",
                data.efc_solref[r]
            );
        }
    }

    // ---- T9: Defaults cascade for solreffriction → AC10 ----
    // This test verifies the model field level — defaults cascade is verified
    // by confirming non-default values appear in model.dof_solref. The cascade
    // itself is in sim-mjcf (parser → defaults → builder), tested via MJCF
    // round-trip. Here we verify the model field reads correctly at assembly.
    #[test]
    fn t9_defaults_cascade_solreffriction() {
        let custom_solref = [0.03, 0.7];
        let model = build_hinge_friction_model(1.0, custom_solref, [0.9, 0.95, 0.001, 0.5, 2.0]);

        // Model field should have the custom value (not default [0.02, 1.0])
        assert_eq!(model.dof_solref[0], custom_solref);

        // Assembly should use it
        let mut data = model.make_data();
        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let friction_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::FrictionLoss))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(friction_rows.len(), 1);
        assert_eq!(
            data.efc_solref[friction_rows[0]], custom_solref,
            "efc_solref should be custom {custom_solref:?}, got {:?}",
            data.efc_solref[friction_rows[0]]
        );
    }

    // ---- T10: Non-default solreffriction in constraint rows → AC11 ----
    #[test]
    fn t10_nondefault_solref_in_constraint_rows() {
        let custom_solref = [0.05, 0.8];
        let custom_solimp = [0.85, 0.9, 0.002, 0.6, 1.5];
        let model = build_hinge_friction_model(1.0, custom_solref, custom_solimp);

        let mut data = model.make_data();
        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let friction_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::FrictionLoss))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(friction_rows.len(), 1, "Expected 1 friction row");
        let r = friction_rows[0];

        assert_eq!(
            data.efc_solref[r], custom_solref,
            "efc_solref = {:?}, expected {custom_solref:?}",
            data.efc_solref[r]
        );
        assert_eq!(
            data.efc_solimp[r], custom_solimp,
            "efc_solimp = {:?}, expected {custom_solimp:?}",
            data.efc_solimp[r]
        );

        // Also verify: frictionloss=0 → no friction row
        let model_no_fl = build_hinge_friction_model(0.0, custom_solref, custom_solimp);
        let mut data_no_fl = model_no_fl.make_data();
        assemble_unified_constraints(&model_no_fl, &mut data_no_fl, &qacc_smooth);
        let fl_count = data_no_fl
            .efc_type
            .iter()
            .filter(|t| matches!(t, ConstraintType::FrictionLoss))
            .count();
        assert_eq!(fl_count, 0, "frictionloss=0 should produce 0 friction rows");
    }

    // ---- T11: Tendon friction solref end-to-end → AC12 ----
    #[test]
    fn t11_tendon_friction_solref_end_to_end() {
        let tendon_solref = [0.04, 0.9];
        let tendon_solimp = [0.85, 0.92, 0.002, 0.4, 1.8];

        // Build model with hinge joint + tendon with frictionloss
        let mut model = build_hinge_friction_model(
            0.0, // no DOF frictionloss
            [0.02, 1.0],
            [0.9, 0.95, 0.001, 0.5, 2.0],
        );

        // Add 1 tendon with frictionloss and custom solreffriction
        model.ntendon = 1;
        model.tendon_range = vec![(-1.0, 1.0)];
        model.tendon_limited = vec![false];
        model.tendon_stiffness = vec![0.0];
        model.tendon_damping = vec![0.0];
        model.tendon_frictionloss = vec![1.0];
        model.tendon_solref_fri = vec![tendon_solref];
        model.tendon_solimp_fri = vec![tendon_solimp];
        model.tendon_solref_lim = vec![[0.02, 1.0]];
        model.tendon_solimp_lim = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.tendon_margin = vec![0.0];
        model.tendon_invweight0 = vec![0.0];
        model.tendon_length0 = vec![0.0];
        model.tendon_num = vec![1];
        model.tendon_adr = vec![0];
        model.tendon_name = vec![Some("tendon0".into())];
        model.tendon_group = vec![0];
        model.tendon_rgba = vec![[0.5, 0.5, 0.5, 1.0]];
        model.tendon_lengthspring = vec![[0.0, 0.0]];

        let mut data = model.make_data();
        // Set up tendon Jacobian (1 DOF: coefficient = 1.0)
        data.ten_J[0][0] = 1.0;
        data.ten_velocity[0] = 0.0;
        data.ten_length[0] = 0.0;

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        // Should have exactly 1 friction row (tendon friction, no DOF friction)
        let friction_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::FrictionLoss))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(friction_rows.len(), 1, "Expected 1 tendon friction row");
        let r = friction_rows[0];

        assert_eq!(
            data.efc_solref[r], tendon_solref,
            "tendon efc_solref = {:?}, expected {tendon_solref:?}",
            data.efc_solref[r]
        );
        assert_eq!(
            data.efc_solimp[r], tendon_solimp,
            "tendon efc_solimp = {:?}, expected {tendon_solimp:?}",
            data.efc_solimp[r]
        );
    }
}

#[cfg(test)]
mod dt33_tendon_margin_tests {
    //! DT-33 — Tendon margin tests (T1–T6, T12).
    //! Verifies tendon limit activation with margin.

    use crate::constraint::assembly::assemble_unified_constraints;
    use crate::types::{ConstraintType, DISABLE_LIMIT, MjJointType, Model};
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    /// Build a model with 1 hinge joint + 1 limited tendon for margin testing.
    fn build_tendon_limit_model(
        tendon_range: (f64, f64),
        tendon_margin: f64,
        ten_length: f64,
    ) -> (Model, f64) {
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-10.0, 10.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".into())];
        model.jnt_margin = vec![0.0];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];
        model.dof_solref = vec![[0.02, 1.0]];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];

        // Add 1 limited tendon
        model.ntendon = 1;
        model.tendon_range = vec![tendon_range];
        model.tendon_limited = vec![true];
        model.tendon_stiffness = vec![0.0];
        model.tendon_damping = vec![0.0];
        model.tendon_frictionloss = vec![0.0];
        model.tendon_solref_fri = vec![[0.02, 1.0]];
        model.tendon_solimp_fri = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.tendon_solref_lim = vec![[0.02, 1.0]];
        model.tendon_solimp_lim = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.tendon_margin = vec![tendon_margin];
        model.tendon_invweight0 = vec![0.0];
        model.tendon_length0 = vec![0.0];
        model.tendon_num = vec![1];
        model.tendon_adr = vec![0];
        model.tendon_name = vec![Some("tendon0".into())];
        model.tendon_group = vec![0];
        model.tendon_rgba = vec![[0.5, 0.5, 0.5, 1.0]];
        model.tendon_lengthspring = vec![[0.0, 0.0]];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        (model, ten_length)
    }

    fn run_assembly(model: &Model, ten_length: f64) -> crate::types::Data {
        let mut data = model.make_data();
        data.ten_length[0] = ten_length;
        data.ten_J[0][0] = 1.0; // simple 1-DOF tendon
        data.ten_velocity[0] = 0.0;

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(model, &mut data, &qacc_smooth);
        data
    }

    // ---- T1: Tendon margin=0 regression → AC1 ----
    #[test]
    fn t1_tendon_margin_zero_regression() {
        let (model, ten_length) = build_tendon_limit_model((-1.0, 1.0), 0.0, -1.5);
        let data = run_assembly(&model, ten_length);

        let limit_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::LimitTendon))
            .map(|(i, _)| i)
            .collect();

        // ten_length=-1.5, lower limit=-1.0 → dist = -1.5 - (-1.0) = -0.5 < 0.0 → active
        assert!(
            !limit_rows.is_empty(),
            "Expected at least 1 tendon limit row"
        );
        let r = limit_rows[0];
        assert!(
            (data.efc_margin[r] - 0.0).abs() < 1e-15,
            "efc_margin = {}, expected 0.0",
            data.efc_margin[r]
        );
        assert!(
            (data.efc_pos[r] - (-0.5)).abs() < 1e-10,
            "efc_pos = {}, expected -0.5",
            data.efc_pos[r]
        );
    }

    // ---- T2: Tendon margin>0 pre-activation (lower limit) → AC2, AC3, AC4 ----
    #[test]
    fn t2_tendon_margin_pre_activation_lower() {
        let (model, ten_length) = build_tendon_limit_model((-1.0, 1.0), 0.1, -0.95);
        let data = run_assembly(&model, ten_length);

        let limit_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::LimitTendon))
            .map(|(i, _)| i)
            .collect();

        // dist = -0.95 - (-1.0) = 0.05 < 0.1 → pre-activated
        assert!(
            !limit_rows.is_empty(),
            "Expected tendon limit pre-activated (dist=0.05 < margin=0.1)"
        );
        let r = limit_rows[0];
        assert!(
            (data.efc_margin[r] - 0.1).abs() < 1e-15,
            "efc_margin = {}, expected 0.1",
            data.efc_margin[r]
        );
        assert!(
            (data.efc_pos[r] - 0.05).abs() < 1e-10,
            "efc_pos = {}, expected 0.05",
            data.efc_pos[r]
        );
    }

    // ---- T3: Tendon margin>0 pre-activation (upper limit) → AC3 ----
    #[test]
    fn t3_tendon_margin_pre_activation_upper() {
        let (model, ten_length) = build_tendon_limit_model((-1.0, 1.0), 0.1, 0.95);
        let data = run_assembly(&model, ten_length);

        let limit_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::LimitTendon))
            .map(|(i, _)| i)
            .collect();

        // dist_upper = 1.0 - 0.95 = 0.05 < 0.1 → pre-activated
        assert!(
            !limit_rows.is_empty(),
            "Expected upper tendon limit pre-activated (dist=0.05 < margin=0.1)"
        );
        let r = limit_rows[0];
        assert!(
            (data.efc_margin[r] - 0.1).abs() < 1e-15,
            "efc_margin = {}, expected 0.1",
            data.efc_margin[r]
        );
        assert!(
            (data.efc_pos[r] - 0.05).abs() < 1e-10,
            "efc_pos = {}, expected 0.05",
            data.efc_pos[r]
        );
    }

    // ---- T4: DISABLE_LIMIT ignores tendon margin → AC5 ----
    #[test]
    fn t4_disable_limit_ignores_tendon_margin() {
        let (mut model, ten_length) = build_tendon_limit_model((-1.0, 1.0), 0.5, 0.0);
        model.disableflags |= DISABLE_LIMIT;
        let data = run_assembly(&model, ten_length);

        let limit_count = data
            .efc_type
            .iter()
            .filter(|t| matches!(t, ConstraintType::LimitTendon))
            .count();
        assert_eq!(
            limit_count, 0,
            "DISABLE_LIMIT: expected 0 tendon limit rows, got {limit_count}"
        );
    }

    // ---- T5: Negative margin shrinks activation zone → AC6 ----
    #[test]
    fn t5_negative_margin_shrinks_activation() {
        // Case A: dist = -0.05, margin = -0.1. Is -0.05 < -0.1? No → no row
        let (model_a, len_a) = build_tendon_limit_model((-1.0, 1.0), -0.1, -1.05);
        let data_a = run_assembly(&model_a, len_a);
        let count_a = data_a
            .efc_type
            .iter()
            .filter(|t| matches!(t, ConstraintType::LimitTendon))
            .count();
        assert_eq!(
            count_a, 0,
            "Case A: violation -0.05 does not exceed dead zone |margin|=0.1"
        );

        // Case B: dist = -0.15, margin = -0.1. Is -0.15 < -0.1? Yes → row
        let (model_b, len_b) = build_tendon_limit_model((-1.0, 1.0), -0.1, -1.15);
        let data_b = run_assembly(&model_b, len_b);
        let count_b = data_b
            .efc_type
            .iter()
            .filter(|t| matches!(t, ConstraintType::LimitTendon))
            .count();
        assert!(
            count_b >= 1,
            "Case B: violation -0.15 exceeds dead zone |margin|=0.1"
        );
    }

    // ---- T6: Large margin overlap — both limits active → AC7 ----
    #[test]
    fn t6_large_margin_both_limits_active() {
        // range=(-0.5, 0.5), margin=0.6, ten_length=0.0
        // lower dist = 0.0 - (-0.5) = 0.5 < 0.6 → active
        // upper dist = 0.5 - 0.0 = 0.5 < 0.6 → active
        let (model, ten_length) = build_tendon_limit_model((-0.5, 0.5), 0.6, 0.0);
        let data = run_assembly(&model, ten_length);

        let limit_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::LimitTendon))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(
            limit_rows.len(),
            2,
            "Expected 2 tendon limit rows (both active), got {}",
            limit_rows.len()
        );
    }

    // ---- T12: Multi-tendon indexing → correct margin per tendon ----
    #[test]
    fn t12_multi_tendon_correct_margin_per_tendon() {
        // 3 tendons with different margins
        let mut model = Model::empty();
        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".into()), Some("body".into())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-10.0, 10.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.qpos_spring = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".into())];
        model.jnt_margin = vec![0.0];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];
        model.dof_solref = vec![[0.02, 1.0]];
        model.dof_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];

        // 3 tendons: margins 0.0, 0.1, 0.2
        // All limited with range=(-1.0, 1.0)
        model.ntendon = 3;
        model.tendon_range = vec![(-1.0, 1.0); 3];
        model.tendon_limited = vec![true; 3];
        model.tendon_stiffness = vec![0.0; 3];
        model.tendon_damping = vec![0.0; 3];
        model.tendon_frictionloss = vec![0.0; 3];
        model.tendon_solref_fri = vec![[0.02, 1.0]; 3];
        model.tendon_solimp_fri = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 3];
        model.tendon_solref_lim = vec![[0.02, 1.0]; 3];
        model.tendon_solimp_lim = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 3];
        model.tendon_margin = vec![0.0, 0.1, 0.2];
        model.tendon_invweight0 = vec![0.0; 3];
        model.tendon_length0 = vec![0.0; 3];
        model.tendon_num = vec![1, 1, 1];
        model.tendon_adr = vec![0, 1, 2];
        model.tendon_name = vec![Some("t0".into()), Some("t1".into()), Some("t2".into())];
        model.tendon_group = vec![0; 3];
        model.tendon_rgba = vec![[0.5, 0.5, 0.5, 1.0]; 3];
        model.tendon_lengthspring = vec![[0.0, 0.0]; 3];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::new(0.0, 0.0, -9.81);

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        // All tendons at length = -0.95 (lower dist = 0.05)
        // T0: margin=0.0, dist=0.05 → 0.05 < 0.0? No → no row
        // T1: margin=0.1, dist=0.05 → 0.05 < 0.1? Yes → row
        // T2: margin=0.2, dist=0.05 → 0.05 < 0.2? Yes → row
        for t in 0..3 {
            data.ten_length[t] = -0.95;
            data.ten_J[t][0] = 1.0;
            data.ten_velocity[t] = 0.0;
        }

        let qacc_smooth = DVector::zeros(model.nv);
        assemble_unified_constraints(&model, &mut data, &qacc_smooth);

        let limit_rows: Vec<usize> = data
            .efc_type
            .iter()
            .enumerate()
            .filter(|(_, t)| matches!(t, ConstraintType::LimitTendon))
            .map(|(i, _)| i)
            .collect();

        // T0 (margin=0.0) → no row; T1 (margin=0.1) → 1 row; T2 (margin=0.2) → 1 row
        assert_eq!(
            limit_rows.len(),
            2,
            "Expected 2 tendon limit rows (T1+T2 active, T0 inactive), got {}",
            limit_rows.len()
        );

        // Verify margins on the two active rows
        assert!(
            (data.efc_margin[limit_rows[0]] - 0.1).abs() < 1e-15,
            "First active row margin = {}, expected 0.1",
            data.efc_margin[limit_rows[0]]
        );
        assert!(
            (data.efc_margin[limit_rows[1]] - 0.2).abs() < 1e-15,
            "Second active row margin = {}, expected 0.2",
            data.efc_margin[limit_rows[1]]
        );
    }
}
