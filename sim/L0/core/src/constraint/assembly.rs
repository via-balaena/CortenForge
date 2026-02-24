//! Constraint row assembly for all constraint types.
//!
//! Populates the unified `efc_*` arrays on [`Data`] with constraint Jacobians,
//! parameters, and metadata for equality, friction, limit, and contact
//! constraints. Corresponds to MuJoCo's `engine_core_constraint.c`.

use nalgebra::{DMatrix, DVector, Vector3};

use crate::constraint::equality::{
    extract_connect_jacobian, extract_distance_jacobian, extract_joint_equality_jacobian,
    extract_tendon_equality_jacobian, extract_weld_jacobian,
};
use crate::constraint::impedance::{
    MJ_MINVAL, ball_limit_axis_angle, compute_aref, compute_diag_approx_exact, compute_impedance,
    compute_kbip, compute_regularization, normalize_quat4,
};
use crate::constraint::jacobian::compute_contact_jacobian;
use crate::types::{ConstraintState, ConstraintType, Data, EqualityType, MjJointType, Model};

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

    // Equality constraints
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

    // DOF friction loss
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

    // Joint limits (MuJoCo convention: dist < 0 means violated)
    for jnt_id in 0..model.njnt {
        if !model.jnt_limited[jnt_id] {
            continue;
        }
        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                let (limit_min, limit_max) = model.jnt_range[jnt_id];
                let q = data.qpos[model.jnt_qpos_adr[jnt_id]];
                // Lower limit: dist = q - limit_min (negative when violated)
                if q - limit_min < 0.0 {
                    nefc += 1;
                }
                // Upper limit: dist = limit_max - q (negative when violated)
                if limit_max - q < 0.0 {
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
                if dist < 0.0 {
                    // margin = 0.0 (see S6)
                    nefc += 1;
                }
            }
            MjJointType::Free => {
                // MuJoCo does not support free joint limits.
                // Silently ignore — no constraint rows.
            }
        }
    }

    // Tendon limits (MuJoCo convention: dist < 0 means violated)
    for t in 0..model.ntendon {
        if !model.tendon_limited[t] {
            continue;
        }
        let (limit_min, limit_max) = model.tendon_range[t];
        let length = data.ten_length[t];
        // Lower tendon limit: dist = length - limit_min (negative when too short)
        if length - limit_min < 0.0 {
            nefc += 1;
        }
        // Upper tendon limit: dist = limit_max - length (negative when too long)
        if limit_max - length < 0.0 {
            nefc += 1;
        }
    }

    // Contacts
    // §32: pyramidal contacts emit 2*(dim-1) facet rows instead of dim rows.
    for c in &data.contacts {
        let is_pyramidal = c.dim >= 3 && model.cone == 0 && c.mu[0] >= 1e-10;
        if is_pyramidal {
            nefc += 2 * (c.dim - 1);
        } else {
            nefc += c.dim;
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

    // Helper: populate per-row metadata and compute impedance, KBIP, aref, diagApprox, R, D.
    // Must be called for each row after J row and pos/vel/margin are set.
    macro_rules! finalize_row {
        ($solref:expr, $solimp:expr, $pos:expr, $margin:expr, $vel:expr, $floss:expr,
         $ctype:expr, $dim_val:expr, $id_val:expr, $mu_val:expr) => {{
            let sr: [f64; 2] = $solref;
            let si: [f64; 5] = $solimp;
            let pos_val: f64 = $pos;
            let margin_val: f64 = $margin;
            let vel_val: f64 = $vel;
            let floss_val: f64 = $floss;

            data.efc_type.push($ctype);
            data.efc_pos.push(pos_val);
            data.efc_margin.push(margin_val);
            data.efc_vel[row] = vel_val;
            data.efc_solref.push(sr);
            data.efc_solimp.push(si);
            data.efc_floss.push(floss_val);
            data.efc_mu.push($mu_val);
            data.efc_dim.push($dim_val);
            data.efc_id.push($id_val);

            // Impedance
            let violation = (pos_val - margin_val).abs();
            let imp = compute_impedance(si, violation);
            data.efc_imp.push(imp);

            // diagApprox (exact diagonal via M⁻¹ solve)
            let j_row_slice: Vec<f64> = (0..nv).map(|col| data.efc_J[(row, col)]).collect();
            let diag = compute_diag_approx_exact(&j_row_slice, nv, model, data);
            data.efc_diagApprox.push(diag);

            // Regularization
            let (r_val, d_val) = compute_regularization(imp, diag);
            data.efc_R.push(r_val);
            data.efc_D.push(d_val);

            // KBIP + aref
            let (k, b) = compute_kbip(sr, si);
            data.efc_aref[row] = compute_aref(k, b, imp, pos_val, margin_val, vel_val);

            // efc_b = J_row · qacc_smooth - aref
            let mut j_dot_qacc = 0.0;
            for col in 0..nv {
                j_dot_qacc += data.efc_J[(row, col)] * qacc_smooth[col];
            }
            data.efc_b[row] = j_dot_qacc - data.efc_aref[row];

            row += 1;
        }};
    }

    // === Phase 3: Populate rows ===

    // --- 3a: Equality constraints ---
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id] {
            continue;
        }

        let rows = match model.eq_type[eq_id] {
            EqualityType::Connect => extract_connect_jacobian(model, data, eq_id),
            EqualityType::Weld => extract_weld_jacobian(model, data, eq_id),
            EqualityType::Joint => extract_joint_equality_jacobian(model, data, eq_id),
            EqualityType::Distance => extract_distance_jacobian(model, data, eq_id),
            EqualityType::Tendon => extract_tendon_equality_jacobian(model, data, eq_id),
        };

        let sr = model.eq_solref[eq_id];
        let si = model.eq_solimp[eq_id];
        let nrows = rows.j_rows.nrows();

        for r in 0..nrows {
            // Copy J row
            for col in 0..nv {
                data.efc_J[(row, col)] = rows.j_rows[(r, col)];
            }
            finalize_row!(
                sr,
                si,
                rows.pos[r],
                0.0,
                rows.vel[r],
                0.0,
                ConstraintType::Equality,
                1,
                eq_id,
                [0.0; 5]
            );
        }
    }

    // --- 3a': Flex edge-length constraints (equality block) ---
    for e in 0..model.nflexedge {
        let [v0, v1] = model.flexedge_vert[e];
        let x0 = data.flexvert_xpos[v0];
        let x1 = data.flexvert_xpos[v1];
        let diff = x1 - x0;
        let dist = diff.norm();
        let rest_len = model.flexedge_length0[e];
        let flex_id = model.flexedge_flexid[e];

        if dist < 1e-10 {
            // Degenerate: zero-length edge, skip (fill zeros, finalize_row! handles it)
            finalize_row!(
                model.flex_edge_solref[flex_id],
                model.flex_edge_solimp[flex_id],
                0.0,
                0.0,
                0.0,
                0.0,
                ConstraintType::FlexEdge,
                1,
                e,
                [0.0; 5]
            );
            continue;
        }

        let direction = diff / dist;
        let pos_error = dist - rest_len; // positive = stretched, negative = compressed

        // Jacobian: ∂C/∂x_v0 = -direction, ∂C/∂x_v1 = +direction
        // (§27F) Pinned vertices (dofadr=usize::MAX) have zero Jacobian columns.
        let dof0 = model.flexvert_dofadr[v0];
        let dof1 = model.flexvert_dofadr[v1];
        if dof0 != usize::MAX {
            for k in 0..3 {
                data.efc_J[(row, dof0 + k)] = -direction[k];
            }
        }
        if dof1 != usize::MAX {
            for k in 0..3 {
                data.efc_J[(row, dof1 + k)] = direction[k];
            }
        }

        // Velocity: relative velocity projected onto edge direction
        let vel0 = if dof0 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof0], data.qvel[dof0 + 1], data.qvel[dof0 + 2])
        };
        let vel1 = if dof1 == usize::MAX {
            Vector3::zeros()
        } else {
            Vector3::new(data.qvel[dof1], data.qvel[dof1 + 1], data.qvel[dof1 + 2])
        };
        let vel_error = (vel1 - vel0).dot(&direction);

        finalize_row!(
            model.flex_edge_solref[flex_id],
            model.flex_edge_solimp[flex_id],
            pos_error,
            0.0, // margin
            vel_error,
            0.0, // friction loss
            ConstraintType::FlexEdge,
            1,        // dim
            e,        // id (edge index)
            [0.0; 5]  // mu (no friction on edge constraints)
        );
    }

    // --- 3b: DOF friction loss ---
    for dof_idx in 0..nv {
        let fl = model.dof_frictionloss[dof_idx];
        if fl <= 0.0 {
            continue;
        }
        // Jacobian: 1×nv with 1.0 at dof_idx
        data.efc_J[(row, dof_idx)] = 1.0;
        let vel = data.qvel[dof_idx];
        finalize_row!(
            model.dof_solref[dof_idx],
            model.dof_solimp[dof_idx],
            0.0,
            0.0,
            vel,
            fl,
            ConstraintType::FrictionLoss,
            1,
            dof_idx,
            [0.0; 5]
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
        finalize_row!(
            model.tendon_solref_fri[t],
            model.tendon_solimp_fri[t],
            0.0,
            0.0,
            vel,
            fl,
            ConstraintType::FrictionLoss,
            1,
            t,
            [0.0; 5]
        );
    }

    // --- 3d: Joint limits ---
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

                // MuJoCo convention: dist > 0 = satisfied, dist < 0 = violated.
                // Constraint is instantiated when dist < margin (here margin=0,
                // so when dist < 0, i.e., limit violated).

                // Lower limit: dist = q - limit_min (negative when q < limit_min)
                let dist_lower = q - limit_min;
                if dist_lower < 0.0 {
                    // J = +1 (MuJoCo: jac = -side, side=-1 → jac=+1)
                    data.efc_J[(row, dof_adr)] = 1.0;
                    // pos = dist (negative = violated, MuJoCo convention)
                    // vel = J*qdot = qdot
                    finalize_row!(
                        sr,
                        si,
                        dist_lower,
                        0.0,
                        qdot,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
                    );
                }

                // Upper limit: dist = limit_max - q (negative when q > limit_max)
                let dist_upper = limit_max - q;
                if dist_upper < 0.0 {
                    // J = -1 (MuJoCo: jac = -side, side=+1 → jac=-1)
                    data.efc_J[(row, dof_adr)] = -1.0;
                    // pos = dist (negative = violated, MuJoCo convention)
                    // vel = J*qdot = -qdot
                    finalize_row!(
                        sr,
                        si,
                        dist_upper,
                        0.0,
                        -qdot,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
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

                if dist < 0.0 {
                    // margin = 0.0 (see S6)
                    // Jacobian: -unit_dir on 3 angular DOFs
                    data.efc_J[(row, dof_adr)] = -unit_dir.x;
                    data.efc_J[(row, dof_adr + 1)] = -unit_dir.y;
                    data.efc_J[(row, dof_adr + 2)] = -unit_dir.z;

                    // Constraint-space velocity: J · qvel
                    let vel = -(unit_dir.x * data.qvel[dof_adr]
                        + unit_dir.y * data.qvel[dof_adr + 1]
                        + unit_dir.z * data.qvel[dof_adr + 2]);

                    finalize_row!(
                        model.jnt_solref[jnt_id],
                        model.jnt_solimp[jnt_id],
                        dist,
                        0.0,
                        vel,
                        0.0,
                        ConstraintType::LimitJoint,
                        1,
                        jnt_id,
                        [0.0; 5]
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
        let sr = model.tendon_solref[t];
        let si = model.tendon_solimp[t];

        // MuJoCo convention: dist > 0 = satisfied, dist < 0 = violated.
        // Tendon limits follow the same pattern as joint limits.

        // Lower tendon limit: dist = length - limit_min (negative when too short)
        let dist_lower = length - limit_min;
        if dist_lower < 0.0 {
            // J = +ten_J (MuJoCo convention: pushes length up)
            for col in 0..nv {
                data.efc_J[(row, col)] = data.ten_J[t][col];
            }
            // pos = dist (negative = violated, MuJoCo convention)
            // vel = J·qdot ≈ ten_velocity
            finalize_row!(
                sr,
                si,
                dist_lower,
                0.0,
                vel,
                0.0,
                ConstraintType::LimitTendon,
                1,
                t,
                [0.0; 5]
            );
        }

        // Upper tendon limit: dist = limit_max - length (negative when too long)
        let dist_upper = limit_max - length;
        if dist_upper < 0.0 {
            // J = -ten_J (MuJoCo convention: pushes length down)
            for col in 0..nv {
                data.efc_J[(row, col)] = -data.ten_J[t][col];
            }
            // pos = dist (negative = violated, MuJoCo convention)
            // vel = -ten_velocity
            finalize_row!(
                sr,
                si,
                dist_upper,
                0.0,
                -vel,
                0.0,
                ConstraintType::LimitTendon,
                1,
                t,
                [0.0; 5]
            );
        }
    }

    // --- 3f: Contacts ---
    let contacts = data.contacts.clone(); // Clone to avoid borrow conflict
    // §32: Track pyramidal contact ranges for R-scaling post-processing.
    let mut pyramidal_ranges: Vec<(usize, usize, usize)> = Vec::new(); // (start_row, n_facets, contact_idx)
    for (ci, contact) in contacts.iter().enumerate() {
        let dim = contact.dim;
        let cj = compute_contact_jacobian(model, data, contact);

        let sr_normal = contact.solref;
        let si = contact.solimp;
        // includemargin = margin - gap, computed at contact creation.
        // Flows into compute_impedance (violation threshold) and compute_aref
        // (reference acceleration offset).
        let margin = contact.includemargin;
        let is_elliptic = dim >= 3 && model.cone == 1 && contact.mu[0] >= 1e-10;
        let is_pyramidal = dim >= 3 && model.cone == 0 && contact.mu[0] >= 1e-10;

        if is_pyramidal {
            // §32: Pyramidal friction cone — emit 2*(dim-1) facet rows.
            // Each friction direction d produces two facets:
            //   J_pos = J_normal + μ_d · J_friction_d
            //   J_neg = J_normal - μ_d · J_friction_d
            // All facets share pos, margin, solref (NOT solreffriction).
            let n_facets = 2 * (dim - 1);
            let start_row = row;

            // MuJoCo: cpos[0] = cpos[1] = con->dist; cmargin[0] = cmargin[1] = con->includemargin;
            // These are set ONCE before the loop, reused for ALL facet pairs.
            let pos = -contact.depth;

            for d in 1..dim {
                let mu_d = contact.mu[d - 1]; // friction coefficient for direction d

                // Positive facet: J_normal + μ_d · J_friction_d
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(0, col)] + mu_d * cj[(d, col)];
                }
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += data.efc_J[(row, col)] * data.qvel[col];
                }
                finalize_row!(
                    sr_normal,
                    si,
                    pos,
                    margin,
                    vel,
                    0.0,
                    ConstraintType::ContactPyramidal,
                    n_facets,
                    ci,
                    contact.mu
                );

                // Negative facet: J_normal - μ_d · J_friction_d
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(0, col)] - mu_d * cj[(d, col)];
                }
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += data.efc_J[(row, col)] * data.qvel[col];
                }
                finalize_row!(
                    sr_normal,
                    si,
                    pos,
                    margin,
                    vel,
                    0.0,
                    ConstraintType::ContactPyramidal,
                    n_facets,
                    ci,
                    contact.mu
                );
            }

            pyramidal_ranges.push((start_row, n_facets, ci));
        } else {
            // Elliptic or frictionless: emit dim rows (existing path).
            let ctype = if is_elliptic {
                ConstraintType::ContactElliptic
            } else {
                ConstraintType::ContactFrictionless
            };

            // §31: solreffriction selection for elliptic friction rows.
            let has_solreffriction = is_elliptic
                && (contact.solreffriction[0] != 0.0 || contact.solreffriction[1] != 0.0);

            for r in 0..dim {
                for col in 0..nv {
                    data.efc_J[(row, col)] = cj[(r, col)];
                }

                // pos: row 0 = signed distance, rows 1+ = 0.
                let pos = if r == 0 { -contact.depth } else { 0.0 };
                let margin_r = if r == 0 { margin } else { 0.0 };

                // §31: select solref for this row.
                let sr = if r > 0 && has_solreffriction {
                    contact.solreffriction
                } else {
                    sr_normal
                };

                // vel: J_row · qvel
                let mut vel = 0.0;
                for col in 0..nv {
                    vel += cj[(r, col)] * data.qvel[col];
                }

                finalize_row!(sr, si, pos, margin_r, vel, 0.0, ctype, dim, ci, contact.mu);
            }
        }
    }

    // §32: Post-process R scaling for pyramidal contacts.
    // TODO(§32): AC12/AC13 cross-validation against MuJoCo reference data.
    // MuJoCo's mj_makeImpedance computes R per-row from each row's own diagApprox,
    // then overrides all facet rows with Rpy = 2 · μ_reg² · R[first_facet].
    // R[first_facet] was already computed by finalize_row! using the first facet's
    // actual Jacobian (J_normal + μ[0]·J_friction_0), NOT the pure normal Jacobian.
    for &(start_row, n_facets, ci) in &pyramidal_ranges {
        let contact = &contacts[ci];

        // Use R already computed by finalize_row! for the first facet row.
        let r_first_facet = data.efc_R[start_row];

        // μ_reg = friction[0] · √(1/impratio)
        let mu_reg = contact.mu[0] * (1.0 / model.impratio).sqrt();
        let rpy = (2.0 * mu_reg * mu_reg * r_first_facet).max(MJ_MINVAL);

        for row_idx in start_row..start_row + n_facets {
            data.efc_R[row_idx] = rpy;
            data.efc_D[row_idx] = 1.0 / rpy;
        }
    }

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
