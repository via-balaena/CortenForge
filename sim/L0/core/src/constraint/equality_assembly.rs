//! Equality constraint row assembly — connect, weld, joint, distance, tendon
//! and flex equalities.
//!
//! Populates the equality rows in the unified `efc_*` arrays.
//! Called from [`super::assembly::assemble_unified_constraints`] during Phase 3a.

use nalgebra::DVector;

use crate::constraint::equality::{
    extract_connect_jacobian, extract_distance_jacobian, extract_joint_equality_jacobian,
    extract_tendon_equality_jacobian, extract_weld_jacobian,
};
use crate::constraint::impedance::compute_diag_approx_bodyweight;
use crate::linalg::mju_norm;
use crate::types::{ConstraintType, Data, EqualityType, Model};

use super::assembly::{finalize_constraint_row, finalize_constraint_row_imp};

/// Assemble equality constraint rows (Phase 3a), in equality order.
///
/// Populates efc_J, efc_type, efc_pos, efc_margin, efc_vel, and all impedance
/// fields for each equality constraint. Increments `row` for each row
/// written.
pub(super) fn assemble_equality_rows(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
) {
    let sleep_filter = crate::island::constraint_sleep_filter(model, data);
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id]
            || (sleep_filter && crate::island::equality_asleep(model, data, eq_id))
        {
            continue;
        }

        let rows = match model.eq_type[eq_id] {
            EqualityType::Flex => {
                assemble_flex_equality_rows(model, data, nv, row, qacc_smooth, eq_id);
                continue;
            }
            EqualityType::Connect => extract_connect_jacobian(model, data, eq_id),
            EqualityType::Weld => extract_weld_jacobian(model, data, eq_id),
            EqualityType::Joint => extract_joint_equality_jacobian(model, data, eq_id),
            EqualityType::Distance => extract_distance_jacobian(model, data, eq_id),
            EqualityType::Tendon => extract_tendon_equality_jacobian(model, data, eq_id),
        };

        // All rows empty (two bodies without dofs, say): no rows, as MuJoCo's
        // `mj_addConstraint`.
        if rows.j_rows.iter().all(|&x| x == 0.0) {
            continue;
        }

        let sr = model.eq_solref[eq_id];
        let si = model.eq_solimp[eq_id];
        let nrows = rows.j_rows.nrows();
        // A connect's 3 rows and a weld's 6 take one impedance, at the norm of
        // their violation (MuJoCo `getposdim`); each row keeps its own `pos`.
        let shared_violation = match model.eq_type[eq_id] {
            EqualityType::Connect | EqualityType::Weld => Some(mju_norm(&rows.pos[..nrows])),
            EqualityType::Joint
            | EqualityType::Distance
            | EqualityType::Tendon
            | EqualityType::Flex => None,
        };

        for r in 0..nrows {
            // Copy J row
            for col in 0..nv {
                data.efc_J[(*row, col)] = rows.j_rows[(r, col)];
            }
            let bw =
                compute_diag_approx_bodyweight(model, data, ConstraintType::Equality, eq_id, r);
            finalize_constraint_row_imp(
                model,
                data,
                nv,
                row,
                qacc_smooth,
                sr,
                si,
                rows.pos[r],
                0.0,
                rows.vel[r],
                0.0,
                ConstraintType::Equality,
                1,
                eq_id,
                [0.0; 5],
                bw,
                shared_violation.unwrap_or_else(|| rows.pos[r].abs()),
            );
        }
    }
}

/// A flex equality's rows: one per non-rigid edge of flex `eq_obj1id`, at its
/// length's departure from the rest length, with the equality's `solref` and
/// `solimp` and the edge's `flexedge_invweight0` as the diagonal
/// approximation (MuJoCo `engine_core_constraint.c:616-643`, `:1153-1165`).
/// A non-rigid edge has a vertex with dofs and a unit direction, so its row
/// is never empty.
fn assemble_flex_equality_rows(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
    eq_id: usize,
) {
    let f = model.eq_obj1id[eq_id];
    for e in model.flex_edgeadr[f]..model.flex_edgeadr[f] + model.flex_edgenum[f] {
        if model.flexedge_rigid[e] {
            continue;
        }
        // At any length, as MuJoCo's: `mj_flex_edge` gives a short edge a
        // direction.
        let pos = data.flexedge_length[e] - model.flexedge_length0[e];
        let rowadr = model.flexedge_J_rowadr[e];
        for k in rowadr..rowadr + model.flexedge_J_rownnz[e] {
            data.efc_J[(*row, model.flexedge_J_colind[k])] = data.flexedge_J[k];
        }
        let vel = data.flexedge_velocity[e];
        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            qacc_smooth,
            model.eq_solref[eq_id],
            model.eq_solimp[eq_id],
            pos,
            0.0,
            vel,
            0.0,
            ConstraintType::Equality,
            1,
            eq_id,
            [0.0; 5],
            model.flexedge_invweight0[e],
        );
    }
}
