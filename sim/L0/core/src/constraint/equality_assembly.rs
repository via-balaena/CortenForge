//! Equality constraint row assembly — connect, weld, joint, distance, tendon,
//! and flex edge-length constraints.
//!
//! Populates equality and flex-edge rows in the unified `efc_*` arrays.
//! Called from [`super::assembly::assemble_unified_constraints`] during Phase 3a.

use nalgebra::DVector;

use crate::constraint::equality::{
    extract_connect_jacobian, extract_distance_jacobian, extract_joint_equality_jacobian,
    extract_tendon_equality_jacobian, extract_weld_jacobian,
};
use crate::constraint::impedance::{MJ_MINVAL, compute_diag_approx_bodyweight};
use crate::types::{ConstraintType, Data, EqualityType, Model};

use super::assembly::finalize_constraint_row;

/// Assemble equality constraint rows (Phase 3a) and flex edge-length constraints (Phase 3a').
///
/// Populates efc_J, efc_type, efc_pos, efc_margin, efc_vel, and all impedance
/// fields for each equality constraint and flex edge. Increments `row` for each
/// row written.
pub(super) fn assemble_equality_rows(
    model: &Model,
    data: &mut Data,
    nv: usize,
    row: &mut usize,
    qacc_smooth: &DVector<f64>,
) {
    // --- Equality constraints (connect, weld, joint, distance, tendon) ---
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
                data.efc_J[(*row, col)] = rows.j_rows[(r, col)];
            }
            let bw =
                compute_diag_approx_bodyweight(model, data, ConstraintType::Equality, eq_id, r);
            finalize_constraint_row(
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
            );
        }
    }

    // --- Flex edge-length constraints (equality block) ---
    for e in 0..model.nflexedge {
        let rest_len = model.flexedge_length0[e];
        let flex_id = model.flexedge_flexid[e];
        let dist = data.flexedge_length[e];

        if dist < 1e-10 {
            // Degenerate: zero-length edge, skip (fill zeros)
            finalize_constraint_row(
                model,
                data,
                nv,
                row,
                qacc_smooth,
                model.flex_edge_solref[flex_id],
                model.flex_edge_solimp[flex_id],
                0.0,
                0.0,
                0.0,
                0.0,
                ConstraintType::FlexEdge,
                1,
                e,
                [0.0; 5],
                MJ_MINVAL,
            );
            continue;
        }

        let pos_error = dist - rest_len; // positive = stretched, negative = compressed

        // §42A-i: Scatter pre-computed edge Jacobian into dense efc_J row.
        let rowadr = model.flexedge_J_rowadr[e];
        let rownnz = model.flexedge_J_rownnz[e];
        for j in 0..rownnz {
            let col = model.flexedge_J_colind[rowadr + j];
            data.efc_J[(*row, col)] = data.flexedge_J[rowadr + j];
        }

        // Velocity error: read from pre-computed Data field
        let vel_error = data.flexedge_velocity[e];

        finalize_constraint_row(
            model,
            data,
            nv,
            row,
            qacc_smooth,
            model.flex_edge_solref[flex_id],
            model.flex_edge_solimp[flex_id],
            pos_error,
            0.0, // margin
            vel_error,
            0.0, // friction loss
            ConstraintType::FlexEdge,
            1,        // dim
            e,        // id (edge index)
            [0.0; 5], // mu (no friction on edge constraints)
            MJ_MINVAL,
        );
    }
}
