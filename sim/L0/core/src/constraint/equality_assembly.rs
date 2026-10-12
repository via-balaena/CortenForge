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

use super::assembly::{finalize_constraint_row, finalize_constraint_row_imp};

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
    let sleep_filter = crate::island::constraint_sleep_filter(model, data);
    for eq_id in 0..model.neq {
        if !model.eq_active[eq_id]
            || (sleep_filter && crate::island::equality_asleep(model, data, eq_id))
        {
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
        // A connect's 3 rows and a weld's 6 take one impedance, at the norm of
        // their violation (MuJoCo `getposdim`); each row keeps its own `pos`.
        let shared_violation = match model.eq_type[eq_id] {
            EqualityType::Connect | EqualityType::Weld => Some(mju_norm(&rows.pos[..nrows])),
            EqualityType::Joint | EqualityType::Distance | EqualityType::Tendon => None,
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

    // --- Flex edge-length constraints (equality block) ---
    for e in 0..model.nflexedge {
        let rest_len = model.flexedge_length0[e];
        let flex_id = model.flexedge_flexid[e];
        // At any length, as MuJoCo's (`engine_core_constraint.c:626-640`):
        // `mj_flex_edge` gives a short edge a direction.
        let dist = data.flexedge_length[e];

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

/// `‖v‖` in MuJoCo's order (`mju_norm` = `sqrt(mju_dot)`,
/// `engine_util_blas.c:698-765`): four interleaved sums of squares, added as
/// `(r0 + r2) + (r1 + r3)`, then the remaining one to three left to right.
fn mju_norm(v: &[f64]) -> f64 {
    let n = v.len();
    let (mut r0, mut r1, mut r2, mut r3) = (0.0, 0.0, 0.0, 0.0);
    let mut i = 0;
    while i + 4 <= n {
        r0 += v[i] * v[i];
        r1 += v[i + 1] * v[i + 1];
        r2 += v[i + 2] * v[i + 2];
        r3 += v[i + 3] * v[i + 3];
        i += 4;
    }
    let mut sum = (r0 + r2) + (r1 + r3);
    match n - i {
        3 => sum += v[i] * v[i] + v[i + 1] * v[i + 1] + v[i + 2] * v[i + 2],
        2 => sum += v[i] * v[i] + v[i + 1] * v[i + 1],
        1 => sum += v[i] * v[i],
        _ => {}
    }
    sum.sqrt()
}

#[cfg(test)]
mod tests {
    use super::mju_norm;

    /// A weld's six violations where MuJoCo's summation order differs from
    /// a left-to-right sum in the last place: MuJoCo 3.5.0's `mju_norm` gives
    /// 0.013731086045903286; summed left to right, 0.01373108604590329.
    #[test]
    fn mju_norm_sums_in_mujoco_order() {
        let v = [
            -0.006_066, -0.005_913, 0.002_481, 0.008_006, 0.006_809, -0.000_411,
        ];
        assert_eq!(
            mju_norm(&v).to_bits(),
            0.013_731_086_045_903_286_f64.to_bits()
        );
    }
}
