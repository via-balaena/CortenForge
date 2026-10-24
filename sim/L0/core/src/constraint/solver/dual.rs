//! What MuJoCo's dual solvers, PGS ([`super::pgs`]) and noslip
//! ([`super::noslip`]), share (`engine_solver.c:70-310`): the rows' inverse
//! inertia `efc_AR` read by row, by block and on its diagonal, the change in
//! the dual cost a move makes, the warmstart PGS starts from
//! (`engine_forward.c:614-649`), and the map back to the dofs that ends both
//! (`dualFinish`).
//!
//! `efc_AR` is dense (row by row, `nefc × nefc`) unless the rows' sparse
//! columns are kept (`Data::efc_AR_rownnz`), as MuJoCo's is at the sparse
//! size ([`Model::is_sparse`]).

use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::factor::mj_solve_m;
use crate::linalg::{cholesky_solve_in_place, dot_sparse, mju_dot, mju_max};
use crate::types::flags::disabled;
use crate::types::{DISABLE_WARMSTART, Data, Model};

use super::compute_qfrc_constraint_from_efc;
use super::primal::{PrimalMass, constraint_update_at};

/// Whether `efc_AR` keeps each row's sparse columns.
fn sparse_ar(data: &Data) -> bool {
    !data.efc_AR_rownnz.is_empty()
}

/// Row `i` of `efc_AR`: its values and, at the sparse size, their columns.
fn ar_row(data: &Data, i: usize) -> (&[f64], Option<&[usize]>) {
    if sparse_ar(data) {
        let span = data.efc_AR_rowadr[i]..data.efc_AR_rowadr[i] + data.efc_AR_rownnz[i];
        (&data.efc_AR[span.clone()], Some(&data.efc_AR_colind[span]))
    } else {
        let nefc = data.efc_type.len();
        (&data.efc_AR[i * nefc..(i + 1) * nefc], None)
    }
}

/// `1/diag(AR)`, or with `sub_r` `1/max(mjMINVAL, AR − R)` on the diagonal
/// (`ARdiaginv`, `engine_solver.c:82-112`).
pub(super) fn ar_diag_inv(data: &Data, sub_r: bool) -> Vec<f64> {
    (0..data.efc_type.len())
        .map(|i| {
            let diag = match ar_row(data, i) {
                (vals, Some(cols)) => {
                    // A row always holds its diagonal (`mju_sqrMatTDSparse`).
                    let k = cols.iter().position(|&c| c == i).unwrap_or(0);
                    vals[k]
                }
                (vals, None) => vals[i],
            };
            1.0 / if sub_r {
                mju_max(MJ_MINVAL, diag - data.efc_R[i])
            } else {
                diag
            }
        })
        .collect()
}

/// The `n × n` block of `AR` at row and column `start`, row by row, into
/// `ac`; with `sub_r` its diagonal less `R`, at least 1e-10 (`extractBlock`,
/// `engine_solver.c:117-172`). At the sparse size each of the block's rows
/// is read from the place its first row has column `start`.
pub(super) fn extract_block(data: &Data, ac: &mut [f64], start: usize, n: usize, sub_r: bool) {
    if sparse_ar(data) {
        let (_, cols) = ar_row(data, start);
        let k = cols
            .and_then(|cols| cols.iter().position(|&c| c == start))
            .unwrap_or(0);
        for j in 0..n {
            let adr = data.efc_AR_rowadr[start + j] + k;
            ac[j * n..(j + 1) * n].copy_from_slice(&data.efc_AR[adr..adr + n]);
        }
    } else {
        let nefc = data.efc_type.len();
        for j in 0..n {
            let adr = start + (start + j) * nefc;
            ac[j * n..(j + 1) * n].copy_from_slice(&data.efc_AR[adr..adr + n]);
        }
    }
    if sub_r {
        for j in 0..n {
            ac[j * (n + 1)] -= data.efc_R[start + j];
            ac[j * (n + 1)] = mju_max(1e-10, ac[j * (n + 1)]);
        }
    }
}

/// The residual of rows `i..i + dim`, `efc_b + AR·efc_force`, with `sub_r`
/// less `R·efc_force` (`residual`, `engine_solver.c:177-202`).
pub(super) fn residual(data: &Data, res: &mut [f64], i: usize, dim: usize, sub_r: bool) {
    let force = data.efc_force.as_slice();
    for (j, r) in res[..dim].iter_mut().enumerate() {
        let dot = match ar_row(data, i + j) {
            (vals, Some(cols)) => dot_sparse(vals, |k| force[cols[k]]),
            (vals, None) => mju_dot(vals, force),
        };
        *r = data.efc_b[i + j] + dot;
    }
    if sub_r {
        for (j, r) in res[..dim].iter_mut().enumerate() {
            *r -= data.efc_R[i + j] * force[i + j];
        }
    }
}

/// The change in the dual cost of moving a block's forces from `old` to
/// `force`, `½·δ'·A·δ + δ·res`; a rise above 1e-10 restores `old` and counts
/// as none (`costChange`, `engine_solver.c:207-228`).
pub(super) fn cost_change(a: &[f64], force: &mut [f64], old: &[f64], res: &[f64]) -> f64 {
    let dim = force.len();
    let mut change = if dim == 1 {
        let delta = force[0] - old[0];
        0.5 * delta * delta * a[0] + delta * res[0]
    } else {
        let mut delta = [0.0; 6];
        for j in 0..dim {
            delta[j] = force[j] - old[j];
        }
        let delta = &delta[..dim];
        0.5 * mul_vec_mat_vec(delta, a, delta) + mju_dot(delta, res)
    };
    if change > 1e-10 {
        force.copy_from_slice(old);
        change = 0.0;
    }
    change
}

/// `v'·A·w` as `mju_mulVecMatVec` (`engine_util_blas.c:806-812`): each row's
/// `mju_dot` with `w`, times `v`'s entry, summed in order from 0.
pub(super) fn mul_vec_mat_vec(v: &[f64], a: &[f64], w: &[f64]) -> f64 {
    let n = v.len();
    let mut res = 0.0;
    for i in 0..n {
        res += v[i] * mju_dot(&a[i * n..(i + 1) * n], w);
    }
    res
}

/// PGS's warmstart (`warmstart`, `engine_forward.c:614-649`): with the
/// warmstart enabled, each row's state and force at `jar = J·qacc_warmstart −
/// aref` ([`constraint_update_at`]), kept unless their dual cost
/// `f·b + ½·f·(AR·f)` is positive, then zero forces; disabled, zero forces.
pub fn pgs_warmstart(model: &Model, data: &mut Data) {
    if disabled(model, DISABLE_WARMSTART) {
        data.efc_force.fill(0.0);
        return;
    }
    let warm = data.qacc_warmstart.clone();
    constraint_update_at(model, data, warm.as_slice());
    let nefc = data.efc_type.len();
    let force = data.efc_force.as_slice();
    let mut cost = mju_dot(force, data.efc_b.as_slice());
    let ar_f: Vec<f64> = (0..nefc)
        .map(|r| match ar_row(data, r) {
            (vals, Some(cols)) => dot_sparse(vals, |k| force[cols[k]]),
            (vals, None) => mju_dot(vals, force),
        })
        .collect();
    cost += 0.5 * mju_dot(force, &ar_f);
    if cost > 0.0 {
        data.efc_force.fill(0.0);
        data.qfrc_constraint.fill(0.0);
    }
}

/// The forces to the dofs and the acceleration from them (`dualFinish`,
/// `engine_solver.c:70-77`): `qfrc_constraint = J'·efc_force`,
/// `qacc = M⁻¹·qfrc_constraint + qacc_smooth`.
pub(super) fn dual_finish(model: &Model, data: &mut Data, mass: &PrimalMass) {
    compute_qfrc_constraint_from_efc(model, data);
    let mut qacc = data.qfrc_constraint.clone();
    match mass {
        PrimalMass::Tree => mj_solve_m(model, data, qacc.as_mut_slice()),
        PrimalMass::Implicit { factor, .. } => cholesky_solve_in_place(factor, &mut qacc),
    }
    qacc += &data.qacc_smooth;
    data.qacc = qacc;
}
