//! The PGS (projected Gauss-Seidel) solver, MuJoCo's `mj_solPGS`
//! (`engine_solver.c:316-531`), operation for operation: from the warmstart's
//! forces or zero ([`super::dual::pgs_warmstart`]), each sweep moves each
//! row's force, or an elliptic contact's block of forces, to the minimum of
//! the dual cost over the rows' inverse inertia `efc_AR`, keeps a move only
//! if it lowers the cost, and stops when the sweep's improvement, scaled by
//! the model's mean inertia, falls under the tolerance. Each row's residual
//! reads its row of `efc_AR`, so at the dense size a sweep's work grows as
//! nefc², as MuJoCo's.

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::dual_state;
use crate::constraint::solver::primal::{MJ_NSOLVER, PrimalMass};
use crate::constraint::solver::qcqp;
use crate::linalg::{mju_dot, mju_max};
use crate::types::{ConstraintType, Data, Model, SolverStat};

use super::dual::{ar_diag_inv, cost_change, dual_finish, extract_block, residual};

/// PGS over every row ([`Model::solver_iterations`] at most), from
/// `Data::efc_force` as the warmstart left it, ending with the forces mapped
/// to the dofs and the acceleration made from them (`dualFinish`). Writes
/// `efc_force`, `efc_state`, `qfrc_constraint`, `qacc`, the iteration count,
/// each iteration's statistics and `solver_nnz` (`AR`'s nonzeros,
/// `engine_solver.c:516-524`).
// One function, so the sweep reads as MuJoCo's does; single-letter names
// (`f`, `v`, `x`, `s`) are its own.
#[allow(clippy::many_single_char_names, clippy::too_many_lines)]
pub fn mj_sol_pgs(model: &Model, data: &mut Data, mass: &PrimalMass) {
    let (ne, nf, nefc) = (data.ne, data.nf, data.efc_type.len());
    // `nv` as f64 for the statistics' scale; a model's dof count is far below 2^52.
    #[allow(clippy::cast_precision_loss)]
    let scale = 1.0 / (model.stat_meaninertia * 1.0_f64.max(model.nv as f64));
    let ar_inv = ar_diag_inv(data, false);

    let mut state = std::mem::take(&mut data.efc_state);
    state.resize(nefc, crate::types::ConstraintState::Satisfied);
    dual_state(data, &mut state);

    let mut saved = Vec::new();
    let mut iter = 0;
    while iter < model.solver_iterations {
        let mut improvement = 0.0;
        let mut i = 0;
        while i < nefc {
            let elliptic = data.efc_type[i] == ConstraintType::ContactElliptic;
            let dim = if elliptic {
                data.contacts[data.efc_id[i]].dim
            } else {
                1
            };
            let mut res = [0.0; 6];
            residual(data, &mut res, i, dim, false);
            let mut old = [0.0; 6];
            old[..dim].copy_from_slice(&data.efc_force.as_slice()[i..i + dim]);
            let mut a = [0.0; 36];

            if elliptic {
                let mu = data.efc_mu[i];
                extract_block(data, &mut a, i, dim, false);
                let force = data.efc_force.as_mut_slice();

                // The normal update, or the update along the ray of the forces.
                if force[i] < MJ_MINVAL {
                    force[i] -= res[0] * ar_inv[i];
                    if force[i] < 0.0 {
                        force[i] = 0.0;
                    }
                    force[i + 1..i + dim].fill(0.0);
                } else {
                    let mut v = [0.0; 6];
                    v[..dim].copy_from_slice(&force[i..i + dim]);
                    let mut v1 = [0.0; 6];
                    for r in 0..dim {
                        v1[r] = mju_dot(&a[r * dim..(r + 1) * dim], &v[..dim]);
                    }
                    let denom = mju_dot(&v[..dim], &v1[..dim]);
                    if denom >= MJ_MINVAL {
                        let mut x = -mju_dot(&v[..dim], &res[..dim]) / denom;
                        if force[i] + x * v[0] < 0.0 {
                            x = -v[0] / force[i];
                        }
                        for j in 0..dim {
                            force[i + j] += x * v[j];
                        }
                    }
                }

                // The friction update, the normal force fixed: the block's
                // friction rows, their residual less the old friction's part
                // and with the normal's change.
                let n = dim - 1;
                let mut bc = [0.0; 5];
                let mut ac = [0.0; 25];
                bc[..n].copy_from_slice(&res[1..dim]);
                for j in 0..n {
                    ac[j * n..(j + 1) * n].copy_from_slice(&a[(j + 1) * dim + 1..(j + 2) * dim]);
                    bc[j] -= mju_dot(&ac[j * n..(j + 1) * n], &old[1..dim]);
                    bc[j] += a[(j + 1) * dim] * (force[i] - old[0]);
                }
                if force[i] < MJ_MINVAL {
                    force[i + 1..i + dim].fill(0.0);
                } else {
                    let (mut v, active) = solve_qcqp(&ac[..n * n], &bc[..n], &mu[..n], force[i]);
                    // On the cone's surface: the forces onto the ellipsoid, in
                    // case the QCQP's solution is approximate.
                    if active {
                        let mut s = 0.0;
                        for j in 0..n {
                            s += v[j] * v[j] / (mu[j] * mu[j]);
                        }
                        let s = (force[i] * force[i] / mju_max(MJ_MINVAL, s)).sqrt();
                        for x in &mut v[..n] {
                            *x *= s;
                        }
                    }
                    force[i + 1..i + dim].copy_from_slice(&v[..n]);
                }
            } else {
                let force = data.efc_force.as_mut_slice();
                force[i] -= res[0] * ar_inv[i];
                if i >= ne && i < ne + nf {
                    let floss = data.efc_floss[i];
                    if force[i] < -floss {
                        force[i] = -floss;
                    } else if force[i] > floss {
                        force[i] = floss;
                    }
                } else if i >= ne + nf && force[i] < 0.0 {
                    force[i] = 0.0;
                }
                a[0] = 1.0 / ar_inv[i];
            }

            improvement -= cost_change(
                &a[..dim * dim],
                &mut data.efc_force.as_mut_slice()[i..i + dim],
                &old[..dim],
                &res[..dim],
            );
            i += dim;
        }

        // The rows' states after the sweep, how many changed, and the
        // sweep's statistics (`engine_solver.c:489-499`).
        let old_state = state.clone();
        let nactive = dual_state(data, &mut state);
        let nchange = old_state.iter().zip(&state).filter(|(a, b)| a != b).count();
        improvement *= scale;
        if iter < MJ_NSOLVER {
            saved.push(SolverStat {
                improvement,
                gradient: 0.0,
                lineslope: 0.0,
                nactive,
                nchange,
                neval: 0,
                nupdate: 0,
            });
        }
        iter += 1;
        if improvement < model.solver_tolerance {
            break;
        }
    }

    data.efc_state = state;
    data.solver_niter = iter;
    data.solver_stat = saved;
    data.solver_nnz = if data.efc_AR_rownnz.is_empty() {
        nefc * nefc
    } else {
        data.efc_AR_rownnz.iter().sum()
    };
    dual_finish(model, data, mass);
}

/// MuJoCo's QCQP for an elliptic contact's `n` friction forces
/// (`mju_QCQP2`, `mju_QCQP3`, `mju_QCQP`, `engine_util_solve.c`), by `n`.
// MuJoCo's QCQP names: the block `a`, the bias `b`, the frictions `d`, the
// normal force `r`.
#[allow(clippy::many_single_char_names)]
pub(super) fn solve_qcqp(a: &[f64], b: &[f64], d: &[f64], r: f64) -> ([f64; 5], bool) {
    let mut out = [0.0; 5];
    let active = match b.len() {
        2 => {
            let (v, active) =
                qcqp::qcqp2([[a[0], a[1]], [a[2], a[3]]], [b[0], b[1]], [d[0], d[1]], r);
            out[..2].copy_from_slice(&v);
            active
        }
        3 => {
            let (v, active) = qcqp::qcqp3(
                [[a[0], a[1], a[2]], [a[3], a[4], a[5]], [a[6], a[7], a[8]]],
                [b[0], b[1], b[2]],
                [d[0], d[1], d[2]],
                r,
            );
            out[..3].copy_from_slice(&v);
            active
        }
        n => {
            let (v, active) = qcqp::qcqp_nd(a, b, d, r, n);
            out[..n].copy_from_slice(&v);
            active
        }
    };
    (out, active)
}
