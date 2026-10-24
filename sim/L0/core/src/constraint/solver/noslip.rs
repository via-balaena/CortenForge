//! The noslip solver, MuJoCo's `mj_solNoSlip` (`engine_solver.c:537-747`),
//! operation for operation: after the main solver, sweeps of projected
//! Gauss-Seidel over the friction rows alone — friction loss, then each
//! contact's friction in row order — on the rows' inverse inertia without
//! their regularization (`efc_AR` less `R`), the normal forces fixed.

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::dual_state;
use crate::constraint::solver::primal::{MJ_NSOLVER, PrimalMass};
use crate::linalg::{mju_dot, mju_max};
use crate::types::{ConstraintType, Data, Model, SolverStat};

use super::dual::{ar_diag_inv, cost_change, dual_finish, extract_block, residual};
use super::pgs::solve_qcqp;

/// Noslip's sweeps ([`Model::noslip_iterations`] at most) from the main
/// solver's forces, ending with the forces mapped to the dofs and the
/// acceleration made from them (`dualFinish`). Its sweeps are counted in
/// `solver_niter`, their statistics appended to the main solver's while there
/// are fewer than 200, and `efc_state` left as [`dual_state`] classifies the
/// final forces.
// One function, so the sweeps read as MuJoCo's do; single-letter names
// (`y`, `k0`, `k1`, `v`, `s`) are its own.
#[allow(clippy::many_single_char_names, clippy::too_many_lines)]
pub fn mj_sol_noslip(model: &Model, data: &mut Data, mass: &PrimalMass) {
    let (ne, nf, nefc) = (data.ne, data.nf, data.efc_type.len());
    // `nv` as f64 for the statistics' scale; a model's dof count is far below 2^52.
    #[allow(clippy::cast_precision_loss)]
    let scale = 1.0 / (model.stat_meaninertia * 1.0_f64.max(model.nv as f64));
    let ar_inv = ar_diag_inv(data, true);

    let mut state = std::mem::take(&mut data.efc_state);
    state.resize(nefc, crate::types::ConstraintState::Satisfied);
    dual_state(data, &mut state);

    let mut iter = 0;
    while iter < model.noslip_iterations {
        let mut improvement = 0.0;
        // The regularization's cost, which this solve leaves out, at the
        // first sweep (`engine_solver.c:563-567`).
        if iter == 0 {
            for i in 0..nefc {
                improvement += 0.5 * data.efc_force[i] * data.efc_force[i] * data.efc_R[i];
            }
        }

        // Dry friction.
        for i in ne..ne + nf {
            let mut res = [0.0];
            residual(data, &mut res, i, 1, true);
            let old = data.efc_force[i];
            let floss = data.efc_floss[i];
            let force = data.efc_force.as_mut_slice();
            force[i] -= res[0] * ar_inv[i];
            if force[i] < -floss {
                force[i] = -floss;
            } else if force[i] > floss {
                force[i] = floss;
            }
            let delta = force[i] - old;
            improvement -= 0.5 * delta * delta / ar_inv[i] + delta * res[0];
        }

        // Contact friction.
        let mut i = ne + nf;
        while i < nefc {
            match data.efc_type[i] {
                ConstraintType::ContactPyramidal => {
                    let dim = data.contacts[data.efc_id[i]].dim;
                    // Each pair of opposing facets.
                    for j in (i..i + 2 * (dim - 1)).step_by(2) {
                        let mut res = [0.0; 2];
                        residual(data, &mut res, j, 2, true);
                        let old = [data.efc_force[j], data.efc_force[j + 1]];
                        let mut ac = [0.0; 4];
                        extract_block(data, &mut ac, j, 2, true);
                        let mut bc = res;
                        for k in 0..2 {
                            bc[k] -= mju_dot(&ac[k * 2..k * 2 + 2], &old);
                        }
                        let force = data.efc_force.as_mut_slice();
                        // f0 = mid + y, f1 = mid − y.
                        let mid = 0.5 * (force[j] + force[j + 1]);
                        let k1 = ac[0] + ac[3] - ac[1] - ac[2];
                        let k0 = mid * (ac[0] - ac[3]) + bc[0] - bc[1];
                        if k1 < MJ_MINVAL {
                            force[j] = mid;
                            force[j + 1] = mid;
                        } else {
                            let y = -k0 / k1;
                            if y < -mid {
                                force[j] = 0.0;
                                force[j + 1] = 2.0 * mid;
                            } else if y > mid {
                                force[j] = 2.0 * mid;
                                force[j + 1] = 0.0;
                            } else {
                                force[j] = mid + y;
                                force[j + 1] = mid - y;
                            }
                        }
                        improvement -= cost_change(&ac, &mut force[j..j + 2], &old, &res);
                    }
                    i += 2 * (dim - 1);
                }
                ConstraintType::ContactElliptic => {
                    let dim = data.contacts[data.efc_id[i]].dim;
                    let mu = data.efc_mu[i];
                    let n = dim - 1;
                    let mut res = [0.0; 5];
                    residual(data, &mut res, i + 1, n, true);
                    let mut old = [0.0; 5];
                    old[..n].copy_from_slice(&data.efc_force.as_slice()[i + 1..i + dim]);
                    let mut ac = [0.0; 25];
                    extract_block(data, &mut ac, i + 1, n, true);
                    let mut bc = res;
                    for j in 0..n {
                        bc[j] -= mju_dot(&ac[j * n..(j + 1) * n], &old[..n]);
                    }
                    let force = data.efc_force.as_mut_slice();
                    if force[i] < MJ_MINVAL {
                        force[i + 1..i + dim].fill(0.0);
                    } else {
                        let (mut v, active) =
                            solve_qcqp(&ac[..n * n], &bc[..n], &mu[..n], force[i]);
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
                    improvement -= cost_change(
                        &ac[..n * n],
                        &mut force[i + 1..i + dim],
                        &old[..n],
                        &res[..n],
                    );
                    i += dim;
                }
                _ => i += 1,
            }
        }

        // The rows' states, how many changed, and the sweep's statistics
        // after the main solver's (`engine_solver.c:716-729`).
        let old_state = state.clone();
        let nactive = dual_state(data, &mut state);
        let nchange = old_state.iter().zip(&state).filter(|(a, b)| a != b).count();
        improvement *= scale;
        if data.solver_niter + iter < MJ_NSOLVER {
            data.solver_stat.push(SolverStat {
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
        if improvement < model.noslip_tolerance {
            break;
        }
    }

    data.efc_state = state;
    data.solver_niter += iter;
    dual_finish(model, data, mass);
}
