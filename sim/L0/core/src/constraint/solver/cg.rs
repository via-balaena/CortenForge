//! Conjugate Gradient (CG) solver for constrained dynamics.
//!
//! Primal CG solver operating in acceleration space with M⁻¹ preconditioning
//! and Polak-Ribière direction updates. Shares the line search infrastructure
//! (`primal_prepare`, `primal_search`) and constraint classifier
//! (`classify_constraint_states`) with the Newton solver.
//!
//! Corresponds to MuJoCo's `mj_solCG` (§29.6).

use nalgebra::DVector;

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::pgs::classify_constraint_states;
use crate::constraint::solver::primal::{evaluate_cost_at, primal_prepare, primal_search};
use crate::linalg::mj_solve_sparse;
use crate::mujoco_pipeline::recover_newton;
use crate::types::{ConstraintState, Data, Model, SolverStat};

/// Primal CG solver operating on all constraint types in acceleration space.
///
/// Implements MuJoCo's `mj_solCG` (§29.6) which calls `mj_solPrimal(flg_Newton=false)`:
/// - Shares constraint evaluation (classify_constraint_states) with Newton
/// - Shares line search (primal_prepare, primal_search) with Newton
/// - Preconditioner: M⁻¹ (via LDL solve) instead of Newton's H⁻¹
/// - Direction: Polak-Ribiere conjugate gradient instead of Newton direction
/// - No cone Hessian (flg_HessianCone = false)
///
/// After convergence, `data.efc_force` and `data.efc_jar` are populated.
/// The caller must then compute `qfrc_constraint = J^T · efc_force`.
#[allow(clippy::cast_precision_loss)]
pub fn cg_solve_unified(model: &Model, data: &mut Data) {
    let nv = model.nv;

    // qacc_smooth, qfrc_smooth, and efc_* arrays are already populated by
    // mj_fwd_constraint() before dispatching to this solver.
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = data.qfrc_smooth.clone();
    let meaninertia = data.stat_meaninertia;
    let nefc = data.efc_type.len();

    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth);
        data.qfrc_constraint.fill(0.0);
        data.solver_niter = 0;
        data.solver_stat.clear();
        return;
    }

    let scale = 1.0 / (meaninertia * (1.0_f64).max(nv as f64));
    let tolerance = model.solver_tolerance;

    // Initialize efc arrays
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // Warmstart selection (same as Newton). CG always uses raw qM.
    let cost_warmstart = evaluate_cost_at(
        data,
        model,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
        &data.qM,
    );
    let cost_smooth = evaluate_cost_at(
        data,
        model,
        &qacc_smooth,
        &qacc_smooth,
        &qfrc_smooth,
        &data.qM,
    );

    let mut qacc = if cost_warmstart < cost_smooth {
        data.qacc_warmstart.clone()
    } else {
        qacc_smooth.clone()
    };

    // Compute Ma = M · qacc
    let mut ma = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma[r] += data.qM[(r, c)] * qacc[c];
        }
    }

    // Initial constraint classification (no cone Hessian for CG)
    classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

    // CG-specific: compute gradient and M⁻¹ preconditioned direction
    let (rowadr, rownnz, colind) = model.qld_csr();

    // grad = Ma - qfrc_smooth - J^T · efc_force
    let mut qfrc_constraint_local = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f != 0.0 {
            for col in 0..nv {
                qfrc_constraint_local[col] += data.efc_J[(i, col)] * f;
            }
        }
    }
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint_local[k];
    }

    // M⁻¹ preconditioner: mgrad = M⁻¹ · grad
    let mut mgrad = grad.clone();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut mgrad,
    );

    // Initial search = -mgrad (steepest descent for first iteration)
    let mut search = -mgrad.clone();

    // Pre-loop convergence check
    let grad_norm: f64 = grad.iter().map(|x| x * x).sum::<f64>().sqrt();
    if scale * grad_norm < tolerance {
        data.solver_niter = 0;
        data.solver_stat.clear();
        recover_newton(model, data, &qacc, &qfrc_smooth);
        return;
    }

    // === ITERATE ===
    let max_iters = model.solver_iterations;
    let max_ls_iter = model.ls_iterations.max(20);
    let ls_tolerance = model.ls_tolerance;
    let mut converged = false;
    let mut solver_stats: Vec<SolverStat> = Vec::with_capacity(max_iters);
    let mut grad_old = grad.clone();
    let mut mgrad_old = mgrad.clone();
    // Polak-Ribiere needs previous-iteration gradient; initial values
    // are overwritten in step 5 before the first read in step 7.
    // Force-read to silence unused_assignments warning.
    let _ = (&grad_old, &mgrad_old);

    for iter in 0..max_iters {
        // 1. Compute mv = M·search, jv = J·search (for line search)
        let mut mv = DVector::<f64>::zeros(nv);
        for r in 0..nv {
            for c in 0..nv {
                mv[r] += data.qM[(r, c)] * search[c];
            }
        }
        let mut jv = DVector::<f64>::zeros(nefc);
        for i in 0..nefc {
            for col in 0..nv {
                jv[i] += data.efc_J[(i, col)] * search[col];
            }
        }

        // Compute lineslope: grad · search / ||search||
        let search_norm = search.iter().map(|x| x * x).sum::<f64>().sqrt();
        let lineslope = if search_norm > 0.0 {
            grad.iter()
                .zip(search.iter())
                .map(|(g, s)| g * s)
                .sum::<f64>()
                / search_norm
        } else {
            0.0
        };

        // Snapshot constraint states for nchange tracking
        let old_states = data.efc_state.clone();

        // 2. LINE SEARCH (shared with Newton)
        let pq = primal_prepare(
            data,
            nv,
            &qacc,
            &qacc_smooth,
            &ma,
            &qfrc_smooth,
            &search,
            &mv,
            &jv,
        );
        let (alpha, nline) = primal_search(
            data,
            &pq,
            jv.as_slice(),
            tolerance,
            ls_tolerance,
            max_ls_iter,
            scale,
        );

        if alpha == 0.0 {
            let nactive = data
                .efc_state
                .iter()
                .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
                .count();
            solver_stats.push(SolverStat {
                improvement: 0.0,
                gradient: scale * grad_norm,
                lineslope,
                nactive,
                nchange: 0,
                nline,
            });
            break;
        }

        // 3. UPDATE qacc, ma, efc_jar
        for k in 0..nv {
            qacc[k] += alpha * search[k];
            ma[k] += alpha * mv[k];
        }
        for i in 0..nefc {
            data.efc_jar[i] += alpha * jv[i];
        }

        // 4. UPDATE CONSTRAINTS (no cone Hessian for CG)
        let old_cost = data.efc_cost;
        classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

        // 5. Compute gradient
        grad_old = grad.clone();
        mgrad_old = mgrad.clone();

        qfrc_constraint_local.fill(0.0);
        for i in 0..nefc {
            let f = data.efc_force[i];
            if f != 0.0 {
                for col in 0..nv {
                    qfrc_constraint_local[col] += data.efc_J[(i, col)] * f;
                }
            }
        }
        for k in 0..nv {
            grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint_local[k];
        }

        // 6. M⁻¹ preconditioner
        mgrad = grad.clone();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut mgrad,
        );

        // 7. Polak-Ribiere direction update
        // iter 0: pure steepest descent (beta=0).
        // iter>0: beta = grad^T · (mgrad - mgrad_old) / grad_old^T · mgrad_old
        let beta = if iter == 0 {
            0.0
        } else {
            let mgrad_diff: DVector<f64> = &mgrad - &mgrad_old;
            let num = grad.dot(&mgrad_diff);
            let den = grad_old.dot(&mgrad_old);
            if den.abs() < MJ_MINVAL {
                0.0
            } else {
                (num / den).max(0.0)
            }
        };
        search = -&mgrad + beta * &search;

        // 8. CONVERGENCE CHECK
        let improvement = scale * (old_cost - data.efc_cost);
        let gradient = scale * grad.iter().map(|x| x * x).sum::<f64>().sqrt();

        let nactive = data
            .efc_state
            .iter()
            .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
            .count();
        let nchange = old_states
            .iter()
            .zip(data.efc_state.iter())
            .filter(|(old, new)| old != new)
            .count();

        solver_stats.push(SolverStat {
            improvement,
            gradient,
            lineslope,
            nactive,
            nchange,
            nline,
        });

        if improvement < tolerance && gradient < tolerance {
            converged = true;
            break;
        }
    }

    // === RECOVER ===
    data.solver_niter = solver_stats.len();
    data.solver_stat = solver_stats;
    recover_newton(model, data, &qacc, &qfrc_smooth);

    // If not converged, we still use the best result we have
    let _ = converged;
}
