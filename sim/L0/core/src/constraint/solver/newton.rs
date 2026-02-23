//! Newton solver outer loop (§15.8).
//!
//! Computes qacc, qfrc_constraint, efc_state, efc_force, efc_jar, efc_cost
//! directly via reduced primal optimization. Returns early with `NewtonResult`
//! indicating success or failure mode for PGS fallback.
//!
//! Corresponds to the Newton solver path in MuJoCo's `engine_solver.c`.

use nalgebra::{DMatrix, DVector};

use crate::constraint::solver::hessian::{
    NV_SPARSE_THRESHOLD, SparseHessian, assemble_hessian, hessian_cone, hessian_incremental,
};
use crate::constraint::solver::pgs::classify_constraint_states;
use crate::constraint::solver::primal::{
    compute_gradient_and_search, compute_gradient_and_search_sparse, evaluate_cost_at,
    primal_prepare, primal_search,
};
use crate::types::{ConstraintState, Data, Model, SolverStat};

/// Result of the Newton solver. Used by Step 14 (PGS fallback).
#[derive(Debug)]
pub enum NewtonResult {
    /// Solver converged successfully.
    Converged,
    /// Cholesky factorization failed (Hessian not positive definite).
    CholeskyFailed,
    /// Maximum iterations reached without convergence.
    MaxIterationsExceeded,
}

/// Newton solver outer loop (§15.8).
///
/// Computes qacc, qfrc_constraint, efc_state, efc_force, efc_jar, efc_cost
/// directly via reduced primal optimization. Returns early with `NewtonResult`
/// indicating success or failure mode for PGS fallback.
///
/// DT-35: `m_eff` is the effective mass matrix — `M_impl` when
/// `ImplicitSpringDamper` is active, `data.qM` otherwise. All M·v products,
/// Hessian assembly, and cost evaluation use `m_eff`.
#[allow(clippy::too_many_lines, clippy::cast_precision_loss)]
pub fn newton_solve(
    model: &Model,
    data: &mut Data,
    m_eff: &DMatrix<f64>,
    qfrc_eff: &DVector<f64>,
    implicit_sd: bool,
) -> NewtonResult {
    let nv = model.nv;

    // === INITIALIZE ===
    // qacc_smooth, qfrc_smooth, and efc_* arrays are already populated by
    // mj_fwd_constraint() before dispatching to this solver.
    // Use the explicit qfrc_eff parameter (which may contain implicit spring/damper
    // corrections) rather than reading from data.qfrc_smooth via side-channel mutation.
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = qfrc_eff.clone();
    let meaninertia = data.stat_meaninertia;
    let nefc = data.efc_type.len();

    // No constraints → unconstrained solution
    if nefc == 0 {
        data.qacc.copy_from(&qacc_smooth);
        data.qfrc_constraint.fill(0.0);
        data.solver_niter = 0;
        data.solver_stat.clear();
        return NewtonResult::Converged;
    }

    let scale = 1.0 / (meaninertia * (1.0_f64).max(nv as f64));
    let tolerance = model.solver_tolerance;

    // Initialize efc_state and efc_force vectors to correct size
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // --- Warmstart selection ---
    // Evaluate cost at qacc_warmstart
    let cost_warmstart = evaluate_cost_at(
        data,
        model,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
        m_eff,
    );
    // Evaluate cost at qacc_smooth (Gauss = 0, only constraint cost from efc_b)
    let cost_smooth =
        evaluate_cost_at(data, model, &qacc_smooth, &qacc_smooth, &qfrc_smooth, m_eff);

    let mut qacc = if cost_warmstart < cost_smooth {
        data.qacc_warmstart.clone()
    } else {
        qacc_smooth.clone()
    };

    // Compute Ma = M_eff · qacc (DT-35: uses M_impl when ImplicitSpringDamper)
    let mut ma = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma[r] += m_eff[(r, c)] * qacc[c];
        }
    }

    // Initial constraint classification
    classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

    // Select dense vs sparse Hessian path (Phase C).
    // Sparse path: full refactorization each iteration, no incremental updates.
    // Dense path: incremental rank-1 updates + cone Hessian augmentation.
    let use_sparse = nv > NV_SPARSE_THRESHOLD;

    // Initial Hessian + gradient + search direction
    let mut chol_l_dense: Option<DMatrix<f64>> = None;
    let mut sparse_h: Option<SparseHessian> = None;

    let (mut grad, mut search) = if use_sparse {
        let Ok(sh) = SparseHessian::assemble(model, data, nv, implicit_sd) else {
            data.solver_niter = 0;
            data.solver_stat.clear();
            return NewtonResult::CholeskyFailed;
        };
        let gs = compute_gradient_and_search_sparse(data, nv, &ma, &qfrc_smooth, &sh);
        sparse_h = Some(sh);
        gs
    } else {
        let Ok(l) = assemble_hessian(data, nv, m_eff) else {
            data.solver_niter = 0;
            data.solver_stat.clear();
            return NewtonResult::CholeskyFailed;
        };
        let gs = compute_gradient_and_search(data, nv, &ma, &qfrc_smooth, &l);
        chol_l_dense = Some(l);
        gs
    };

    // Pre-loop convergence check
    let grad_norm: f64 = grad.iter().map(|x| x * x).sum::<f64>().sqrt();
    if scale * grad_norm < tolerance {
        // Already converged — go to RECOVER
        data.solver_niter = 0;
        data.solver_stat.clear();
        recover_newton(model, data, &qacc, &qfrc_smooth);
        return NewtonResult::Converged;
    }

    // === ITERATE (Phase B: exact Newton line search + incremental Hessian) ===
    let max_iters = model.solver_iterations;
    let max_ls_iter = model.ls_iterations.max(20);
    let ls_tolerance = model.ls_tolerance;
    let mut converged = false;
    let mut solver_stats = Vec::with_capacity(max_iters);

    // Precompute Mv = M_eff*search and Jv = J*search for the initial search direction
    let mut mv = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            mv[r] += m_eff[(r, c)] * search[c];
        }
    }
    let mut jv = DVector::<f64>::zeros(nefc);
    for i in 0..nefc {
        for col in 0..nv {
            jv[i] += data.efc_J[(i, col)] * search[col];
        }
    }

    for _iter in 0..max_iters {
        // Compute lineslope before line search: grad · search / ||search||
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

        // 1. LINE SEARCH (Phase B: exact Newton)
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
            // Record final stat entry for the alpha=0 iteration
            let nactive = data
                .efc_state
                .iter()
                .filter(|s| matches!(s, ConstraintState::Quadratic | ConstraintState::Cone))
                .count();
            solver_stats.push(SolverStat {
                improvement: 0.0,
                gradient: scale * grad.iter().map(|x| x * x).sum::<f64>().sqrt(),
                lineslope,
                nactive,
                nchange: 0,
                nline,
            });
            converged = true; // No improvement — treat as converged (local minimum)
            break;
        }

        // 2. MOVE
        for k in 0..nv {
            qacc[k] += alpha * search[k];
            ma[k] += alpha * mv[k];
        }
        for i in 0..nefc {
            data.efc_jar[i] += alpha * jv[i];
        }

        // 3. UPDATE CONSTRAINTS (save old states for incremental Hessian)
        let old_states = data.efc_state.clone();
        let old_cost = data.efc_cost;
        classify_constraint_states(model, data, &qacc, &qacc_smooth, &qfrc_smooth);

        // 4-5. HESSIAN + GRADIENT (dense vs sparse)
        let (g, s) = if use_sparse {
            // Sparse path: full refactorization each iteration
            let sh = sparse_h.as_mut().unwrap_or_else(|| unreachable!());
            if sh.refactor(model, data, implicit_sd).is_err() {
                // Refactorization failed — try full reassembly
                if let Ok(new_sh) = SparseHessian::assemble(model, data, nv, implicit_sd) {
                    *sh = new_sh;
                } else {
                    data.solver_niter = solver_stats.len();
                    data.solver_stat = solver_stats;
                    return NewtonResult::CholeskyFailed;
                }
            }
            compute_gradient_and_search_sparse(data, nv, &ma, &qfrc_smooth, sh)
        } else {
            // Dense path: incremental rank-1 updates + cone Hessian
            let chol_l = chol_l_dense.as_mut().unwrap_or_else(|| unreachable!());
            if hessian_incremental(data, nv, chol_l, &old_states, m_eff).is_err() {
                // Incremental update failed — fall back to full reassembly
                if let Ok(l) = assemble_hessian(data, nv, m_eff) {
                    *chol_l = l;
                } else {
                    data.solver_niter = solver_stats.len();
                    data.solver_stat = solver_stats;
                    return NewtonResult::CholeskyFailed;
                }
            }

            // Use cone Hessian if any cone contacts
            let l_for_solve = if data.ncone > 0 {
                match hessian_cone(data, nv, chol_l) {
                    Ok(lc) => lc,
                    Err(_) => chol_l.clone(),
                }
            } else {
                chol_l.clone()
            };

            compute_gradient_and_search(data, nv, &ma, &qfrc_smooth, &l_for_solve)
        };
        grad = g;

        // 6. CONVERGENCE CHECK + SOLVER STATISTICS
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

        if improvement < tolerance || gradient < tolerance {
            converged = true;
            break;
        }

        // 7. SEARCH DIRECTION + recompute Mv, Jv for next iteration
        search = s;

        // Recompute Mv = M_eff * search
        mv.fill(0.0);
        for r in 0..nv {
            for c in 0..nv {
                mv[r] += m_eff[(r, c)] * search[c];
            }
        }
        // Recompute Jv = J * search
        jv.fill(0.0);
        for i in 0..nefc {
            for col in 0..nv {
                jv[i] += data.efc_J[(i, col)] * search[col];
            }
        }
    }

    // === RECOVER ===
    data.solver_niter = solver_stats.len();
    data.solver_stat = solver_stats;
    recover_newton(model, data, &qacc, &qfrc_smooth);

    if converged {
        NewtonResult::Converged
    } else {
        NewtonResult::MaxIterationsExceeded
    }
}

/// RECOVER block: write final solver qacc to Data.
///
/// For Newton and CG, this writes the primal solution `qacc` that was computed
/// directly by the solver. The remaining post-processing (qfrc_constraint,
/// limit forces) is handled centrally in `mj_fwd_constraint()`.
pub fn recover_newton(
    _model: &Model,
    data: &mut Data,
    qacc: &DVector<f64>,
    _qfrc_smooth: &DVector<f64>,
) {
    data.qacc.copy_from(qacc);
}
