//! PGS (Projected Gauss-Seidel) solver and constraint state classification.
//!
//! Contains the dual-space PGS solver (`pgs_solve_unified`), the primal-space
//! constraint classifier (`classify_constraint_states`), and sparse Delassus
//! computation via incremental qacc tracking.
//!
//! Corresponds to MuJoCo's `mj_solPGS` (§29.3) and `PrimalUpdateConstraint`
//! (§15.1).
//!
//! ## Performance
//!
//! The solver uses incremental qacc tracking instead of the dense nefc×nefc
//! Delassus matrix. This reduces per-iteration cost from O(nefc²) to
//! O(nefc × nv), which is critical for SDF collision scenes with hundreds
//! of contacts (nefc > 1000, nv ~ 12).

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::mj_solve_sparse;
use crate::types::{ConstraintState, ConstraintType, Data, Model, SolverStat};

/// Precomputed data for sparse PGS iteration.
///
/// Instead of the full nefc×nefc Delassus matrix, stores the nv×nefc
/// matrix M⁻¹·J^T and derived quantities. Per-iteration residuals are
/// computed as O(nv) dot products against the tracked qacc vector.
struct SparsePgsData {
    /// M⁻¹ · J^T (nv × nefc) — precomputed via sparse LDL solves.
    minv_jt: DMatrix<f64>,
    /// Diagonal of AR: AR_ii = J_row_i · minv_jt_col_i + R_i.
    ar_diag: Vec<f64>,
    /// Inverse diagonal: 1/AR_ii (0 if AR_ii ≈ 0).
    ar_diag_inv: Vec<f64>,
    /// Effective bias: b_eff_i = efc_b_i - J_row_i · qacc_smooth.
    /// Residual = b_eff_i + J_row_i · qacc + R_i * f_i.
    b_eff: Vec<f64>,
}

/// Prepare sparse PGS data structures.
///
/// Cost: O(nefc × nv²) for the sparse LDL solves + O(nefc × nv) for
/// diagonal and bias precomputation. Avoids the O(nefc² × nv) cost of
/// building the full Delassus matrix.
fn prepare_sparse_pgs(model: &Model, data: &Data, qacc_smooth: &DVector<f64>) -> SparsePgsData {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    // Compute M⁻¹ · J^T column by column via sparse LDL
    let (rowadr, rownnz, colind) = model.qld_csr();
    let mut minv_jt = DMatrix::zeros(nv, nefc);
    for col in 0..nefc {
        let mut buf = DVector::zeros(nv);
        for r in 0..nv {
            buf[r] = data.efc_J[(col, r)];
        }
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut buf,
        );
        for r in 0..nv {
            minv_jt[(r, col)] = buf[r];
        }
    }

    // Precompute AR diagonal and inverse diagonal
    let mut ar_diag = vec![0.0_f64; nefc];
    let mut ar_diag_inv = vec![0.0_f64; nefc];
    for i in 0..nefc {
        let mut d = data.efc_R[i];
        for k in 0..nv {
            d += data.efc_J[(i, k)] * minv_jt[(k, i)];
        }
        ar_diag[i] = d;
        ar_diag_inv[i] = if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d };
    }

    // Precompute effective bias: b_eff_i = efc_b_i - J_row_i · qacc_smooth
    let b_eff: Vec<f64> = (0..nefc)
        .map(|i| {
            let mut j_qacc = 0.0;
            for k in 0..nv {
                j_qacc += data.efc_J[(i, k)] * qacc_smooth[k];
            }
            data.efc_b[i] - j_qacc
        })
        .collect();

    SparsePgsData {
        minv_jt,
        ar_diag,
        ar_diag_inv,
        b_eff,
    }
}

/// Compute residual for a single constraint row using qacc.
///
/// `res_i = b_eff_i + J_row_i · qacc + R_i * f_i`
///
/// Cost: O(nv) — dot product of J row with qacc.
#[inline]
fn row_residual(
    efc_j: &DMatrix<f64>,
    qacc: &DVector<f64>,
    b_eff: &[f64],
    efc_r: &[f64],
    efc_force: &DVector<f64>,
    row: usize,
    nv: usize,
) -> f64 {
    let mut j_qacc = 0.0;
    for k in 0..nv {
        j_qacc += efc_j[(row, k)] * qacc[k];
    }
    b_eff[row] + j_qacc + efc_r[row] * efc_force[row]
}

/// Update qacc after changing force on row `i` by `delta_f`.
///
/// `qacc += delta_f * minv_jt_col_i`
///
/// Cost: O(nv).
#[inline]
fn update_qacc(
    qacc: &mut DVector<f64>,
    minv_jt: &DMatrix<f64>,
    row: usize,
    delta_f: f64,
    nv: usize,
) {
    for k in 0..nv {
        qacc[k] += delta_f * minv_jt[(k, row)];
    }
}

/// Compute a dim×dim AR subblock on the fly.
///
/// `AR_block[j][k] = J_row_(base+j) · minv_jt_col_(base+k) + (j==k)*R[base+j]`
///
/// Cost: O(dim² × nv). For dim=4, nv=12: 192 flops.
fn compute_ar_block(
    efc_j: &DMatrix<f64>,
    minv_jt: &DMatrix<f64>,
    efc_r: &[f64],
    base: usize,
    dim: usize,
    nv: usize,
) -> Vec<Vec<f64>> {
    let mut block = vec![vec![0.0_f64; dim]; dim];
    for j in 0..dim {
        for k in 0..dim {
            let mut val = 0.0;
            for r in 0..nv {
                val += efc_j[(base + j, r)] * minv_jt[(r, base + k)];
            }
            if j == k {
                val += efc_r[base + j];
            }
            block[j][k] = val;
        }
    }
    block
}

/// Unified PGS solver with sparse qacc tracking.
///
/// Implements MuJoCo's `mj_solPGS` (§29.3) with O(nefc × nv) per-iteration
/// cost instead of O(nefc²). Uses incremental qacc tracking:
///
/// - Track `qacc = qacc_smooth + M⁻¹ · J^T · f`
/// - Residual: `res_i = b_eff_i + J_row_i · qacc + R_i · f_i` (O(nv))
/// - Update: `qacc += delta_f · minv_jt_col_i` (O(nv))
///
/// AR subblocks for elliptic contacts are computed on-the-fly from
/// `J_block × minv_jt_block` (O(dim² × nv) per block).
// PGS solver inlined as a single function so the elliptic / pyramidal / friction-loss branching and the per-row update loop read end-to-end; single-letter names follow the published PGS notation.
#[allow(
    clippy::many_single_char_names,
    clippy::needless_range_loop,
    clippy::suspicious_operation_groupings,
    clippy::too_many_lines
)]
pub fn pgs_solve_unified(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    if nefc == 0 {
        return;
    }
    let nv = model.nv;

    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = data.qfrc_smooth.clone();

    // Prepare sparse data structures (O(nefc × nv²) — no nefc×nefc matrix)
    let sp = prepare_sparse_pgs(model, data, &qacc_smooth);

    // Initialize efc arrays for classification
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // Warmstart: classify at qacc_warmstart to get warmstart forces
    classify_constraint_states(
        model,
        data,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
    );
    let warm_forces = data.efc_force.clone();

    // Evaluate warmstart dual cost using qacc: O(nefc × nv) instead of O(nefc²)
    // qacc_warm = qacc_smooth + minv_jt · warm_forces
    let mut qacc_warm = qacc_smooth.clone();
    for i in 0..nefc {
        if warm_forces[i] != 0.0 {
            for k in 0..nv {
                qacc_warm[k] += warm_forces[i] * sp.minv_jt[(k, i)];
            }
        }
    }
    let mut dual_cost_warm = 0.0;
    for i in 0..nefc {
        // AR · f_i = J_i · (qacc_warm - qacc_smooth) + R_i · f_i
        let mut j_delta = 0.0;
        for k in 0..nv {
            j_delta += data.efc_J[(i, k)] * (qacc_warm[k] - qacc_smooth[k]);
        }
        let ar_f_i = j_delta + data.efc_R[i] * warm_forces[i];
        dual_cost_warm += 0.5 * warm_forces[i] * ar_f_i + warm_forces[i] * data.efc_b[i];
    }

    // Initialize qacc and forces from warmstart or cold start
    let mut qacc = if dual_cost_warm < 0.0 {
        data.efc_force.copy_from(&warm_forces);
        qacc_warm
    } else {
        data.efc_force.fill(0.0);
        qacc_smooth.clone()
    };

    let max_iters = model.solver_iterations;
    // `niter` and `nv` are usize → f64 for convergence diagnostics; bounded by solver budget, far below 2^52.
    #[allow(clippy::cast_precision_loss)]
    let scale = 1.0 / (data.stat_meaninertia * 1.0_f64.max(nv as f64));
    let tolerance = model.solver_tolerance;

    let mut iter = 0;
    let mut solver_stats: Vec<SolverStat> = Vec::with_capacity(max_iters);

    while iter < max_iters {
        let mut improvement = 0.0_f64;

        let mut i = 0;
        while i < nefc {
            let ctype = data.efc_type[i];
            let dim = data.efc_dim[i];

            // Elliptic contacts: two-phase ray update + QCQP
            if matches!(ctype, ConstraintType::ContactElliptic) && dim > 1 {
                let mu = data.efc_mu[i];

                // Save old forces
                let old_force: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

                // Compute residual for the block: O(dim × nv)
                let mut res = vec![0.0_f64; dim];
                for j in 0..dim {
                    res[j] = row_residual(
                        &data.efc_J,
                        &qacc,
                        &sp.b_eff,
                        &data.efc_R,
                        &data.efc_force,
                        i + j,
                        nv,
                    );
                }

                // Compute AR subblock on the fly: O(dim² × nv)
                let ar_block = compute_ar_block(&data.efc_J, &sp.minv_jt, &data.efc_R, i, dim, nv);

                // ---- Phase 1: Ray update ----
                if data.efc_force[i] < MJ_MINVAL {
                    data.efc_force[i] -= res[0] * sp.ar_diag_inv[i];
                    if data.efc_force[i] < 0.0 {
                        data.efc_force[i] = 0.0;
                    }
                    for j in 1..dim {
                        data.efc_force[i + j] = 0.0;
                    }
                } else {
                    let v: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

                    // v1 = AR_block * v
                    let mut v1 = vec![0.0_f64; dim];
                    for j in 0..dim {
                        for k in 0..dim {
                            v1[j] += ar_block[j][k] * v[k];
                        }
                    }

                    let denom: f64 = v.iter().zip(v1.iter()).map(|(a, b)| a * b).sum();
                    if denom >= MJ_MINVAL {
                        let x_step =
                            -v.iter().zip(res.iter()).map(|(a, b)| a * b).sum::<f64>() / denom;
                        let x_clamped = if data.efc_force[i] + x_step * v[0] < 0.0 {
                            -data.efc_force[i] / v[0]
                        } else {
                            x_step
                        };
                        for j in 0..dim {
                            data.efc_force[i + j] += x_clamped * v[j];
                        }
                    }
                }

                // ---- Phase 2: Friction QCQP (normal fixed) ----
                let fn_current = data.efc_force[i];
                let fdim = dim - 1;

                if fn_current < MJ_MINVAL {
                    for j in 1..dim {
                        data.efc_force[i + j] = 0.0;
                    }
                } else {
                    // Build friction-friction subblock from ar_block
                    let mut ac_flat = vec![0.0_f64; fdim * fdim];
                    let mut bc = vec![0.0_f64; fdim];
                    for j in 0..fdim {
                        for k in 0..fdim {
                            ac_flat[j * fdim + k] = ar_block[1 + j][1 + k];
                        }
                        bc[j] = res[j + 1];
                        for k in 0..fdim {
                            bc[j] -= ac_flat[j * fdim + k] * old_force[1 + k];
                        }
                        bc[j] += ar_block[1 + j][0] * (fn_current - old_force[0]);
                    }

                    let (fric_result, active) = if fdim == 2 {
                        let ac2 = [[ac_flat[0], ac_flat[1]], [ac_flat[2], ac_flat[3]]];
                        let (r, a) = crate::constraint::solver::qcqp::qcqp2(
                            ac2,
                            [bc[0], bc[1]],
                            [mu[0], mu[1]],
                            fn_current,
                        );
                        (r.to_vec(), a)
                    } else if fdim == 3 {
                        let ac3 = [
                            [ac_flat[0], ac_flat[1], ac_flat[2]],
                            [ac_flat[3], ac_flat[4], ac_flat[5]],
                            [ac_flat[6], ac_flat[7], ac_flat[8]],
                        ];
                        let (r, a) = crate::constraint::solver::qcqp::qcqp3(
                            ac3,
                            [bc[0], bc[1], bc[2]],
                            [mu[0], mu[1], mu[2]],
                            fn_current,
                        );
                        (r.to_vec(), a)
                    } else {
                        crate::constraint::solver::qcqp::qcqp_nd(
                            &ac_flat,
                            &bc,
                            &mu[..fdim],
                            fn_current,
                            fdim,
                        )
                    };

                    if active {
                        let mut ssq = 0.0_f64;
                        for j in 0..fdim {
                            if mu[j] > MJ_MINVAL {
                                ssq += (fric_result[j] / mu[j]).powi(2);
                            }
                        }
                        let s = (fn_current * fn_current / ssq.max(MJ_MINVAL)).sqrt();
                        for j in 0..fdim {
                            data.efc_force[i + 1 + j] = fric_result[j] * s;
                        }
                    } else {
                        for j in 0..fdim {
                            data.efc_force[i + 1 + j] = fric_result[j];
                        }
                    }
                }

                // ---- Cost guard using AR subblock ----
                let cost_change = {
                    let mut cc = 0.0_f64;
                    for j in 0..dim {
                        let delta_j = data.efc_force[i + j] - old_force[j];
                        cc += delta_j * res[j];
                        for k in 0..dim {
                            let delta_k = data.efc_force[i + k] - old_force[k];
                            cc += 0.5 * delta_j * ar_block[j][k] * delta_k;
                        }
                    }
                    if cc > 1e-10 {
                        // Revert
                        data.efc_force.as_mut_slice()[i..i + dim].copy_from_slice(&old_force);
                        0.0
                    } else {
                        cc
                    }
                };

                // Update qacc for force changes in this block
                for j in 0..dim {
                    let delta = data.efc_force[i + j] - old_force[j];
                    if delta != 0.0 {
                        update_qacc(&mut qacc, &sp.minv_jt, i + j, delta, nv);
                    }
                }

                improvement -= cost_change;
                i += dim;
            } else {
                // Scalar constraint: single-row GS update + projection
                let old_force = data.efc_force[i];

                // Compute residual: O(nv) instead of O(nefc)
                let res = row_residual(
                    &data.efc_J,
                    &qacc,
                    &sp.b_eff,
                    &data.efc_R,
                    &data.efc_force,
                    i,
                    nv,
                );

                // GS update
                data.efc_force[i] -= res * sp.ar_diag_inv[i];

                // Project per constraint type
                match ctype {
                    ConstraintType::Equality | ConstraintType::FlexEdge => {}
                    ConstraintType::FrictionLoss => {
                        let fl = data.efc_floss[i];
                        data.efc_force[i] = data.efc_force[i].clamp(-fl, fl);
                    }
                    ConstraintType::LimitJoint
                    | ConstraintType::LimitTendon
                    | ConstraintType::ContactFrictionless
                    | ConstraintType::ContactPyramidal
                    | ConstraintType::ContactElliptic => {
                        data.efc_force[i] = data.efc_force[i].max(0.0);
                    }
                }

                // Cost guard using AR diagonal
                let delta_f = data.efc_force[i] - old_force;
                let cost_change = if delta_f.abs() > 0.0 {
                    let cc = 0.5 * delta_f * delta_f * sp.ar_diag[i] + delta_f * res;
                    if cc > 1e-10 {
                        data.efc_force[i] = old_force;
                        0.0
                    } else {
                        // Update qacc incrementally
                        update_qacc(&mut qacc, &sp.minv_jt, i, delta_f, nv);
                        cc
                    }
                } else {
                    0.0
                };
                improvement -= cost_change;

                i += 1;
            }
        }

        improvement *= scale;

        solver_stats.push(SolverStat {
            improvement,
            gradient: 0.0,
            lineslope: 0.0,
            nactive: 0,
            nchange: 0,
            nline: 0,
        });

        iter += 1;

        if improvement < tolerance {
            break;
        }
    }

    data.solver_niter = iter;
    data.solver_stat = solver_stats;
}

/// Classify constraint states, compute forces, and evaluate total cost.
///
/// Implements PrimalUpdateConstraint from §15.1:
/// - Per-row state machine (Quadratic, Satisfied, LinearNeg, LinearPos, Cone)
/// - Force computation from state
/// - Total cost = Gauss term + constraint penalties
///
/// # Arguments
/// * `model` - Model with solver parameters
/// * `data` - Data with efc_* arrays (modified: efc_state, efc_force, efc_cost)
/// * `qacc` - Current acceleration estimate
/// * `qacc_smooth` - Unconstrained smooth acceleration (M⁻¹ · qfrc_smooth)
/// * `qfrc_smooth` - Smooth force (excluding friction loss)
// Friction pyramid update takes the full per-contact state (normal/tangent/multipliers/work buffers).
#[allow(clippy::too_many_arguments)]
// Single-letter names (a, b, c, p, q, r) follow the standard PGS friction-pyramid notation.
#[allow(clippy::many_single_char_names)]
pub fn classify_constraint_states(
    model: &Model,
    data: &mut Data,
    qacc: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
) {
    let nv = model.nv;
    let nefc = data.efc_type.len();

    // Reset cone Hessian tracking for this classification pass
    data.ncone = 0;
    for hc in &mut data.efc_cone_hessian {
        *hc = None;
    }

    // Compute jar = J · qacc - aref for all rows
    for i in 0..nefc {
        let mut j_dot_qacc = 0.0;
        for col in 0..nv {
            j_dot_qacc += data.efc_J[(i, col)] * qacc[col];
        }
        data.efc_jar[i] = j_dot_qacc - data.efc_aref[i];
    }

    // Compute Ma = M · qacc
    let mut ma = DVector::<f64>::zeros(nv);
    for row_idx in 0..nv {
        for col_idx in 0..nv {
            ma[row_idx] += data.qM[(row_idx, col_idx)] * qacc[col_idx];
        }
    }

    // Classify each row and compute forces
    let mut constraint_cost = 0.0;
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let jar = data.efc_jar[i];
        let d = data.efc_D[i];
        let r = data.efc_R[i];

        match ctype {
            ConstraintType::Equality | ConstraintType::FlexEdge => {
                data.efc_state[i] = ConstraintState::Quadratic;
                data.efc_force[i] = -d * jar;
                constraint_cost += 0.5 * d * jar * jar;
                i += 1;
            }

            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                let threshold = r * floss;
                if jar >= threshold {
                    data.efc_state[i] = ConstraintState::LinearPos;
                    data.efc_force[i] = -floss;
                    constraint_cost += floss * jar - 0.5 * r * floss * floss;
                } else if jar <= -threshold {
                    data.efc_state[i] = ConstraintState::LinearNeg;
                    data.efc_force[i] = floss;
                    constraint_cost += -floss * jar - 0.5 * r * floss * floss;
                } else {
                    data.efc_state[i] = ConstraintState::Quadratic;
                    data.efc_force[i] = -d * jar;
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }

            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
                if jar < 0.0 {
                    data.efc_state[i] = ConstraintState::Quadratic;
                    data.efc_force[i] = -d * jar;
                    constraint_cost += 0.5 * d * jar * jar;
                } else {
                    data.efc_state[i] = ConstraintState::Satisfied;
                    data.efc_force[i] = 0.0;
                }
                i += 1;
            }

            ConstraintType::ContactElliptic => {
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0];

                let u0 = data.efc_jar[i] * mu;
                let n = u0;

                let mut t_sq = 0.0;
                for j in 1..dim {
                    let friction_j = data.efc_mu[i][j - 1];
                    let uj = data.efc_jar[i + j] * friction_j;
                    t_sq += uj * uj;
                }
                let t = t_sq.sqrt();
                let t_min = MJ_MINVAL;

                if n >= mu * t || (t < t_min && n >= 0.0) {
                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Satisfied;
                        data.efc_force[i + j] = 0.0;
                    }
                } else if mu * n + t <= 0.0 || (t < t_min && n < 0.0) {
                    for j in 0..dim {
                        let jar_j = data.efc_jar[i + j];
                        let d_j = data.efc_D[i + j];
                        data.efc_state[i + j] = ConstraintState::Quadratic;
                        data.efc_force[i + j] = -d_j * jar_j;
                        constraint_cost += 0.5 * d_j * jar_j * jar_j;
                    }
                } else {
                    let d_normal = data.efc_D[i];
                    let dm = d_normal / (mu * mu * (1.0 + mu * mu));
                    let n_minus_mu_t = n - mu * t;

                    constraint_cost += 0.5 * dm * n_minus_mu_t * n_minus_mu_t;

                    let f_normal = -dm * n_minus_mu_t * mu;
                    data.efc_force[i] = f_normal;

                    for j in 1..dim {
                        let friction_j = data.efc_mu[i][j - 1];
                        let uj = data.efc_jar[i + j] * friction_j;
                        data.efc_force[i + j] = (dm * n_minus_mu_t * mu / t) * uj * friction_j;
                    }

                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Cone;
                    }

                    let ci = data.efc_id[i];
                    let t_safe = t.max(MJ_MINVAL);
                    let mut hc = DMatrix::<f64>::zeros(dim, dim);

                    hc[(0, 0)] = 1.0;

                    let mut u_vec = vec![0.0_f64; dim];
                    u_vec[0] = u0;
                    for (j, u_j) in u_vec.iter_mut().enumerate().skip(1) {
                        let friction_j = data.efc_mu[i][j - 1];
                        *u_j = data.efc_jar[i + j] * friction_j;
                    }

                    for j in 1..dim {
                        let val = -mu * u_vec[j] / t_safe;
                        hc[(0, j)] = val;
                        hc[(j, 0)] = val;
                    }

                    for k in 1..dim {
                        for j in 1..dim {
                            hc[(k, j)] = mu * n / (t_safe * t_safe * t_safe) * u_vec[k] * u_vec[j];
                        }
                        hc[(k, k)] += mu * mu - mu * n / t_safe;
                    }

                    let mut scale_vec = vec![0.0_f64; dim];
                    scale_vec[0] = mu;
                    scale_vec[1..dim].copy_from_slice(&data.efc_mu[i][..(dim - 1)]);
                    for k in 0..dim {
                        for j in 0..dim {
                            hc[(k, j)] *= scale_vec[k] * scale_vec[j] * dm;
                        }
                    }

                    data.efc_cone_hessian[ci] = Some(hc);
                    data.ncone += 1;
                }
                i += dim;
            }
        }
    }

    // Gauss term: ½·(Ma − qfrc_smooth)·(qacc − qacc_smooth)
    let mut cost_gauss = 0.0;
    for k in 0..nv {
        cost_gauss += (ma[k] - qfrc_smooth[k]) * (qacc[k] - qacc_smooth[k]);
    }
    cost_gauss *= 0.5;

    data.efc_cost = cost_gauss + constraint_cost;
}
