//! Noslip post-processor: suppresses residual contact slip (§15.10).
//!
//! Runs a modified PGS pass on friction force rows only, without
//! regularization, using the current normal forces as fixed cone limits.
//! This matches MuJoCo's `mj_solNoSlip`.

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::qcqp;
use crate::linalg::mj_solve_sparse;
use crate::types::{ConstraintType, Data, Model};

/// Tag for each noslip row, used to select the correct projection.
#[derive(Clone, Copy)]
enum NoslipRowKind {
    /// Friction-loss row: clamp to [-floss, +floss].
    FrictionLoss { floss: f64 },
    /// Elliptic contact friction row: cone projection with parent contact.
    /// `group_start`: local index within submatrix of first friction row of this contact.
    /// `group_len`: number of friction rows for this contact.
    /// `contact_efc_start`: efc row index of the parent contact's normal row.
    EllipticFriction {
        group_start: usize,
        group_len: usize,
        contact_efc_start: usize,
    },
    /// Pyramidal contact row: 2x2 block solve on pairs.
    /// `group_start`: local index of first row of this contact in submatrix.
    /// `group_len`: total rows for this contact = 2*(dim-1).
    PyramidalFriction {
        group_start: usize,
        group_len: usize,
    },
}

/// Noslip post-processor: suppresses residual contact slip (§15.10).
///
/// Runs a modified PGS pass on friction force rows only, without
/// regularization, using the current normal forces as fixed cone limits.
/// This matches MuJoCo's `mj_solNoSlip`.
///
/// Called after any solver (Newton, PGS, CG) when `model.noslip_iterations > 0`.
/// Also called after Newton's PGS fallback path. Matches MuJoCo's `mj_solNoSlip`.
///
/// Processes two row ranges (NOT equality rows):
/// 1. Friction-loss rows: PGS update with interval clamping [-floss, +floss]
/// 2. Contact friction rows: elliptic QCQP or pyramidal 2x2 block solve
///
/// All noslip processing uses the UNREGULARIZED Delassus matrix A (without R).
/// This matches MuJoCo's flg_subR=1 in ARdiaginv() and residual().
// Single-letter names follow the Delassus-matrix notation; cast lints are usize → f64 for residual norms; indexed loops mutate parallel multiplier arrays at the same row index.
#[allow(
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::needless_range_loop
)]
pub fn noslip_postprocess(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nefc = data.efc_type.len();
    let noslip_iter = model.noslip_iterations;
    let noslip_tol = model.noslip_tolerance;

    if noslip_iter == 0 || nefc == 0 {
        return;
    }

    // =========================================================================
    // 1. Identify noslip-eligible rows: friction-loss + contact friction.
    //    Each row is tagged with its type for projection in the PGS loop.
    // =========================================================================

    let mut noslip_rows: Vec<usize> = Vec::new();
    let mut noslip_kinds: Vec<NoslipRowKind> = Vec::new();

    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let dim = data.efc_dim[i];

        match ctype {
            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                noslip_rows.push(i);
                noslip_kinds.push(NoslipRowKind::FrictionLoss { floss });
                i += 1;
            }
            ConstraintType::ContactElliptic | ConstraintType::ContactFrictionless => {
                if dim >= 3 {
                    let group_start = noslip_rows.len();
                    let group_len = dim - 1;
                    for j in 1..dim {
                        noslip_rows.push(i + j);
                        noslip_kinds.push(NoslipRowKind::EllipticFriction {
                            group_start,
                            group_len,
                            contact_efc_start: i,
                        });
                    }
                }
                i += dim;
            }
            ConstraintType::ContactPyramidal => {
                // §33 S4: Pyramidal contacts have 2*(dim-1) rows, all are friction.
                // Include them for noslip processing.
                let nrows = dim; // all rows are friction edges for pyramidal
                let group_start = noslip_rows.len();
                for j in 0..nrows {
                    noslip_rows.push(i + j);
                    noslip_kinds.push(NoslipRowKind::PyramidalFriction {
                        group_start,
                        group_len: nrows,
                    });
                }
                i += dim;
            }
            _ => {
                i += 1;
            }
        }
    }

    let n = noslip_rows.len();
    if n == 0 {
        return;
    }

    // =========================================================================
    // 2. Build noslip Delassus submatrix A (UNREGULARIZED) and effective bias.
    //
    //    For each noslip row i, we compute M⁻¹ · J[row_i]^T, then dot against
    //    ALL nefc Jacobian rows (not just noslip rows). This gives:
    //      - a_sub[fi,fj] = J[row_i] · M⁻¹ · J[row_j]^T  (noslip-to-noslip)
    //      - b_eff[fi] = efc_b[row_i] + Σ_{j NOT noslip} A[row_i,j] * efc_force[j]
    //
    //    The PGS iteration then uses: res = b_eff[fi] + Σ_k a_sub[fi,k] * f[k]
    //    which equals the full-matrix residual: efc_b[i] + Σ_{j=0..nefc} A[i,j]*f[j]
    //
    //    This matches MuJoCo's mj_solNoSlip which uses efc_b (not efc_jar) and
    //    the full Delassus row including cross-coupling to non-noslip constraints.
    // =========================================================================

    // Build global→local index map for noslip rows
    let mut noslip_local: Vec<Option<usize>> = vec![None; nefc];
    for (fi, &row) in noslip_rows.iter().enumerate() {
        noslip_local[row] = Some(fi);
    }

    let mut a_sub = DMatrix::<f64>::zeros(n, n);
    let mut b_eff: Vec<f64> = Vec::with_capacity(n);
    for (fi, &row_i) in noslip_rows.iter().enumerate() {
        // Solve M⁻¹ · J[row_i]^T
        let mut minv_ji = DVector::<f64>::zeros(nv);
        for col in 0..nv {
            minv_ji[col] = data.efc_J[(row_i, col)];
        }
        let (rowadr, rownnz, colind) = model.qld_csr();
        mj_solve_sparse(
            rowadr,
            rownnz,
            colind,
            &data.qLD_data,
            &data.qLD_diag_inv,
            &mut minv_ji,
        );

        // Start from efc_b (constraint bias, fixed at assembly)
        let mut b_i = data.efc_b[row_i];

        // Dot against ALL nefc rows
        for j in 0..nefc {
            let mut dot = 0.0;
            for col in 0..nv {
                dot += data.efc_J[(j, col)] * minv_ji[col];
            }
            if let Some(fj) = noslip_local[j] {
                // Noslip row: store in submatrix
                a_sub[(fi, fj)] = dot;
            } else {
                // Non-noslip row: absorb cross-coupling into effective bias
                b_i += dot * data.efc_force[j];
            }
        }

        b_eff.push(b_i);
    }

    // 3. Extract current forces
    let mut f: Vec<f64> = noslip_rows.iter().map(|&r| data.efc_force[r]).collect();

    // Precompute unregularized diagonal inverse
    let diag_inv: Vec<f64> = (0..n)
        .map(|fi| {
            let d = a_sub[(fi, fi)];
            if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d }
        })
        .collect();

    // Convergence scaling (matches CG/Newton: 1/(meaninertia * max(1, nv)))
    let conv_scale = 1.0 / (data.stat_meaninertia * (1.0_f64).max(nv as f64));

    // =========================================================================
    // 4. PGS iterations with per-type projection
    // =========================================================================
    for iter_num in 0..noslip_iter {
        let mut improvement = 0.0_f64;

        // Iter-0 cost correction: account for regularization cost removed by
        // unregularized noslip pass. Matches MuJoCo's mj_solNoSlip iter==0 branch.
        if iter_num == 0 {
            for fi in 0..n {
                let row = noslip_rows[fi];
                improvement += 0.5 * f[fi] * f[fi] * data.efc_R[row];
            }
        }

        // Phase A: Friction-loss rows — scalar PGS + interval clamping
        for fi in 0..n {
            if let NoslipRowKind::FrictionLoss { floss } = noslip_kinds[fi] {
                // Unregularized residual: b_eff + A*f
                let mut residual = b_eff[fi];
                for k in 0..n {
                    residual += a_sub[(fi, k)] * f[k];
                }
                let old = f[fi];
                f[fi] -= residual * diag_inv[fi];
                f[fi] = f[fi].clamp(-floss, floss);
                let delta = f[fi] - old;
                // Cost change: ½δ²·A_diag + δ·residual (negative = improvement)
                improvement -= 0.5 * delta * delta * a_sub[(fi, fi)] + delta * residual;
            }
        }

        // Phase B: Elliptic contact friction — QCQP cone projection
        {
            let mut fi = 0;
            while fi < n {
                if let NoslipRowKind::EllipticFriction {
                    group_start,
                    group_len,
                    contact_efc_start,
                } = noslip_kinds[fi]
                {
                    if fi == group_start {
                        let normal_force = data.efc_force[contact_efc_start];
                        let mu = data.efc_mu[contact_efc_start];
                        let fn_abs = normal_force.abs();

                        // Save old friction forces
                        let old_forces: Vec<f64> =
                            (0..group_len).map(|j| f[group_start + j]).collect();

                        // Compute residuals for all friction rows
                        let mut residuals: Vec<f64> = Vec::with_capacity(group_len);
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            let mut res = b_eff[idx];
                            for k in 0..n {
                                res += a_sub[(idx, k)] * f[k];
                            }
                            residuals.push(res);
                        }

                        if fn_abs < MJ_MINVAL {
                            // Zero normal force: zero all friction
                            for local_j in 0..group_len {
                                f[group_start + local_j] = 0.0;
                            }
                        } else {
                            // Build friction-friction subblock Ac and bias bc
                            let mut ac_flat = vec![0.0_f64; group_len * group_len];
                            let mut bc = vec![0.0_f64; group_len];
                            for j in 0..group_len {
                                for k in 0..group_len {
                                    ac_flat[j * group_len + k] =
                                        a_sub[(group_start + j, group_start + k)];
                                }
                                // bc[j] = res[j] - Σ Ac[j,k] * old[k]
                                bc[j] = residuals[j];
                                for k in 0..group_len {
                                    bc[j] -= ac_flat[j * group_len + k] * old_forces[k];
                                }
                            }

                            // QCQP dispatch
                            let (fric_result, active) = if group_len == 2 {
                                let ac2 = [[ac_flat[0], ac_flat[1]], [ac_flat[2], ac_flat[3]]];
                                let (r, a) =
                                    qcqp::qcqp2(ac2, [bc[0], bc[1]], [mu[0], mu[1]], fn_abs);
                                (r.to_vec(), a)
                            } else if group_len == 3 {
                                let ac3 = [
                                    [ac_flat[0], ac_flat[1], ac_flat[2]],
                                    [ac_flat[3], ac_flat[4], ac_flat[5]],
                                    [ac_flat[6], ac_flat[7], ac_flat[8]],
                                ];
                                let (r, a) = qcqp::qcqp3(
                                    ac3,
                                    [bc[0], bc[1], bc[2]],
                                    [mu[0], mu[1], mu[2]],
                                    fn_abs,
                                );
                                (r.to_vec(), a)
                            } else {
                                qcqp::qcqp_nd(&ac_flat, &bc, &mu[..group_len], fn_abs, group_len)
                            };

                            // Ellipsoidal rescale if constraint active
                            if active {
                                let mut ssq = 0.0_f64;
                                for j in 0..group_len {
                                    if mu[j] > MJ_MINVAL {
                                        ssq += (fric_result[j] / mu[j]).powi(2);
                                    }
                                }
                                let s = (fn_abs * fn_abs / ssq.max(MJ_MINVAL)).sqrt();
                                for j in 0..group_len {
                                    f[group_start + j] = fric_result[j] * s;
                                }
                            } else {
                                f[group_start..group_start + group_len]
                                    .copy_from_slice(&fric_result[..group_len]);
                            }
                        }

                        // Accumulate cost improvement for the whole group
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            let delta = f[idx] - old_forces[local_j];
                            improvement -= 0.5 * delta * delta * a_sub[(idx, idx)]
                                + delta * residuals[local_j];
                        }

                        fi += group_len;
                        continue;
                    }
                }
                fi += 1;
            }
        }

        // Phase C: Pyramidal contact friction — 2x2 block solve on pairs
        {
            let mut fi = 0;
            while fi < n {
                if let NoslipRowKind::PyramidalFriction {
                    group_start,
                    group_len,
                } = noslip_kinds[fi]
                {
                    if fi == group_start {
                        // Process pairs: (0,1), (2,3), ..., (group_len-2, group_len-1)
                        let mut k = 0;
                        while k < group_len {
                            if k + 1 >= group_len {
                                break;
                            }
                            let j0 = group_start + k;
                            let j1 = group_start + k + 1;

                            let old = [f[j0], f[j1]];
                            let mid = 0.5 * (old[0] + old[1]);

                            // 2x2 block of unregularized A
                            let a00 = a_sub[(j0, j0)];
                            let a11 = a_sub[(j1, j1)];
                            let a01 = a_sub[(j0, j1)];

                            // Unregularized residual for both rows
                            let mut res0 = b_eff[j0];
                            let mut res1 = b_eff[j1];
                            for c in 0..n {
                                res0 += a_sub[(j0, c)] * f[c];
                                res1 += a_sub[(j1, c)] * f[c];
                            }

                            // Constant part: bc = res - A * old
                            let bc0 = res0 - a00 * old[0] - a01 * old[1];
                            let bc1 = res1 - a01 * old[0] - a11 * old[1];

                            // 1D optimization over y (change of variables: f0=mid+y, f1=mid-y)
                            let k1 = a00 + a11 - 2.0 * a01; // curvature
                            let k0 = mid * (a00 - a11) + bc0 - bc1; // gradient at y=0

                            if k1 < MJ_MINVAL {
                                // Degenerate curvature: split evenly
                                f[j0] = mid;
                                f[j1] = mid;
                            } else {
                                let y = -k0 / k1;
                                // Clamp: f0 >= 0 and f1 >= 0 ⟹ |y| <= mid
                                if y < -mid {
                                    f[j0] = 0.0;
                                    f[j1] = 2.0 * mid;
                                } else if y > mid {
                                    f[j0] = 2.0 * mid;
                                    f[j1] = 0.0;
                                } else {
                                    f[j0] = mid + y;
                                    f[j1] = mid - y;
                                }
                            }

                            // Cost rollback: revert if cost increased
                            let d0 = f[j0] - old[0];
                            let d1 = f[j1] - old[1];
                            let cost = 0.5 * (d0 * d0 * a00 + d1 * d1 * a11 + 2.0 * d0 * d1 * a01)
                                + d0 * res0
                                + d1 * res1;
                            if cost > 1e-10 {
                                f[j0] = old[0];
                                f[j1] = old[1];
                            } else {
                                // Accumulate cost improvement (cost is negative = improvement)
                                improvement -= cost;
                            }

                            k += 2;
                        }

                        fi += group_len;
                        continue;
                    }
                }
                fi += 1;
            }
        }

        // Cost-based convergence (matches MuJoCo's CG/Newton pattern)
        if improvement * conv_scale < noslip_tol {
            break;
        }
    }

    // =========================================================================
    // 5. Write back updated forces
    // =========================================================================
    for (fi, &row) in noslip_rows.iter().enumerate() {
        data.efc_force[row] = f[fi];
    }

    // 6. Recompute qfrc_constraint = J^T · efc_force
    data.qfrc_constraint.fill(0.0);
    for i_row in 0..nefc {
        let force = data.efc_force[i_row];
        if force == 0.0 {
            continue;
        }
        for col in 0..nv {
            data.qfrc_constraint[col] += data.efc_J[(i_row, col)] * force;
        }
    }

    // 7. Recompute qacc = M⁻¹ · (qfrc_smooth + qfrc_constraint)
    for k in 0..nv {
        data.qacc[k] = data.qfrc_applied[k] + data.qfrc_actuator[k] + data.qfrc_passive[k]
            - data.qfrc_bias[k]
            + data.qfrc_constraint[k];
    }
    let (rowadr, rownnz, colind) = model.qld_csr();
    mj_solve_sparse(
        rowadr,
        rownnz,
        colind,
        &data.qLD_data,
        &data.qLD_diag_inv,
        &mut data.qacc,
    );
}
