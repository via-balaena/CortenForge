//! Noslip post-processor: suppresses residual contact slip (§15.10).
//!
//! Runs a modified PGS pass on friction force rows only, without
//! regularization, using the current normal forces as fixed cone limits.
//! This matches MuJoCo's `mj_solNoSlip`.

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::primal::MJ_NSOLVER;
use crate::constraint::solver::qcqp;
use crate::dynamics::factor::mj_solve_m;
use crate::linalg::{cholesky_solve_in_place, mju_dot};
use crate::types::{ConstraintState, ConstraintType, Data, Model, SolverStat};

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
/// regularization, using the current normal forces as fixed cone limits:
/// MuJoCo's `mj_solNoSlip` in its sweeps, counts, statistics and states. Its
/// residuals come from a dense J·M⁻¹·Jᵀ rather than MuJoCo's `efc_AR`, so it
/// rounds otherwise (ledger L142).
///
/// Called after any solver (Newton, PGS, CG) when `model.noslip_iterations > 0`.
/// Leaves `efc_force` and `qfrc_constraint`;
/// the forward pass makes the acceleration from them (`mj_fwd_acceleration`).
/// Its sweeps are counted in `solver_niter` and each one's statistics are
/// appended to `solver_stat` after the solver's, and `efc_state` is left as
/// [`dual_state`] classifies the final forces (`engine_solver.c:556`,
/// `:716-741`).
///
/// Processes two row ranges (NOT equality rows):
/// 1. Friction-loss rows: PGS update with interval clamping [-floss, +floss]
/// 2. Contact friction rows: elliptic QCQP or pyramidal 2x2 block solve
///
/// All noslip processing uses the UNREGULARIZED Delassus matrix A (without R).
/// This matches MuJoCo's flg_subR=1 in ARdiaginv() and residual().
///
/// Under the implicit spring-damper integrator `m_impl_factor` is the
/// Cholesky factor of `M_impl`, which takes the mass matrix's place, as it
/// does for Newton, CG and PGS.
// Single-letter names follow the Delassus-matrix notation; cast lints are usize → f64 for residual norms; indexed loops mutate parallel multiplier arrays at the same row index.
#[allow(
    clippy::many_single_char_names,
    clippy::cast_precision_loss,
    clippy::needless_range_loop
)]
pub fn noslip_postprocess(model: &Model, data: &mut Data, m_impl_factor: Option<&DMatrix<f64>>) {
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

    // No early return without noslip rows: MuJoCo's sweep still runs, and
    // is counted and recorded.
    let n = noslip_rows.len();

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
        match m_impl_factor {
            Some(factor) => cholesky_solve_in_place(factor, &mut minv_ji),
            None => mj_solve_m(model, data, minv_ji.as_mut_slice()),
        }

        // Start from efc_b (the constraint bias the solve started from)
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

    // The statistics' scale, MuJoCo's: the model's mean inertia
    // (`engine_solver.c:550`).
    let conv_scale = 1.0 / (model.stat_meaninertia * (1.0_f64).max(nv as f64));

    // The rows' states before the first sweep, as MuJoCo's `dualState`.
    let mut state = std::mem::take(&mut data.efc_state);
    dual_state(data, &mut state);

    // =========================================================================
    // 4. PGS iterations with per-type projection
    // =========================================================================
    let mut iter_num = 0;
    while iter_num < noslip_iter {
        let mut improvement = 0.0_f64;

        // Iter-0 cost correction: account for regularization cost removed by
        // unregularized noslip pass, over every row as MuJoCo's mj_solNoSlip
        // iter==0 branch (`engine_solver.c:563-567`).
        if iter_num == 0 {
            for row in 0..nefc {
                improvement += 0.5 * data.efc_force[row] * data.efc_force[row] * data.efc_R[row];
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
                    && fi == group_start
                {
                    let normal_force = data.efc_force[contact_efc_start];
                    let mu = data.efc_mu[contact_efc_start];
                    let fn_abs = normal_force.abs();

                    // Save old friction forces
                    let old_forces: Vec<f64> = (0..group_len).map(|j| f[group_start + j]).collect();

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

                    // Friction-friction subblock Ac and bias bc
                    let mut ac_flat = vec![0.0_f64; group_len * group_len];
                    let mut bc = vec![0.0_f64; group_len];
                    for j in 0..group_len {
                        for k in 0..group_len {
                            ac_flat[j * group_len + k] = a_sub[(group_start + j, group_start + k)];
                        }
                        // bc[j] = res[j] - Σ Ac[j,k] * old[k]
                        bc[j] = residuals[j];
                        for k in 0..group_len {
                            bc[j] -= ac_flat[j * group_len + k] * old_forces[k];
                        }
                    }

                    if fn_abs < MJ_MINVAL {
                        // Zero normal force: zero all friction
                        for local_j in 0..group_len {
                            f[group_start + local_j] = 0.0;
                        }
                    } else {
                        // QCQP dispatch
                        let (fric_result, active) = if group_len == 2 {
                            let ac2 = [[ac_flat[0], ac_flat[1]], [ac_flat[2], ac_flat[3]]];
                            let (r, a) = qcqp::qcqp2(ac2, [bc[0], bc[1]], [mu[0], mu[1]], fn_abs);
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

                    improvement -= cost_change(
                        &ac_flat,
                        &mut f[group_start..group_start + group_len],
                        &old_forces,
                        &residuals,
                    );

                    fi += group_len;
                    continue;
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
                    && fi == group_start
                {
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
                fi += 1;
            }
        }

        // The sweep's forces, then the rows' states from them and the sweep's
        // statistics after the solver's (`engine_solver.c:716-729`).
        for (fi, &row) in noslip_rows.iter().enumerate() {
            data.efc_force[row] = f[fi];
        }
        let old_state = state.clone();
        let nactive = dual_state(data, &mut state);
        let nchange = old_state.iter().zip(&state).filter(|(a, b)| a != b).count();
        let improvement = improvement * conv_scale;
        // Kept after the solver's while there are fewer than mjNSOLVER
        // (`engine_solver.c:728-729`).
        if data.solver_niter + iter_num < MJ_NSOLVER {
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
        iter_num += 1;

        // Cost-based convergence (matches MuJoCo's CG/Newton pattern)
        if improvement < noslip_tol {
            break;
        }
    }
    data.efc_state = state;
    data.solver_niter += iter_num;

    // 5. Recompute qfrc_constraint = J^T · efc_force
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
}

/// MuJoCo's `costChange` (`engine_solver.c:207-228`) for a block of more
/// than one row (an elliptic contact's friction): the change in the dual
/// cost of moving its forces from `old` to `force`, ½·δᵀ·A·δ + δ·res for
/// δ = `force` − `old`, `a` the block row by row and `res` its residual at
/// `old`. A rise above 1e-10 restores `old` and counts as none.
fn cost_change(a: &[f64], force: &mut [f64], old: &[f64], res: &[f64]) -> f64 {
    let dim = force.len();
    let delta: Vec<f64> = force.iter().zip(old).map(|(f, o)| f - o).collect();
    let mut quad = 0.0;
    for (i, &di) in delta.iter().enumerate() {
        quad += di * mju_dot(&a[i * dim..(i + 1) * dim], &delta);
    }
    let change = 0.5 * quad + mju_dot(&delta, res);
    if change > 1e-10 {
        force.copy_from_slice(old);
        return 0.0;
    }
    change
}

/// Each row's state from its force alone, MuJoCo's `dualState`
/// (`engine_solver.c:233-310`); returns the rows not satisfied. Equality rows
/// are quadratic; a friction-loss row is linear at its bound, the side
/// opposite the primal's; a limit or non-elliptic contact row is quadratic
/// while its force is positive; an elliptic contact's rows take its zone in
/// the regular cone.
fn dual_state(data: &Data, state: &mut [ConstraintState]) -> usize {
    let (ne, nf, nefc) = (data.ne, data.nf, data.efc_type.len());
    let force = &data.efc_force;
    let mut nactive = ne + nf;
    state[..ne].fill(ConstraintState::Quadratic);
    for i in ne..ne + nf {
        let floss = data.efc_floss[i];
        state[i] = if force[i] <= -floss {
            ConstraintState::LinearPos
        } else if force[i] >= floss {
            ConstraintState::LinearNeg
        } else {
            ConstraintState::Quadratic
        };
    }
    let mut i = ne + nf;
    while i < nefc {
        if data.efc_type[i] != ConstraintType::ContactElliptic {
            if force[i] <= 0.0 {
                state[i] = ConstraintState::Satisfied;
            } else {
                state[i] = ConstraintState::Quadratic;
                nactive += 1;
            }
            i += 1;
            continue;
        }
        // μ here is friction[0]; MuJoCo's `con->mu` is friction[0]·√(R[i+1]/R[i])
        // (`engine_core_constraint.c:1540`), as in `constraint_update` (book P38).
        let dim = data.efc_dim[i];
        let friction = &data.efc_mu[i];
        let mu = friction[0];
        let n = force[i] / mu;
        let t = (1..dim)
            .map(|j| (force[i + j] / friction[j - 1]).powi(2))
            .sum::<f64>()
            .sqrt();
        let zone = if mu * n >= t {
            ConstraintState::Satisfied
        } else if n + mu * t <= 0.0 {
            nactive += dim;
            ConstraintState::Quadratic
        } else {
            nactive += dim;
            ConstraintState::Cone
        };
        state[i..i + dim].fill(zone);
        i += dim;
    }
    nactive
}

#[cfg(test)]
#[allow(clippy::float_cmp)]
mod tests {
    use super::cost_change;

    /// A move that lowers the dual cost is kept and counts its change, the
    /// block's cross terms included; one that raises it by more than 1e-10
    /// is undone and counts as none (MuJoCo's `costChange`). Exact in binary.
    #[test]
    fn cost_change_keeps_a_fall_and_undoes_a_rise() {
        let a = [2.0, 0.5, 0.5, 1.0];
        let (old, res) = ([0.0, 0.0], [-1.0, 0.0]);
        // δ = (0.5, −0.5): ½·δᵀAδ = ½·0.5, δ·res = −0.5 (the diagonal alone
        // would give ½·0.75).
        let mut force = [0.5, -0.5];
        assert_eq!(cost_change(&a, &mut force, &old, &res), -0.25);
        assert_eq!(force, [0.5, -0.5]);
        // δ = (−0.5, 0): ½·0.5 + 0.5, a rise.
        let mut force = [-0.5, 0.0];
        assert_eq!(cost_change(&a, &mut force, &old, &res), 0.0);
        assert_eq!(force, old);
        // With no residual, δ = (d, 0) changes the cost by d²: a rise of 2⁻³⁴
        // (under 1e-10) is kept and counted, one of 2⁻³² (over it) undone.
        let still = [0.0, 0.0];
        let small = 2f64.powi(-17);
        let mut force = [small, 0.0];
        assert_eq!(cost_change(&a, &mut force, &old, &still), 2f64.powi(-34));
        assert_eq!(force, [small, 0.0]);
        let mut force = [2.0 * small, 0.0];
        assert_eq!(cost_change(&a, &mut force, &old, &still), 0.0);
        assert_eq!(force, old);
    }
}
