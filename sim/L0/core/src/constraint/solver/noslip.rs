//! Noslip post-processor: suppresses residual contact slip (§15.10).
//!
//! Runs a modified PGS pass on friction force rows only, without
//! regularization, using the current normal forces as fixed cone limits.
//! This matches MuJoCo's `mj_solNoSlip`.
//!
//! Also contains the elliptic cone projector used by both PGS and noslip.

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::mj_solve_sparse;
use crate::types::{ConstraintType, Data, Model};

/// Project contact forces onto the elliptic friction cone (§32.4).
///
/// Enforces the dual cone constraint: normal force ≥ 0 and weighted friction
/// norm ≤ normal force. Used by both PGS (per-contact group projection) and
/// noslip (implicit through QCQP).
///
/// Cone shape: ||(λ₁/μ₁, λ₂/μ₂, ...)|| ≤ λ_n
pub fn project_elliptic_cone(lambda: &mut [f64], mu: &[f64; 5], dim: usize) {
    // Step 1: Enforce unilateral constraint (normal force must be non-negative)
    // Negative normal force = separating contact = release completely
    if lambda[0] < 0.0 {
        for l in lambda.iter_mut().take(dim) {
            *l = 0.0;
        }
        return;
    }

    // Step 2: Clamp friction components where mu ≈ 0 (infinite resistance = no sliding)
    for i in 1..dim {
        if mu[i - 1] <= 1e-12 {
            lambda[i] = 0.0;
        }
    }

    // Step 3: Compute weighted friction norm (elliptic cone radius)
    // s = sqrt( Σ (λ_i / μ_i)² ) for i = 1..dim-1
    let mut s_sq = 0.0;
    for i in 1..dim {
        if mu[i - 1] > 1e-12 {
            s_sq += (lambda[i] / mu[i - 1]).powi(2);
        }
    }
    let s = s_sq.sqrt();

    // Step 4: If friction exceeds cone boundary, scale to boundary
    // Cone constraint: s ≤ λ_n, i.e., ||(λ_i/μ_i)|| ≤ λ_n
    if s > lambda[0] && s > 1e-10 {
        let scale = lambda[0] / s;
        for l in lambda.iter_mut().take(dim).skip(1) {
            *l *= scale;
        }
    }
}

/// Noslip QCQP for 2 friction DOFs (condim=3): solve the constrained tangential subproblem.
///
/// Minimizes  `½(f−f_unc)ᵀ·A·(f−f_unc)` subject to `Σ(f_j/μ_j)² ≤ f_n²`
/// where `f_unc` is the unconstrained GS solution.
///
/// This matches MuJoCo's `mju_QCQP2`:
/// - Scale to unit-friction space: `y_j = f_j / μ_j`
/// - If unconstrained solution is inside cone → return it
/// - Otherwise Newton iteration on dual Lagrange multiplier λ
///
/// `a` is the 2×2 tangential Delassus subblock (unregularized).
/// `f_unc` is the unconstrained GS update for the 2 friction DOFs.
/// `mu` is the friction coefficients for these 2 DOFs.
/// `fn_abs` is the absolute normal force (cone radius).
///
/// Returns the projected (f[0], f[1]).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
fn noslip_qcqp2(a: [[f64; 2]; 2], f_unc: [f64; 2], mu: [f64; 2], fn_abs: f64) -> [f64; 2] {
    // Scale to unit-friction space: y = f / mu, A_s = D·A·D, b_s stays in y-space
    // The unconstrained solution in y-space: y_unc = f_unc / mu
    let y_unc = [
        f_unc[0] / mu[0].max(MJ_MINVAL),
        f_unc[1] / mu[1].max(MJ_MINVAL),
    ];

    // Check if unconstrained solution is inside cone: ||y|| ≤ fn
    let r2 = fn_abs * fn_abs;
    let norm2 = y_unc[0] * y_unc[0] + y_unc[1] * y_unc[1];
    if norm2 <= r2 {
        return f_unc;
    }

    // Need to project: solve (A_s + λI)·y = A_s·y_unc with ||y||² = r²
    // Scale Delassus: A_s[i,j] = mu[i] * A[i,j] * mu[j]
    let a_s = [
        [a[0][0] * mu[0] * mu[0], a[0][1] * mu[0] * mu[1]],
        [a[1][0] * mu[1] * mu[0], a[1][1] * mu[1] * mu[1]],
    ];

    // RHS in y-space: g = A_s · y_unc
    let g = [
        a_s[0][0] * y_unc[0] + a_s[0][1] * y_unc[1],
        a_s[1][0] * y_unc[0] + a_s[1][1] * y_unc[1],
    ];

    // Newton on λ: φ(λ) = ||y(λ)||² − r² = 0
    // y(λ) = (A_s + λI)⁻¹ · g
    let mut lam = 0.0_f64;
    for _ in 0..20 {
        // (A_s + λI) for 2×2
        let m00 = a_s[0][0] + lam;
        let m11 = a_s[1][1] + lam;
        let m01 = a_s[0][1];
        let det = m00 * m11 - m01 * m01;
        if det.abs() < MJ_MINVAL {
            break;
        }
        let inv_det = 1.0 / det;

        // y = M⁻¹ · g
        let y0 = (m11 * g[0] - m01 * g[1]) * inv_det;
        let y1 = (-m01 * g[0] + m00 * g[1]) * inv_det;

        let phi = y0 * y0 + y1 * y1 - r2;
        if phi.abs() < 1e-10 {
            // Converged — unscale and return
            return [y0 * mu[0], y1 * mu[1]];
        }

        // φ'(λ) = -2 · yᵀ · M⁻¹ · y
        let my0 = (m11 * y0 - m01 * y1) * inv_det;
        let my1 = (-m01 * y0 + m00 * y1) * inv_det;
        let dphi = -2.0 * (y0 * my0 + y1 * my1);

        if dphi.abs() < MJ_MINVAL {
            break;
        }
        lam -= phi / dphi;
        lam = lam.max(0.0); // λ ≥ 0 (dual feasibility)
    }

    // Final solve with converged λ
    let m00 = a_s[0][0] + lam;
    let m11 = a_s[1][1] + lam;
    let m01 = a_s[0][1];
    let det = m00 * m11 - m01 * m01;
    if det.abs() < MJ_MINVAL {
        // Degenerate: simple rescaling fallback
        let s = norm2.sqrt();
        if s > MJ_MINVAL {
            let scale = fn_abs / s;
            return [f_unc[0] * scale, f_unc[1] * scale];
        }
        return [0.0, 0.0];
    }
    let inv_det = 1.0 / det;
    let y0 = (m11 * g[0] - m01 * g[1]) * inv_det;
    let y1 = (-m01 * g[0] + m00 * g[1]) * inv_det;

    // Exact rescale to cone boundary for numerical safety
    let yn2 = y0 * y0 + y1 * y1;
    if yn2 > r2 && yn2 > MJ_MINVAL {
        let s = fn_abs / yn2.sqrt();
        [y0 * mu[0] * s, y1 * mu[1] * s]
    } else {
        [y0 * mu[0], y1 * mu[1]]
    }
}

/// Noslip QCQP for 3 friction DOFs (condim=4): solve the constrained tangential subproblem.
///
/// Same algorithm as `noslip_qcqp2` but for 3×3 system. Uses cofactor inverse.
#[allow(clippy::many_single_char_names)]
fn noslip_qcqp3(a: [[f64; 3]; 3], f_unc: [f64; 3], mu: [f64; 3], fn_abs: f64) -> [f64; 3] {
    let y_unc = [
        f_unc[0] / mu[0].max(MJ_MINVAL),
        f_unc[1] / mu[1].max(MJ_MINVAL),
        f_unc[2] / mu[2].max(MJ_MINVAL),
    ];

    let r2 = fn_abs * fn_abs;
    let norm2 = y_unc[0] * y_unc[0] + y_unc[1] * y_unc[1] + y_unc[2] * y_unc[2];
    if norm2 <= r2 {
        return f_unc;
    }

    // Scale Delassus: A_s[i,j] = mu[i] * A[i,j] * mu[j]
    let mut a_s = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            a_s[i][j] = a[i][j] * mu[i] * mu[j];
        }
    }

    // g = A_s · y_unc
    let mut g = [0.0_f64; 3];
    for i in 0..3 {
        for j in 0..3 {
            g[i] += a_s[i][j] * y_unc[j];
        }
    }

    // Newton on λ
    let mut lam = 0.0_f64;
    let mut y = [0.0_f64; 3];
    for _ in 0..20 {
        // M = A_s + λI
        let mut m = a_s;
        m[0][0] += lam;
        m[1][1] += lam;
        m[2][2] += lam;

        // 3×3 cofactor inverse
        let cof00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let cof01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
        let cof02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        let det = m[0][0] * cof00 + m[0][1] * cof01 + m[0][2] * cof02;
        if det.abs() < MJ_MINVAL {
            break;
        }
        let inv_det = 1.0 / det;

        let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
        let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
        let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
        let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];

        // y = M⁻¹ · g (cofactor inverse is transposed)
        y[0] = (cof00 * g[0] + cof10 * g[1] + cof20 * g[2]) * inv_det;
        y[1] = (cof01 * g[0] + cof11 * g[1] + cof21 * g[2]) * inv_det;
        y[2] = (cof02 * g[0] + cof12 * g[1] + cof22 * g[2]) * inv_det;

        let phi = y[0] * y[0] + y[1] * y[1] + y[2] * y[2] - r2;
        if phi.abs() < 1e-10 {
            return [y[0] * mu[0], y[1] * mu[1], y[2] * mu[2]];
        }

        // φ'(λ) = -2 · yᵀ · M⁻¹ · y
        let my0 = (cof00 * y[0] + cof10 * y[1] + cof20 * y[2]) * inv_det;
        let my1 = (cof01 * y[0] + cof11 * y[1] + cof21 * y[2]) * inv_det;
        let my2 = (cof02 * y[0] + cof12 * y[1] + cof22 * y[2]) * inv_det;
        let dphi = -2.0 * (y[0] * my0 + y[1] * my1 + y[2] * my2);

        if dphi.abs() < MJ_MINVAL {
            break;
        }
        lam -= phi / dphi;
        lam = lam.max(0.0);
    }

    // Final solve
    let mut m = a_s;
    m[0][0] += lam;
    m[1][1] += lam;
    m[2][2] += lam;
    let cof00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    let cof01 = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]);
    let cof02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    let det = m[0][0] * cof00 + m[0][1] * cof01 + m[0][2] * cof02;
    if det.abs() < MJ_MINVAL {
        let s = norm2.sqrt();
        if s > MJ_MINVAL {
            let scale = fn_abs / s;
            return [f_unc[0] * scale, f_unc[1] * scale, f_unc[2] * scale];
        }
        return [0.0, 0.0, 0.0];
    }
    let inv_det = 1.0 / det;
    let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
    let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
    let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
    let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
    let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
    let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    y[0] = (cof00 * g[0] + cof10 * g[1] + cof20 * g[2]) * inv_det;
    y[1] = (cof01 * g[0] + cof11 * g[1] + cof21 * g[2]) * inv_det;
    y[2] = (cof02 * g[0] + cof12 * g[1] + cof22 * g[2]) * inv_det;

    let yn2 = y[0] * y[0] + y[1] * y[1] + y[2] * y[2];
    if yn2 > r2 && yn2 > MJ_MINVAL {
        let s = fn_abs / yn2.sqrt();
        [y[0] * mu[0] * s, y[1] * mu[1] * s, y[2] * mu[2] * s]
    } else {
        [y[0] * mu[0], y[1] * mu[1], y[2] * mu[2]]
    }
}

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
    for _iter in 0..noslip_iter {
        let mut improvement = 0.0_f64;

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

                        // Save old forces for cost tracking
                        let old_forces: Vec<f64> =
                            (0..group_len).map(|j| f[group_start + j]).collect();

                        // Compute residuals for all friction rows in this group
                        let mut residuals: Vec<f64> = Vec::with_capacity(group_len);
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            let mut res = b_eff[idx];
                            for k in 0..n {
                                res += a_sub[(idx, k)] * f[k];
                            }
                            residuals.push(res);
                        }

                        // Unconstrained GS update for all friction rows
                        let mut f_unc: Vec<f64> = Vec::with_capacity(group_len);
                        for local_j in 0..group_len {
                            let idx = group_start + local_j;
                            f_unc.push(f[idx] - residuals[local_j] * diag_inv[idx]);
                        }

                        // QCQP projection based on dimension
                        if fn_abs < MJ_MINVAL {
                            // Zero normal force: zero all friction
                            for local_j in 0..group_len {
                                f[group_start + local_j] = 0.0;
                            }
                        } else if group_len == 2 {
                            // condim=3: 2 friction DOFs — use QCQP2
                            let a_block = [
                                [
                                    a_sub[(group_start, group_start)],
                                    a_sub[(group_start, group_start + 1)],
                                ],
                                [
                                    a_sub[(group_start + 1, group_start)],
                                    a_sub[(group_start + 1, group_start + 1)],
                                ],
                            ];
                            let result =
                                noslip_qcqp2(a_block, [f_unc[0], f_unc[1]], [mu[0], mu[1]], fn_abs);
                            f[group_start] = result[0];
                            f[group_start + 1] = result[1];
                        } else if group_len == 3 {
                            // condim=4: 3 friction DOFs — use QCQP3
                            let a_block = [
                                [
                                    a_sub[(group_start, group_start)],
                                    a_sub[(group_start, group_start + 1)],
                                    a_sub[(group_start, group_start + 2)],
                                ],
                                [
                                    a_sub[(group_start + 1, group_start)],
                                    a_sub[(group_start + 1, group_start + 1)],
                                    a_sub[(group_start + 1, group_start + 2)],
                                ],
                                [
                                    a_sub[(group_start + 2, group_start)],
                                    a_sub[(group_start + 2, group_start + 1)],
                                    a_sub[(group_start + 2, group_start + 2)],
                                ],
                            ];
                            let result = noslip_qcqp3(
                                a_block,
                                [f_unc[0], f_unc[1], f_unc[2]],
                                [mu[0], mu[1], mu[2]],
                                fn_abs,
                            );
                            f[group_start] = result[0];
                            f[group_start + 1] = result[1];
                            f[group_start + 2] = result[2];
                        } else {
                            // condim=6 or higher: simple rescaling fallback
                            f[group_start..group_start + group_len]
                                .copy_from_slice(&f_unc[..group_len]);
                            let mut s_sq = 0.0;
                            for local_j in 0..group_len {
                                let mu_j = mu[local_j];
                                if mu_j > MJ_MINVAL {
                                    s_sq += (f[group_start + local_j] / mu_j).powi(2);
                                }
                            }
                            let s = s_sq.sqrt();
                            if s > fn_abs && s > MJ_MINVAL {
                                let rescale = fn_abs / s;
                                for local_j in 0..group_len {
                                    f[group_start + local_j] *= rescale;
                                }
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
                            if cost > MJ_MINVAL {
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
