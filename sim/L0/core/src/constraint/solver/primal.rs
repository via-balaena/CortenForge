//! Shared CG/Newton primal infrastructure.
//!
//! Contains gradient+search computation, precomputed quadratic line-search
//! structures (`PrimalQuad`, `PrimalPoint`), and the three-phase exact Newton
//! line search (`primal_prepare`, `primal_eval`, `primal_search`).
//! Also provides `evaluate_cost_at` for warmstart comparison.
//!
//! Corresponds to the shared primal routines in MuJoCo's `engine_solver.c`
//! (§15.4–15.5).

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::linalg::cholesky_solve_in_place;
use crate::mujoco_pipeline::SparseHessian;
use crate::types::{ConstraintType, Data, Model};

/// Compute gradient and preconditioned search direction (PrimalUpdateGradient, §15.4).
///
/// Returns (grad, search) where:
/// - `qfrc_constraint = J^T · efc_force`
/// - `grad = Ma - qfrc_smooth - qfrc_constraint`
/// - `search = -H⁻¹ · grad` (via Cholesky solve)
pub fn compute_gradient_and_search(
    data: &Data,
    nv: usize,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    chol_l: &DMatrix<f64>,
) -> (DVector<f64>, DVector<f64>) {
    let nefc = data.efc_type.len();

    // qfrc_constraint = J^T · efc_force
    let mut qfrc_constraint = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            qfrc_constraint[col] += data.efc_J[(i, col)] * f;
        }
    }

    // grad = Ma - qfrc_smooth - qfrc_constraint
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint[k];
    }

    // search = -H⁻¹ · grad = -solve(L, grad)
    let mut search = grad.clone();
    cholesky_solve_in_place(chol_l, &mut search);
    for k in 0..nv {
        search[k] = -search[k];
    }

    (grad, search)
}

/// Sparse variant of `compute_gradient_and_search` using `SparseHessian::solve`.
pub fn compute_gradient_and_search_sparse(
    data: &Data,
    nv: usize,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    sparse_h: &SparseHessian,
) -> (DVector<f64>, DVector<f64>) {
    let nefc = data.efc_type.len();

    // qfrc_constraint = J^T · efc_force
    let mut qfrc_constraint = DVector::<f64>::zeros(nv);
    for i in 0..nefc {
        let f = data.efc_force[i];
        if f == 0.0 {
            continue;
        }
        for col in 0..nv {
            qfrc_constraint[col] += data.efc_J[(i, col)] * f;
        }
    }

    // grad = Ma - qfrc_smooth - qfrc_constraint
    let mut grad = DVector::<f64>::zeros(nv);
    for k in 0..nv {
        grad[k] = ma[k] - qfrc_smooth[k] - qfrc_constraint[k];
    }

    // search = -H⁻¹ · grad via sparse LDL^T solve
    let mut search = grad.clone();
    sparse_h.solve(&mut search);
    for k in 0..nv {
        search[k] = -search[k];
    }

    (grad, search)
}

// ==========================================================================
// Phase B: Exact 1D Newton line search infrastructure (§15.5)
// ==========================================================================

/// Precomputed quadratic polynomials for fast 1D cost evaluation along search direction.
///
/// For constraint i: cost(alpha) depends on `jar_trial = jar + alpha * jv`
/// For the Gauss regularisation: cost is exactly quadratic in alpha.
pub struct PrimalQuad {
    /// Per-constraint quadratic coefficients.
    /// For Quadratic/Equality rows:   [0.5*D*jar², D*jar*jv, 0.5*D*jv²]
    /// For LinearPos:                  [+floss*jar - 0.5*R*floss², +floss*jv, 0]
    /// For LinearNeg:                  [-floss*jar - 0.5*R*floss², -floss*jv, 0]
    /// For Satisfied:                  [0, 0, 0]
    /// For Contact rows: use raw values, re-classify per alpha in primal_eval.
    pub quad: Vec<[f64; 3]>,

    /// Per-elliptic-cone extra coefficients.
    /// Each entry: [U0, V0, UU, UV, VV, Dm] for the rescaled cone decomposition.
    /// Indexed by sequential cone index (not efc row).
    pub cone_quad: Vec<[f64; 6]>,

    /// Mapping from sequential cone index to the starting efc row and dim.
    pub cone_map: Vec<(usize, usize)>,

    /// Gauss term quadratic: [constant, linear, quadratic].
    /// cost_gauss(alpha) = quad_gauss[0] + alpha*quad_gauss[1] + alpha²*quad_gauss[2]
    pub quad_gauss: [f64; 3],

    /// Norm of the search direction ||search||.
    pub snorm: f64,
}

/// A point on the 1D cost curve, evaluated at step size `alpha`.
#[derive(Debug, Clone, Copy)]
pub struct PrimalPoint {
    /// Step size.
    pub alpha: f64,
    /// [dcost/dalpha, d²cost/dalpha²]
    pub deriv: [f64; 2],
    /// Total cost at this alpha (used for diagnostics / future cost monitoring).
    #[allow(dead_code)]
    pub cost: f64,
}

/// Precompute quadratic polynomial coefficients for the 1D line search (§15.5).
///
/// Called once per Newton iteration before entering the line search. The
/// precomputed coefficients allow O(nefc) cost+derivative evaluation at each
/// trial alpha without any matrix-vector products.
#[allow(clippy::many_single_char_names, clippy::too_many_arguments)]
pub fn primal_prepare(
    data: &Data,
    _nv: usize,
    qacc: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    ma: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    search: &DVector<f64>,
    mv: &DVector<f64>,
    jv: &DVector<f64>,
) -> PrimalQuad {
    let nefc = data.efc_type.len();
    let mut quad = vec![[0.0_f64; 3]; nefc];
    let mut cone_quad = Vec::new();
    let mut cone_map = Vec::new();

    let mut i = 0;
    while i < nefc {
        let jar_i = data.efc_jar[i];
        let d_i = data.efc_D[i];
        let jv_i = jv[i];

        if data.efc_type[i] == ConstraintType::ContactElliptic {
            let dim = data.efc_dim[i];
            let mu = data.efc_mu[i][0];

            // Per-row quadratic (for bottom zone / per-component evaluation)
            for j in 0..dim {
                let jar_j = data.efc_jar[i + j];
                let d_j = data.efc_D[i + j];
                let jv_j = jv[i + j];
                quad[i + j] = [
                    0.5 * d_j * jar_j * jar_j,
                    d_j * jar_j * jv_j,
                    0.5 * d_j * jv_j * jv_j,
                ];
            }

            // Cone-specific: rescale to circular cone space
            let u0 = jar_i * mu;
            let v0 = jv_i * mu;
            let mut uu = 0.0;
            let mut uv = 0.0;
            let mut vv = 0.0;

            for j in 1..dim {
                let friction_j = data.efc_mu[i][j - 1];
                let uj = data.efc_jar[i + j] * friction_j;
                let vj = jv[i + j] * friction_j;
                uu += uj * uj;
                uv += uj * vj;
                vv += vj * vj;
            }

            let dm = d_i / (mu * mu * (1.0 + mu * mu));
            cone_quad.push([u0, v0, uu, uv, vv, dm]);
            cone_map.push((i, dim));

            i += dim;
        } else {
            // Scalar constraint: store raw D·jar quadratic
            quad[i] = [
                0.5 * d_i * jar_i * jar_i,
                d_i * jar_i * jv_i,
                0.5 * d_i * jv_i * jv_i,
            ];
            i += 1;
        }
    }

    // Gauss quadratic: cost(alpha) = G0 + alpha*G1 + alpha²*G2
    // G0 = 0.5 * (Ma - qfrc_smooth) · (qacc - qacc_smooth)  [already in data.efc_cost - constraint_cost]
    // G1 = search · Ma - qfrc_smooth · search = search · (Ma - qfrc_smooth)
    // G2 = 0.5 * search · Mv
    let nv = search.len();
    let mut g1 = 0.0;
    let mut g2 = 0.0;
    for k in 0..nv {
        g1 += search[k] * (ma[k] - qfrc_smooth[k]);
        g2 += search[k] * mv[k];
    }
    g2 *= 0.5;

    // G0 = 0.5 * (Ma - qfrc_smooth) · (qacc - qacc_smooth)
    // Exact since Gauss cost is purely quadratic in qacc (linear in alpha).
    let mut g0 = 0.0;
    for k in 0..nv {
        g0 += (ma[k] - qfrc_smooth[k]) * (qacc[k] - qacc_smooth[k]);
    }
    g0 *= 0.5;

    let snorm: f64 = search.iter().map(|x| x * x).sum::<f64>().sqrt();

    PrimalQuad {
        quad,
        cone_quad,
        cone_map,
        quad_gauss: [g0, g1, g2],
        snorm,
    }
}

/// Evaluate cost, first derivative, and second derivative at alpha along the search direction.
///
/// Uses precomputed `PrimalQuad` coefficients for O(nefc) evaluation without
/// matrix-vector products. This is the inner loop of `PrimalSearch`.
#[allow(clippy::many_single_char_names)]
pub fn primal_eval(data: &Data, pq: &PrimalQuad, jv: &[f64], alpha: f64) -> PrimalPoint {
    let nefc = data.efc_type.len();
    let mut cost = 0.0;
    let mut deriv = [0.0_f64; 2];

    // Batch quadratic accumulator for simple constraints and bottom-zone cones
    let mut quad_total = [0.0_f64; 3];

    let mut cone_idx = 0;
    let mut i = 0;
    while i < nefc {
        match data.efc_type[i] {
            ConstraintType::Equality | ConstraintType::FlexEdge => {
                // Always quadratic
                quad_total[0] += pq.quad[i][0];
                quad_total[1] += pq.quad[i][1];
                quad_total[2] += pq.quad[i][2];
                i += 1;
            }

            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                let r_i = data.efc_R[i];
                let threshold = r_i * floss;
                // Trial jar at alpha
                let jar_trial = data.efc_jar[i] + alpha * jv[i];

                if jar_trial >= threshold {
                    // LinearPos
                    cost += floss * jar_trial - 0.5 * r_i * floss * floss;
                    deriv[0] += floss * jv[i];
                    // deriv[1] += 0 (linear)
                } else if jar_trial <= -threshold {
                    // LinearNeg
                    cost += -floss * jar_trial - 0.5 * r_i * floss * floss;
                    deriv[0] += -floss * jv[i];
                    // deriv[1] += 0
                } else {
                    // Quadratic
                    quad_total[0] += pq.quad[i][0];
                    quad_total[1] += pq.quad[i][1];
                    quad_total[2] += pq.quad[i][2];
                }
                i += 1;
            }

            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
                // Re-classify at trial alpha
                let jar_trial = data.efc_jar[i] + alpha * jv[i];
                if jar_trial < 0.0 {
                    // Quadratic (active)
                    quad_total[0] += pq.quad[i][0];
                    quad_total[1] += pq.quad[i][1];
                    quad_total[2] += pq.quad[i][2];
                }
                // else: Satisfied — zero cost
                i += 1;
            }

            ConstraintType::ContactElliptic => {
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0];
                let cq = &pq.cone_quad[cone_idx];
                let (_, _) = pq.cone_map[cone_idx];
                cone_idx += 1;

                // Compute N(alpha) and T(alpha) in rescaled cone space
                let n_alpha = cq[0] + alpha * cq[1]; // U0 + alpha*V0
                let t_sq = (cq[2] + alpha * (2.0 * cq[3] + alpha * cq[4])).max(0.0); // UU + 2*alpha*UV + alpha²*VV
                let t_alpha = t_sq.sqrt();
                let dm = cq[5];

                let t_min = MJ_MINVAL;

                if n_alpha >= mu * t_alpha || (t_alpha < t_min && n_alpha >= 0.0) {
                    // Top zone: satisfied, zero cost
                } else if mu * n_alpha + t_alpha <= 0.0 || (t_alpha < t_min && n_alpha < 0.0) {
                    // Bottom zone: per-row quadratic
                    for j in 0..dim {
                        quad_total[0] += pq.quad[i + j][0];
                        quad_total[1] += pq.quad[i + j][1];
                        quad_total[2] += pq.quad[i + j][2];
                    }
                } else {
                    // Middle zone: cone cost
                    let nmt = n_alpha - mu * t_alpha;
                    cost += 0.5 * dm * nmt * nmt;

                    // First derivative: d(cost)/d(alpha)
                    // d(N)/d(alpha) = V0
                    // d(T)/d(alpha) = (UV + alpha*VV) / T (when T > 0)
                    let n1 = cq[1]; // V0
                    let t1 = if t_alpha > t_min {
                        (cq[3] + alpha * cq[4]) / t_alpha
                    } else {
                        0.0
                    };
                    let nmt1 = n1 - mu * t1; // d(N-mu*T)/d(alpha)
                    deriv[0] += dm * nmt * nmt1;

                    // Second derivative: d²(cost)/d(alpha)²
                    // d²(T)/d(alpha)² = (VV - t1²) / T (when T > 0)
                    let t2 = if t_alpha > t_min {
                        (cq[4] - t1 * t1) / t_alpha
                    } else {
                        0.0
                    };
                    deriv[1] += dm * (nmt1 * nmt1 + nmt * (-mu * t2));
                }

                i += dim;
            }
        }
    }

    // Evaluate batched quadratic total
    let q_cost = quad_total[0] + alpha * quad_total[1] + alpha * alpha * quad_total[2];
    let q_d1 = quad_total[1] + 2.0 * alpha * quad_total[2];
    let q_d2 = 2.0 * quad_total[2];
    cost += q_cost;
    deriv[0] += q_d1;
    deriv[1] += q_d2;

    // Add Gauss term (exactly quadratic in alpha)
    cost += pq.quad_gauss[0] + alpha * pq.quad_gauss[1] + alpha * alpha * pq.quad_gauss[2];
    deriv[0] += pq.quad_gauss[1] + 2.0 * alpha * pq.quad_gauss[2];
    deriv[1] += 2.0 * pq.quad_gauss[2];

    PrimalPoint { alpha, deriv, cost }
}

/// Three-phase exact Newton line search (§15.5 PrimalSearch).
///
/// Finds the step size `alpha` that minimizes the 1D cost along the search
/// direction. Uses the precomputed `PrimalQuad` from `primal_prepare` and
/// evaluates via `primal_eval`.
///
/// # Phases
/// 1. **Initial Newton step** from alpha=0. If derivative is small enough, done.
/// 2. **Bracket search**: step in the direction of the initial derivative until
///    the derivative sign changes, establishing a bracket [lo, hi].
/// 3. **Bracketed refinement**: three candidates (Newton from lo, Newton from hi,
///    midpoint). Pick the best converged candidate, or tighten the bracket.
///
/// Returns the optimal alpha, or 0.0 if no improvement is possible.
#[allow(clippy::many_single_char_names)]
pub fn primal_search(
    data: &Data,
    pq: &PrimalQuad,
    jv: &[f64],
    tolerance: f64,
    ls_tolerance: f64,
    max_ls_iter: usize,
    scale: f64,
) -> (f64, usize) {
    let mut neval: usize = 0;

    // Gradient tolerance for convergence
    let gtol = tolerance * ls_tolerance * pq.snorm / scale.max(MJ_MINVAL);
    if gtol <= 0.0 || pq.snorm < MJ_MINVAL {
        return (0.0, neval);
    }

    // Phase 1: Initial Newton step from alpha=0
    let p0 = primal_eval(data, pq, jv, 0.0);
    neval += 1;

    // Check if gradient at 0 is already small (unconstrained minimum at 0)
    if p0.deriv[0].abs() < gtol {
        return (0.0, neval);
    }

    // Newton step: alpha1 = -f'(0) / f''(0)
    let alpha1 = if p0.deriv[1] > MJ_MINVAL {
        -p0.deriv[0] / p0.deriv[1]
    } else {
        // Hessian too small; take unit step in descent direction
        if p0.deriv[0] < 0.0 { 1.0 } else { -1.0 }
    };

    // Guard: step must be positive (descent direction)
    if alpha1 <= 0.0 {
        return (0.0, neval);
    }

    let p1 = primal_eval(data, pq, jv, alpha1);
    neval += 1;

    // Check convergence at p1
    if p1.deriv[0].abs() < gtol {
        return (alpha1, neval);
    }

    // Phase 2: One-sided bracket search
    // Determine search direction based on sign of p1's derivative
    let mut lo;
    let mut hi;
    if p1.deriv[0] < 0.0 {
        // Derivative still negative — minimum is to the right of p1
        lo = p1;
        hi = p1; // will be updated
        let mut step = alpha1; // step size doubles each iteration
        for _ in 0..max_ls_iter {
            step *= 2.0;
            let trial = lo.alpha + step;
            let pt = primal_eval(data, pq, jv, trial);
            neval += 1;
            if pt.deriv[0].abs() < gtol {
                return (pt.alpha, neval);
            }
            if pt.deriv[0] > 0.0 {
                // Derivative changed sign — bracket found: [lo, pt]
                hi = pt;
                break;
            }
            // Still negative — continue stepping right
            lo = pt;
            step = pt.alpha - lo.alpha; // geometric increase
        }
        // If hi was never updated (loop exhausted), return best we have
        if hi.alpha <= lo.alpha {
            return (lo.alpha, neval);
        }
    } else {
        // Derivative positive at p1 — minimum is between 0 and p1
        lo = p0;
        hi = p1;
    }

    // Phase 3: Bracketed refinement
    for _ in 0..max_ls_iter {
        // Bracket width check
        let width = hi.alpha - lo.alpha;
        if width < MJ_MINVAL {
            break;
        }

        // Three candidates:
        // 1. Newton step from lo
        let c1 = if lo.deriv[1] > MJ_MINVAL {
            let a = lo.alpha - lo.deriv[0] / lo.deriv[1];
            if a > lo.alpha && a < hi.alpha {
                Some(a)
            } else {
                None
            }
        } else {
            None
        };

        // 2. Newton step from hi
        let c2 = if hi.deriv[1] > MJ_MINVAL {
            let a = hi.alpha - hi.deriv[0] / hi.deriv[1];
            if a > lo.alpha && a < hi.alpha {
                Some(a)
            } else {
                None
            }
        } else {
            None
        };

        // 3. Midpoint
        let mid = 0.5 * (lo.alpha + hi.alpha);

        // Evaluate all candidates, pick the best
        let mut best = None;
        let mut best_deriv_abs = f64::MAX;

        for alpha in [c1, c2, Some(mid)].into_iter().flatten() {
            let pt = primal_eval(data, pq, jv, alpha);
            neval += 1;
            if pt.deriv[0].abs() < gtol {
                return (pt.alpha, neval);
            }
            if pt.deriv[0].abs() < best_deriv_abs {
                best_deriv_abs = pt.deriv[0].abs();
                best = Some(pt);
            }
        }

        // Tighten the bracket using the best candidate
        if let Some(pt) = best {
            if pt.deriv[0] < 0.0 {
                lo = pt;
            } else {
                hi = pt;
            }
        } else {
            break;
        }
    }

    // Return the endpoint with smaller |derivative|
    let alpha = if lo.deriv[0].abs() < hi.deriv[0].abs() {
        lo.alpha
    } else {
        hi.alpha
    };
    (alpha, neval)
}

/// Evaluate total cost (Gauss + constraint) at a trial acceleration.
///
/// Does not modify any data fields — purely evaluative.
/// Used by warmstart comparison in `newton_solve`.
/// DT-35: `m_eff` is `M_impl` when `ImplicitSpringDamper` is active.
#[allow(clippy::many_single_char_names)]
pub fn evaluate_cost_at(
    data: &Data,
    model: &Model,
    qacc_trial: &DVector<f64>,
    qacc_smooth: &DVector<f64>,
    qfrc_smooth: &DVector<f64>,
    m_eff: &DMatrix<f64>,
) -> f64 {
    let nv = model.nv;
    let nefc = data.efc_type.len();

    // Compute jar_trial = J · qacc_trial - aref
    let mut jar_trial = DVector::<f64>::zeros(nefc);
    for i in 0..nefc {
        let mut j_dot_qacc = 0.0;
        for col in 0..nv {
            j_dot_qacc += data.efc_J[(i, col)] * qacc_trial[col];
        }
        jar_trial[i] = j_dot_qacc - data.efc_aref[i];
    }

    // Compute Ma_trial = M_eff · qacc_trial
    let mut ma_trial = DVector::<f64>::zeros(nv);
    for r in 0..nv {
        for c in 0..nv {
            ma_trial[r] += m_eff[(r, c)] * qacc_trial[c];
        }
    }

    // Gauss term: ½·(Ma_trial - qfrc_smooth)·(qacc_trial - qacc_smooth)
    let mut cost_gauss = 0.0;
    for k in 0..nv {
        cost_gauss += (ma_trial[k] - qfrc_smooth[k]) * (qacc_trial[k] - qacc_smooth[k]);
    }
    cost_gauss *= 0.5;

    // Constraint cost
    let mut constraint_cost = 0.0;
    let mut i = 0;
    while i < nefc {
        let ctype = data.efc_type[i];
        let jar = jar_trial[i];
        let d = data.efc_D[i];
        let r = data.efc_R[i];

        match ctype {
            ConstraintType::Equality | ConstraintType::FlexEdge => {
                constraint_cost += 0.5 * d * jar * jar;
                i += 1;
            }
            ConstraintType::FrictionLoss => {
                let floss = data.efc_floss[i];
                let threshold = r * floss;
                if jar >= threshold {
                    constraint_cost += floss * jar - 0.5 * r * floss * floss;
                } else if jar <= -threshold {
                    constraint_cost += -floss * jar - 0.5 * r * floss * floss;
                } else {
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }
            ConstraintType::LimitJoint
            | ConstraintType::LimitTendon
            | ConstraintType::ContactFrictionless
            | ConstraintType::ContactPyramidal => {
                if jar < 0.0 {
                    constraint_cost += 0.5 * d * jar * jar;
                }
                i += 1;
            }
            ConstraintType::ContactElliptic => {
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0];

                let u0 = jar_trial[i] * mu;
                let n = u0;
                let mut t_sq = 0.0;
                for j in 1..dim {
                    let friction_j = data.efc_mu[i][j - 1];
                    let uj = jar_trial[i + j] * friction_j;
                    t_sq += uj * uj;
                }
                let t = t_sq.sqrt();
                let t_min = MJ_MINVAL;

                if n >= mu * t || (t < t_min && n >= 0.0) {
                    // Top → zero cost
                } else if mu * n + t <= 0.0 || (t < t_min && n < 0.0) {
                    // Bottom → per-row quadratic
                    for j in 0..dim {
                        let jar_j = jar_trial[i + j];
                        let d_j = data.efc_D[i + j];
                        constraint_cost += 0.5 * d_j * jar_j * jar_j;
                    }
                } else {
                    // Middle → cone cost
                    let d_normal = data.efc_D[i];
                    let dm = d_normal / (mu * mu * (1.0 + mu * mu));
                    let n_minus_mu_t = n - mu * t;
                    constraint_cost += 0.5 * dm * n_minus_mu_t * n_minus_mu_t;
                }
                i += dim;
            }
        }
    }

    cost_gauss + constraint_cost
}
