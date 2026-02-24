//! PGS (Projected Gauss-Seidel) solver and constraint state classification.
//!
//! Contains the dual-space PGS solver (`pgs_solve_unified`), the primal-space
//! constraint classifier (`classify_constraint_states`), and the per-row
//! Delassus matrix computation (`compute_delassus_regularized`).
//!
//! Corresponds to MuJoCo's `mj_solPGS` (§29.3) and `PrimalUpdateConstraint`
//! (§15.1).

use nalgebra::{DMatrix, DVector};

use crate::constraint::impedance::MJ_MINVAL;
use crate::constraint::solver::noslip::project_elliptic_cone;
use crate::linalg::mj_solve_sparse;
use crate::types::{ConstraintState, ConstraintType, Data, Model};

/// Compute the regularized Delassus matrix AR = J·M⁻¹·J^T + diag(R).
///
/// This is the full `nefc × nefc` dense matrix used by the PGS solver.
/// Each column of M⁻¹·J^T is computed via a sparse LDL solve.
fn compute_delassus_regularized(model: &Model, data: &Data) -> DMatrix<f64> {
    let nefc = data.efc_type.len();
    let nv = model.nv;

    // Step 1: Compute M⁻¹ · J^T column by column
    let (rowadr, rownnz, colind) = model.qld_csr();
    let mut minv_jt = DMatrix::zeros(nv, nefc);
    for col in 0..nefc {
        // Copy J row (transposed = column of J^T) into a DVector for the solver
        let mut buf = DVector::zeros(nv);
        for r in 0..nv {
            buf[r] = data.efc_J[(col, r)];
        }
        // Solve M · x = J[col,:]^T → x = M⁻¹ · J[col,:]^T
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

    // Step 2: AR = J · (M⁻¹ · J^T) + diag(R)
    let mut ar = &data.efc_J * &minv_jt;
    for i in 0..nefc {
        ar[(i, i)] += data.efc_R[i];
    }

    ar
}

/// Unified PGS solver operating on all constraint types in dual (force) space.
///
/// Implements MuJoCo's `mj_solPGS` (§29.3):
/// - Builds AR = J·M⁻¹·J^T + diag(R) (regularized Delassus matrix)
/// - Gauss-Seidel iteration with per-type projection
/// - Cost guard: reverts updates that increase dual cost
/// - Elliptic contacts: group projection via `project_elliptic_cone`
///
/// After convergence, `data.efc_force` contains the constraint forces.
/// The caller must then compute `qfrc_constraint = J^T · efc_force`.
pub fn pgs_solve_unified(model: &Model, data: &mut Data) {
    let nefc = data.efc_type.len();
    if nefc == 0 {
        return;
    }

    // Build regularized Delassus matrix
    let ar = compute_delassus_regularized(model, data);

    // Precompute inverse diagonal for scalar GS updates
    let ar_diag_inv: Vec<f64> = (0..nefc)
        .map(|i| {
            let d = ar[(i, i)];
            if d.abs() < MJ_MINVAL { 0.0 } else { 1.0 / d }
        })
        .collect();

    // Warmstart: use classify_constraint_states to map qacc_warmstart → efc_force,
    // then compare dual cost vs cold start (zero forces).
    // This matches MuJoCo's universal warmstart (§29.11).
    let qacc_smooth = data.qacc_smooth.clone();
    let qfrc_smooth = data.qfrc_smooth.clone();

    // Initialize efc arrays for classification
    data.efc_state.resize(nefc, ConstraintState::Quadratic);
    data.efc_force = DVector::zeros(nefc);
    data.efc_jar = DVector::zeros(nefc);

    // Classify at qacc_warmstart to get warmstart forces
    classify_constraint_states(
        model,
        data,
        &data.qacc_warmstart.clone(),
        &qacc_smooth,
        &qfrc_smooth,
    );
    let warm_forces = data.efc_force.clone();

    // Evaluate dual cost: ½·f^T·AR·f + f^T·b
    let mut dual_cost_warm = 0.0;
    for i in 0..nefc {
        let mut ar_f_i = 0.0;
        for j in 0..nefc {
            ar_f_i += ar[(i, j)] * warm_forces[j];
        }
        dual_cost_warm += 0.5 * warm_forces[i] * ar_f_i + warm_forces[i] * data.efc_b[i];
    }

    // Cold start cost is zero (f=0 → cost=0)
    // Use warmstart only if it produces lower dual cost
    if dual_cost_warm < 0.0 {
        data.efc_force.copy_from(&warm_forces);
    } else {
        data.efc_force.fill(0.0);
    }

    let max_iters = model.solver_iterations;

    for _iter in 0..max_iters {
        let mut i = 0;
        while i < nefc {
            let ctype = data.efc_type[i];
            let dim = data.efc_dim[i];

            // Elliptic contacts: group projection
            if matches!(ctype, ConstraintType::ContactElliptic) && dim > 1 {
                // Save old force for cost guard
                let old_force: Vec<f64> = data.efc_force.as_slice()[i..i + dim].to_vec();

                // Compute residual for all rows in this group
                for j in 0..dim {
                    let mut res = data.efc_b[i + j];
                    for c in 0..nefc {
                        res += ar[(i + j, c)] * data.efc_force[c];
                    }
                    // GS update: subtract residual scaled by diagonal inverse
                    data.efc_force[i + j] -= res * ar_diag_inv[i + j];
                }

                // Project onto elliptic friction cone
                let mu = data.efc_mu[i];
                project_elliptic_cone(&mut data.efc_force.as_mut_slice()[i..i + dim], &mu, dim);

                // Cost guard: revert if dual cost increased
                let delta: Vec<f64> = (0..dim)
                    .map(|j| data.efc_force[i + j] - old_force[j])
                    .collect();
                let cost_change =
                    pgs_cost_change(&ar, &delta, &data.efc_b, &data.efc_force, i, dim, nefc);
                if cost_change > 1e-10 {
                    data.efc_force.as_mut_slice()[i..i + dim].copy_from_slice(&old_force);
                }

                i += dim;
            } else {
                // Scalar constraint: single-row GS update + projection
                let old_force = data.efc_force[i];

                // Compute residual: res = b[i] + Σ AR[i,c] * force[c]
                let mut res = data.efc_b[i];
                for c in 0..nefc {
                    res += ar[(i, c)] * data.efc_force[c];
                }

                // GS update
                data.efc_force[i] -= res * ar_diag_inv[i];

                // Project per constraint type
                match ctype {
                    // Equality: bilateral (unclamped)
                    ConstraintType::Equality | ConstraintType::FlexEdge => {}

                    // Friction loss: box clamp [-floss, +floss]
                    ConstraintType::FrictionLoss => {
                        let fl = data.efc_floss[i];
                        data.efc_force[i] = data.efc_force[i].clamp(-fl, fl);
                    }

                    // Limits, frictionless/pyramidal contacts, and elliptic dim=1: unilateral (force >= 0)
                    ConstraintType::LimitJoint
                    | ConstraintType::LimitTendon
                    | ConstraintType::ContactFrictionless
                    | ConstraintType::ContactPyramidal
                    | ConstraintType::ContactElliptic => {
                        data.efc_force[i] = data.efc_force[i].max(0.0);
                    }
                }

                // Cost guard
                let delta_f = data.efc_force[i] - old_force;
                if delta_f.abs() > 0.0 {
                    // cost_change = 0.5 * delta^2 * AR[i,i] + delta * res_before_update
                    // where res_before_update = res (computed above before GS update)
                    let cost_change = 0.5 * delta_f * delta_f * ar[(i, i)] + delta_f * res;
                    if cost_change > 1e-10 {
                        data.efc_force[i] = old_force;
                    }
                }

                i += 1;
            }
        }
    }

    data.solver_niter = max_iters;
    data.solver_stat.clear();
}

/// Compute cost change for PGS cost guard (multi-row case).
///
/// For a block update δ at rows [i..i+dim):
/// cost_change = 0.5 · δ^T · AR_block · δ + δ^T · res_old
/// where res_old is the residual BEFORE the GS update.
fn pgs_cost_change(
    ar: &DMatrix<f64>,
    delta: &[f64],
    efc_b: &DVector<f64>,
    efc_force: &DVector<f64>,
    i: usize,
    dim: usize,
    nefc: usize,
) -> f64 {
    // Reconstruct the residual before the update:
    // res_old[j] = b[i+j] + Σ AR[i+j, c] * (force[c] - delta[j] if c==i+j else force[c])
    // = b[i+j] + Σ AR[i+j, c] * force[c] - AR[i+j, i+j] * delta[j]
    // But we need the ORIGINAL residual (before force was updated).
    // force_old[j] = force[j] - delta[j], so:
    // res_old[j] = b[i+j] + Σ_{c not in block} AR[i+j, c] * force[c] + Σ_{j' in block} AR[i+j, i+j'] * (force[i+j'] - delta[j'])
    let mut cost = 0.0;
    for j in 0..dim {
        // Compute residual at old force values
        let mut res_old = efc_b[i + j];
        for c in 0..nefc {
            if c >= i && c < i + dim {
                res_old += ar[(i + j, c)] * (efc_force[c] - delta[c - i]);
            } else {
                res_old += ar[(i + j, c)] * efc_force[c];
            }
        }
        cost += delta[j] * res_old;
    }
    // Quadratic term: 0.5 * δ^T * AR_block * δ
    for j in 0..dim {
        for k in 0..dim {
            cost += 0.5 * delta[j] * ar[(i + j, i + k)] * delta[k];
        }
    }
    cost
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
#[allow(clippy::too_many_arguments)]
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
                // Always Quadratic (two-sided soft equality)
                data.efc_state[i] = ConstraintState::Quadratic;
                data.efc_force[i] = -d * jar;
                constraint_cost += 0.5 * d * jar * jar;
                i += 1;
            }

            ConstraintType::FrictionLoss => {
                // Huber threshold at ±R·floss
                let floss = data.efc_floss[i];
                let threshold = r * floss;
                if jar >= threshold {
                    // LinearPos
                    data.efc_state[i] = ConstraintState::LinearPos;
                    data.efc_force[i] = -floss;
                    constraint_cost += floss * jar - 0.5 * r * floss * floss;
                } else if jar <= -threshold {
                    // LinearNeg
                    data.efc_state[i] = ConstraintState::LinearNeg;
                    data.efc_force[i] = floss;
                    constraint_cost += -floss * jar - 0.5 * r * floss * floss;
                } else {
                    // Quadratic
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
                // jar < 0 → Quadratic; jar >= 0 → Satisfied
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
                // Three-zone classification over all dim rows
                let dim = data.efc_dim[i];
                let mu = data.efc_mu[i][0]; // Primary friction coefficient

                // Compute U vector and N, T
                // U[0] = jar[i] · μ (normal, scaled)
                // U[j] = jar[i+j] · friction[j-1]  for j = 1..dim-1
                // friction[j-1] = mu[j-1] from the 5-element mu array
                let u0 = data.efc_jar[i] * mu;
                let n = u0;

                let mut t_sq = 0.0;
                for j in 1..dim {
                    let friction_j = data.efc_mu[i][j - 1]; // mu[j-1]
                    let uj = data.efc_jar[i + j] * friction_j;
                    t_sq += uj * uj;
                }
                let t = t_sq.sqrt();

                let t_min = MJ_MINVAL; // 1e-15

                // Three-zone classification
                if n >= mu * t || (t < t_min && n >= 0.0) {
                    // Top (separated) → Satisfied
                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Satisfied;
                        data.efc_force[i + j] = 0.0;
                    }
                } else if mu * n + t <= 0.0 || (t < t_min && n < 0.0) {
                    // Bottom (fully active) → Quadratic, per-row
                    for j in 0..dim {
                        let jar_j = data.efc_jar[i + j];
                        let d_j = data.efc_D[i + j];
                        data.efc_state[i + j] = ConstraintState::Quadratic;
                        data.efc_force[i + j] = -d_j * jar_j;
                        constraint_cost += 0.5 * d_j * jar_j * jar_j;
                    }
                } else {
                    // Middle (cone surface) → Cone state
                    let d_normal = data.efc_D[i];
                    let dm = d_normal / (mu * mu * (1.0 + mu * mu));
                    let n_minus_mu_t = n - mu * t;

                    // Cost: ½·Dm·(N − μ·T)²
                    constraint_cost += 0.5 * dm * n_minus_mu_t * n_minus_mu_t;

                    // Forces:
                    // f[i]   = −Dm · NmT · μ  (normal force)
                    // f[i+j] = (Dm · NmT · μ / T) · U[j] · friction[j−1]  for j = 1..dim-1
                    let f_normal = -dm * n_minus_mu_t * mu;
                    data.efc_force[i] = f_normal;

                    for j in 1..dim {
                        let friction_j = data.efc_mu[i][j - 1];
                        let uj = data.efc_jar[i + j] * friction_j;
                        data.efc_force[i + j] = (dm * n_minus_mu_t * mu / t) * uj * friction_j;
                    }

                    // Replicate Cone state to all dim rows
                    for j in 0..dim {
                        data.efc_state[i + j] = ConstraintState::Cone;
                    }

                    // Build per-contact cone Hessian H_c (dim × dim)
                    // MuJoCo formula: H_raw then scale by diag(mu, friction) · Dm
                    let ci = data.efc_id[i]; // contact index
                    let t_safe = t.max(MJ_MINVAL);
                    let mut hc = DMatrix::<f64>::zeros(dim, dim);

                    // H_raw[0,0] = 1
                    hc[(0, 0)] = 1.0;

                    // Compute U vector (scaled jar)
                    let mut u_vec = vec![0.0_f64; dim];
                    u_vec[0] = u0; // jar[i] * mu
                    for (j, u_j) in u_vec.iter_mut().enumerate().skip(1) {
                        let friction_j = data.efc_mu[i][j - 1];
                        *u_j = data.efc_jar[i + j] * friction_j;
                    }

                    // H_raw[0,j] = -mu * U[j] / T  for j >= 1
                    // H_raw[j,0] = H_raw[0,j]  (symmetric)
                    for j in 1..dim {
                        let val = -mu * u_vec[j] / t_safe;
                        hc[(0, j)] = val;
                        hc[(j, 0)] = val;
                    }

                    // H_raw[k,j] = mu * N / T³ * U[k] * U[j]  for k,j >= 1
                    // diagonal: += mu² - mu * N / T
                    for k in 1..dim {
                        for j in 1..dim {
                            hc[(k, j)] = mu * n / (t_safe * t_safe * t_safe) * u_vec[k] * u_vec[j];
                        }
                        hc[(k, k)] += mu * mu - mu * n / t_safe;
                    }

                    // Scale: H = diag(mu, friction[0..]) · H_raw · diag(mu, friction[0..]) · Dm
                    // Build scale vector
                    let mut scale = vec![0.0_f64; dim];
                    scale[0] = mu;
                    scale[1..dim].copy_from_slice(&data.efc_mu[i][..(dim - 1)]);
                    // Apply: H[k,j] *= scale[k] * scale[j] * dm
                    for k in 0..dim {
                        for j in 0..dim {
                            hc[(k, j)] *= scale[k] * scale[j] * dm;
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
