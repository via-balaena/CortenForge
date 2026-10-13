//! Linear algebra utilities: Cholesky, sparse solve, the sparse LU of the
//! implicit integrator.
//!
//! Pure math routines with no pipeline state dependencies. Used by the
//! forward dynamics pipeline (dense Cholesky for implicit integration,
//! sparse solve for explicit/Newton paths) and analytical derivatives.

use std::cmp::Ordering;

use crate::constraint::impedance::MJ_MINVAL;
use crate::types::StepError;
use nalgebra::{DMatrix, DVector};

/// `Σ vec1[k]·vec2(k)` in MuJoCo's order (`mju_dotSparse`,
/// `engine_util_sparse.h:159-185`): four partial sums over each block of
/// four, joined as `(s0 + s2) + (s1 + s3)`, then the rest one by one.
/// `vec2(k)` is the entry of the other vector that pairs with `vec1[k]`.
pub(crate) fn dot_sparse(vec1: &[f64], vec2: impl Fn(usize) -> f64) -> f64 {
    let n = vec1.len();
    let (mut s0, mut s1, mut s2, mut s3) = (0.0, 0.0, 0.0, 0.0);
    let mut i = 0;
    while i + 4 <= n {
        s0 += vec1[i] * vec2(i);
        s1 += vec1[i + 1] * vec2(i + 1);
        s2 += vec1[i + 2] * vec2(i + 2);
        s3 += vec1[i + 3] * vec2(i + 3);
        i += 4;
    }
    let mut res = (s0 + s2) + (s1 + s3);
    while i < n {
        res += vec1[i] * vec2(i);
        i += 1;
    }
    res
}

// ============================================================================
// Dense Cholesky
// ============================================================================

/// In-place Cholesky (LL^T) factorization. Overwrites the lower triangle of `m` with L.
/// The upper triangle is left unchanged. Returns `Err(StepError::CholeskyFailed)` if
/// the matrix is not positive definite.
///
/// Zero allocations — operates entirely on borrowed data.
pub(crate) fn cholesky_in_place(m: &mut DMatrix<f64>) -> Result<(), StepError> {
    let n = m.nrows();
    for j in 0..n {
        // Diagonal: L[j,j] = sqrt(M[j,j] - Σ(L[j,k]² for k < j))
        let mut diag = m[(j, j)];
        for k in 0..j {
            diag -= m[(j, k)] * m[(j, k)];
        }
        if diag <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let ljj = diag.sqrt();
        m[(j, j)] = ljj;

        // Off-diagonal: L[i,j] = (M[i,j] - Σ(L[i,k]·L[j,k] for k < j)) / L[j,j]
        for i in (j + 1)..n {
            let mut sum = m[(i, j)];
            for k in 0..j {
                sum -= m[(i, k)] * m[(j, k)];
            }
            m[(i, j)] = sum / ljj;
        }
    }
    Ok(())
}

/// Solve L·L^T·x = b in place, where L is stored in the lower triangle of `l`.
/// On entry `x` contains b; on exit `x` contains the solution.
///
/// Zero allocations — operates entirely on borrowed data.
pub(crate) fn cholesky_solve_in_place(l: &DMatrix<f64>, x: &mut DVector<f64>) {
    let n = l.nrows();

    // Forward substitution: L·y = b
    for j in 0..n {
        for k in 0..j {
            x[j] -= l[(j, k)] * x[k];
        }
        x[j] /= l[(j, j)];
    }

    // Back substitution: L^T·z = y
    for j in (0..n).rev() {
        for k in (j + 1)..n {
            x[j] -= l[(k, j)] * x[k];
        }
        x[j] /= l[(j, j)];
    }
}

/// Dense Cholesky rank-1 update: given L such that L·L^T = H, compute L' in-place
/// such that L'·L'^T = H + v·v^T.
///
/// Uses the Linpack DCHUD algorithm (Givens rotations). O(n²).
/// The vector `v` is used as workspace and modified.
///
/// Returns `Err(StepError::CholeskyFailed)` if the diagonal becomes non-positive.
// Single-letter names (a, b, c, x, y) follow the standard linear-algebra notation; the multiply-add expression is left as `a*b + c` for clarity against the published formula.
#[allow(clippy::many_single_char_names, clippy::imprecise_flops)]
pub(crate) fn cholesky_rank1_update(l: &mut DMatrix<f64>, v: &mut [f64]) -> Result<(), StepError> {
    let n = l.nrows();
    debug_assert_eq!(v.len(), n);

    for j in 0..n {
        let a = l[(j, j)];
        let b = v[j];
        let r = (a * a + b * b).sqrt();
        if r <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let c = a / r;
        let s = b / r;
        l[(j, j)] = r;

        // Apply Givens rotation to remaining entries in column j and v
        for i in (j + 1)..n {
            let t = l[(i, j)];
            l[(i, j)] = c * t + s * v[i];
            v[i] = -s * t + c * v[i];
        }
    }
    Ok(())
}

/// Dense Cholesky rank-1 downdate: given L such that L·L^T = H, compute L' in-place
/// such that L'·L'^T = H - v·v^T.
///
/// Uses the Linpack DCHDD algorithm (forward solve + reverse Givens). O(n²).
/// The vector `v` is used as workspace and modified.
///
/// Returns `Err(StepError::CholeskyFailed)` if the result would be indefinite.
// Single-letter names follow the standard linear-algebra notation; the multiply-add form is left as `a*b + c` for clarity against the published formula. (`dead_code` retained for symmetry with the active variant.)
#[allow(dead_code, clippy::many_single_char_names, clippy::imprecise_flops)]
pub(crate) fn cholesky_rank1_downdate(
    l: &mut DMatrix<f64>,
    v: &mut [f64],
) -> Result<(), StepError> {
    let n = l.nrows();
    debug_assert_eq!(v.len(), n);

    // Step 1: Forward solve L·p = v, store p in v
    for j in 0..n {
        for k in 0..j {
            v[j] -= l[(j, k)] * v[k];
        }
        if l[(j, j)].abs() < MJ_MINVAL {
            return Err(StepError::CholeskyFailed);
        }
        v[j] /= l[(j, j)];
    }

    // Step 2: Check positive-definiteness: alpha² = 1 - ||p||² > 0
    let p_norm_sq: f64 = v.iter().map(|x| x * x).sum();
    let alpha_sq = 1.0 - p_norm_sq;
    if alpha_sq <= 0.0 {
        return Err(StepError::CholeskyFailed);
    }
    let mut alpha = alpha_sq.sqrt();

    // Step 3: Reverse Givens rotations to produce L'
    for j in (0..n).rev() {
        let a = alpha;
        let b = v[j];
        let r = (a * a + b * b).sqrt();
        if r <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
        let c = a / r;
        let s = b / r;
        alpha = r;

        // Update column j of L
        for i in (j + 1)..n {
            let t = l[(i, j)];
            // Accumulate the p-vector contribution back
            l[(i, j)] = (t - s * v[i]) / c;
            v[i] = c * v[i] - s * t;
        }

        // Update diagonal
        l[(j, j)] *= c;
        if l[(j, j)] <= 0.0 {
            return Err(StepError::CholeskyFailed);
        }
    }

    Ok(())
}

// ============================================================================
// Sparse Triangular Solve
// ============================================================================

/// Solve `L^T D L x = b` using the sparse factorization from `mj_factor_sparse`.
///
/// Matches MuJoCo's `mj_solveLD`:
/// - Off-diagonal entries at positions `0..rownnz[i]-1`
/// - Diagonal phase uses precomputed `qld_diag_inv[i]` (multiply, not divide)
///
/// On entry `x` contains `b`; on exit `x` contains the solution.
/// Zero allocations — operates entirely on borrowed data.
#[allow(non_snake_case)]
pub fn mj_solve_sparse(
    rowadr: &[usize],
    rownnz: &[usize],
    colind: &[usize],
    qld_data: &[f64],
    qld_diag_inv: &[f64],
    x: &mut DVector<f64>,
) {
    let nv = x.len();

    // Phase 1: Solve L^T y = b (scatter: propagate each DOF to its ancestors).
    // Off-diagonal entries only: positions 0..rownnz[i]-1.
    // Zero-skip: if x[i] == 0 the scatter is a no-op (MuJoCo: `if ((x_i = x[i]))`).
    // Diagonal-only skip: rownnz == 1 means no off-diagonals (MuJoCo: `if (rownnz[i] == 1)`).
    for i in (0..nv).rev() {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let xi = x[i];
        if xi == 0.0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[colind[start + k]] -= qld_data[start + k] * xi;
        }
    }

    // Phase 2: Solve D z = y (multiply by precomputed inverse, matching MuJoCo's diaginv).
    for i in 0..nv {
        x[i] *= qld_diag_inv[i];
    }

    // Phase 3: Solve L w = z (gather from ancestors)
    // Off-diagonal entries only: positions 0..rownnz[i]-1.
    // Diagonal-only skip: rownnz == 1 means no off-diagonals (MuJoCo: `if (rownnz[i] == 1)`).
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[i] -= qld_data[start + k] * x[colind[start + k]];
        }
    }
}

/// Batch solve `L^T D L X = B` for multiple right-hand sides simultaneously.
///
/// Matches MuJoCo's `mj_solveLD` with `n > 1`: the outer loop sweeps CSR metadata
/// once per DOF, the inner loop iterates across `n` vectors. This gives O(1) CSR
/// metadata loads vs O(n) for `n` separate `mj_solve_sparse` calls.
///
/// `x` is an nv × n column-major matrix (nalgebra `DMatrix`). Each column is an
/// independent RHS; on exit each column contains the corresponding solution.
///
/// Includes zero-skip in L^T phase (per-vector) and diagonal-only row skip.
#[allow(non_snake_case)]
pub(crate) fn mj_solve_sparse_batch(
    rowadr: &[usize],
    rownnz: &[usize],
    colind: &[usize],
    qld_data: &[f64],
    qld_diag_inv: &[f64],
    x: &mut DMatrix<f64>,
) {
    let nv = x.nrows();
    let n = x.ncols();

    // Phase 1: Solve L^T Y = B (scatter across all vectors).
    for i in (0..nv).rev() {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for v in 0..n {
            let xi = x[(i, v)];
            if xi == 0.0 {
                continue;
            }
            for k in 0..nnz_offdiag {
                x[(colind[start + k], v)] -= qld_data[start + k] * xi;
            }
        }
    }

    // Phase 2: Solve D Z = Y (multiply by precomputed inverse).
    for i in 0..nv {
        let inv_di = qld_diag_inv[i];
        for v in 0..n {
            x[(i, v)] *= inv_di;
        }
    }

    // Phase 3: Solve L W = Z (gather across all vectors).
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for v in 0..n {
            let mut acc = 0.0;
            for k in 0..nnz_offdiag {
                acc += qld_data[start + k] * x[(colind[start + k], v)];
            }
            x[(i, v)] -= acc;
        }
    }
}

// ============================================================================
// Sparse LU on the dof tree (MuJoCo's qLU)
// ============================================================================

/// Factor `lu`, a matrix on a pattern holding, in ascending columns, every
/// pair of dofs on one branch (`Model::D_*`), as `(U + I)·L` in place: MuJoCo
/// 3.5.0's `mju_factorLUSparse` (`engine_util_solve.c:763-847`), from the last
/// row up, with no pivoting and, on a tree's pattern, no fill-in. `index`
/// lists the rows to factor (MuJoCo's awake dofs); every row when `None`.
///
/// # Errors
///
/// [`StepError::LuSingular`] at a diagonal smaller than mjMINVAL in
/// magnitude, where MuJoCo stops with an error.
pub(crate) fn lu_factor_sparse(
    lu: &mut [f64],
    rownnz: &[usize],
    rowadr: &[usize],
    colind: &[usize],
    index: Option<&[usize]>,
) -> Result<(), StepError> {
    let n = index.map_or(rownnz.len(), <[usize]>::len);
    let row = |r: usize| index.map_or(r, |ix| ix[r]);
    let mut remaining: Vec<usize> = (0..n).map(|r| rownnz[row(r)]).collect();
    for r in (0..n).rev() {
        let i = row(r);
        remaining[r] -= 1;
        let ii = rowadr[i] + remaining[r];
        debug_assert_eq!(
            colind[ii], i,
            "row {i}: the last entry left is not its diagonal"
        );
        if lu[ii].abs() < MJ_MINVAL {
            return Err(StepError::LuSingular);
        }
        for c in (0..r).rev() {
            let j = row(c);
            let ji = rowadr[j] + remaining[c] - 1;
            if colind[ji] != i {
                continue;
            }
            remaining[c] -= 1;
            lu[ji] /= lu[ii];
            let lu_ji = lu[ji];
            // Row j's columns left of `ji` hold row i's left of its diagonal:
            // both are the dofs on one branch with them, below i.
            let (mut icnt, mut jcnt) = (rowadr[i], rowadr[j]);
            while jcnt < rowadr[j] + remaining[c] {
                match colind[icnt].cmp(&colind[jcnt]) {
                    Ordering::Equal => {
                        lu[jcnt] -= lu[icnt] * lu_ji;
                        icnt += 1;
                        jcnt += 1;
                    }
                    Ordering::Greater => jcnt += 1,
                    Ordering::Less => {
                        debug_assert!(false, "rows {i} and {j}: the factor would need fill-in");
                        icnt += 1;
                    }
                }
            }
        }
    }
    Ok(())
}

/// Solve `M·res = vec` with [`lu_factor_sparse`]'s factor of `M`: MuJoCo
/// 3.5.0's `mju_solveLUSparse` (`engine_util_solve.c:851-883`), `(U + I)`
/// from the last row up, then `L` from the first, each row's sum taken as
/// [`dot_sparse`] takes it. `diag` is each row's diagonal position
/// (`Model::D_diag`); `index` as [`lu_factor_sparse`]'s.
#[allow(clippy::too_many_arguments)]
pub(crate) fn lu_solve_sparse(
    res: &mut [f64],
    lu: &[f64],
    vec: &[f64],
    rownnz: &[usize],
    rowadr: &[usize],
    diag: &[usize],
    colind: &[usize],
    index: Option<&[usize]>,
) {
    let n = index.map_or(rownnz.len(), <[usize]>::len);
    let row = |k: usize| index.map_or(k, |ix| ix[k]);
    for k in (0..n).rev() {
        let i = row(k);
        res[i] = vec[i];
        let adr = rowadr[i] + diag[i] + 1;
        let nnz = rowadr[i] + rownnz[i] - adr;
        if nnz > 0 {
            let sum = dot_sparse(&lu[adr..adr + nnz], |t| res[colind[adr + t]]);
            res[i] -= sum;
        }
    }
    for k in 0..n {
        let i = row(k);
        let (adr, d) = (rowadr[i], diag[i]);
        if d > 0 {
            let sum = dot_sparse(&lu[adr..adr + d], |t| res[colind[adr + t]]);
            res[i] -= sum;
        }
        res[i] /= lu[adr + d];
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod cholesky_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Generate a random SPD matrix of size n×n.
    fn random_spd(n: usize, seed: u64) -> DMatrix<f64> {
        // Deterministic pseudo-random via simple LCG
        let mut state = seed;
        let mut next = || -> f64 {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            ((state >> 33) as f64) / f64::from(u32::MAX) - 0.5
        };

        // A = random matrix, then M = A^T * A + n*I (guarantees SPD)
        let a = DMatrix::from_fn(n, n, |_, _| next());
        a.transpose() * &a + DMatrix::identity(n, n) * (n as f64)
    }

    #[test]
    fn in_place_cholesky_matches_nalgebra() {
        for &n in &[1, 2, 3, 5, 10, 20] {
            let m = random_spd(n, 42 + n as u64);
            let rhs = DVector::from_fn(n, |i, _| (i as f64 + 1.0) * 0.7);

            // nalgebra reference
            let chol_ref = m.clone().cholesky().expect("nalgebra cholesky failed");
            let x_ref = chol_ref.solve(&rhs);

            // Our in-place implementation
            let mut m_inplace = m.clone();
            cholesky_in_place(&mut m_inplace).expect("in-place cholesky failed");

            let mut x_ours = rhs.clone();
            cholesky_solve_in_place(&m_inplace, &mut x_ours);

            // Compare solutions
            for i in 0..n {
                assert_relative_eq!(x_ours[i], x_ref[i], epsilon = 1e-12, max_relative = 1e-12);
            }
        }
    }

    #[test]
    fn in_place_cholesky_rejects_non_spd() {
        // Zero matrix is not SPD
        let mut m = DMatrix::zeros(3, 3);
        assert!(cholesky_in_place(&mut m).is_err());

        // Negative diagonal
        let mut m = DMatrix::identity(3, 3);
        m[(1, 1)] = -1.0;
        assert!(cholesky_in_place(&mut m).is_err());
    }

    #[test]
    fn in_place_cholesky_1x1() {
        let mut m = DMatrix::from_element(1, 1, 4.0);
        cholesky_in_place(&mut m).unwrap();
        assert_relative_eq!(m[(0, 0)], 2.0, epsilon = 1e-15);

        let mut x = DVector::from_element(1, 6.0);
        cholesky_solve_in_place(&m, &mut x);
        // 4 * x = 6 => x = 1.5
        assert_relative_eq!(x[0], 1.5, epsilon = 1e-15);
    }
}
