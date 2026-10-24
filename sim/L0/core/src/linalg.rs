//! Linear algebra utilities: Cholesky, the sparse dot product, the sparse
//! LU of the implicit integrator.
//!
//! Pure math routines with no pipeline state dependencies. The mass
//! matrix's factor and solve are `dynamics::factor`'s.

use std::cmp::Ordering;

use crate::constraint::impedance::MJ_MINVAL;
use crate::types::StepError;
use nalgebra::{DMatrix, DVector};

/// `Σ vec1[k]·vec2[k]` in MuJoCo's order (`mju_dot`,
/// `engine_util_blas.c:704-766`): four partial sums over each block of
/// four, joined as `(s0 + s2) + (s1 + s3)`, then the remaining one to three
/// products summed and added at once.
pub(crate) fn mju_dot(vec1: &[f64], vec2: &[f64]) -> f64 {
    let n = vec1.len();
    let (mut s0, mut s1, mut s2, mut s3) = (0.0, 0.0, 0.0, 0.0);
    let mut i = 0;
    while i + 4 <= n {
        s0 += vec1[i] * vec2[i];
        s1 += vec1[i + 1] * vec2[i + 1];
        s2 += vec1[i + 2] * vec2[i + 2];
        s3 += vec1[i + 3] * vec2[i + 3];
        i += 4;
    }
    let mut res = (s0 + s2) + (s1 + s3);
    match n - i {
        3 => res += vec1[i] * vec2[i] + vec1[i + 1] * vec2[i + 1] + vec1[i + 2] * vec2[i + 2],
        2 => res += vec1[i] * vec2[i] + vec1[i + 1] * vec2[i + 1],
        1 => res += vec1[i] * vec2[i],
        _ => {}
    }
    res
}

/// `max(a, b)` as MuJoCo's `mju_max` (`engine_util_misc.c:1296-1302`): `a`
/// when `a ≥ b`, else `b`. A NaN `b` gives NaN, a NaN `a` gives `b`; Rust's
/// `f64::max` gives the other value for either.
#[inline]
pub(crate) fn mju_max(a: f64, b: f64) -> f64 {
    if a >= b { a } else { b }
}

/// `min(a, b)` as MuJoCo's `mju_min` (`engine_util_misc.c:1286-1292`): `a`
/// when `a ≤ b`, else `b`, a NaN in either place as [`mju_max`] takes it.
#[inline]
pub(crate) fn mju_min(a: f64, b: f64) -> f64 {
    if a <= b { a } else { b }
}

/// The dot product of two 6-vectors in MuJoCo's order (`mji_dot6`,
/// `engine_inline.h:422-429`): `((v0·w0 + v2·w2) + (v1·w1 + v3·w3)) +
/// (v4·w4 + v5·w5)`, as `mju_dot` groups six but without its 0 start.
pub(crate) fn mji_dot6(vec1: &[f64], vec2: &[f64]) -> f64 {
    ((vec1[0] * vec2[0] + vec1[2] * vec2[2]) + (vec1[1] * vec2[1] + vec1[3] * vec2[3]))
        + (vec1[4] * vec2[4] + vec1[5] * vec2[5])
}

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

/// [`dot_sparse`] over `(column, value)` entries, `Σ value·vec[column]` in
/// `mju_dotSparse`'s grouping: the first entries in fours, the rest one by one.
pub(crate) fn dot_sparse_entries(
    entries: impl Iterator<Item = (usize, f64)> + Clone,
    vec: &[f64],
) -> f64 {
    let full = entries.clone().count() / 4 * 4;
    let mut s = [0.0_f64; 4];
    for (k, (c, v)) in entries.clone().take(full).enumerate() {
        s[k % 4] += v * vec[c];
    }
    let mut res = (s[0] + s[2]) + (s[1] + s[3]);
    for (c, v) in entries.skip(full) {
        res += v * vec[c];
    }
    res
}

/// `‖v‖` in MuJoCo's order (`mju_norm` = `sqrt(mju_dot)`,
/// `engine_util_blas.c:698-765`): four interleaved sums of squares, added as
/// `(r0 + r2) + (r1 + r3)`, then the remaining one to three left to right.
pub(crate) fn mju_norm(v: &[f64]) -> f64 {
    let n = v.len();
    let (mut r0, mut r1, mut r2, mut r3) = (0.0, 0.0, 0.0, 0.0);
    let mut i = 0;
    while i + 4 <= n {
        r0 += v[i] * v[i];
        r1 += v[i + 1] * v[i + 1];
        r2 += v[i + 2] * v[i + 2];
        r3 += v[i + 3] * v[i + 3];
        i += 4;
    }
    let mut sum = (r0 + r2) + (r1 + r3);
    match n - i {
        3 => sum += v[i] * v[i] + v[i + 1] * v[i + 1] + v[i + 2] * v[i + 2],
        2 => sum += v[i] * v[i] + v[i + 1] * v[i + 1],
        1 => sum += v[i] * v[i],
        _ => {}
    }
    sum.sqrt()
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

/// Factor the dense row-major `n × n` matrix `mat` as `L·L'` in place, as
/// MuJoCo 3.5.0's `mju_cholFactor` does (`engine_util_solve.c:33-62`): `L`
/// in the lower triangle, the upper triangle left as it was. A diagonal below
/// `mindiag` is replaced by `mindiag`; the count of the others is returned (the
/// rank).
pub(crate) fn mju_chol_factor(mat: &mut [f64], n: usize, mindiag: f64) -> usize {
    let mut rank = n;
    for j in 0..n {
        let mut tmp = mat[j * (n + 1)];
        if j > 0 {
            tmp -= mju_dot(&mat[j * n..j * n + j], &mat[j * n..j * n + j]);
        }
        if tmp < mindiag {
            tmp = mindiag;
            rank -= 1;
        }
        mat[j * (n + 1)] = tmp.sqrt();
        let inv = 1.0 / mat[j * (n + 1)];
        for i in j + 1..n {
            let dot = mju_dot(&mat[i * n..i * n + j], &mat[j * n..j * n + j]);
            mat[i * n + j] = (mat[i * n + j] - dot) * inv;
        }
    }
    rank
}

/// Solve `L·L'·x = b` in place with [`mju_chol_factor`]'s factor, as MuJoCo
/// 3.5.0's `mju_cholSolve` does (`engine_util_solve.c:66-92`). `x` holds `b`
/// on entry.
pub(crate) fn mju_chol_solve(mat: &[f64], x: &mut [f64], n: usize) {
    for i in 0..n {
        if i > 0 {
            let dot = mju_dot(&mat[i * n..i * n + i], &x[..i]);
            x[i] -= dot;
        }
        x[i] /= mat[i * (n + 1)];
    }
    for i in (0..n).rev() {
        for j in i + 1..n {
            x[i] -= mat[j * n + i] * x[j];
        }
        x[i] /= mat[i * (n + 1)];
    }
}

/// Update [`mju_chol_factor`]'s factor in place to that of `L·L' + x·x'`
/// (`plus`) or `L·L' − x·x'`, as MuJoCo 3.5.0's `mju_cholUpdate` does
/// (`engine_util_solve.c:96-136`). `x` is overwritten. A new diagonal square
/// below mjMINVAL is replaced by mjMINVAL; the count of the others is returned
/// (the rank).
// Single-letter names are mju_cholUpdate's own.
#[allow(clippy::many_single_char_names)]
pub(crate) fn mju_chol_update(mat: &mut [f64], x: &mut [f64], n: usize, plus: bool) -> usize {
    let mut rank = n;
    for k in 0..n {
        if x[k] != 0.0 {
            let lkk = mat[k * (n + 1)];
            let mut tmp = lkk * lkk + if plus { x[k] * x[k] } else { -x[k] * x[k] };
            if tmp < MJ_MINVAL {
                tmp = MJ_MINVAL;
                rank -= 1;
            }
            let r = tmp.sqrt();
            let c = r / lkk;
            let cinv = 1.0 / c;
            let s = x[k] / lkk;
            mat[k * (n + 1)] = r;
            if plus {
                for i in k + 1..n {
                    mat[i * n + k] = (mat[i * n + k] + s * x[i]) * cinv;
                }
            } else {
                for i in k + 1..n {
                    mat[i * n + k] = (mat[i * n + k] - s * x[i]) * cinv;
                }
            }
            for i in k + 1..n {
                x[i] = c * x[i] - s * mat[i * n + k];
            }
        }
    }
    rank
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
#[allow(clippy::too_many_arguments)] // `mju_solveLUSparse`'s arguments, one for one
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

    /// `H` row-major, factored by `mju_chol_factor`.
    fn row_major_factor(h: &DMatrix<f64>) -> Vec<f64> {
        let n = h.nrows();
        let mut l: Vec<f64> = (0..n * n).map(|k| h[(k / n, k % n)]).collect();
        assert_eq!(mju_chol_factor(&mut l, n, MJ_MINVAL), n, "full rank");
        l
    }

    /// `L·L'` from a row-major factor's lower triangle.
    fn product(l: &[f64], n: usize) -> DMatrix<f64> {
        DMatrix::from_fn(n, n, |i, j| {
            (0..=i.min(j)).map(|k| l[i * n + k] * l[j * n + k]).sum()
        })
    }

    /// `mju_chol_update` removes a row from the factor: updating the factor of
    /// `H` with `−v·v'` gives the factor of `H − v·v'`, and with `+v·v'` that
    /// of `H + v·v'`, as MuJoCo's `mju_cholUpdate` (`flg_plus` 0 and 1). (The
    /// parent's Newton removed a row by negating `v` and adding it, which adds
    /// it again.) `v` is half the factor's first column, so `H − v·v'` is
    /// positive definite.
    #[test]
    fn chol_update_minus_removes_row() {
        for n in 2..=6 {
            let h = random_spd(n, 7 + n as u64);
            let l0 = row_major_factor(&h);
            let v = DVector::from_fn(n, |i, _| 0.5 * l0[i * n]);
            let tol = 1e-14 * h.amax();
            for (plus, want) in [
                (false, &h - &v * v.transpose()),
                (true, &h + &v * v.transpose()),
            ] {
                let mut l = l0.clone();
                let mut x: Vec<f64> = v.iter().copied().collect();
                assert_eq!(mju_chol_update(&mut l, &mut x, n, plus), n, "full rank");
                let got = product(&l, n);
                for i in 0..n {
                    for j in 0..n {
                        assert!(
                            (got[(i, j)] - want[(i, j)]).abs() <= tol,
                            "n {n}, plus {plus}: [{i},{j}] {:e} vs {:e}",
                            got[(i, j)],
                            want[(i, j)]
                        );
                    }
                }
            }
        }
    }

    /// `mju_chol_solve` with `mju_chol_factor`'s factor solves `H·x = b`.
    #[test]
    fn chol_solve_solves() {
        for n in 1..=7 {
            let h = random_spd(n, 99 + n as u64);
            let l = row_major_factor(&h);
            let b: Vec<f64> = (0..n).map(|i| (i as f64 + 1.0) * 0.3).collect();
            let mut x = b.clone();
            mju_chol_solve(&l, &mut x, n);
            let hx = &h * DVector::from_column_slice(&x);
            for i in 0..n {
                assert!((hx[i] - b[i]).abs() <= 1e-13, "n {n}: row {i}");
            }
        }
    }
}

#[cfg(test)]
mod mju_norm_tests {
    use super::mju_norm;

    /// A weld's six violations where MuJoCo's summation order differs from
    /// a left-to-right sum in the last place: MuJoCo 3.5.0's `mju_norm` gives
    /// 0.013731086045903286; summed left to right, 0.013731086045903288.
    /// And a connect's three, where `a² + (b² + c²)` gives
    /// 0.0003823124437736077 and MuJoCo's order 0.00038231244377360775.
    #[test]
    fn mju_norm_sums_in_mujoco_order() {
        let v = [
            -0.006_066, -0.005_913, 0.002_481, 0.008_006, 0.006_809, -0.000_411,
        ];
        assert_eq!(
            mju_norm(&v).to_bits(),
            0.013_731_086_045_903_286_f64.to_bits()
        );
        let v = [
            -4.012_437_229_592_179e-5,
            -1.493_753_111_448_576_4e-4,
            -3.496_281_679_618_837_6e-4,
        ];
        assert_eq!(
            mju_norm(&v).to_bits(),
            0.000_382_312_443_773_607_75_f64.to_bits()
        );
    }
}
