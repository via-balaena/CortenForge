//! MuJoCo 3.5.0's eigen-decomposition of a symmetric 3×3 matrix, and the
//! principal moments and axes of a full inertia it makes from it.
//!
//! A transcription of `mjuu_eig3` and `mjuu_fullInertia` (`user_util.cc`) in
//! plain arithmetic: each product and sum in the C source's order, with no
//! fused multiply-add, as the build the census's oracle compiles.

use super::orientation::{MJ_EPS, normvec};

/// `kEigEPS` (`user_util.cc:660`): the Jacobi iteration's stop thresholds and
/// the sort's tie margin.
const EIG_EPS: f64 = 1e-12;

/// `mjuu_eig3` (`user_util.cc:661-754`): the eigenvalues of the symmetric
/// matrix `mat` (row-major) and the quaternion (w, x, y, z) of its eigenvector
/// frame.
///
/// Jacobi iteration, at most 500 sweeps, the rotation kept as a quaternion; it
/// stops when the largest off-diagonal element is below 1e-12 or a sweep's
/// cosine above 1 − 1e-12. The eigenvalues then go in decreasing order by a
/// 0, 1, 0 bubble that swaps only where the first is more than 1e-12 below the
/// second, each swap turning the frame a quarter turn. A non-finite `mat`
/// runs the 500 sweeps and returns non-finite values.
pub fn eig3(mat: &[f64; 9]) -> ([f64; 3], [f64; 4]) {
    let mut quat = [1.0, 0.0, 0.0, 0.0];
    let mut eigval = [0.0; 3];
    for _ in 0..500 {
        // D = eigvecᵀ · mat · eigvec, its diagonal the eigenvalues.
        let eigvec = quat2mat(&quat);
        let d = mulmat(&mulmat(&transpose(&eigvec), mat), &eigvec);
        eigval = [d[0], d[4], d[8]];

        // The largest off-diagonal element, and the axis a turn about would
        // zero it.
        let (rk, ck, rotk) = if d[1].abs() > d[2].abs() && d[1].abs() > d[5].abs() {
            (0, 1, 2)
        } else if d[2].abs() > d[5].abs() {
            (0, 2, 1)
        } else {
            (1, 2, 0)
        };
        if d[3 * rk + ck].abs() < EIG_EPS {
            break;
        }

        // The 2×2 symmetric Schur decomposition.
        let tau = (d[4 * ck] - d[4 * rk]) / (2.0 * d[3 * rk + ck]);
        let t = if tau >= 0.0 {
            1.0 / (tau + (1.0 + tau * tau).sqrt())
        } else {
            -1.0 / (-tau + (1.0 + tau * tau).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        if c > 1.0 - EIG_EPS {
            break;
        }

        // The turn as a quaternion, accumulated.
        let mut turn = [0.0; 4];
        let half_sin = (0.5 - 0.5 * c).sqrt();
        turn[rotk + 1] = if tau >= 0.0 { -half_sin } else { half_sin };
        if rotk == 1 {
            turn[rotk + 1] = -turn[rotk + 1];
        }
        turn[0] = (1.0 - turn[rotk + 1] * turn[rotk + 1]).sqrt();
        normvec(&mut turn);
        quat = mulquat(&quat, &turn);
        normvec(&mut quat);
    }

    for j in 0..3 {
        let j1 = j % 2;
        if eigval[j1] + EIG_EPS < eigval[j1 + 1] {
            eigval.swap(j1, j1 + 1);
            // MuJoCo's own literal, which is not the double nearest 1/√2
            // (`f64::consts::FRAC_1_SQRT_2`): its bits are the turn's.
            #[allow(clippy::approx_constant)]
            let half = 0.707_106_781_186_548;
            let mut turn = [0.0; 4];
            turn[0] = half;
            turn[(j1 + 2) % 3 + 1] = half;
            quat = mulquat(&quat, &turn);
            normvec(&mut quat);
        }
    }
    (eigval, quat)
}

/// `mjuu_fullInertia` (`user_util.cc:785-811`): the principal moments, in
/// decreasing order, and the frame (w, x, y, z) of the inertia
/// `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`; MuJoCo's refusal when the smallest moment
/// is below mjEPS.
pub fn full_inertia(full: &[f64; 6]) -> Result<([f64; 3], [f64; 4]), &'static str> {
    let mat = [
        full[0], full[3], full[4], //
        full[3], full[1], full[5], //
        full[4], full[5], full[2],
    ];
    let (eigval, quat) = eig3(&mat);
    if eigval[2] < MJ_EPS {
        return Err("inertia must have positive eigenvalues");
    }
    Ok((eigval, quat))
}

/// `mjuu_quat2mat`: the rotation matrix (row-major) of the quaternion, exactly
/// the identity for (1, 0, 0, 0).
fn quat2mat(q: &[f64; 4]) -> [f64; 9] {
    // MuJoCo's exact test for the identity, which skips the products.
    #[allow(clippy::float_cmp)]
    if *q == [1.0, 0.0, 0.0, 0.0] {
        return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    }
    let q00 = q[0] * q[0];
    let q01 = q[0] * q[1];
    let q02 = q[0] * q[2];
    let q03 = q[0] * q[3];
    let q11 = q[1] * q[1];
    let q12 = q[1] * q[2];
    let q13 = q[1] * q[3];
    let q22 = q[2] * q[2];
    let q23 = q[2] * q[3];
    let q33 = q[3] * q[3];
    [
        q00 + q11 - q22 - q33,
        2.0 * (q12 - q03),
        2.0 * (q13 + q02),
        2.0 * (q12 + q03),
        q00 - q11 + q22 - q33,
        2.0 * (q23 - q01),
        2.0 * (q13 - q02),
        2.0 * (q23 + q01),
        q00 - q11 - q22 + q33,
    ]
}

/// `mjuu_transposemat`.
fn transpose(m: &[f64; 9]) -> [f64; 9] {
    [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]]
}

/// `mjuu_mulmat`: `a · b`, row-major.
fn mulmat(a: &[f64; 9], b: &[f64; 9]) -> [f64; 9] {
    let mut r = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
        }
    }
    r
}

/// `mjuu_mulquat`: `qa · qb`, made unit by `normvec` as MuJoCo's is.
fn mulquat(qa: &[f64; 4], qb: &[f64; 4]) -> [f64; 4] {
    let mut r = [
        qa[0] * qb[0] - qa[1] * qb[1] - qa[2] * qb[2] - qa[3] * qb[3],
        qa[0] * qb[1] + qa[1] * qb[0] + qa[2] * qb[3] - qa[3] * qb[2],
        qa[0] * qb[2] - qa[1] * qb[3] + qa[2] * qb[0] + qa[3] * qb[1],
        qa[0] * qb[3] + qa[1] * qb[2] - qa[2] * qb[1] + qa[3] * qb[0],
    ];
    normvec(&mut r);
    r
}

#[cfg(test)]
// The values compared are exact: a diagonal matrix's own entries, untouched.
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn a_diagonal_matrix_is_its_own_decomposition_sorted_decreasing() {
        let (val, _) = eig3(&[1.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 2.0]);
        assert_eq!(val, [3.0, 2.0, 1.0]);
    }

    #[test]
    fn moments_within_the_tie_margin_keep_their_order() {
        // 1 and 1 + 1e-13: not more than 1e-12 apart, so no swap and no turn.
        let (val, quat) = eig3(&[1.0, 0.0, 0.0, 0.0, 1.0 + 1e-13, 0.0, 0.0, 0.0, 0.5]);
        assert_eq!(val, [1.0, 1.0 + 1e-13, 0.5]);
        assert_eq!(quat, [1.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn a_non_finite_matrix_returns_non_finite_values_in_bounded_time() {
        let (val, quat) = eig3(&[1.0, f64::NAN, 0.0, f64::NAN, 1.0, 0.0, 0.0, 0.0, 1.0]);
        assert!(val.iter().chain(quat.iter()).any(|x| !x.is_finite()));
    }

    #[test]
    fn a_full_inertia_with_a_moment_below_mj_eps_is_refused() {
        assert_eq!(
            full_inertia(&[1.0, 1.0, 1e-15, 0.0, 0.0, 0.0]),
            Err("inertia must have positive eigenvalues")
        );
    }
}
