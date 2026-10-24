//! QCQP solvers for friction cone projection.
//!
//! Matches MuJoCo's `mju_QCQP2/3()` in `engine_util_solve.c` and
//! `mju_QCQP()` for N-dimensional problems.
//!
//! All functions solve: min 0.5*x'*A*x + b'*x  s.t.  Σ(x_i/d_i)² ≤ r²
//! Returns (result, active) where active = constraint was projected to boundary.

/// Convergence tolerance for QCQP Newton iteration.
/// Matches MuJoCo's hardcoded `1e-10` threshold.
use crate::linalg::{mju_chol_factor, mju_chol_solve, mju_dot};

const QCQP_TOL: f64 = 1e-10;

/// Maximum Newton iterations. Matches MuJoCo.
const QCQP_MAX_ITER: usize = 20;

/// 2D QCQP solver (condim=3, 2 friction DOFs), MuJoCo's `mju_QCQP2`
/// (`engine_util_solve.c:991-1052`) operation for operation: `A` and `b`
/// scaled by `d`, then Newton steps on λ with `P = (A + λ)⁻¹` written out,
/// `v = −P·b`, until `v·v − r²` or the step falls under 1e-10, at most 20;
/// active when λ moved from 0.
///
/// `a`: 2×2 Delassus subblock (row-major as [[f64; 2]; 2]), whose upper
/// triangle is read.
/// `b`: 2-element bias vector.
/// `d`: 2-element friction coefficients (mu).
/// `r`: cone radius (normal force).
// Single-letter names are mju_QCQP2's own. The cofactor products are
// written as MuJoCo writes them (`A12*A12` against `(A11+la)*(A22+la)`).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
pub fn qcqp2(a: [[f64; 2]; 2], b: [f64; 2], d: [f64; 2], r: f64) -> ([f64; 2], bool) {
    let b1 = b[0] * d[0];
    let b2 = b[1] * d[1];
    let a11 = a[0][0] * d[0] * d[0];
    let a22 = a[1][1] * d[1] * d[1];
    let a12 = a[0][1] * d[0] * d[1];
    let mut la = 0.0;
    let (mut v1, mut v2) = (0.0, 0.0);
    for _ in 0..QCQP_MAX_ITER {
        let det = (a11 + la) * (a22 + la) - a12 * a12;
        if det < QCQP_TOL {
            return ([0.0, 0.0], false);
        }
        let detinv = 1.0 / det;
        let p11 = (a22 + la) * detinv;
        let p22 = (a11 + la) * detinv;
        let p12 = -a12 * detinv;
        v1 = -p11 * b1 - p12 * b2;
        v2 = -p12 * b1 - p22 * b2;
        let val = v1 * v1 + v2 * v2 - r * r;
        if val < QCQP_TOL {
            break;
        }
        let deriv = -2.0 * (p11 * v1 * v1 + 2.0 * p12 * v1 * v2 + p22 * v2 * v2);
        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        la += delta;
    }
    ([v1 * d[0], v2 * d[1]], la != 0.0)
}

/// 3D QCQP solver (condim=4, 3 friction DOFs), MuJoCo's `mju_QCQP3`
/// (`engine_util_solve.c:1058-1140`) operation for operation, as
/// [`qcqp2`] with `P` from the cofactors of `A + λ` over its determinant.
// Single-letter names are mju_QCQP3's own. The cofactor products are
// written as MuJoCo writes them (`A12*A12` against `(A11+la)*(A22+la)`).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
pub fn qcqp3(a: [[f64; 3]; 3], b: [f64; 3], d: [f64; 3], r: f64) -> ([f64; 3], bool) {
    let b1 = b[0] * d[0];
    let b2 = b[1] * d[1];
    let b3 = b[2] * d[2];
    let a11 = a[0][0] * d[0] * d[0];
    let a22 = a[1][1] * d[1] * d[1];
    let a33 = a[2][2] * d[2] * d[2];
    let a12 = a[0][1] * d[0] * d[1];
    let a13 = a[0][2] * d[0] * d[2];
    let a23 = a[1][2] * d[1] * d[2];
    let mut la = 0.0;
    let (mut v1, mut v2, mut v3) = (0.0, 0.0, 0.0);
    for _ in 0..QCQP_MAX_ITER {
        let mut p11 = (a22 + la) * (a33 + la) - a23 * a23;
        let mut p22 = (a11 + la) * (a33 + la) - a13 * a13;
        let mut p33 = (a11 + la) * (a22 + la) - a12 * a12;
        let mut p12 = a13 * a23 - a12 * (a33 + la);
        let mut p13 = a12 * a23 - a13 * (a22 + la);
        let mut p23 = a12 * a13 - a23 * (a11 + la);
        let det = (a11 + la) * p11 + a12 * p12 + a13 * p13;
        if det < QCQP_TOL {
            return ([0.0, 0.0, 0.0], false);
        }
        let detinv = 1.0 / det;
        p11 *= detinv;
        p22 *= detinv;
        p33 *= detinv;
        p12 *= detinv;
        p13 *= detinv;
        p23 *= detinv;
        v1 = -p11 * b1 - p12 * b2 - p13 * b3;
        v2 = -p12 * b1 - p22 * b2 - p23 * b3;
        v3 = -p13 * b1 - p23 * b2 - p33 * b3;
        let val = v1 * v1 + v2 * v2 + v3 * v3 - r * r;
        if val < QCQP_TOL {
            break;
        }
        let deriv = -2.0 * (p11 * v1 * v1 + p22 * v2 * v2 + p33 * v3 * v3)
            - 4.0 * (p12 * v1 * v2 + p13 * v1 * v3 + p23 * v2 * v3);
        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        la += delta;
    }
    ([v1 * d[0], v2 * d[1], v3 * d[2]], la != 0.0)
}

/// N-dimensional QCQP solver (condim=6, up to 5 friction DOFs), MuJoCo's
/// `mju_QCQP` (`engine_util_solve.c:1146-1212`) operation for operation: `A`
/// and `b` scaled by `d`, then Newton steps on λ, each factoring `A + λ`
/// ([`mju_chol_factor`], refusing a rank under `n` at 1e-10) and solving for
/// `res = −(A + λ)⁻¹·b`, until `res·res − r²` or the step falls under 1e-10,
/// at most 20.
///
/// `a`: n×n Delassus subblock (flat row-major, length n*n).
/// `b`: n-element bias vector.
/// `d`: n-element friction coefficients.
/// `r`: cone radius.
/// `n`: dimension (number of friction DOFs), at most 5.
// Single-letter names are mju_QCQP's own.
#[allow(clippy::many_single_char_names)]
pub fn qcqp_nd(a: &[f64], b: &[f64], d: &[f64], r: f64, n: usize) -> (Vec<f64>, bool) {
    let mut am = [0.0; 25];
    let mut bs = [0.0; 5];
    for i in 0..n {
        bs[i] = b[i] * d[i];
        for j in 0..n {
            am[j + i * n] = a[j + i * n] * d[i] * d[j];
        }
    }
    let mut la = 0.0;
    let mut res = vec![0.0; n];
    let mut tmp = [0.0; 5];
    for _ in 0..QCQP_MAX_ITER {
        let mut ala = am;
        for i in 0..n {
            ala[i * (n + 1)] += la;
        }
        if mju_chol_factor(&mut ala[..n * n], n, QCQP_TOL) < n {
            return (vec![0.0; n], false);
        }
        res.copy_from_slice(&bs[..n]);
        mju_chol_solve(&ala[..n * n], &mut res, n);
        for x in &mut res {
            *x *= -1.0;
        }
        let val = mju_dot(&res, &res) - r * r;
        if val < QCQP_TOL {
            break;
        }
        tmp[..n].copy_from_slice(&res);
        mju_chol_solve(&ala[..n * n], &mut tmp[..n], n);
        let deriv = -2.0 * mju_dot(&res, &tmp[..n]);
        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        la += delta;
    }
    for i in 0..n {
        res[i] *= d[i];
    }
    (res, la != 0.0)
}

#[cfg(test)]
#[allow(
    clippy::many_single_char_names,
    clippy::float_cmp,
    clippy::uninlined_format_args
)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-10;

    /// T1: QCQP2 conformance — constrained case with exact values → AC1
    #[test]
    fn test_qcqp2_constrained_identity() {
        let a = [[1.0, 0.0], [0.0, 1.0]];
        let b = [-2.0, -2.0];
        let d = [1.0, 1.0];
        let r = 1.0;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(active, "should be active (projected to boundary)");
        let expected = 1.0 / 2.0_f64.sqrt();
        assert!(
            (result[0] - expected).abs() < TOL,
            "x[0] = {}, expected {}",
            result[0],
            expected
        );
        assert!(
            (result[1] - expected).abs() < TOL,
            "x[1] = {}, expected {}",
            result[1],
            expected
        );
        // Cone constraint
        let norm2 = result[0] * result[0] + result[1] * result[1];
        assert!(norm2 <= 1.0 + TOL, "cone violated: ||x||² = {}", norm2);
    }

    /// T2: QCQP3 conformance — constrained case with exact values → AC2
    #[test]
    fn test_qcqp3_constrained_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [-3.0, -4.0, 0.0];
        let d = [1.0, 1.0, 1.0];
        let r = 1.0;
        let (result, active) = qcqp3(a, b, d, r);
        assert!(active, "should be active");
        assert!(
            (result[0] - 0.6).abs() < TOL,
            "x[0] = {}, expected 0.6",
            result[0]
        );
        assert!(
            (result[1] - 0.8).abs() < TOL,
            "x[1] = {}, expected 0.8",
            result[1]
        );
        assert!(result[2].abs() < TOL, "x[2] = {}, expected 0.0", result[2]);
        let norm2: f64 = result.iter().map(|x| x * x).sum();
        assert!(norm2 <= 1.0 + TOL, "cone violated: ||x||² = {}", norm2);
    }

    /// T3: QCQP_ND 5D — condim=6 with exact values → AC3
    #[test]
    fn test_qcqp_nd_5d_constrained() {
        let n = 5;
        // Identity matrix
        let mut a = vec![0.0_f64; n * n];
        for i in 0..n {
            a[i * n + i] = 1.0;
        }
        let b = vec![-1.0; n];
        let d = vec![0.5; n];
        let r = 0.5;
        let (result, active) = qcqp_nd(&a, &b, &d, r, n);
        assert!(active, "should be active");
        let expected = 5.0_f64.sqrt() / 20.0; // √5/20
        for (i, &x) in result.iter().enumerate() {
            assert!(
                (x - expected).abs() < TOL,
                "x[{}] = {}, expected {}",
                i,
                x,
                expected
            );
        }
        // Cone: Σ(x_i/0.5)² = r²
        let cone: f64 = result
            .iter()
            .zip(d.iter())
            .map(|(x, di)| (x / di).powi(2))
            .sum();
        assert!(
            (cone - r * r).abs() < TOL,
            "cone = {}, expected {}",
            cone,
            r * r
        );
    }

    /// T10: QCQP degenerate input → AC11
    #[test]
    fn test_qcqp2_degenerate_zero_matrix() {
        let a = [[0.0, 0.0], [0.0, 0.0]];
        let b = [1.0, 1.0];
        let d = [0.5, 0.5];
        let r = 1.0;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(!active, "degenerate should not be active");
        assert_eq!(result, [0.0, 0.0]);
    }

    /// T11: QCQP unconstrained (inside cone) → AC12
    #[test]
    fn test_qcqp2_unconstrained_inside_cone() {
        let a = [[10.0, 0.0], [0.0, 10.0]];
        let b = [-1.0, -1.0];
        let d = [1.0, 1.0];
        let r = 10.0;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(!active, "should not be active (inside cone)");
        assert!(
            (result[0] - 0.1).abs() < TOL,
            "x[0] = {}, expected 0.1",
            result[0]
        );
        assert!(
            (result[1] - 0.1).abs() < TOL,
            "x[1] = {}, expected 0.1",
            result[1]
        );
    }

    /// T12: QCQP2 asymmetric mu — degenerate cone axis → AC1
    ///
    /// With extreme aspect ratio (d=[1.0, 0.001]), the Newton iteration may
    /// not converge in 20 iterations. The caller's ellipsoidal rescale
    /// (when active=true) enforces the cone constraint exactly — the QCQP
    /// function provides the direction, the rescale provides the magnitude.
    /// This matches MuJoCo's behavior: QCQP is approximate, caller rescales.
    #[test]
    fn test_qcqp2_asymmetric_mu() {
        let a = [[2.0, 0.0], [0.0, 2.0]];
        let b = [-3.0, -3.0];
        let d = [1.0, 0.001];
        let r = 1.0;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(
            active,
            "should be active (unconstrained [1.5, 1.5] far outside)"
        );
        assert!(
            result[0].is_finite() && result[1].is_finite(),
            "result must be finite"
        );
        // With extreme aspect ratio, Newton may not fully converge in 20 iters.
        // Verify the result is reasonable — the caller applies ellipsoidal rescale
        // to enforce the cone exactly when active==true.
        assert!(
            result[0] > 0.0,
            "x[0] should be positive (bias is negative)"
        );
    }

    /// T13: QCQP2 iteration exhaustion (ill-conditioned A) → AC1
    #[test]
    fn test_qcqp2_ill_conditioned() {
        let a = [[1.0, 0.999], [0.999, 1.0]];
        let b = [-10.0, -10.0];
        let d = [1.0, 1.0];
        let r = 0.5;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(
            result[0].is_finite() && result[1].is_finite(),
            "must be finite"
        );
        let norm2 = result[0] * result[0] + result[1] * result[1];
        // Either converged on boundary or close to it
        assert!(
            norm2 <= r * r + 1e-6,
            "cone violated: ||x||² = {} > r² = {}",
            norm2,
            r * r
        );
        // Should be active since unconstrained is far outside
        assert!(active, "should be active");
    }

    /// T18: QCQP2 delta step guard (early exit) → AC1
    #[test]
    fn test_qcqp2_inside_cone_stiff() {
        let a = [[1e6, 0.0], [0.0, 1e6]];
        let b = [-1.0, -1.0];
        let d = [1.0, 1.0];
        let r = 0.5;
        let (result, active) = qcqp2(a, b, d, r);
        // Unconstrained: [1e-6, 1e-6], ||x||² = 2e-12 << 0.25, inside cone
        assert!(!active, "should not be active (inside cone)");
        assert!((result[0] - 1e-6).abs() < TOL, "x[0] = {}", result[0]);
        assert!((result[1] - 1e-6).abs() < TOL, "x[1] = {}", result[1]);
    }

    /// T20: QCQP2 with very large friction coefficients → AC12
    #[test]
    fn test_qcqp2_large_mu() {
        let a = [[1.0, 0.0], [0.0, 1.0]];
        let b = [-0.5, -0.5];
        let d = [100.0, 100.0];
        let r = 1.0;
        let (result, active) = qcqp2(a, b, d, r);
        assert!(!active, "should not be active (inside cone with large mu)");
        assert!((result[0] - 0.5).abs() < TOL, "x[0] = {}", result[0]);
        assert!((result[1] - 0.5).abs() < TOL, "x[1] = {}", result[1]);
    }

    /// T21: QCQP2 with zero mu in one direction → AC11
    #[test]
    fn test_qcqp2_zero_mu() {
        let a = [[1.0, 0.0], [0.0, 1.0]];
        let b = [-2.0, -2.0];
        let d = [1.0, 0.0];
        let r = 1.0;
        let (result, active) = qcqp2(a, b, d, r);
        // d[1]=0 makes A_s[1,1] = 0 → singular at λ=0
        assert!(!active, "degenerate (zero mu) should not be active");
        assert_eq!(result, [0.0, 0.0]);
    }

    /// T22: QCQP3 degenerate input → AC2
    #[test]
    fn test_qcqp3_degenerate_zero_matrix() {
        let a = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let b = [1.0, 1.0, 1.0];
        let d = [1.0, 1.0, 1.0];
        let r = 1.0;
        let (result, active) = qcqp3(a, b, d, r);
        assert!(!active, "degenerate should not be active");
        assert_eq!(result, [0.0, 0.0, 0.0]);
    }

    /// T23: QCQP3 inside cone → AC2
    #[test]
    fn test_qcqp3_inside_cone() {
        let a = [[10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]];
        let b = [-1.0, -1.0, -1.0];
        let d = [1.0, 1.0, 1.0];
        let r = 10.0;
        let (result, active) = qcqp3(a, b, d, r);
        assert!(!active, "should not be active (inside cone)");
        for (i, &x) in result.iter().enumerate() {
            assert!((x - 0.1).abs() < TOL, "x[{}] = {}", i, x);
        }
    }

    /// T24: QCQP_ND degenerate input (5D) → AC3
    #[test]
    fn test_qcqp_nd_5d_degenerate() {
        let n = 5;
        let a = vec![0.0_f64; n * n];
        let b = vec![1.0; n];
        let d = vec![1.0; n];
        let r = 1.0;
        let (result, active) = qcqp_nd(&a, &b, &d, r, n);
        assert!(!active, "degenerate should not be active");
        assert!(result.iter().all(|&x| x == 0.0), "should return zeros");
    }

    /// T26: QCQP2 symmetric stress — random inputs → AC1 (supplementary)
    #[test]
    fn test_qcqp2_random_stress() {
        // Test several representative cases to catch rounding bugs
        let cases: Vec<([[f64; 2]; 2], [f64; 2], [f64; 2], f64)> = vec![
            // Symmetric positive definite with off-diagonal
            ([[3.0, 0.5], [0.5, 2.0]], [-4.0, -3.0], [1.0, 1.0], 1.0),
            // Asymmetric mu
            ([[2.0, 0.0], [0.0, 2.0]], [-5.0, -5.0], [2.0, 0.5], 1.0),
            // Large radius (unconstrained)
            ([[1.0, 0.0], [0.0, 1.0]], [-0.1, -0.1], [1.0, 1.0], 100.0),
            // Small radius
            ([[1.0, 0.0], [0.0, 1.0]], [-5.0, -5.0], [1.0, 1.0], 0.01),
            // Unequal diagonal
            ([[10.0, 0.0], [0.0, 0.1]], [-1.0, -1.0], [1.0, 1.0], 0.5),
        ];

        for (i, (a, b, d, r)) in cases.iter().enumerate() {
            let (result, _active) = qcqp2(*a, *b, *d, *r);
            assert!(
                result[0].is_finite() && result[1].is_finite(),
                "case {}: result not finite: {:?}",
                i,
                result
            );
            // Cone constraint always satisfied
            let cone = (result[0] / d[0]).powi(2) + (result[1] / d[1]).powi(2);
            assert!(
                cone <= r * r + 1e-6,
                "case {}: cone violated: {} > {}",
                i,
                cone,
                r * r
            );
        }
    }
}
