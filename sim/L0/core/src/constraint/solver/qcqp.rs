//! QCQP solvers for friction cone projection.
//!
//! Matches MuJoCo's `mju_QCQP2/3()` in `engine_util_solve.c` and
//! `mju_QCQP()` for N-dimensional problems.
//!
//! All functions solve: min 0.5*x'*A*x + b'*x  s.t.  Σ(x_i/d_i)² ≤ r²
//! Returns (result, active) where active = constraint was projected to boundary.

/// Convergence tolerance for QCQP Newton iteration.
/// Matches MuJoCo's hardcoded `1e-10` threshold.
const QCQP_TOL: f64 = 1e-10;

/// Maximum Newton iterations. Matches MuJoCo.
const QCQP_MAX_ITER: usize = 20;

/// 2D QCQP solver (condim=3, 2 friction DOFs).
///
/// Matches `mju_QCQP2` in `engine_util_solve.c`.
///
/// `a`: 2×2 Delassus subblock (row-major as [[f64; 2]; 2]).
/// `b`: 2-element bias vector.
/// `d`: 2-element friction coefficients (mu).
/// `r`: cone radius (normal force).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
pub fn qcqp2(a: [[f64; 2]; 2], b: [f64; 2], d: [f64; 2], r: f64) -> ([f64; 2], bool) {
    // Scale to unit sphere
    let bs = [b[0] * d[0], b[1] * d[1]];
    let a_s = [
        [a[0][0] * d[0] * d[0], a[0][1] * d[0] * d[1]],
        [a[1][0] * d[1] * d[0], a[1][1] * d[1] * d[1]],
    ];
    let r2 = r * r;

    // Newton iteration on dual λ
    let mut lam = 0.0_f64;
    let mut v0 = 0.0_f64;
    let mut v1 = 0.0_f64;

    for _ in 0..QCQP_MAX_ITER {
        let m00 = a_s[0][0] + lam;
        let m11 = a_s[1][1] + lam;
        let m01 = a_s[0][1];
        let det = m00 * m11 - m01 * m01;
        if det < QCQP_TOL {
            return ([0.0, 0.0], false);
        }
        let idet = 1.0 / det;

        // y = -(A_s + λI)⁻¹ * b_s
        v0 = -(m11 * bs[0] - m01 * bs[1]) * idet;
        v1 = -(-m01 * bs[0] + m00 * bs[1]) * idet;

        let val = v0 * v0 + v1 * v1 - r2;
        if val < QCQP_TOL {
            break;
        }

        // φ'(λ) = -2 * y' * (A_s+λI)⁻¹ * y
        let pv0 = (m11 * v0 - m01 * v1) * idet;
        let pv1 = (-m01 * v0 + m00 * v1) * idet;
        let deriv = -2.0 * (v0 * pv0 + v1 * pv1);

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    // Unscale: x_i = y_i * d_i
    ([v0 * d[0], v1 * d[1]], lam != 0.0)
}

/// 3D QCQP solver (condim=4, 3 friction DOFs).
///
/// Matches `mju_QCQP3` in `engine_util_solve.c`. Uses 3×3 cofactor inverse.
#[allow(clippy::many_single_char_names)]
pub fn qcqp3(a: [[f64; 3]; 3], b: [f64; 3], d: [f64; 3], r: f64) -> ([f64; 3], bool) {
    // Scale to unit sphere
    let mut bs = [0.0_f64; 3];
    let mut a_s = [[0.0_f64; 3]; 3];
    for i in 0..3 {
        bs[i] = b[i] * d[i];
        for j in 0..3 {
            a_s[i][j] = a[i][j] * d[i] * d[j];
        }
    }
    let r2 = r * r;

    let mut lam = 0.0_f64;
    let mut v = [0.0_f64; 3];

    for _ in 0..QCQP_MAX_ITER {
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
        if det < QCQP_TOL {
            return ([0.0, 0.0, 0.0], false);
        }
        let idet = 1.0 / det;

        let cof10 = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]);
        let cof11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let cof12 = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]);
        let cof20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let cof21 = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]);
        let cof22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];

        // y = -(M⁻¹) * b_s  (cofactor inverse is transposed)
        v[0] = -(cof00 * bs[0] + cof10 * bs[1] + cof20 * bs[2]) * idet;
        v[1] = -(cof01 * bs[0] + cof11 * bs[1] + cof21 * bs[2]) * idet;
        v[2] = -(cof02 * bs[0] + cof12 * bs[1] + cof22 * bs[2]) * idet;

        let val = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - r2;
        if val < QCQP_TOL {
            break;
        }

        // φ'(λ) = -2 * y' * M⁻¹ * y
        let pv0 = (cof00 * v[0] + cof10 * v[1] + cof20 * v[2]) * idet;
        let pv1 = (cof01 * v[0] + cof11 * v[1] + cof21 * v[2]) * idet;
        let pv2 = (cof02 * v[0] + cof12 * v[1] + cof22 * v[2]) * idet;
        let deriv = -2.0 * (v[0] * pv0 + v[1] * pv1 + v[2] * pv2);

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    ([v[0] * d[0], v[1] * d[1], v[2] * d[2]], lam != 0.0)
}

/// N-dimensional QCQP solver (condim=6, up to 5 friction DOFs).
///
/// Matches `mju_QCQP` in `engine_util_solve.c`. Uses Cholesky factorization.
///
/// `a`: n×n Delassus subblock (flat row-major, length n*n).
/// `b`: n-element bias vector.
/// `d`: n-element friction coefficients.
/// `r`: cone radius.
/// `n`: dimension (number of friction DOFs).
#[allow(clippy::many_single_char_names)]
pub fn qcqp_nd(a: &[f64], b: &[f64], d: &[f64], r: f64, n: usize) -> (Vec<f64>, bool) {
    // Scale to unit sphere
    let mut bs = vec![0.0_f64; n];
    let mut a_s = vec![0.0_f64; n * n];
    for i in 0..n {
        bs[i] = b[i] * d[i];
        for j in 0..n {
            a_s[i * n + j] = a[i * n + j] * d[i] * d[j];
        }
    }
    let r2 = r * r;

    let mut lam = 0.0_f64;
    let mut v = vec![0.0_f64; n];

    for _ in 0..QCQP_MAX_ITER {
        // M = A_s + λI
        let mut m_data = a_s.clone();
        for i in 0..n {
            m_data[i * n + i] += lam;
        }

        // Cholesky factorization: M = L * L^T
        let mut l = vec![0.0_f64; n * n];
        let mut spd = true;
        for i in 0..n {
            for j in 0..=i {
                let mut sum = m_data[i * n + j];
                for k in 0..j {
                    sum -= l[i * n + k] * l[j * n + k];
                }
                if i == j {
                    if sum < QCQP_TOL {
                        spd = false;
                        break;
                    }
                    l[i * n + j] = sum.sqrt();
                } else {
                    l[i * n + j] = sum / l[j * n + j];
                }
            }
            if !spd {
                break;
            }
        }
        if !spd {
            return (vec![0.0; n], false);
        }

        // Solve L * z = -b_s (forward substitution)
        let mut z = vec![0.0_f64; n];
        for i in 0..n {
            let mut sum = -bs[i];
            for k in 0..i {
                sum -= l[i * n + k] * z[k];
            }
            z[i] = sum / l[i * n + i];
        }

        // Solve L^T * y = z (back substitution)
        for i in (0..n).rev() {
            let mut sum = z[i];
            for k in (i + 1)..n {
                sum -= l[k * n + i] * v[k];
            }
            v[i] = sum / l[i * n + i];
        }

        let val: f64 = v.iter().map(|yi| yi * yi).sum::<f64>() - r2;
        if val < QCQP_TOL {
            break;
        }

        // P*v for derivative: solve L*L^T * pv = v
        let mut pz = vec![0.0_f64; n];
        for i in 0..n {
            let mut sum = v[i];
            for k in 0..i {
                sum -= l[i * n + k] * pz[k];
            }
            pz[i] = sum / l[i * n + i];
        }
        let mut pv = vec![0.0_f64; n];
        for i in (0..n).rev() {
            let mut sum = pz[i];
            for k in (i + 1)..n {
                sum -= l[k * n + i] * pv[k];
            }
            pv[i] = sum / l[i * n + i];
        }

        let deriv: f64 = -2.0 * v.iter().zip(pv.iter()).map(|(vi, pi)| vi * pi).sum::<f64>();

        let delta = -val / deriv;
        if delta < QCQP_TOL {
            break;
        }
        lam += delta;
    }

    // Unscale
    let result: Vec<f64> = v.iter().zip(d.iter()).map(|(yi, di)| yi * di).collect();
    (result, lam != 0.0)
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
