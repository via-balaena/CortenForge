//! Constraint impedance and regularization computation.
//!
//! Computes per-row impedance (stiffness/damping from solref/solimp),
//! diagonal approximation, and regularization for the constraint system.
//! Corresponds to the impedance machinery in MuJoCo's
//! `engine_core_constraint.c` (§15.1).

use nalgebra::{DVector, UnitQuaternion, Vector3};

use crate::linalg::{cholesky_in_place, cholesky_solve_in_place};
use crate::types::{Data, Model};

// Canonical copy in dynamics/crba.rs. Retained here for non-CRBA uses
// (quaternion_to_axis_angle, constraint body mass lookup) until those
// functions are extracted in later phases.
const MIN_INERTIA_THRESHOLD: f64 = 1e-10;

/// Default solref parameters [timeconst, dampratio] (MuJoCo defaults).
///
/// timeconst = 0.02 seconds gives a 50 Hz natural frequency.
/// dampratio = 1.0 gives critical damping.
pub const DEFAULT_SOLREF: [f64; 2] = [0.02, 1.0];

/// Default solimp parameters [d0, d_width, width, midpoint, power] (MuJoCo defaults).
///
/// These control the constraint impedance profile:
/// - d0 = 0.9: Impedance at zero violation
/// - d_width = 0.95: Impedance at full violation width (endpoint)
/// - width = 0.001: Transition zone size (meters or radians)
/// - midpoint = 0.5: Midpoint of the sigmoid transition curve
/// - power = 2.0: Power of the sigmoid transition curve
pub const DEFAULT_SOLIMP: [f64; 5] = [0.9, 0.95, 0.001, 0.5, 2.0];

/// Minimum value to prevent division by zero (matches MuJoCo's mjMINVAL).
pub const MJ_MINVAL: f64 = 1e-15;

/// Compute impedance from solimp parameters and constraint violation (§15.1).
///
/// Maps violation distance through a configurable sigmoid curve defined by
/// solimp = [d0, d_width, width, midpoint, power]:
/// - At zero violation: returns d0
/// - At full width: returns d_width
/// - In between: smooth transition via power-law sigmoid
///
/// # Returns
/// Impedance value in (0, 1), clamped to [MIN_IMPEDANCE, MAX_IMPEDANCE].
#[inline]
pub fn compute_impedance(solimp: [f64; 5], violation: f64) -> f64 {
    // MuJoCo clamps impedance to [mjMINIMP, mjMAXIMP] = [0.0001, 0.9999]
    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;
    // Threshold for treating floating-point values as equal
    const EPS: f64 = 1e-12;
    // Minimum width to avoid division by zero (MuJoCo uses mjMINVAL ≈ 1e-15,
    // we use a wider margin since f64 precision is ~1e-16)
    const MIN_WIDTH: f64 = 1e-10;

    debug_assert!(
        violation >= 0.0,
        "compute_impedance expects non-negative violation, got {violation}"
    );

    // Clamp inputs to valid ranges, matching MuJoCo's getsolparam():
    //   d0, d_width, midpoint ∈ [mjMINIMP, mjMAXIMP]
    //   width ≥ 0
    //   power ≥ 1
    let d0 = solimp[0].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let d_width = solimp[1].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let width = solimp[2].max(0.0);
    let midpoint = solimp[3].clamp(MJ_MIN_IMP, MJ_MAX_IMP);
    let power = solimp[4].max(1.0);

    // Flat function: d0 ≈ d_width or width is negligible
    if (d0 - d_width).abs() < EPS || width <= MIN_WIDTH {
        return 0.5 * (d0 + d_width);
    }

    // Normalized position: x = violation / width, clamped to [0, 1]
    let x = (violation / width).min(1.0);

    // Saturated at full width — return d_width
    if x >= 1.0 {
        return d_width;
    }

    // At zero violation — return d0
    if x == 0.0 {
        return d0;
    }

    // Compute sigmoid y(x) ∈ [0, 1]
    let y = if (power - 1.0).abs() < EPS {
        // Linear case (power ≈ 1): y = x regardless of midpoint
        x
    } else if x <= midpoint {
        // Lower half: y = a * x^p where a = 1 / midpoint^(p-1)
        let a = 1.0 / midpoint.powf(power - 1.0);
        a * x.powf(power)
    } else {
        // Upper half: y = 1 - b * (1-x)^p where b = 1 / (1-midpoint)^(p-1)
        let b = 1.0 / (1.0 - midpoint).powf(power - 1.0);
        1.0 - b * (1.0 - x).powf(power)
    };

    // Interpolate: d = d0 + y * (d_width - d0)
    d0 + y * (d_width - d0)
}

/// Convert a quaternion error to axis-angle representation.
///
/// For a unit quaternion `q = (w, x, y, z) = (cos(θ/2), sin(θ/2) * axis)`,
/// returns `θ * axis` as a Vector3.
///
/// This is more accurate than the small-angle approximation `2 * [x, y, z]`
/// which has ~10% error at 90° and ~36% error at 180°.
///
/// # Arguments
/// * `quat` - A unit quaternion representing the rotation error
///
/// # Returns
/// Axis-angle representation as `angle * axis` (Vector3)
#[inline]
pub fn quaternion_to_axis_angle(quat: &UnitQuaternion<f64>) -> Vector3<f64> {
    let q = quat.quaternion();
    let (qw, qx, qy, qz) = (q.w, q.i, q.j, q.k);

    // Handle identity quaternion (no rotation)
    let sin_half_angle_sq = qx * qx + qy * qy + qz * qz;
    if sin_half_angle_sq < MIN_INERTIA_THRESHOLD {
        return Vector3::zeros();
    }

    let sin_half_angle = sin_half_angle_sq.sqrt();

    // Compute full angle: θ = 2 * atan2(||xyz||, w)
    // This handles all cases including w < 0 (angle > π)
    let angle = 2.0 * sin_half_angle.atan2(qw);

    // Axis is normalized [x, y, z] / ||xyz||
    // Result is angle * axis
    Vector3::new(qx, qy, qz) * (angle / sin_half_angle)
}

/// Compute KBIP stiffness K and damping B from solref parameters (§15.1).
///
/// Returns `(K, B)` where:
/// - Standard mode (solref[0] > 0):
///   - K = 1/(dmax²·timeconst²·dampratio²)
///   - B = 2/(dmax·timeconst)          when dampratio > 0
///   - B = -dampratio/dmax             when dampratio ≤ 0
/// - Direct mode (solref[0] ≤ 0): K = -solref[0]/dmax², B = -solref[1]/dmax
///
/// `dmax` = solimp[1] (clamped to [mjMINIMP, mjMAXIMP]).
///
/// S4.15: When `DISABLE_REFSAFE` is NOT set (default), standard-mode
/// `solref[0]` is clamped to `max(solref[0], 2 * timestep)`. This prevents
/// excessively stiff constraints that would require sub-timestep response.
///
/// Matches MuJoCo's `mj_makeImpedance` in `engine_core_constraint.c`.
pub fn compute_kbip(model: &Model, solref: [f64; 2], solimp: [f64; 5]) -> (f64, f64) {
    use crate::types::DISABLE_REFSAFE;
    use crate::types::flags::disabled;

    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;

    let dmax = solimp[1].clamp(MJ_MIN_IMP, MJ_MAX_IMP);

    if solref[0] > 0.0 {
        // Standard mode: solref = [timeconst, dampratio]
        // S4.15: Clamp timeconst to 2*timestep unless refsafe is disabled.
        let timeconst = if disabled(model, DISABLE_REFSAFE) {
            solref[0]
        } else {
            solref[0].max(2.0 * model.timestep)
        };
        let dampratio = solref[1];

        let k = 1.0 / (dmax * dmax * timeconst * timeconst * dampratio * dampratio).max(MJ_MINVAL);
        let b = if dampratio > 0.0 {
            2.0 / (dmax * timeconst).max(MJ_MINVAL)
        } else {
            -dampratio / dmax.max(MJ_MINVAL)
        };
        (k, b)
    } else {
        // Direct mode: solref = [-stiffness, -damping]
        let k = -solref[0] / (dmax * dmax).max(MJ_MINVAL);
        let b = -solref[1] / dmax.max(MJ_MINVAL);
        (k, b)
    }
}

/// Compute reference acceleration aref for a single constraint row (§15.1).
///
/// `aref = -B * vel - K * imp * (pos - margin)`
///
/// This is the MuJoCo KBIP convention where impedance multiplies only the
/// position term, NOT the velocity term.
#[inline]
pub fn compute_aref(k: f64, b: f64, imp: f64, pos: f64, margin: f64, vel: f64) -> f64 {
    -b * vel - k * imp * (pos - margin)
}

/// Normalize a quaternion [w, x, y, z]. Returns identity if norm < 1e-10.
/// Matches MuJoCo's mju_normalize4 and our normalize_quaternion() convention.
#[inline]
pub fn normalize_quat4(q: [f64; 4]) -> [f64; 4] {
    let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if norm > 1e-10 {
        [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm]
    } else {
        [1.0, 0.0, 0.0, 0.0] // identity
    }
}

/// Compute ball joint limit quantities from a unit quaternion [w, x, y, z].
/// Returns `(unit_dir, angle)` where:
///   - `angle = |theta| >= 0` (unsigned rotation magnitude)
///   - `unit_dir = sign(theta) * axis` (Jacobian direction)
///
/// When `angle < 1e-10` (near-identity), returns `(Vector3::z(), 0.0)`.
/// The unit_dir value is arbitrary in this case — the caller must not use it
/// because `dist = limit - 0 > 0` means no constraint is instantiated.
///
/// Matches MuJoCo's `mju_quat2Vel(angleAxis, q, 1)` followed by
/// `value = mju_normalize3(angleAxis)`.
pub fn ball_limit_axis_angle(q: [f64; 4]) -> (Vector3<f64>, f64) {
    // Step 1: mju_quat2Vel with dt=1
    let v = Vector3::new(q[1], q[2], q[3]);
    let sin_half = v.norm();

    // Guard: near-identity → angle ≈ 0, axis undefined.
    if sin_half < 1e-10 {
        return (Vector3::z(), 0.0);
    }

    let axis = v / sin_half; // unit rotation axis
    let mut theta = 2.0 * sin_half.atan2(q[0]);
    if theta > std::f64::consts::PI {
        theta -= 2.0 * std::f64::consts::PI;
    }

    // Step 2: |theta * axis| = |theta|, direction = sign(theta) * axis.
    let angle = theta.abs();
    let unit_dir = if theta >= 0.0 { axis } else { -axis };

    (unit_dir, angle)
}

/// Compute exact diagonal approximation of A = J·M⁻¹·J^T for one row (§15.12 step 2).
///
/// Solves M·w = J_i^T via the sparse LDL factorization, then computes
/// diagApprox_i = J_i · w (one dot product). O(nv) per row.
///
/// Falls back to dense Cholesky if sparse factorization is not valid.
pub fn compute_diag_approx_exact(j_row: &[f64], nv: usize, model: &Model, data: &Data) -> f64 {
    // Solve M * w = J_row^T
    let mut w = DVector::zeros(nv);
    for i in 0..nv {
        w[i] = j_row[i];
    }

    if data.qLD_valid {
        // Use sparse LDL: forward/back substitution
        mj_solve_sparse_vec(model, data, &mut w);
    } else {
        // Fallback: use dense M and solve via Cholesky
        // Build a copy of qM and factor it
        let mut m_copy = data.qM.clone();
        // Cholesky failure means M is singular; w will be approximate.
        // The Newton solver detects and handles this via PGS fallback.
        if cholesky_in_place(&mut m_copy).is_err() {
            return MJ_MINVAL;
        }
        cholesky_solve_in_place(&m_copy, &mut w);
    }

    // diagApprox = J_row · w = J_row · M⁻¹ · J_row^T
    let mut result = 0.0;
    for i in 0..nv {
        result += j_row[i] * w[i];
    }
    result.max(MJ_MINVAL)
}

/// Solve M·x = b using the sparse LDL factorization (in-place).
///
/// Forward substitution: L^T · z = b, then D · y = z, then L · x = y.
/// Matches the tree-sparse structure from mj_factor_sparse.
///
/// `rownnz[i]` includes diagonal (last element); off-diagonal count = `rownnz[i] - 1`.
pub fn mj_solve_sparse_vec(model: &Model, data: &Data, x: &mut DVector<f64>) {
    let nv = model.nv;
    let (rowadr, rownnz, colind) = model.qld_csr();

    // Forward substitution: solve L^T * z = x (L is unit lower triangular)
    // Process from leaves to root (high index to low)
    // Zero-skip + diagonal-only skip matching MuJoCo's mj_solveLD.
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
            x[colind[start + k]] -= data.qLD_data[start + k] * xi;
        }
    }

    // Diagonal solve: z = z * D^-1 (multiply by precomputed inverse, matching MuJoCo).
    for i in 0..nv {
        x[i] *= data.qLD_diag_inv[i];
    }

    // Back substitution: solve L * x = z
    // Process from root to leaves (low index to high)
    // Diagonal-only skip: rownnz == 1 means no off-diagonals.
    for i in 0..nv {
        let nnz_offdiag = rownnz[i] - 1;
        if nnz_offdiag == 0 {
            continue;
        }
        let start = rowadr[i];
        for k in 0..nnz_offdiag {
            x[i] -= data.qLD_data[start + k] * x[colind[start + k]];
        }
    }
}

/// Compute regularization R and constraint stiffness D from impedance and diagApprox (§15.1).
///
/// R = max(mjMINVAL, (1 - imp) / imp * diagApprox)
/// D = 1 / R
#[inline]
pub fn compute_regularization(imp: f64, diag_approx: f64) -> (f64, f64) {
    let r = ((1.0 - imp) / imp * diag_approx).max(MJ_MINVAL);
    let d = 1.0 / r;
    (r, d)
}

// ==========================================================================
// Tests — moved from monolith (Phase 12)
// ==========================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod impedance_tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Default solimp: [d0=0.9, d_width=0.95, width=0.001, midpoint=0.5, power=2.0]
    const DEFAULT: [f64; 5] = DEFAULT_SOLIMP;

    // ========================================================================
    // compute_impedance unit tests
    // ========================================================================

    #[test]
    fn test_impedance_at_zero_violation() {
        // At zero violation, impedance should be d0
        let d = compute_impedance(DEFAULT, 0.0);
        assert_relative_eq!(d, 0.9, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_at_full_width() {
        // At violation >= width, impedance should be d_width
        let d = compute_impedance(DEFAULT, 0.001);
        assert_relative_eq!(d, 0.95, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_beyond_width() {
        // Beyond width, impedance should saturate at d_width
        let d = compute_impedance(DEFAULT, 0.01);
        assert_relative_eq!(d, 0.95, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_at_midpoint() {
        // At midpoint of transition, impedance should be between d0 and d_width.
        // For power=2 and midpoint=0.5:
        //   x = 0.5*width / width = 0.5
        //   y(0.5) = a * 0.5^2 where a = 1/0.5^(2-1) = 2
        //   y = 2 * 0.25 = 0.5
        //   d = 0.9 + 0.5 * (0.95 - 0.9) = 0.925
        let d = compute_impedance(DEFAULT, 0.0005);
        assert_relative_eq!(d, 0.925, epsilon = 1e-3);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "non-negative violation")]
    fn test_impedance_rejects_negative_violation() {
        // Negative violation is a caller bug — debug_assert catches it
        compute_impedance(DEFAULT, -0.0005);
    }

    #[test]
    fn test_impedance_flat_when_d0_equals_dwidth() {
        // When d0 == d_width, impedance is constant
        let solimp = [0.9, 0.9, 0.001, 0.5, 2.0];
        let d_zero = compute_impedance(solimp, 0.0);
        let d_mid = compute_impedance(solimp, 0.0005);
        let d_full = compute_impedance(solimp, 0.001);
        assert_relative_eq!(d_zero, 0.9, epsilon = 1e-4);
        assert_relative_eq!(d_mid, 0.9, epsilon = 1e-4);
        assert_relative_eq!(d_full, 0.9, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_linear_power() {
        // With power=1, transition should be linear
        let solimp = [0.5, 1.0, 0.01, 0.5, 1.0];
        let d_quarter = compute_impedance(solimp, 0.0025);
        // x = 0.25, y = 0.25 (linear), d = 0.5 + 0.25*0.5 = 0.625
        assert_relative_eq!(d_quarter, 0.625, epsilon = 1e-3);

        let d_half = compute_impedance(solimp, 0.005);
        // x = 0.5, y = 0.5 (linear), d = 0.5 + 0.5*0.5 = 0.75
        assert_relative_eq!(d_half, 0.75, epsilon = 1e-3);
    }

    #[test]
    fn test_impedance_clamped_to_valid_range() {
        // Impedance should be clamped to (0, 1) even with extreme solimp values
        let solimp_low = [0.0, 0.0, 0.001, 0.5, 2.0];
        let d = compute_impedance(solimp_low, 0.0);
        assert!(d >= 0.0001, "Impedance should be at least MIN_IMPEDANCE");

        let solimp_high = [1.0, 1.0, 0.001, 0.5, 2.0];
        let d = compute_impedance(solimp_high, 0.0);
        assert!(d <= 0.9999, "Impedance should be at most MAX_IMPEDANCE");
    }

    #[test]
    fn test_impedance_monotonic_increasing() {
        // With d_width > d0, impedance should increase with violation
        let solimp = [0.5, 0.95, 0.01, 0.5, 2.0];
        let mut prev = compute_impedance(solimp, 0.0);
        for i in 1..=10 {
            let pos = f64::from(i) * 0.001;
            let d = compute_impedance(solimp, pos);
            assert!(
                d >= prev - 1e-10,
                "Impedance should be monotonically increasing: d({})={} < d({})={}",
                pos - 0.001,
                prev,
                pos,
                d
            );
            prev = d;
        }
    }

    #[test]
    fn test_impedance_zero_width() {
        // Zero width should return average of d0 and d_width
        let solimp = [0.5, 0.9, 0.0, 0.5, 2.0];
        let d = compute_impedance(solimp, 0.1);
        assert_relative_eq!(d, 0.7, epsilon = 1e-4);
    }

    #[test]
    fn test_impedance_extreme_midpoint_no_nan() {
        // midpoint=0 would cause division by zero without input clamping.
        // Clamped to 0.0001, so this must produce a finite result.
        let solimp = [0.5, 0.9, 0.01, 0.0, 3.0];
        let d = compute_impedance(solimp, 0.005);
        assert!(d.is_finite(), "midpoint=0 should not produce NaN, got {d}");
        assert!(d > 0.0 && d < 1.0);

        // midpoint=1 same issue on the upper branch
        let solimp = [0.5, 0.9, 0.01, 1.0, 3.0];
        let d = compute_impedance(solimp, 0.005);
        assert!(d.is_finite(), "midpoint=1 should not produce NaN, got {d}");
        assert!(d > 0.0 && d < 1.0);
    }

    #[test]
    fn test_impedance_power_below_one_clamped() {
        // power < 1 is invalid in MuJoCo (clamped to 1). Should behave as linear.
        let solimp = [0.5, 1.0, 0.01, 0.5, 0.5]; // power=0.5, clamped to 1.0
        let d = compute_impedance(solimp, 0.005);
        // With power clamped to 1 (linear): x=0.5, y=0.5, d = 0.5 + 0.5*0.5 = 0.75
        assert_relative_eq!(d, 0.75, epsilon = 1e-3);
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod ball_limit_tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    /// Build quaternion [w, x, y, z] for rotation of `angle_deg` degrees about `axis`.
    fn quat_from_axis_angle_deg(axis: [f64; 3], angle_deg: f64) -> [f64; 4] {
        let half = (angle_deg / 2.0_f64).to_radians();
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let s = half.sin() / norm;
        [half.cos(), axis[0] * s, axis[1] * s, axis[2] * s]
    }

    #[test]
    fn test_ball_limit_axis_angle_90deg_about_z() {
        // 90 deg rotation about Z: q = (cos(45 deg), 0, 0, sin(45 deg))
        let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 90.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, PI / 2.0, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.z, 1.0, epsilon = 1e-10); // +Z (theta > 0)
        assert!(unit_dir.x.abs() < 1e-10);
        assert!(unit_dir.y.abs() < 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_50deg_about_oblique() {
        // 50 deg about axis (0.6, 0.8, 0) -- matches acceptance criterion 8
        let q = quat_from_axis_angle_deg([0.6, 0.8, 0.0], 50.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 50.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(unit_dir.x, 0.6, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.y, 0.8, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_200deg_wraps() {
        // 200 deg about Z -> wraps to theta = -160 deg, angle = 160 deg
        let q = quat_from_axis_angle_deg([0.0, 0.0, 1.0], 200.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 160.0_f64.to_radians(), epsilon = 1e-10);
        // theta < 0 after wrap, so unit_dir = sign(-) * z = -z
        assert_relative_eq!(unit_dir.z, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_identity() {
        let q = [1.0, 0.0, 0.0, 0.0];
        let (_, angle) = ball_limit_axis_angle(q);
        assert!(angle < 1e-10, "identity should have zero rotation angle");
        // unit_dir is arbitrary (z-axis default) -- not asserted
    }

    #[test]
    fn test_ball_limit_axis_angle_negative_quat() {
        // -q represents the same rotation as q. Verify identical output.
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 60.0);
        let neg_q = [-q[0], -q[1], -q[2], -q[3]];
        let (dir1, angle1) = ball_limit_axis_angle(q);
        let (dir2, angle2) = ball_limit_axis_angle(neg_q);
        assert_relative_eq!(angle1, angle2, epsilon = 1e-10);
        assert_relative_eq!(dir1, dir2, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_exactly_180deg() {
        // Boundary case: exactly pi rotation
        let q = quat_from_axis_angle_deg([0.0, 1.0, 0.0], 180.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, PI, epsilon = 1e-10);
        assert_relative_eq!(unit_dir.y, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_exact_boundary() {
        // Rotation angle exactly equals a typical limit -- verify angle is precise
        let q = quat_from_axis_angle_deg([1.0, 0.0, 0.0], 45.0);
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert_relative_eq!(angle, 45.0_f64.to_radians(), epsilon = 1e-10);
        assert_relative_eq!(unit_dir.x, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_ball_limit_axis_angle_near_zero_quaternion() {
        // A quaternion with near-zero norm (degenerate) should be caught by
        // normalize_quat4 (returns identity) before reaching ball_limit_axis_angle.
        // But ball_limit_axis_angle itself should also handle near-zero sin_half
        // gracefully -- returning angle = 0 and an arbitrary direction.
        let q = [1.0, 1e-15, 1e-15, 1e-15]; // Nearly identity, sin_half ~ 1.7e-15
        let (_, angle) = ball_limit_axis_angle(q);
        assert!(
            angle < 1e-10,
            "near-zero sin_half should produce angle ~ 0: got {angle}"
        );
        // unit_dir is arbitrary (z-axis default) -- not asserted
    }
}
