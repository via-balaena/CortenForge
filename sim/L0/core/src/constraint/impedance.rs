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
/// Matches MuJoCo's `mj_makeImpedance` in `engine_core_constraint.c`.
pub fn compute_kbip(solref: [f64; 2], solimp: [f64; 5]) -> (f64, f64) {
    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;

    let dmax = solimp[1].clamp(MJ_MIN_IMP, MJ_MAX_IMP);

    if solref[0] > 0.0 {
        // Standard mode: solref = [timeconst, dampratio]
        let timeconst = solref[0];
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
