//! Constraint impedance and regularization computation.
//!
//! Computes per-row impedance (stiffness/damping from solref/solimp),
//! diagonal approximation, and regularization for the constraint system.
//! Corresponds to the impedance machinery in MuJoCo's
//! `engine_core_constraint.c` (§15.1).

use nalgebra::{DVector, Vector3};

use crate::dynamics::factor::mj_solve_m;
use crate::linalg::{cholesky_in_place, cholesky_solve_in_place, mju_max, mju_min};
use crate::types::{Data, Model};

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

/// Whether a `solref` or a `solreffriction` mixes MuJoCo's two formats,
/// standard (both values positive) and direct (neither): one value positive,
/// the other not (`getsolparam`, `engine_core_constraint.c:1335`, `:1346`).
fn is_mixed(reference: [f64; 2]) -> bool {
    (reference[0] > 0.0) != (reference[1] > 0.0)
}

/// A row's `solref` as MuJoCo's `getsolparam` takes it
/// (`engine_core_constraint.c:1334-1338`): a mixed one is replaced by
/// [`DEFAULT_SOLREF`], with MuJoCo's warning. MuJoCo warns once per
/// constraint, this once per row.
pub fn checked_solref(solref: [f64; 2]) -> [f64; 2] {
    if is_mixed(solref) {
        log::warn!("mixed solref format, replacing with default");
        DEFAULT_SOLREF
    } else {
        solref
    }
}

/// A contact's `solreffriction` as `getsolparam` takes it
/// (`engine_core_constraint.c:1345-1349`): a mixed one is replaced by
/// (0, 0), its default, under which an elliptic contact's friction rows take
/// `solref` (`mj_makeImpedance`, `:1489-1491`).
pub fn checked_solreffriction(solreffriction: [f64; 2]) -> [f64; 2] {
    if is_mixed(solreffriction) {
        log::warn!("solreffriction values should have the same sign, replacing with default");
        [0.0, 0.0]
    } else {
        solreffriction
    }
}

/// `a` to the power `b`, as MuJoCo's `power()` (`engine_core_constraint.c:1400-1409`):
/// exactly `a` for 1 and `a*a` for 2 (the default), which `powf` need not give.
#[inline]
// MuJoCo compares the power exactly (`engine_core_constraint.c:1403-1405`).
#[allow(clippy::float_cmp)]
fn mj_power(a: f64, b: f64) -> f64 {
    if b == 1.0 {
        a
    } else if b == 2.0 {
        a * a
    } else {
        a.powf(b)
    }
}

/// Compute impedance from solimp parameters and constraint violation (§15.1).
///
/// Maps violation distance through a configurable sigmoid curve defined by
/// solimp = [d0, d_width, width, midpoint, power]:
/// - At zero violation: returns d0
/// - At full width: returns d_width
/// - In between: smooth transition via power-law sigmoid
///
/// The curve is flat when `d0 == d_width` or `width <= mjMINVAL`, and
/// linear when `power == 1`, each compared exactly, as MuJoCo's
/// `getimpedance` (`engine_core_constraint.c:1416`, `:1439`).
///
/// # Returns
/// Impedance value in (0, 1), clamped to [MIN_IMPEDANCE, MAX_IMPEDANCE].
#[inline]
// MuJoCo compares the flat and linear cases exactly.
#[allow(clippy::float_cmp)]
pub fn compute_impedance(solimp: [f64; 5], violation: f64) -> f64 {
    // MuJoCo clamps impedance to [mjMINIMP, mjMAXIMP] = [0.0001, 0.9999]
    const MJ_MIN_IMP: f64 = 0.0001;
    const MJ_MAX_IMP: f64 = 0.9999;

    // A NaN passes: a contact at a NaN distance (a geom turned by a NaN, as
    // MuJoCo's collision functions list it) reaches here, and its impedance
    // is NaN, as MuJoCo's `getimpedance` gives (ledger L209).
    debug_assert!(
        !(violation < 0.0),
        "compute_impedance expects non-negative violation, got {violation}"
    );

    // Clamp inputs to valid ranges as MuJoCo's getsolparam() does
    // (`engine_core_constraint.c:1357-1361`):
    //   d0, d_width, midpoint ∈ [mjMINIMP, mjMAXIMP]
    //   width ≥ 0
    //   power ≥ 1
    let clamp = |v: f64| mju_min(MJ_MAX_IMP, mju_max(MJ_MIN_IMP, v));
    let d0 = clamp(solimp[0]);
    let d_width = clamp(solimp[1]);
    let width = mju_max(0.0, solimp[2]);
    let midpoint = clamp(solimp[3]);
    let power = mju_max(1.0, solimp[4]);

    // Flat function
    if d0 == d_width || width <= MJ_MINVAL {
        return 0.5 * (d0 + d_width);
    }

    // Normalized position: x = violation / width, saturated at either end
    // (`getimpedance`, `engine_core_constraint.c:1422-1435`). A NaN is
    // neither and goes on to the curve, which gives NaN.
    let x = violation / width;
    if x >= 1.0 {
        return d_width;
    }
    if x <= 0.0 {
        return d0;
    }

    // Compute sigmoid y(x) ∈ [0, 1]
    let y = if power == 1.0 {
        // Linear case: y = x regardless of midpoint
        x
    } else if x <= midpoint {
        // Lower half: y = a * x^p where a = 1 / midpoint^(p-1)
        let a = 1.0 / mj_power(midpoint, power - 1.0);
        a * mj_power(x, power)
    } else {
        // Upper half: y = 1 - b * (1-x)^p where b = 1 / (1-midpoint)^(p-1)
        let b = 1.0 / mj_power(1.0 - midpoint, power - 1.0);
        1.0 - b * mj_power(1.0 - x, power)
    };

    // Interpolate: d = d0 + y * (d_width - d0)
    d0 + y * (d_width - d0)
}

/// Compute KBIP stiffness K and damping B from solref parameters (§15.1).
///
/// Returns `(K, B)` where:
/// - Standard mode (`solref[0] > 0`):
///   - K = 1/(dmax²·timeconst²·dampratio²)
///   - B = 2/(dmax·timeconst)          when dampratio > 0
///   - B = -dampratio/dmax             when dampratio ≤ 0
/// - Direct mode (`solref[0] ≤ 0`): `K = -solref[0]/dmax²`, `B = -solref[1]/dmax`
///
/// `dmax` = `solimp[1]` (clamped to `[mjMINIMP, mjMAXIMP]`).
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

        let k = 1.0
            / mju_max(
                MJ_MINVAL,
                dmax * dmax * timeconst * timeconst * dampratio * dampratio,
            );
        let b = if dampratio > 0.0 {
            2.0 / mju_max(MJ_MINVAL, dmax * timeconst)
        } else {
            -dampratio / mju_max(MJ_MINVAL, dmax)
        };
        (k, b)
    } else {
        // Direct mode: solref = [-stiffness, -damping]
        let k = -solref[0] / mju_max(MJ_MINVAL, dmax * dmax);
        let b = -solref[1] / mju_max(MJ_MINVAL, dmax);
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

/// A ball joint limit's direction and angle from a unit quaternion
/// `[w, x, y, z]`, as MuJoCo's: `mju_quat2Vel(angleAxis, q, 1)`, then
/// `value = mju_normalize3(angleAxis)` (`engine_core_constraint.c:824-828`).
/// Returns `(unit_dir, angle)`: `angle = |theta|`, and `unit_dir` the axis
/// times the sign of `theta`, the x axis below mjMINVAL (`mju_normalize3`).
pub fn ball_limit_axis_angle(q: [f64; 4]) -> (Vector3<f64>, f64) {
    let mut angle_axis = crate::quat::quat_to_vel(&q, 1.0);
    let angle = crate::quat::normalize3(&mut angle_axis);
    (Vector3::from(angle_axis), angle)
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
        mj_solve_m(model, data, w.as_mut_slice());
    } else {
        // Fallback: use dense M and solve via Cholesky
        // Build a copy of qM and factor it
        let mut m_copy = data.qM.clone();
        // Cholesky failure means M is singular: the floor below.
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

/// Body-weight diagonal approximation for row `row_offset` of equality
/// constraint `id`, as MuJoCo's `mj_diagApprox` (`engine_core_constraint.c:1093-1287`):
/// an O(1) approximation of `diag(J · M⁻¹ · J^T)` from the inverse weights at
/// `qpos0` instead of solving `M · w = J^T`. A connect's rows take its two
/// bodies' translational weights, a weld's first three rows those and its
/// last three their rotational ones, a joint or tendon equality its first
/// object's `dof_invweight0` or `tendon_invweight0` plus its second's when it
/// has one. No floor: MuJoCo floors only `R` (`mj_makeImpedance`).
#[inline]
pub fn compute_diag_approx_bodyweight(model: &Model, id: usize, row_offset: usize) -> f64 {
    use crate::types::EqualityType;
    if id >= model.neq {
        return MJ_MINVAL;
    }
    let body = |b: usize, comp: usize| model.body_invweight0.get(b).map_or(0.0, |w| w[comp]);
    let dof = |jnt: usize| {
        if jnt < model.njnt {
            model
                .dof_invweight0
                .get(model.jnt_dof_adr[jnt])
                .copied()
                .unwrap_or(0.0)
        } else {
            0.0
        }
    };
    let tendon = |t: usize| model.tendon_invweight0.get(t).copied().unwrap_or(0.0);
    let (obj1, obj2) = (model.eq_obj1id[id], model.eq_obj2id[id]);
    match model.eq_type[id] {
        EqualityType::Connect => body(obj1, 0) + body(obj2, 0),
        EqualityType::Weld => {
            let comp = usize::from(row_offset >= 3);
            body(obj1, comp) + body(obj2, comp)
        }
        EqualityType::Joint => {
            let w = dof(obj1);
            if obj2 == usize::MAX { w } else { w + dof(obj2) }
        }
        EqualityType::Tendon => {
            let w = tendon(obj1);
            if obj2 == usize::MAX {
                w
            } else {
                w + tendon(obj2)
            }
        }
        // Not reached: a flex row's diagonal approximation is its
        // edge's `flexedge_invweight0`, which
        // `assemble_flex_equality_rows` passes itself.
        EqualityType::Flex => MJ_MINVAL,
        // This crate's own equality (MuJoCo 3.5.0 has none): its geoms'
        // bodies' translational weights, as a connect's.
        EqualityType::Distance => {
            let geom_body = |g: usize| model.geom_body.get(g).copied().unwrap_or(0);
            body(geom_body(obj1), 0) + body(geom_body(obj2), 0)
        }
    }
}

/// Compute regularization R and constraint stiffness D from impedance and diagApprox (§15.1).
///
/// R = max(mjMINVAL, (1 - imp) · diagApprox / imp), in MuJoCo's order of
/// operations (`mj_makeImpedance`, `engine_core_constraint.c:1484`), its
/// `mju_max`: a NaN impedance gives a NaN R
/// D = 1 / R
#[inline]
pub fn compute_regularization(imp: f64, diag_approx: f64) -> (f64, f64) {
    let r = mju_max(MJ_MINVAL, (1.0 - imp) * diag_approx / imp);
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

    /// MuJoCo's `power()` returns `a*a` for 2 and `a` for 1
    /// (`engine_core_constraint.c:1400-1409`), which `pow` need not equal: at
    /// these two points, one on each branch of the sigmoid, libm's `pow(·, 2)`
    /// moves the impedance by one unit in the last place on macOS (Linux not
    /// measured). The values are MuJoCo's `getimpedance` arithmetic (`:1413-1461`).
    #[test]
    fn impedance_squares_as_mujoco_power() {
        // Opaque, as a model's solimp is: a constant power lets the compiler
        // fold `powf(x, 2.0)` into `x*x`.
        let solimp = std::hint::black_box([0.9, 0.95, 1.0, 0.5, 2.0]);
        for (violation, mujoco) in [
            (0.43254, 0.918_709_085_16),
            (0.50964, 0.925_954_707_039_999_9),
        ] {
            let imp = compute_impedance(solimp, violation);
            assert_eq!(
                imp.to_bits(),
                f64::to_bits(mujoco),
                "at {violation}: {imp} against {mujoco}"
            );
        }
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "non-negative violation")]
    fn test_impedance_rejects_negative_violation() {
        // Negative violation is a caller bug — debug_assert catches it
        compute_impedance(DEFAULT, -0.0005);
    }

    #[test]
    fn test_impedance_admits_nan_violation() {
        // A contact at a NaN distance (a geom turned by a NaN) reaches here;
        // the debug check lets it pass, as MuJoCo's `getimpedance` has none,
        // and the impedance is NaN, as MuJoCo's (ledger L209)
        assert!(compute_impedance(DEFAULT, f64::NAN).is_nan());
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
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert!(angle < 1e-10, "identity should have zero rotation angle");
        // MuJoCo's `mju_normalize3` gives the x axis below mjMINVAL.
        assert_eq!(unit_dir, Vector3::x());
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
        // Nearly the identity, sin_half ~ 1.7e-15, above mjMINVAL: the angle is
        // tiny and the direction is the rotation's own axis, as MuJoCo's.
        let q = [1.0, 1e-15, 1e-15, 1e-15];
        let (unit_dir, angle) = ball_limit_axis_angle(q);
        assert!(
            angle < 1e-10,
            "near-zero sin_half should produce angle ~ 0: got {angle}"
        );
        assert_relative_eq!(unit_dir.x, 1.0 / 3.0_f64.sqrt(), epsilon = 1e-12);
        assert_relative_eq!(unit_dir.y, unit_dir.x);
        assert_relative_eq!(unit_dir.z, unit_dir.x);
    }
}
