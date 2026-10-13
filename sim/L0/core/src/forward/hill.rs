//! Hill-type muscle dynamics and force-length-velocity curves.
//!
//! Implements MuJoCo's piecewise-quadratic muscle model: active FL curve,
//! FV curve, passive force, and activation dynamics. Corresponds to
//! `engine_util_misc.c` (mju_muscleGain, mju_muscleBias).

use crate::constraint::impedance::MJ_MINVAL;

// ============================================================================
// MuJoCo Muscle Force-Length-Velocity Curves
// ============================================================================
// These implement MuJoCo's exact piecewise-quadratic muscle curves from
// engine_util_misc.c: mju_muscleGain (FL, FV) and mju_muscleBias (FP).

/// Active force-length curve: piecewise quadratic bump (MuJoCo
/// `mju_muscleGainLength`, `engine_util_misc.c:627-649`).
/// Returns 0 outside `[lmin, lmax]` (and for a NaN length), peak 1.0 at `L = 1.0`.
#[must_use]
pub fn muscle_gain_length(length: f64, lmin: f64, lmax: f64) -> f64 {
    if lmin <= length && length <= lmax {
        let a = 0.5 * (lmin + 1.0); // midpoint of [lmin, 1]
        let b = 0.5 * (1.0 + lmax); // midpoint of [1, lmax]

        return if length <= a {
            let x = (length - lmin) / (a - lmin).max(MJ_MINVAL);
            0.5 * x * x
        } else if length <= 1.0 {
            let x = (1.0 - length) / (1.0 - a).max(MJ_MINVAL);
            1.0 - 0.5 * x * x
        } else if length <= b {
            let x = (length - 1.0) / (b - 1.0).max(MJ_MINVAL);
            1.0 - 0.5 * x * x
        } else {
            let x = (lmax - length) / (lmax - b).max(MJ_MINVAL);
            0.5 * x * x
        };
    }
    0.0
}

/// Force-velocity curve: piecewise quadratic.
/// `velocity` is normalized by `L0 * vmax` so `V = -1` is max shortening.
#[must_use]
pub fn muscle_gain_velocity(velocity: f64, fvmax: f64) -> f64 {
    let y = fvmax - 1.0;
    if velocity <= -1.0 {
        0.0
    } else if velocity <= 0.0 {
        (velocity + 1.0) * (velocity + 1.0)
    } else if velocity <= y {
        fvmax - (y - velocity) * (y - velocity) / y.max(MJ_MINVAL)
    } else {
        fvmax
    }
}

/// Passive force curve: zero below `L = 1.0`, quadratic onset, linear beyond midpoint.
/// [`muscle_bias`] computes the same curve times the force, in MuJoCo's order.
#[must_use]
pub fn muscle_passive_force(length: f64, lmax: f64, fpmax: f64) -> f64 {
    let b = 0.5 * (1.0 + lmax);
    if length <= 1.0 {
        0.0
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(MJ_MINVAL);
        fpmax * 0.5 * x * x
    } else {
        let x = (length - b) / (b - 1.0).max(MJ_MINVAL);
        fpmax * (0.5 + x)
    }
}

/// A muscle's peak force: `prm[2]`, or `prm[3] / max(mjMINVAL, acc0)` where
/// that is negative.
///
/// MuJoCo leaves a negative force in the model and resolves it at each call
/// (`engine_util_misc.c:665-667`, `:708-710`). `prm` is the actuator's
/// `gainprm` for its gain and its `biasprm` for its bias.
#[must_use]
pub fn muscle_f0(prm: &[f64; 9], acc0: f64) -> f64 {
    if prm[2] < 0.0 {
        prm[3] / acc0.max(MJ_MINVAL)
    } else {
        prm[2]
    }
}

/// The optimal length `L0` of a muscle and its normalized length at `len`.
pub fn muscle_length(len: f64, lengthrange: (f64, f64), prm: &[f64; 9]) -> (f64, f64) {
    let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(MJ_MINVAL);
    (l0, prm[0] + (len - lengthrange.0) / l0.max(MJ_MINVAL))
}

/// A muscle's active force per unit input (MuJoCo `mju_muscleGain`,
/// `engine_util_misc.c:653-693`), `prm` its `gainprm`.
#[must_use]
pub fn muscle_gain(len: f64, vel: f64, lengthrange: (f64, f64), acc0: f64, prm: &[f64; 9]) -> f64 {
    let force = muscle_f0(prm, acc0);
    let (l0, length) = muscle_length(len, lengthrange, prm);
    let velocity = vel / (l0 * prm[6]).max(MJ_MINVAL);
    -force * muscle_gain_length(length, prm[4], prm[5]) * muscle_gain_velocity(velocity, prm[8])
}

/// A muscle's passive force (MuJoCo `mju_muscleBias`,
/// `engine_util_misc.c:697-729`), `prm` its `biasprm`.
#[must_use]
pub fn muscle_bias(len: f64, lengthrange: (f64, f64), acc0: f64, prm: &[f64; 9]) -> f64 {
    let force = muscle_f0(prm, acc0);
    let (_, length) = muscle_length(len, lengthrange, prm);
    let (lmax, fpmax) = (prm[5], prm[7]);
    let b = 0.5 * (1.0 + lmax);
    if length <= 1.0 {
        0.0
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(MJ_MINVAL);
        -force * fpmax * 0.5 * x * x
    } else {
        let x = (length - b) / (b - 1.0).max(MJ_MINVAL);
        -force * fpmax * (0.5 + x)
    }
}

// ============================================================================
// Muscle Activation Dynamics
// ============================================================================

/// Quintic smoothstep (C2-continuous Hermite): MuJoCo's `mju_sigmoid`
/// (`engine_util_misc.c:1888-1900`), in its arithmetic.
pub fn sigmoid(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x >= 1.0 {
        return 1.0;
    }
    x * x * x * (3.0 * x * (2.0 * x - 5.0) + 10.0)
}

/// Compute d(act)/dt for muscle activation dynamics.
///
/// Follows Millard et al. (2013) with activation-dependent time constants:
///   `tau_act_eff   = tau_act   * (0.5 + 1.5 * act)`
///   `tau_deact_eff = tau_deact / (0.5 + 1.5 * act)`
///
/// When `tausmooth` is at least mjMINVAL, uses quintic sigmoid to blend
/// between `tau_act` and `tau_deact` instead of a hard switch at
/// `ctrl == act`. MuJoCo's `mju_muscleDynamics` and
/// `mju_muscleDynamicsTimescale` (`engine_util_misc.c:733-770`).
#[must_use]
pub fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 10]) -> f64 {
    let ctrl_clamped = ctrl.clamp(0.0, 1.0);
    let act_clamped = act.clamp(0.0, 1.0);

    // Activation-dependent effective time constants (Millard et al. 2013)
    let tau_act = dynprm[0] * (0.5 + 1.5 * act_clamped);
    let tau_deact = dynprm[1] / (0.5 + 1.5 * act_clamped);
    let tausmooth = dynprm[2];

    let dctrl = ctrl_clamped - act;

    // Select time constant
    let tau = if tausmooth < MJ_MINVAL {
        // Hard switch
        if dctrl > 0.0 { tau_act } else { tau_deact }
    } else {
        // Smooth blending via quintic sigmoid, centred at dctrl = 0
        tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / tausmooth + 0.5)
    };

    dctrl / tau.max(MJ_MINVAL)
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant,
    clippy::uninlined_format_args,
    clippy::while_float
)]
mod tests {
    use super::*;
    use crate::forward::actuation::{hill_active_fl, hill_force_velocity, hill_passive_fl};

    /// A NaN length gives no active force: MuJoCo's length curve tests
    /// `lmin <= L && L <= lmax`, false for NaN, and `mju_muscleGain` returns
    /// -0.0 there (MuJoCo 3.5.0, measured).
    #[test]
    fn nan_length_has_no_active_force() {
        let prm = [0.75, 1.05, 300.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2];
        let gain = muscle_gain(f64::NAN, 0.1, (-1.0, 1.0), 1.0, &prm);
        assert_eq!(gain.to_bits(), (-0.0_f64).to_bits());
    }

    // ---- AC #3: FL curve shape ----
    #[test]
    fn test_fl_curve_shape() {
        // Peak at optimal length
        assert!((muscle_gain_length(1.0, 0.5, 1.6) - 1.0).abs() < 1e-10);
        // Zero at lmin
        assert!((muscle_gain_length(0.5, 0.5, 1.6)).abs() < 1e-10);
        // Zero at lmax
        assert!((muscle_gain_length(1.6, 0.5, 1.6)).abs() < 1e-10);
        // Zero outside range
        assert_eq!(muscle_gain_length(0.3, 0.5, 1.6), 0.0);
        assert_eq!(muscle_gain_length(1.7, 0.5, 1.6), 0.0);
        // Monotonically increasing from lmin to 1.0
        let fl_075 = muscle_gain_length(0.75, 0.5, 1.6);
        let fl_090 = muscle_gain_length(0.90, 0.5, 1.6);
        assert!(fl_075 > 0.0 && fl_075 < fl_090);
        assert!(fl_090 < 1.0);
    }

    // ---- AC #4: FV curve shape ----
    #[test]
    fn test_fv_curve_shape() {
        // Isometric (V = 0)
        assert!((muscle_gain_velocity(0.0, 1.2) - 1.0).abs() < 1e-10);
        // Max shortening (V = -1)
        assert!((muscle_gain_velocity(-1.0, 1.2)).abs() < 1e-10);
        // Eccentric plateau
        assert!((muscle_gain_velocity(0.2, 1.2) - 1.2).abs() < 1e-10);
        // Below max shortening
        assert_eq!(muscle_gain_velocity(-1.5, 1.2), 0.0);
    }

    // ---- AC #5: FP curve shape ----
    #[test]
    fn test_fp_curve_shape() {
        // No passive below optimal
        assert_eq!(muscle_passive_force(0.8, 1.6, 1.3), 0.0);
        assert_eq!(muscle_passive_force(1.0, 1.6, 1.3), 0.0);
        // At midpoint b = 0.5*(1+1.6) = 1.3, FP = fpmax * 0.5
        let fp_at_b = muscle_passive_force(1.3, 1.6, 1.3);
        assert!((fp_at_b - 1.3 * 0.5).abs() < 1e-10);
        // Above midpoint: linear
        let fp_above = muscle_passive_force(1.4, 1.6, 1.3);
        assert!(fp_above > fp_at_b);
    }

    // ---- AC #2: Activation asymmetry ----
    #[test]
    fn test_activation_asymmetry() {
        let dynprm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let dt = 0.001;

        // Time to reach 90% from 0 with ctrl=1
        let mut act_up: f64 = 0.0;
        let mut steps_up = 0;
        while act_up < 0.9 {
            let dact = muscle_activation_dynamics(1.0, act_up, &dynprm);
            act_up += dt * dact;
            act_up = act_up.clamp(0.0, 1.0);
            steps_up += 1;
            if steps_up > 10000 {
                break;
            }
        }

        // Time to reach 10% from 1 with ctrl=0
        let mut act_down: f64 = 1.0;
        let mut steps_down = 0;
        while act_down > 0.1 {
            let dact = muscle_activation_dynamics(0.0, act_down, &dynprm);
            act_down += dt * dact;
            act_down = act_down.clamp(0.0, 1.0);
            steps_down += 1;
            if steps_down > 10000 {
                break;
            }
        }

        assert!(
            steps_up < steps_down,
            "Activation should be faster than deactivation: {} steps up vs {} steps down",
            steps_up,
            steps_down
        );
    }

    // ---- Sigmoid unit tests ----
    #[test]
    fn test_sigmoid_boundaries() {
        assert_eq!(sigmoid(0.0), 0.0);
        assert_eq!(sigmoid(1.0), 1.0);
        assert_eq!(sigmoid(-0.5), 0.0);
        assert_eq!(sigmoid(1.5), 1.0);
        // Midpoint should be 0.5
        assert!((sigmoid(0.5) - 0.5).abs() < 1e-10);
    }

    // ---- Hill curve unit tests (from hill_muscle_tests) ----
    #[test]
    fn test_hill_active_fl_curve() {
        // Peak at optimal length
        assert!(
            (hill_active_fl(1.0) - 1.0).abs() < 1e-10,
            "FL(1.0) should be 1.0"
        );
        // Zero at boundaries
        assert_eq!(
            hill_active_fl(0.49),
            0.0,
            "FL(0.49) should be 0 (below lmin)"
        );
        assert_eq!(
            hill_active_fl(1.61),
            0.0,
            "FL(1.61) should be 0 (above lmax)"
        );
        // At boundaries
        assert!(hill_active_fl(0.5) > 0.0, "FL(0.5) should be > 0 (at lmin)");
        assert!(hill_active_fl(1.6) > 0.0, "FL(1.6) should be > 0 (at lmax)");
        // Monotonically increasing from 0.5 to 1.0
        assert!(hill_active_fl(0.7) < hill_active_fl(0.9));
        assert!(hill_active_fl(0.9) < hill_active_fl(1.0));
    }

    #[test]
    fn test_hill_force_velocity_curve() {
        // Isometric
        assert!(
            (hill_force_velocity(0.0) - 1.0).abs() < 1e-10,
            "FV(0) should be 1.0"
        );
        // Max shortening
        assert_eq!(hill_force_velocity(-1.0), 0.0, "FV(-1) should be 0");
        assert_eq!(hill_force_velocity(-1.5), 0.0, "FV(-1.5) should be 0");
        // Eccentric
        assert!(hill_force_velocity(0.5) > 1.0, "FV(0.5) should be > 1.0");
        // Concentric
        assert!(
            hill_force_velocity(-0.5) > 0.0 && hill_force_velocity(-0.5) < 1.0,
            "FV(-0.5) should be in (0, 1)"
        );
    }

    #[test]
    fn test_hill_passive_fl_curve() {
        // No passive force at or below optimal length
        assert_eq!(hill_passive_fl(0.9), 0.0, "FP(0.9) should be 0");
        assert_eq!(hill_passive_fl(1.0), 0.0, "FP(1.0) should be 0");
        // Positive passive force beyond optimal
        assert!(hill_passive_fl(1.5) > 0.0, "FP(1.5) should be > 0");
        // Monotonically increasing
        assert!(hill_passive_fl(1.3) < hill_passive_fl(1.5));
    }

    // ---- T1: Activation dynamics — hard switch (AC1) ----
    #[test]
    fn test_hill_activation_hard_switch() {
        let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.8, 0.3, &prm);
        // V1: act_clamped=0.3, tau_act=0.01×0.95=0.0095, dctrl=0.5>0 →
        // tau=0.0095, act_dot=0.5/0.0095=52.6315789...
        assert!(
            (act_dot - 52.631_578_947_368_42).abs() < 1e-10,
            "hard-switch act_dot = {act_dot}"
        );
    }

    // ---- T1 additional: V2-V6 boundary conditions ----
    #[test]
    fn test_hill_activation_deactivation() {
        // V2: dctrl < 0 → deactivation path
        let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.2, 0.6, &prm);
        // act_clamped=0.6, tau_deact=0.04/(0.5+0.9)=0.04/1.4, dctrl=-0.4
        let tau_deact = 0.04 / (0.5 + 1.5 * 0.6);
        let expected = -0.4 / tau_deact;
        assert!(
            (act_dot - expected).abs() < 1e-10,
            "deactivation act_dot = {act_dot}, expected {expected}"
        );
    }

    #[test]
    fn test_hill_activation_at_zero() {
        // V3: act = 0 boundary
        let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.5, 0.0, &prm);
        // act_clamped=0.0, tau_act=0.01×0.5=0.005, dctrl=0.5
        let expected = 0.5 / 0.005;
        assert!(
            (act_dot - expected).abs() < 1e-10,
            "act=0 act_dot = {act_dot}, expected {expected}"
        );
    }

    #[test]
    fn test_hill_activation_at_one() {
        // V4: act = 1 boundary
        let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.5, 1.0, &prm);
        // act_clamped=1.0, tau_deact=0.04/(0.5+1.5)=0.04/2.0=0.02, dctrl=-0.5
        let expected = -0.5 / 0.02;
        assert!(
            (act_dot - expected).abs() < 1e-10,
            "act=1 act_dot = {act_dot}, expected {expected}"
        );
    }

    #[test]
    fn test_hill_activation_ctrl_clamped() {
        // V6: ctrl > 1 is clamped
        let prm = [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(1.5, 0.5, &prm);
        // ctrl clamped to 1.0, so same as ctrl=1.0, act=0.5
        let act_dot_ref = muscle_activation_dynamics(1.0, 0.5, &prm);
        assert!(
            (act_dot - act_dot_ref).abs() < 1e-10,
            "clamped ctrl act_dot = {act_dot}, reference = {act_dot_ref}"
        );
    }

    // ---- T2: Activation dynamics — smooth blend (AC2) ----
    /// MuJoCo 3.5.0's `mju_muscleDynamics` on the same inputs, bit for bit:
    /// the sigmoid's argument is `dctrl / tausmooth + 0.5` (here 0.75).
    #[test]
    fn test_hill_activation_smooth_blend() {
        let prm = [0.01, 0.04, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.55, 0.5, &prm);
        assert_eq!(act_dot.to_bits(), 3.443_868_971_547_727_f64.to_bits());
        let prm = [0.01, 0.04, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.3, 0.7, &prm);
        assert_eq!(act_dot.to_bits(), (-15.499_999_999_999_996_f64).to_bits());
    }

    /// The floor on the smoothing width and on the time constant is mjMINVAL
    /// (1e-15), not 1e-10: MuJoCo 3.5.0's `mju_muscleDynamics`, bit for bit. A
    /// width of 1e-12 or 1e-14 blends and one of 1e-16 switches; a time
    /// constant of 1.25e-13 is used as it is and one of 5e-17 (`dynprm[0]`
    /// 1e-16 at activation 0) is raised to 1e-15.
    #[test]
    fn activation_floors_are_mjminval() {
        let prm = [0.01, 0.04, 1e-12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.5 + 1e-13, 0.5, &prm);
        assert_eq!(act_dot.to_bits(), 5.352_395_624_003_173e-12_f64.to_bits());
        let prm = [0.01, 0.04, 1e-14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.5 + 2e-15, 0.5, &prm);
        assert_eq!(act_dot.to_bits(), 1.274_150_622_079_462_5e-13_f64.to_bits());
        let prm = [0.01, 0.04, 1e-16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(1e-17, 0.0, &prm);
        assert_eq!(act_dot.to_bits(), 2e-15_f64.to_bits());
        let prm = [1e-13, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.55, 0.5, &prm);
        assert_eq!(act_dot.to_bits(), 400_000_000_000.000_37_f64.to_bits());
        let prm = [1e-16, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(1.0, 0.0, &prm);
        assert_eq!(act_dot.to_bits(), 999_999_999_999_999.9_f64.to_bits());
    }

    /// MuJoCo 3.5.0's `mju_sigmoid` at a point where `6x² − 15x + 10`
    /// rounds otherwise.
    #[test]
    fn sigmoid_is_mujocos() {
        assert_eq!(
            sigmoid(0.763_774_618_976_614).to_bits(),
            0.910_475_655_329_874_8_f64.to_bits()
        );
    }
}
