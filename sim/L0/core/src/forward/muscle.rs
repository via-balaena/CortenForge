//! Muscle force-length-velocity curves and parameter precomputation.
//!
//! Implements MuJoCo's piecewise-quadratic muscle model: active FL curve,
//! FV curve, passive force, and activation dynamics. Corresponds to
//! `engine_util_misc.c`.

// compute_muscle_params (impl Model) stays in crate::mujoco_pipeline until
// the monolith shim is deleted in Phase 12. It depends on mj_fwd_position,
// mj_crba, mj_jac_site, and mj_solve_sparse — all still routed through the
// monolith re-imports.

// ============================================================================
// MuJoCo Muscle Force-Length-Velocity Curves
// ============================================================================
// These implement MuJoCo's exact piecewise-quadratic muscle curves from
// engine_util_misc.c: mju_muscleGain (FL, FV) and mju_muscleBias (FP).

/// Active force-length curve: piecewise quadratic bump.
/// Returns 0 outside `[lmin, lmax]`, peak 1.0 at `L = 1.0`.
pub fn muscle_gain_length(length: f64, lmin: f64, lmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    if length < lmin || length > lmax {
        return 0.0;
    }
    let a = 0.5 * (lmin + 1.0); // midpoint of [lmin, 1]
    let b = 0.5 * (1.0 + lmax); // midpoint of [1, lmax]

    if length <= a {
        let x = (length - lmin) / (a - lmin).max(EPS);
        0.5 * x * x
    } else if length <= 1.0 {
        let x = (1.0 - length) / (1.0 - a).max(EPS);
        1.0 - 0.5 * x * x
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        1.0 - 0.5 * x * x
    } else {
        let x = (lmax - length) / (lmax - b).max(EPS);
        0.5 * x * x
    }
}

/// Force-velocity curve: piecewise quadratic.
/// `velocity` is normalized by `L0 * vmax` so `V = -1` is max shortening.
pub fn muscle_gain_velocity(velocity: f64, fvmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let y = fvmax - 1.0;
    if velocity <= -1.0 {
        0.0
    } else if velocity <= 0.0 {
        (velocity + 1.0) * (velocity + 1.0)
    } else if velocity <= y {
        fvmax - (y - velocity) * (y - velocity) / y.max(EPS)
    } else {
        fvmax
    }
}

/// Passive force curve: zero below `L = 1.0`, quadratic onset, linear beyond midpoint.
pub fn muscle_passive_force(length: f64, lmax: f64, fpmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    let b = 0.5 * (1.0 + lmax);
    if length <= 1.0 {
        0.0
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        fpmax * 0.5 * x * x
    } else {
        let x = (length - b) / (b - 1.0).max(EPS);
        fpmax * (0.5 + x)
    }
}

// ============================================================================
// Muscle Activation Dynamics
// ============================================================================

/// Quintic smoothstep (C2-continuous Hermite), matching MuJoCo's `mju_sigmoid`.
pub fn sigmoid(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x >= 1.0 {
        return 1.0;
    }
    x * x * x * (6.0 * x * x - 15.0 * x + 10.0)
}

/// Compute d(act)/dt for muscle activation dynamics.
///
/// Follows Millard et al. (2013) with activation-dependent time constants:
///   `tau_act_eff   = tau_act   * (0.5 + 1.5 * act)`
///   `tau_deact_eff = tau_deact / (0.5 + 1.5 * act)`
///
/// When `tausmooth > 0`, uses quintic sigmoid to blend between `tau_act` and
/// `tau_deact` instead of a hard switch at `ctrl == act`.
pub fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 3]) -> f64 {
    let ctrl_clamped = ctrl.clamp(0.0, 1.0);
    let act_clamped = act.clamp(0.0, 1.0);

    // Activation-dependent effective time constants (Millard et al. 2013)
    let tau_act = dynprm[0] * (0.5 + 1.5 * act_clamped);
    let tau_deact = dynprm[1] / (0.5 + 1.5 * act_clamped);
    let tausmooth = dynprm[2];

    let dctrl = ctrl_clamped - act;

    // Select time constant
    let tau = if tausmooth < 1e-10 {
        // Hard switch
        if dctrl > 0.0 { tau_act } else { tau_deact }
    } else {
        // Smooth blending via quintic sigmoid
        tau_deact + (tau_act - tau_deact) * sigmoid(dctrl / tausmooth + 0.5)
    };

    dctrl / tau.max(1e-10)
}

// compute_muscle_params() stays in crate::mujoco_pipeline (impl Model block)
// until Phase 12 monolith deletion. It depends on mj_fwd_position, mj_crba,
// mj_jac_site, and mj_solve_sparse — all still routed through the monolith.
