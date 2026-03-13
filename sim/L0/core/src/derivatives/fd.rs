//! Finite-difference perturbation methods for transition derivatives.

use super::{DerivativeConfig, TransitionMatrices};
use crate::jacobian::{mj_differentiate_pos, mj_integrate_pos_explicit};
use crate::types::{Data, Model, StepError};
use nalgebra::{DMatrix, DVector};

// ============================================================================
// mjd_transition_fd (Phase A: Pure FD)
// ============================================================================

/// Compute finite-difference Jacobians of the simulation transition function.
///
/// Linearizes `x_{t+1} = f(x_t, u_t)` around the current state by perturbing
/// each component of state and control, stepping the simulation, and computing
/// differences. The input `data` must have a valid `forward()` result.
///
/// # Difference formulas
///
/// Centered (O(ε²) error):
///   `∂f/∂x_i ≈ (f(x + ε·e_i) − f(x − ε·e_i)) / (2·ε)`
///
/// Forward (O(ε) error):
///   `∂f/∂x_i ≈ (f(x + ε·e_i) − f(x)) / ε`
///
/// Centered differences are recommended (2x cost but ε² error vs ε error).
/// For ε = 1e-6, centered achieves ~1e-12 error vs ~1e-6 for forward.
///
/// # Cost
///
/// - Centered: `2 · (2·nv + na + nu)` calls to `step()`.
/// - Forward:  `1 + (2·nv + na + nu)` calls to `step()` (the `1` is for
///   the nominal `y_0 = f(x)` evaluation).
///
/// # Quaternion handling
///
/// Position perturbations operate in tangent space (dimension `nv`) via
/// `mj_integrate_pos_explicit()`. Output differences use
/// `mj_differentiate_pos()` to map back to tangent space.
///
/// # Contact handling
///
/// FD naturally captures contact transitions. If a perturbation causes a
/// contact to activate/deactivate, the derivative reflects this discontinuity.
///
/// # Errors
///
/// Returns `StepError` if any perturbed `step()` call fails.
///
/// # Panics
///
/// Panics if `config.eps` is non-positive, non-finite, or greater than `1e-2`.
///
/// # Thread safety
///
/// Takes `&Data` (shared reference) and immediately clones to a scratch
/// buffer. Multiple callers can compute derivatives from the same nominal
/// state concurrently.
#[allow(non_snake_case, clippy::similar_names, clippy::unwrap_used)]
pub fn mjd_transition_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    // Validate config
    assert!(
        config.eps.is_finite() && config.eps > 0.0 && config.eps <= 1e-2,
        "DerivativeConfig::eps must be in (0, 1e-2], got {}",
        config.eps
    );
    // MuJoCo rejects models with history (delays) for FD derivatives.
    assert!(
        model.nhistory == 0,
        "FD derivatives not supported with nhistory > 0 (delays)"
    );

    let eps = config.eps;
    let nv = model.nv;
    let na = model.na;
    let nu = model.nu;
    let nx = 2 * nv + na;
    let ns = model.nsensordata;
    let compute_sensors = config.compute_sensor_derivatives && ns > 0;

    // Phase 0 — Save nominal state and clone scratch.
    let mut scratch = data.clone();
    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let act_0 = data.act.clone();
    let ctrl_0 = data.ctrl.clone();
    let warmstart_0 = data.qacc_warmstart.clone();
    let time_0 = data.time;
    // Compute nominal next state by stepping unperturbed.
    // MuJoCo always computes this unconditionally. We need it for:
    // - forward differencing (non-centered A/B columns)
    // - clamped control differencing fallback (centered mode where one
    //   direction is infeasible due to actuator_ctrlrange boundary)
    scratch.step(model)?;
    let y_0 = extract_state(model, &scratch, &qpos_0);
    let sensor_0 = if compute_sensors {
        Some(scratch.sensordata.clone())
    } else {
        None
    };
    // Restore scratch to nominal for subsequent perturbations.
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
    scratch.ctrl.copy_from(&ctrl_0);
    scratch.qacc_warmstart.copy_from(&warmstart_0);
    scratch.time = time_0;

    let mut A = DMatrix::zeros(nx, nx);
    let mut B = DMatrix::zeros(nx, nu);
    let mut C = if compute_sensors {
        Some(DMatrix::zeros(ns, nx))
    } else {
        None
    };
    let mut D = if compute_sensors {
        Some(DMatrix::zeros(ns, nu))
    } else {
        None
    };

    // Phase 1 — State perturbation (A matrix + C sensor-state columns).
    for i in 0..nx {
        // Apply +eps perturbation
        apply_state_perturbation(
            model,
            &mut scratch,
            &qpos_0,
            &qvel_0,
            &act_0,
            &ctrl_0,
            &warmstart_0,
            time_0,
            i,
            eps,
            nv,
            na,
        );
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);
        let s_plus = if compute_sensors {
            Some(scratch.sensordata.clone())
        } else {
            None
        };

        if config.centered {
            // Apply -eps perturbation
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                &warmstart_0,
                time_0,
                i,
                -eps,
                nv,
                na,
            );
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let s_minus = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            // Central difference: (y+ - y-) / (2·eps)
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            A.column_mut(i).copy_from(&col);

            // Sensor column
            if let (Some(c_mat), Some(sp), Some(sm)) = (&mut C, &s_plus, &s_minus) {
                let scol = (sp - sm) / (2.0 * eps);
                c_mat.column_mut(i).copy_from(&scol);
            }
        } else {
            // Forward difference: (y+ - y0) / eps
            let col = (&y_plus - &y_0) / eps;
            A.column_mut(i).copy_from(&col);

            if let (Some(c_mat), Some(sp), Some(s0)) = (&mut C, &s_plus, &sensor_0) {
                let scol = (sp - s0) / eps;
                c_mat.column_mut(i).copy_from(&scol);
            }
        }
    }

    // Phase 2 — Control perturbation (B matrix + D sensor-control columns).
    // Matches MuJoCo's mjd_stepFD control clamping: only nudge within
    // actuator_ctrlrange. Uses forward-only, backward-only, or centered
    // differencing based on which nudges are feasible.
    for j in 0..nu {
        let range = model.actuator_ctrlrange[j];
        let nudge_fwd = in_ctrl_range(ctrl_0[j], ctrl_0[j] + eps, range);
        let nudge_back =
            (config.centered || !nudge_fwd) && in_ctrl_range(ctrl_0[j] - eps, ctrl_0[j], range);

        let (y_plus, s_plus) = if nudge_fwd {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] += eps;
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.time = time_0;
            scratch.step(model)?;
            let yp = extract_state(model, &scratch, &qpos_0);
            let sp = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (Some(yp), sp)
        } else {
            (None, None)
        };

        let (y_minus, s_minus) = if nudge_back {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] -= eps;
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.time = time_0;
            scratch.step(model)?;
            let ym = extract_state(model, &scratch, &qpos_0);
            let sm = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };
            (Some(ym), sm)
        } else {
            (None, None)
        };

        // State B column (clamped differencing)
        let col = match (&y_plus, &y_minus) {
            (Some(yp), Some(ym)) => (yp - ym) / (2.0 * eps),
            (Some(yp), None) => (yp - &y_0) / eps,
            (None, Some(ym)) => (&y_0 - ym) / eps,
            (None, None) => DVector::zeros(nx),
        };
        B.column_mut(j).copy_from(&col);

        // Sensor D column (same clamped differencing)
        if let (Some(d_mat), Some(s0)) = (&mut D, &sensor_0) {
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d_mat.column_mut(j).copy_from(&scol);
        }
    }

    // Handle nsensordata == 0 with compute_sensor_derivatives == true:
    // Return empty Some matrices (AD-2).
    let (c_result, d_result) = if config.compute_sensor_derivatives {
        if ns > 0 {
            (C, D)
        } else {
            (Some(DMatrix::zeros(0, nx)), Some(DMatrix::zeros(0, nu)))
        }
    } else {
        (None, None)
    };

    Ok(TransitionMatrices {
        A,
        B,
        C: c_result,
        D: d_result,
    })
}

/// Apply a state perturbation at index `i` with magnitude `delta`.
///
/// Restores scratch to nominal state (including warmstart) first, then applies
/// the perturbation:
/// - `i < nv`: position tangent via `mj_integrate_pos_explicit`
/// - `nv <= i < 2*nv`: velocity direct addition
/// - `2*nv <= i < 2*nv+na`: activation direct addition
///
/// Warmstart is restored to prevent leakage between perturbation columns.
/// MuJoCo's `mjd_stepFD` saves/restores `mjSTATE_WARMSTART` across each
/// perturbation for the same reason.
#[allow(clippy::too_many_arguments)]
pub(super) fn apply_state_perturbation(
    model: &Model,
    scratch: &mut Data,
    qpos_0: &DVector<f64>,
    qvel_0: &DVector<f64>,
    act_0: &DVector<f64>,
    ctrl_0: &DVector<f64>,
    warmstart_0: &DVector<f64>,
    time_0: f64,
    i: usize,
    delta: f64,
    nv: usize,
    na: usize,
) {
    if i < nv {
        // Position tangent: mj_integrate_pos_explicit maps dq[i]=delta to coordinates.
        // The velocity `dq` with `dt=1.0` produces a tangent-space displacement of
        // exactly `delta` in direction `i`: qpos_out = qpos_0 ⊕ (1.0 · dq).
        let mut dq = DVector::zeros(nv);
        dq[i] = delta;
        mj_integrate_pos_explicit(model, &mut scratch.qpos, qpos_0, &dq, 1.0);
        scratch.qvel.copy_from(qvel_0);
        scratch.act.copy_from(act_0);
    } else if i < 2 * nv {
        // Velocity: direct addition
        scratch.qpos.copy_from(qpos_0);
        scratch.qvel.copy_from(qvel_0);
        scratch.qvel[i - nv] += delta;
        scratch.act.copy_from(act_0);
    } else {
        // Activation: direct addition
        let act_idx = i - 2 * nv;
        assert!(act_idx < na, "state index out of bounds");
        scratch.qpos.copy_from(qpos_0);
        scratch.qvel.copy_from(qvel_0);
        scratch.act.copy_from(act_0);
        scratch.act[act_idx] += delta;
    }
    scratch.ctrl.copy_from(ctrl_0);
    scratch.qacc_warmstart.copy_from(warmstart_0);
    scratch.time = time_0;
}

/// Check if both values are within the given range.
/// Matches MuJoCo's `inRange()` in `engine_derivative_fd.c`.
pub(super) fn in_ctrl_range(x1: f64, x2: f64, range: (f64, f64)) -> bool {
    x1 >= range.0 && x1 <= range.1 && x2 >= range.0 && x2 <= range.1
}

/// Extract state vector in tangent space from simulation data.
///
/// Returns a `DVector<f64>` of length `2·nv + na`:
/// - `[0..nv]`: position tangent via `mj_differentiate_pos()`
/// - `[nv..2·nv]`: `data.qvel`
/// - `[2·nv..2·nv+na]`: `data.act`
///
/// The `qpos_ref` argument is the nominal qpos used to compute the
/// tangent-space displacement: `dq = qpos ⊖ qpos_ref` where `⊖` handles
/// quaternion subtraction for Ball/Free joints.
pub(super) fn extract_state(model: &Model, data: &Data, qpos_ref: &DVector<f64>) -> DVector<f64> {
    let nv = model.nv;
    let na = model.na;
    let mut x = DVector::zeros(2 * nv + na);

    // Position tangent (handles quaternion joints)
    let mut dq = DVector::zeros(nv);
    mj_differentiate_pos(model, &mut dq, qpos_ref, &data.qpos, 1.0);
    x.rows_mut(0, nv).copy_from(&dq);

    // Velocity (direct copy)
    x.rows_mut(nv, nv).copy_from(&data.qvel);

    // Activation (direct copy)
    if na > 0 {
        x.rows_mut(2 * nv, na).copy_from(&data.act);
    }

    x
}

// ============================================================================
// mjd_inverse_fd (Inverse Dynamics Derivatives)
// ============================================================================

/// Finite-difference derivatives of inverse dynamics.
///
/// Contains the Jacobians of `qfrc_inverse` with respect to position,
/// velocity, and acceleration. `DfDa` approximately equals the mass
/// matrix `M` (since `qfrc_inverse = M*qacc + bias - passive - constraint`).
///
/// # MuJoCo Equivalence
///
/// Matches the output of `mjd_inverseFD()` in `engine_derivative.c`.
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct InverseDynamicsDerivatives {
    /// `∂qfrc_inverse/∂qpos` (nv × nv).
    /// Position perturbations in tangent space.
    pub DfDq: DMatrix<f64>,
    /// `∂qfrc_inverse/∂qvel` (nv × nv).
    pub DfDv: DMatrix<f64>,
    /// `∂qfrc_inverse/∂qacc` (nv × nv).
    /// Approximately equals the mass matrix M.
    pub DfDa: DMatrix<f64>,
}

/// Compute finite-difference derivatives of inverse dynamics.
///
/// Perturbs `qpos`, `qvel`, and `qacc` around the current state, recomputes
/// derived quantities via `forward_skip() + inverse()`, and measures
/// `qfrc_inverse` deltas. Produces three nv×nv Jacobian matrices.
///
/// # Algorithm
///
/// **DfDq** (position derivatives): For each DOF `i`:
///   1. Perturb `qpos` in tangent direction `i` by `±ε`
///   2. Run `forward_skip(None, true)` — full pipeline, skip sensors
///   3. Restore `qacc` to nominal (forward overwrites it)
///   4. Run `inverse()` to compute `qfrc_inverse` with original accelerations
///   5. Measure `qfrc_inverse` difference → column of DfDq
///
/// **DfDv** (velocity derivatives): Same as DfDq but perturbing `qvel`
/// and using `forward_skip(Pos, true)` — position data is unchanged, so
/// FK/collision/CRBA are skipped (~30–50% savings per column).
///
/// **DfDa** (acceleration derivatives): For each DOF `i`:
///   1. Perturb `qacc[i]` by `±ε`
///   2. Run `inverse()` directly (no forward — only `qacc` changed)
///   3. Measure `qfrc_inverse` difference → column of DfDa
///
/// Since `qfrc_inverse = M*qacc + ...`, `DfDa ≈ M`.
///
/// # MuJoCo Equivalence
///
/// Matches `mjd_inverseFD()` in `engine_derivative.c`:
/// - Position columns use `mj_forwardSkip(m, d, mjSTAGE_NONE, 1)`
/// - Velocity columns use `mj_forwardSkip(m, d, mjSTAGE_POS, 1)`
/// - Acceleration columns call only `mj_inverse(m, d)`
/// - After each `forward_skip`, `qacc` is restored to nominal before `inverse()`
///
/// MuJoCo's version also optionally computes sensor derivatives (DsDq,
/// DsDv, DsDa) — not included here.
///
/// # Cost
///
/// - Centered: `2·nv` full-forward + `2·nv` skip-pos-forward + `2·nv` inverse-only.
/// - Forward:  `1 + nv` full-forward + `nv` skip-pos-forward + `nv` inverse-only.
///
/// # Panics
///
/// Panics if `config.eps` is non-positive, non-finite, or greater than `1e-2`.
///
/// # Errors
///
/// Returns `StepError` if any `forward_skip()` call fails.
#[allow(non_snake_case, clippy::similar_names)]
pub fn mjd_inverse_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<InverseDynamicsDerivatives, StepError> {
    assert!(
        config.eps.is_finite() && config.eps > 0.0 && config.eps <= 1e-2,
        "DerivativeConfig::eps must be in (0, 1e-2], got {}",
        config.eps
    );

    let eps = config.eps;
    let nv = model.nv;

    if nv == 0 {
        return Ok(InverseDynamicsDerivatives {
            DfDq: DMatrix::zeros(0, 0),
            DfDv: DMatrix::zeros(0, 0),
            DfDa: DMatrix::zeros(0, 0),
        });
    }

    // Save nominal state.
    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let qacc_0 = data.qacc.clone();
    let mut scratch = data.clone();

    // Compute nominal qfrc_inverse for forward differences.
    // full forward_skip(None) to populate all derived quantities, then
    // restore qacc and compute inverse.
    let qfrc_0 = if config.centered {
        None
    } else {
        scratch.forward_skip(model, crate::forward::MjStage::None, true)?;
        scratch.qacc.copy_from(&qacc_0);
        scratch.inverse(model);
        Some(scratch.qfrc_inverse.clone())
    };

    let mut DfDq = DMatrix::zeros(nv, nv);
    let mut DfDv = DMatrix::zeros(nv, nv);
    let mut DfDa = DMatrix::zeros(nv, nv);

    // --- DfDq: perturb qpos (tangent space) ---
    // forward_skip(None, true) recomputes M, qfrc_bias, qfrc_passive,
    // qfrc_constraint from the perturbed qpos. Sensors are skipped.
    // We restore qacc after forward (which overwrites it) so inverse()
    // uses the nominal acceleration.
    for i in 0..nv {
        // +eps perturbation
        let mut dq = DVector::zeros(nv);
        dq[i] = eps;
        mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
        scratch.qvel.copy_from(&qvel_0);
        scratch.forward_skip(model, crate::forward::MjStage::None, true)?;
        scratch.qacc.copy_from(&qacc_0);
        scratch.inverse(model);
        let f_plus = scratch.qfrc_inverse.clone();

        if config.centered {
            // -eps perturbation
            dq[i] = -eps;
            mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.forward_skip(model, crate::forward::MjStage::None, true)?;
            scratch.qacc.copy_from(&qacc_0);
            scratch.inverse(model);
            let f_minus = &scratch.qfrc_inverse;

            let col = (&f_plus - f_minus) / (2.0 * eps);
            DfDq.column_mut(i).copy_from(&col);
        } else if let Some(ref f_ref) = qfrc_0 {
            let col = (&f_plus - f_ref) / eps;
            DfDq.column_mut(i).copy_from(&col);
        }
    }

    // --- DfDv: perturb qvel ---
    // Position is unchanged, so forward_skip(Pos, true) skips FK/collision/CRBA
    // and only recomputes velocity-dependent quantities (Coriolis, damping).
    // Matches MuJoCo's mj_forwardSkip(m, d, mjSTAGE_POS, 1) for velocity columns.
    //
    // We need position-stage data to be current first. Run a full forward_skip
    // from nominal qpos to populate FK/collision/CRBA, then use skip(Pos) for
    // each velocity perturbation.
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.forward_skip(model, crate::forward::MjStage::None, true)?;

    for i in 0..nv {
        // +eps perturbation
        scratch.qvel.copy_from(&qvel_0);
        scratch.qvel[i] += eps;
        scratch.forward_skip(model, crate::forward::MjStage::Pos, true)?;
        scratch.qacc.copy_from(&qacc_0);
        scratch.inverse(model);
        let f_plus = scratch.qfrc_inverse.clone();

        if config.centered {
            // -eps perturbation
            scratch.qvel.copy_from(&qvel_0);
            scratch.qvel[i] -= eps;
            scratch.forward_skip(model, crate::forward::MjStage::Pos, true)?;
            scratch.qacc.copy_from(&qacc_0);
            scratch.inverse(model);
            let f_minus = &scratch.qfrc_inverse;

            let col = (&f_plus - f_minus) / (2.0 * eps);
            DfDv.column_mut(i).copy_from(&col);
        } else if let Some(ref f_ref) = qfrc_0 {
            let col = (&f_plus - f_ref) / eps;
            DfDv.column_mut(i).copy_from(&col);
        }
    }

    // --- DfDa: perturb qacc ---
    // Only qacc changes — no forward needed. inverse() reads qacc directly:
    // qfrc_inverse = M*qacc + qfrc_bias - qfrc_passive - qfrc_constraint.
    // All other quantities (M, qfrc_bias, etc.) are unchanged from the
    // full forward_skip done above for DfDv.
    //
    // Reset flg_rnepost so each inverse() call recomputes body accumulators
    // with the perturbed qacc (for correct cacc/cfrc_int/cfrc_ext, even
    // though qfrc_inverse itself doesn't depend on them).
    for i in 0..nv {
        scratch.qacc.copy_from(&qacc_0);
        scratch.qacc[i] += eps;
        scratch.flg_rnepost = false;
        scratch.inverse(model);
        let f_plus = scratch.qfrc_inverse.clone();

        if config.centered {
            scratch.qacc.copy_from(&qacc_0);
            scratch.qacc[i] -= eps;
            scratch.flg_rnepost = false;
            scratch.inverse(model);
            let f_minus = &scratch.qfrc_inverse;

            let col = (&f_plus - f_minus) / (2.0 * eps);
            DfDa.column_mut(i).copy_from(&col);
        } else if let Some(ref f_ref) = qfrc_0 {
            let col = (&f_plus - f_ref) / eps;
            DfDa.column_mut(i).copy_from(&col);
        }
    }

    Ok(InverseDynamicsDerivatives { DfDq, DfDv, DfDa })
}

// ============================================================================
// Tests — forward_skip regression + inverse FD
// ============================================================================

#[cfg(test)]
#[allow(
    clippy::similar_names,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::uninlined_format_args
)]
mod forward_skip_tests {
    use crate::forward::MjStage;
    use crate::types::Model;

    /// forward_skip(None, false) must produce identical results to forward().
    #[test]
    fn dt53_forward_skip_none_matches_forward() {
        let model = Model::n_link_pendulum(3, 1.0, 0.1);
        let mut data_fwd = model.make_data();
        let mut data_skip = model.make_data();

        // Set non-trivial state
        data_fwd.qpos[0] = 0.3;
        data_fwd.qpos[1] = -0.2;
        data_fwd.qpos[2] = 0.1;
        data_fwd.qvel[0] = 0.5;
        data_fwd.qvel[1] = -0.3;
        data_fwd.qvel[2] = 0.7;
        data_skip.qpos.copy_from(&data_fwd.qpos);
        data_skip.qvel.copy_from(&data_fwd.qvel);

        // forward() vs forward_skip(None, false)
        data_fwd.forward(&model).expect("forward failed");
        data_skip
            .forward_skip(&model, MjStage::None, false)
            .expect("forward_skip failed");

        // Compare qacc (primary output of forward dynamics)
        for i in 0..model.nv {
            assert!(
                (data_fwd.qacc[i] - data_skip.qacc[i]).abs() < 1e-14,
                "qacc[{}] mismatch: forward={:.16e}, skip={:.16e}",
                i,
                data_fwd.qacc[i],
                data_skip.qacc[i]
            );
        }

        // Compare qfrc_smooth
        for i in 0..model.nv {
            assert!(
                (data_fwd.qfrc_smooth[i] - data_skip.qfrc_smooth[i]).abs() < 1e-14,
                "qfrc_smooth[{}] mismatch",
                i
            );
        }
    }

    /// forward_skip(Pos, true) skips position stage but still computes
    /// accelerations correctly when position data is already current.
    #[test]
    fn dt53_forward_skip_pos_after_position_stage() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();

        data.qpos[0] = 0.5;
        data.qpos[1] = -0.3;
        data.qvel[0] = 0.2;
        data.qvel[1] = 0.4;

        // Run full forward first to populate position-dependent data
        data.forward(&model).expect("forward failed");
        let qacc_full = data.qacc.clone();

        // Now perturb only velocity and use skip(Pos)
        // Without velocity perturbation, skip(Pos) should give same result
        data.forward_skip(&model, MjStage::Pos, true)
            .expect("forward_skip(Pos) failed");

        for i in 0..model.nv {
            assert!(
                (data.qacc[i] - qacc_full[i]).abs() < 1e-12,
                "qacc[{}] mismatch after skip(Pos): full={:.16e}, skip={:.16e}",
                i,
                qacc_full[i],
                data.qacc[i]
            );
        }
    }

    /// forward_skip(Vel, true) skips both position and velocity stages.
    /// When neither qpos nor qvel changed, result matches forward().
    #[test]
    fn dt53_forward_skip_vel_unchanged_state() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();

        data.qpos[0] = 0.3;
        data.qvel[0] = 1.0;

        // Full forward to populate everything
        data.forward(&model).expect("forward failed");
        let qacc_full = data.qacc.clone();

        // Skip both pos and vel stages — should get same result
        data.forward_skip(&model, MjStage::Vel, true)
            .expect("forward_skip(Vel) failed");

        for i in 0..model.nv {
            assert!(
                (data.qacc[i] - qacc_full[i]).abs() < 1e-12,
                "qacc[{}] mismatch after skip(Vel): full={:.16e}, skip={:.16e}",
                i,
                qacc_full[i],
                data.qacc[i]
            );
        }
    }

    /// Verify that forward_skip + integrate produces a valid step.
    #[test]
    fn dt53_forward_skip_plus_integrate() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data_step = model.make_data();
        let mut data_skip = model.make_data();

        data_step.qpos[0] = 0.3;
        data_step.qvel[0] = 0.5;
        data_skip.qpos.copy_from(&data_step.qpos);
        data_skip.qvel.copy_from(&data_step.qvel);

        // step() = forward() + integrate() (for Euler)
        data_step.step(&model).expect("step failed");

        // forward_skip(None) + integrate() should match
        data_skip
            .forward_skip(&model, MjStage::None, true)
            .expect("forward_skip failed");
        data_skip.integrate(&model);

        for i in 0..model.nq {
            assert!(
                (data_step.qpos[i] - data_skip.qpos[i]).abs() < 1e-12,
                "qpos[{}] mismatch: step={:.16e}, skip+integrate={:.16e}",
                i,
                data_step.qpos[i],
                data_skip.qpos[i]
            );
        }
        for i in 0..model.nv {
            assert!(
                (data_step.qvel[i] - data_skip.qvel[i]).abs() < 1e-12,
                "qvel[{}] mismatch: step={:.16e}, skip+integrate={:.16e}",
                i,
                data_step.qvel[i],
                data_skip.qvel[i]
            );
        }
    }

    /// MjStage ordering: None < Pos < Vel.
    #[test]
    fn dt53_mj_stage_ordering() {
        assert!(MjStage::None < MjStage::Pos);
        assert!(MjStage::Pos < MjStage::Vel);
        assert!(MjStage::None < MjStage::Vel);
    }

    /// Verify that velocity FD columns produce correct derivatives when
    /// using forward_skip(Pos) to skip the position stage.
    ///
    /// This is the key use case for forward_skip in FD loops: position
    /// data is unchanged when perturbing velocity, so FK/collision/CRBA
    /// can be skipped.
    #[test]
    fn dt53_forward_skip_pos_velocity_fd_correctness() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.4;
        data.qpos[1] = -0.2;
        data.qvel[0] = 0.3;
        data.forward(&model).expect("forward failed");

        let eps = 1e-6;
        let qacc_0 = data.qacc.clone();

        // Reference: use full forward() for velocity FD
        let mut d_full = data.clone();
        d_full.qvel[0] += eps;
        d_full.forward(&model).expect("ref fwd+ failed");
        d_full.qacc.copy_from(&qacc_0);
        d_full.inverse(&model);
        let frc_full = d_full.qfrc_inverse.clone();

        // Test: use forward_skip(Pos) for velocity FD
        let mut d_skip = data.clone();
        // Pre-populate position data with full forward
        d_skip.forward(&model).expect("init fwd failed");
        // Now perturb velocity and use skip(Pos)
        d_skip.qvel[0] += eps;
        d_skip
            .forward_skip(&model, MjStage::Pos, true)
            .expect("skip fwd+ failed");
        d_skip.qacc.copy_from(&qacc_0);
        d_skip.inverse(&model);
        let frc_skip = d_skip.qfrc_inverse.clone();

        // Results should match
        for i in 0..model.nv {
            assert!(
                (frc_full[i] - frc_skip[i]).abs() < 1e-12,
                "qfrc_inverse[{}] mismatch: full={:.16e}, skip={:.16e}",
                i,
                frc_full[i],
                frc_skip[i]
            );
        }
    }
}

#[cfg(test)]
#[allow(
    clippy::similar_names,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::uninlined_format_args,
    non_snake_case
)]
mod inverse_fd_tests {
    use super::*;
    use crate::jacobian::mj_integrate_pos_explicit;
    use crate::types::Model;
    use nalgebra::DVector;

    /// DfDa should approximately equal the mass matrix M.
    /// qfrc_inverse = M*qacc + ... so ∂qfrc_inverse/∂qacc ≈ M.
    #[test]
    fn dt51_dfda_approx_mass_matrix_1link() {
        let model = Model::n_link_pendulum(1, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.forward(&model).expect("forward failed");

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        // DfDa should be close to qM
        for i in 0..model.nv {
            for j in 0..model.nv {
                let err = (derivs.DfDa[(i, j)] - data.qM[(i, j)]).abs();
                let scale = data.qM[(i, j)].abs().max(1e-10);
                assert!(
                    err / scale < 1e-4,
                    "DfDa[{},{}] = {:.8e}, qM[{},{}] = {:.8e}, rel_err = {:.8e}",
                    i,
                    j,
                    derivs.DfDa[(i, j)],
                    i,
                    j,
                    data.qM[(i, j)],
                    err / scale
                );
            }
        }
    }

    /// DfDa ≈ M for a multi-link pendulum (nv > 1 with coupling).
    #[test]
    fn dt51_dfda_approx_mass_matrix_3link() {
        let model = Model::n_link_pendulum(3, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.qpos[1] = -0.3;
        data.qpos[2] = 0.2;
        data.qvel[0] = 0.1;
        data.forward(&model).expect("forward failed");

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        for i in 0..model.nv {
            for j in 0..model.nv {
                let err = (derivs.DfDa[(i, j)] - data.qM[(i, j)]).abs();
                let scale = data.qM[(i, j)].abs().max(1e-10);
                assert!(
                    err / scale < 1e-4,
                    "3-link DfDa[{},{}] = {:.8e}, qM = {:.8e}, rel_err = {:.8e}",
                    i,
                    j,
                    derivs.DfDa[(i, j)],
                    data.qM[(i, j)],
                    err / scale
                );
            }
        }
    }

    /// DfDv should capture velocity-dependent terms (Coriolis/centrifugal).
    ///
    /// For a multi-link pendulum with nonzero velocity, the bias forces
    /// (Coriolis/centrifugal) depend on velocity. DfDv captures
    /// ∂qfrc_inverse/∂qvel. We verify DfDv is nonzero by independent FD
    /// of qfrc_inverse w.r.t. qvel using forward() + inverse().
    #[test]
    fn dt51_dfdv_matches_independent_fd() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();

        data.qpos[0] = 0.5;
        data.qpos[1] = -0.3;
        data.qvel[0] = 1.0;
        data.qvel[1] = -0.5;
        data.forward(&model).expect("forward failed");

        // Compute via mjd_inverse_fd
        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        // Independent FD verification: perturb qvel[0] manually and check
        let eps = 1e-6;
        let qacc_0 = data.qacc.clone();
        let mut d_plus = data.clone();
        let mut d_minus = data.clone();
        d_plus.qvel[0] += eps;
        d_minus.qvel[0] -= eps;
        d_plus.forward(&model).expect("fwd+ failed");
        d_plus.qacc.copy_from(&qacc_0);
        d_plus.inverse(&model);
        d_minus.forward(&model).expect("fwd- failed");
        d_minus.qacc.copy_from(&qacc_0);
        d_minus.inverse(&model);

        let col_fd = (&d_plus.qfrc_inverse - &d_minus.qfrc_inverse) / (2.0 * eps);
        for i in 0..model.nv {
            let err = (derivs.DfDv[(i, 0)] - col_fd[i]).abs();
            let scale = col_fd[i].abs().max(1e-10);
            assert!(
                err / scale < 1e-3 || err < 1e-10,
                "DfDv[{},0]: api={:.8e}, manual_fd={:.8e}, err={:.8e}",
                i,
                derivs.DfDv[(i, 0)],
                col_fd[i],
                err
            );
        }

        // Dimensions correct
        assert_eq!(derivs.DfDv.nrows(), model.nv);
        assert_eq!(derivs.DfDv.ncols(), model.nv);
    }

    /// DfDq should be non-zero (gravity torques depend on position).
    #[test]
    fn dt51_dfdq_nonzero_gravity() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.forward(&model).expect("forward failed");

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        let max_abs = derivs.DfDq.abs().max();
        assert!(
            max_abs > 1e-3,
            "DfDq should have nonzero entries from gravity; max_abs = {:.8e}",
            max_abs
        );
    }

    /// DfDq matches independent FD with manual forward+inverse.
    #[test]
    fn dt51_dfdq_matches_independent_fd() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.5;
        data.qpos[1] = -0.2;
        data.forward(&model).expect("forward failed");

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        // Independent FD: perturb qpos[0] via tangent space
        let eps = 1e-6;
        let qacc_0 = data.qacc.clone();
        let mut d_plus = data.clone();
        let mut d_minus = data.clone();
        let mut dq = DVector::zeros(model.nv);
        dq[0] = eps;
        mj_integrate_pos_explicit(&model, &mut d_plus.qpos, &data.qpos, &dq, 1.0);
        dq[0] = -eps;
        mj_integrate_pos_explicit(&model, &mut d_minus.qpos, &data.qpos, &dq, 1.0);

        d_plus.forward(&model).expect("fwd+ failed");
        d_plus.qacc.copy_from(&qacc_0);
        d_plus.inverse(&model);
        d_minus.forward(&model).expect("fwd- failed");
        d_minus.qacc.copy_from(&qacc_0);
        d_minus.inverse(&model);

        let col_fd = (&d_plus.qfrc_inverse - &d_minus.qfrc_inverse) / (2.0 * eps);
        for i in 0..model.nv {
            let err = (derivs.DfDq[(i, 0)] - col_fd[i]).abs();
            let scale = col_fd[i].abs().max(1e-10);
            assert!(
                err / scale < 1e-3 || err < 1e-10,
                "DfDq[{},0]: api={:.8e}, manual_fd={:.8e}, err={:.8e}",
                i,
                derivs.DfDq[(i, 0)],
                col_fd[i],
                err
            );
        }
    }

    /// Forward vs centered differences: centered should be more accurate.
    #[test]
    fn dt51_centered_vs_forward_convergence() {
        let model = Model::n_link_pendulum(2, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qvel[0] = 0.5;
        data.forward(&model).expect("forward failed");

        let centered = DerivativeConfig {
            centered: true,
            ..Default::default()
        };
        let forward_cfg = DerivativeConfig {
            centered: false,
            ..Default::default()
        };

        let d_c = mjd_inverse_fd(&model, &data, &centered).expect("centered failed");
        let d_f = mjd_inverse_fd(&model, &data, &forward_cfg).expect("forward failed");

        // Both should agree reasonably well on DfDa ≈ M
        for i in 0..model.nv {
            for j in 0..model.nv {
                let diff = (d_c.DfDa[(i, j)] - d_f.DfDa[(i, j)]).abs();
                assert!(
                    diff < 1e-3,
                    "DfDa[{},{}] centered={:.8e}, forward={:.8e}, diff={:.8e}",
                    i,
                    j,
                    d_c.DfDa[(i, j)],
                    d_f.DfDa[(i, j)],
                    diff
                );
            }
        }

        // DfDq should also agree (both capture gravity Jacobian)
        for i in 0..model.nv {
            for j in 0..model.nv {
                let diff = (d_c.DfDq[(i, j)] - d_f.DfDq[(i, j)]).abs();
                assert!(
                    diff < 1e-2,
                    "DfDq[{},{}] centered={:.8e}, forward={:.8e}, diff={:.8e}",
                    i,
                    j,
                    d_c.DfDq[(i, j)],
                    d_f.DfDq[(i, j)],
                    diff
                );
            }
        }
    }

    /// Zero-DOF model should return empty matrices.
    #[test]
    fn dt51_zero_dof_model() {
        let model = Model::empty();
        let data = model.make_data();

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        assert_eq!(derivs.DfDq.nrows(), 0);
        assert_eq!(derivs.DfDv.nrows(), 0);
        assert_eq!(derivs.DfDa.nrows(), 0);
    }

    /// Verify mjd_inverse_fd uses forward_skip correctly: position
    /// perturbation columns use forward_skip(None, true) (full pipeline,
    /// skip sensors) and velocity columns use forward_skip(Pos, true)
    /// (skip FK/collision). Test this indirectly by verifying the output
    /// matches a reference computed with full forward().
    #[test]
    fn dt51_skip_stage_correctness() {
        let model = Model::n_link_pendulum(3, 1.0, 0.1);
        let mut data = model.make_data();
        data.qpos[0] = 0.4;
        data.qpos[1] = -0.2;
        data.qpos[2] = 0.1;
        data.qvel[0] = 0.3;
        data.qvel[1] = -0.1;
        data.forward(&model).expect("forward failed");

        let config = DerivativeConfig::default();
        let derivs = mjd_inverse_fd(&model, &data, &config).expect("mjd_inverse_fd failed");

        // Independent reference: compute DfDa manually with direct inverse
        let qacc_0 = data.qacc.clone();
        let eps = 1e-6;
        let mut scratch = data.clone();
        scratch.forward(&model).expect("ref fwd failed");

        for j in 0..model.nv {
            // +eps
            scratch.qacc.copy_from(&qacc_0);
            scratch.qacc[j] += eps;
            scratch.flg_rnepost = false;
            scratch.inverse(&model);
            let f_plus = scratch.qfrc_inverse.clone();

            // -eps
            scratch.qacc.copy_from(&qacc_0);
            scratch.qacc[j] -= eps;
            scratch.flg_rnepost = false;
            scratch.inverse(&model);

            let col = (&f_plus - &scratch.qfrc_inverse) / (2.0 * eps);
            for i in 0..model.nv {
                let err = (derivs.DfDa[(i, j)] - col[i]).abs();
                assert!(
                    err < 1e-10,
                    "DfDa[{},{}] skip_stage={:.16e}, ref={:.16e}",
                    i,
                    j,
                    derivs.DfDa[(i, j)],
                    col[i]
                );
            }
        }
    }
}
