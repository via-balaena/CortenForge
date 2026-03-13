//! Muscle force-length-velocity curves and parameter precomputation — entry point.
//!
//! This module re-exports from siblings [`hill`](super::hill) and
//! [`fiber`](super::fiber), and houses the simulation-based length-range
//! estimation (`mj_set_length_range`, `eval_length_range`). Corresponds to
//! `engine_setconst.c` (`mj_setLengthRange`).

// Re-export hill.rs — muscle curves and activation dynamics.
pub use super::hill::{
    muscle_activation_dynamics, muscle_gain_length, muscle_gain_velocity, muscle_passive_force,
};

use super::fiber::build_actuator_moment;
use crate::linalg::mj_solve_sparse;
use crate::types::{
    ActuatorTransmission, BiasType, GainType, LengthRangeError, LengthRangeMode, LengthRangeOpt,
    Model,
};

// ============================================================================
// Simulation-based length-range estimation (DT-59, DT-77)
// ============================================================================

/// Estimate actuator length ranges via simulation.
///
/// For each actuator that passes mode filtering:
/// 1. If `useexisting` and range already set, skip.
/// 2. If `uselimit` and joint/tendon is limited, copy limits (already handled
///    by Phase 1 of `compute_actuator_params`; skipped here via `useexisting`).
/// 3. Otherwise, run two-sided simulation to find min/max length.
///
/// Matches MuJoCo's `mj_setLengthRange()` in `engine_setconst.c`.
pub fn mj_set_length_range(model: &mut Model, opt: &LengthRangeOpt) {
    for i in 0..model.nu {
        // Step 1: Mode filtering
        let is_muscle = matches!(
            model.actuator_gaintype[i],
            GainType::Muscle | GainType::HillMuscle
        ) || matches!(
            model.actuator_biastype[i],
            BiasType::Muscle | BiasType::HillMuscle
        );
        let is_user = model.actuator_gaintype[i] == GainType::User
            || model.actuator_biastype[i] == BiasType::User;

        let skip = match opt.mode {
            LengthRangeMode::None => true,
            LengthRangeMode::Muscle => !is_muscle,
            LengthRangeMode::MuscleUser => !is_muscle && !is_user,
            LengthRangeMode::All => false,
        };
        if skip {
            continue;
        }

        // Step 2: Use existing range (already set by Phase 1 limit copy)
        let (lo, hi) = model.actuator_lengthrange[i];
        if opt.useexisting && lo < hi {
            continue;
        }

        // Step 3: Copy from limits (with uselimit)
        // This is largely redundant with Phase 1 but matches MuJoCo's structure.
        // NOTE (DT-106): gear scaling here is an intentional deviation from
        // MuJoCo's uselimit path — see Phase 1 comment block for full rationale.
        if opt.uselimit {
            let gear = model.actuator_gear[i][0];
            let scale = |a: f64, b: f64| -> (f64, f64) {
                let x = gear * a;
                let y = gear * b;
                (x.min(y), x.max(y))
            };

            match model.actuator_trntype[i] {
                ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
                    let jid = model.actuator_trnid[i][0];
                    if jid < model.njnt && model.jnt_limited[jid] {
                        let (jlo, jhi) = model.jnt_range[jid];
                        model.actuator_lengthrange[i] = scale(jlo, jhi);
                        continue;
                    }
                }
                ActuatorTransmission::Tendon => {
                    let tid = model.actuator_trnid[i][0];
                    if tid < model.ntendon && model.tendon_limited[tid] {
                        let (tlo, thi) = model.tendon_range[tid];
                        model.actuator_lengthrange[i] = scale(tlo, thi);
                        continue;
                    }
                }
                ActuatorTransmission::Site
                | ActuatorTransmission::Body
                | ActuatorTransmission::SliderCrank => {
                    // Site/SliderCrank: configuration-dependent, no static limits to copy.
                    // Body: no length concept. Skip.
                }
            }
        }

        // Step 4: Simulation-based estimation
        // Convergence failure is not fatal — MuJoCo silently leaves the range
        // at (0, 0) and returns success. Only instability (NaN) would be fatal,
        // but we handle that within eval_length_range.
        if let Ok(range) = eval_length_range(model, i, opt) {
            model.actuator_lengthrange[i] = range;
        }
        // On Err: leave at (0, 0) — matches MuJoCo's silent failure behavior
    }
}

/// Run two-sided simulation to estimate actuator length range.
///
/// Side 0: apply negative force → find minimum length.
/// Side 1: apply positive force → find maximum length.
///
/// Runs the full step1/step2 pipeline with gravity, contacts, and passive
/// forces active — matching MuJoCo's `evalAct()` behavior exactly. MuJoCo does
/// NOT disable gravity or contacts during length-range estimation.
fn eval_length_range(
    model: &Model,
    actuator_idx: usize,
    opt: &LengthRangeOpt,
) -> Result<(f64, f64), LengthRangeError> {
    let nv = model.nv;
    if nv == 0 {
        return Err(LengthRangeError::InvalidRange {
            actuator: actuator_idx,
        });
    }

    // Clone model to set LR-specific timestep.
    // IMPORTANT: Do NOT zero gravity or disable contacts — MuJoCo keeps them active.
    let mut lr_model = model.clone();
    lr_model.timestep = opt.timestep;

    let mut lengthrange = [0.0f64; 2];
    let mut lmin_sides = [f64::MAX; 2];
    let mut lmax_sides = [f64::MIN; 2];

    for side in 0..2usize {
        let mut data = lr_model.make_data(); // init at qpos0
        let sign: f64 = if side == 0 { -1.0 } else { 1.0 };

        let mut updated = false;
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let total_steps = (opt.inttotal / opt.timestep).ceil() as usize;
        let measure_start = opt.inttotal - opt.interval;

        for step_idx in 0..total_steps {
            #[allow(clippy::cast_precision_loss)]
            let time = step_idx as f64 * opt.timestep;

            // 1. Velocity damping: qvel *= exp(-dt / max(0.01, timeconst))
            let decay = (-opt.timestep / opt.timeconst.max(0.01)).exp();
            for dof in 0..nv {
                data.qvel[dof] *= decay;
            }

            // 2. Full forward pass (FK, collision, CRBA, gravity, passive, actuation)
            if data.step1(&lr_model).is_err() {
                return Err(LengthRangeError::ConvergenceFailed {
                    actuator: actuator_idx,
                });
            }

            // 3. Build actuator moment vector (transmission Jacobian).
            // Our codebase only populates data.actuator_moment for Site/Body
            // transmissions. For Joint/Tendon, we must build it explicitly.
            // build_actuator_moment uses current data state (site_xmat, ten_J)
            // so it reflects the current configuration after step1.
            let j_vec = build_actuator_moment(&lr_model, &data, actuator_idx);

            let mut x = j_vec.clone();
            let (rowadr, rownnz, colind) = lr_model.qld_csr();
            mj_solve_sparse(
                rowadr,
                rownnz,
                colind,
                &data.qLD_data,
                &data.qLD_diag_inv,
                &mut x,
            );
            let nrm = x.norm().max(1e-15);

            // qfrc_applied = sign * accel * moment / ||M^{-1} moment||
            for dof in 0..nv {
                data.qfrc_applied[dof] = sign * opt.accel * j_vec[dof] / nrm;
            }

            // 4. Force capping
            if opt.maxforce > 0.0 {
                let fnrm = data.qfrc_applied.norm();
                if fnrm > opt.maxforce {
                    let scale = opt.maxforce / fnrm;
                    for dof in 0..nv {
                        data.qfrc_applied[dof] *= scale;
                    }
                }
            }

            // 5. Acceleration + integration (step2)
            if data.step2(&lr_model).is_err() {
                return Err(LengthRangeError::ConvergenceFailed {
                    actuator: actuator_idx,
                });
            }

            // 5b. Instability detection (matching MuJoCo's `if (d->time == 0) return 0`).
            // If the step caused NaN/Inf, mj_check_pos auto-resets data (including time=0).
            // Detect this by checking if divergence occurred.
            if data.divergence_detected() {
                return Err(LengthRangeError::ConvergenceFailed {
                    actuator: actuator_idx,
                });
            }

            // 6. Read actuator length (populated by step1's transmission stage)
            let len = data.actuator_length[actuator_idx];

            // 7. Track min/max during measurement interval
            if time >= measure_start {
                if len < lmin_sides[side] || !updated {
                    lmin_sides[side] = len;
                }
                if len > lmax_sides[side] || !updated {
                    lmax_sides[side] = len;
                }
                updated = true;
            }
        }

        lengthrange[side] = if side == 0 {
            lmin_sides[side]
        } else {
            lmax_sides[side]
        };
    }

    // Step 5: Convergence check
    let dif = lengthrange[1] - lengthrange[0];
    if dif <= 0.0 {
        return Err(LengthRangeError::InvalidRange {
            actuator: actuator_idx,
        });
    }
    for side in 0..2 {
        let oscillation = lmax_sides[side] - lmin_sides[side];
        if oscillation > opt.tolrange * dif {
            return Err(LengthRangeError::ConvergenceFailed {
                actuator: actuator_idx,
            });
        }
    }

    Ok(<(f64, f64)>::from(lengthrange))
}
