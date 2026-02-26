//! Standard 4-stage Runge-Kutta integration.
//!
//! Implements MuJoCo's `mj_RungeKutta` using the classic RK4 Butcher tableau.

use crate::integrate::euler::mj_normalize_quat;
use crate::jacobian::mj_integrate_pos_explicit;
use crate::types::flags::{actuator_disabled, disabled};
use crate::types::{ActuatorDynamics, DISABLE_ACTUATION, Data, Model, StepError};

/// Standard 4-stage Runge-Kutta integration matching MuJoCo's `mj_RungeKutta`.
///
/// # Preconditions
/// - `data.forward()` has already been called (qacc is valid for stage 0).
///
/// # Algorithm
/// Uses the classic RK4 Butcher tableau. Stage 0 reuses qacc from the preceding
/// `forward()` call. Stages 1-3 each call `forward_skip_sensors()` at trial states.
/// Position integration uses `mj_integrate_pos_explicit()` for quaternion correctness.
///
/// After this function returns, derived quantities (xpos, contacts, forces, etc.)
/// are stale from stage 3 and do NOT correspond to the final (qpos, qvel) state.
/// This matches MuJoCo's behavior.
#[allow(clippy::needless_range_loop)] // explicit indexing across parallel arrays (Butcher tableau)
pub fn mj_runge_kutta(model: &Model, data: &mut Data) -> Result<(), StepError> {
    // Butcher tableau
    const RK4_A: [f64; 9] = [
        0.5, 0.0, 0.0, // Stage 1→2
        0.0, 0.5, 0.0, // Stage 2→3
        0.0, 0.0, 1.0, // Stage 3→4
    ];
    const RK4_B: [f64; 4] = [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0];
    const RK4_TIME: [f64; 3] = [0.5, 0.5, 1.0];

    let h = model.timestep;
    let nv = model.nv;

    // 1. SAVE initial state
    data.rk4_qpos_saved.copy_from(&data.qpos);
    data.rk4_qvel[0].copy_from(&data.qvel);
    data.rk4_qacc[0].copy_from(&data.qacc);
    data.rk4_act_saved.copy_from(&data.act);
    data.rk4_act_dot[0].copy_from(&data.act_dot);
    let t0 = data.time;

    // 2. FOR i = 1, 2, 3:
    for i in 1..4usize {
        // 2a. Weighted velocity: dX_vel[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * X[j].qvel[v]
        // (For this tableau, only j = i−1 has a non-zero coefficient.)
        for v in 0..nv {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += RK4_A[(i - 1) * 3 + j] * data.rk4_qvel[j][v];
            }
            data.rk4_dX_vel[v] = sum;
        }

        // 2b. Weighted acceleration: dX_acc[v] = Σ_{j=0}^{2} A[(i-1)*3+j] * F[j].qacc[v]
        for v in 0..nv {
            let mut sum = 0.0;
            for j in 0..3 {
                sum += RK4_A[(i - 1) * 3 + j] * data.rk4_qacc[j][v];
            }
            data.rk4_dX_acc[v] = sum;
        }

        // 2c. Position (manifold): integrate from saved initial position
        mj_integrate_pos_explicit(
            model,
            &mut data.rk4_qpos_stage,
            &data.rk4_qpos_saved,
            &data.rk4_dX_vel,
            h,
        );

        // (§27F) Flex vertex positions now integrated by mj_integrate_pos_explicit
        // above — slide joints are handled by the standard manifold integration path.

        // 2d. Velocity (linear): X[i].qvel = X[0].qvel + h * dX_acc
        // Use split_at_mut for borrow-checker disjointness on rk4_qvel.
        let (head, tail) = data.rk4_qvel.split_at_mut(1);
        for v in 0..nv {
            tail[i - 1][v] = head[0][v] + h * data.rk4_dX_acc[v];
        }

        // 2e. Activation trial state: act = act_saved + h_eff * Σ A[(i-1)*3+j] * act_dot[j]
        // where h_eff depends on dynamics type (Euler vs FilterExact).
        // S4.8: Disabled actuators get zero act_dot, freezing activation.
        for act_i in 0..model.nu {
            let act_adr = model.actuator_act_adr[act_i];
            let is_disabled = disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, act_i);
            for k in 0..model.actuator_act_num[act_i] {
                let a = act_adr + k;
                let mut sum = 0.0;
                for j in 0..3 {
                    let ad = if is_disabled {
                        0.0
                    } else {
                        data.rk4_act_dot[j][a]
                    };
                    sum += RK4_A[(i - 1) * 3 + j] * ad;
                }
                match model.actuator_dyntype[act_i] {
                    ActuatorDynamics::FilterExact => {
                        let tau = model.actuator_dynprm[act_i][0].max(1e-10);
                        data.act[a] = data.rk4_act_saved[a] + sum * tau * (1.0 - (-h / tau).exp());
                    }
                    _ => {
                        data.act[a] = data.rk4_act_saved[a] + h * sum;
                    }
                }
            }
            // §34: Clamp activation to actrange for trial state (replaces muscle-only [0,1])
            if model.actuator_actlimited[act_i] {
                let range = model.actuator_actrange[act_i];
                for k in 0..model.actuator_act_num[act_i] {
                    data.act[act_adr + k] = data.act[act_adr + k].clamp(range.0, range.1);
                }
            }
        }

        // 2f. Set Data state
        data.qpos.copy_from(&data.rk4_qpos_stage);
        data.qvel.copy_from(&tail[i - 1]);

        // 2g. Set Data time
        data.time = t0 + h * RK4_TIME[i - 1];

        // 2h. Evaluate full forward pipeline (without sensors)
        data.forward_skip_sensors(model)?;

        // 2i. Store rates
        data.rk4_qacc[i].copy_from(&data.qacc);
        data.rk4_act_dot[i].copy_from(&data.act_dot);
    }

    // 3. FINAL combination using B weights
    for v in 0..nv {
        let mut vel_sum = 0.0;
        let mut acc_sum = 0.0;
        for j in 0..4 {
            vel_sum += RK4_B[j] * data.rk4_qvel[j][v];
            acc_sum += RK4_B[j] * data.rk4_qacc[j][v];
        }
        data.rk4_dX_vel[v] = vel_sum;
        data.rk4_dX_acc[v] = acc_sum;
    }

    // 4. ADVANCE from initial state
    // Note: qacc_warmstart is now saved at end of step() (§15.9), not here.

    // Restore initial velocity, then advance
    data.qvel.copy_from(&data.rk4_qvel[0]);
    for v in 0..nv {
        data.qvel[v] += h * data.rk4_dX_acc[v];
    }

    // Position on manifold from saved initial position
    mj_integrate_pos_explicit(
        model,
        &mut data.qpos,
        &data.rk4_qpos_saved,
        &data.rk4_dX_vel,
        h,
    );

    // (§27F) Flex vertex positions now integrated by mj_integrate_pos_explicit above.

    mj_normalize_quat(model, data);

    // Advance activation from saved initial state
    // S4.8: Disabled actuators get zero act_dot, freezing activation.
    for act_i in 0..model.nu {
        let act_adr = model.actuator_act_adr[act_i];
        let is_disabled = disabled(model, DISABLE_ACTUATION) || actuator_disabled(model, act_i);
        for k in 0..model.actuator_act_num[act_i] {
            let a = act_adr + k;
            let dact_combined: f64 = (0..4)
                .map(|j| {
                    let ad = if is_disabled {
                        0.0
                    } else {
                        data.rk4_act_dot[j][a]
                    };
                    RK4_B[j] * ad
                })
                .sum();
            match model.actuator_dyntype[act_i] {
                ActuatorDynamics::FilterExact => {
                    let tau = model.actuator_dynprm[act_i][0].max(1e-10);
                    data.act[a] =
                        data.rk4_act_saved[a] + dact_combined * tau * (1.0 - (-h / tau).exp());
                }
                _ => {
                    data.act[a] = data.rk4_act_saved[a] + h * dact_combined;
                }
            }
        }
        // §34: Clamp activation to actrange (replaces muscle-only [0,1])
        if model.actuator_actlimited[act_i] {
            let range = model.actuator_actrange[act_i];
            for k in 0..model.actuator_act_num[act_i] {
                data.act[act_adr + k] = data.act[act_adr + k].clamp(range.0, range.1);
            }
        }
    }

    data.time = t0 + h;

    Ok(())
}
