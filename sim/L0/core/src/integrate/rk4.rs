//! Standard 4-stage Runge-Kutta integration.
//!
//! Implements MuJoCo's `mj_RungeKutta` using the classic RK4 Butcher tableau.

use crate::constraint::impedance::MJ_MINVAL;
use crate::forward::MjStage;
use crate::jacobian::mj_integrate_pos_explicit;
use crate::types::flags::{actuator_disabled, disabled};
use crate::types::{
    ActuatorDynamics, DISABLE_ACTUATION, Data, ENABLE_SLEEP, Model, SleepState, StepError,
};

/// Standard 4-stage Runge-Kutta integration matching MuJoCo's `mj_RungeKutta`.
///
/// # Preconditions
/// - `data.forward()` has already been called (qacc is valid for stage 0).
///
/// # Algorithm
/// Uses the classic RK4 Butcher tableau. Stage 0 reuses qacc from the preceding
/// `forward()` call. Stages 1-3 each call `forward_skip_sensors()` at trial
/// states, which fires `cb_passive` and `cb_control` as `forward()` does, so
/// both callbacks fire four times per step (five on a step that puts a tree to
/// sleep, whose advance runs the forward pass once more).
/// Position integration uses `mj_integrate_pos_explicit()` for quaternion correctness.
///
/// A tree asleep at the step's start is held there through the stages and the
/// advance, and a step that puts a tree to sleep evaluates its forward pass
/// again from the position stage (registry `D-RK4-SLEEP`), so collision and a
/// contact filter callback run once more on that step than in MuJoCo, whose
/// pass starts from the velocity stage.
///
/// After this function returns, derived quantities (xpos, contacts, forces, etc.)
/// are stage 3's, as MuJoCo's, and do NOT correspond to the final (qpos, qvel)
/// state; on a step that puts a tree to sleep they are the step's start state's.
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

    // 0. Sensor history samples, at the step's start state (registry
    // `D-HISTORY-RK4-SENSOR`): MuJoCo inserts them after the stages, from the
    // last stage's kinematics, velocities and forces, so its delayed sensors
    // that read those read values that are not the sensor's at the sample's
    // time.
    crate::history::advance_sensors(model, data);

    // 1. SAVE initial state
    data.rk4_qpos_saved.copy_from(&data.qpos);
    data.rk4_qvel[0].copy_from(&data.qvel);
    data.rk4_qacc[0].copy_from(&data.qacc);
    data.rk4_act_saved.copy_from(&data.act);
    data.rk4_act_dot[0].copy_from(&data.act_dot);
    let t0 = data.time;

    // The trees asleep at the step's start stay there: their dofs take no
    // acceleration in the stages or the advance, so they keep the zero
    // velocity a sleeping tree has (the step's first forward pass woke any
    // other) and their positions. MuJoCo's stages move them with the stale
    // acceleration a sleeping dof keeps, and the stage's forward pass then
    // wakes the tree (registry `D-RK4-SLEEP`). A tree woken during a stage is
    // held until the step ends.
    let hold = model.enableflags & ENABLE_SLEEP != 0 && data.nv_awake < nv;
    if hold {
        data.rk4_dof_asleep.fill(true);
        for idx in 0..data.nv_awake {
            let v = data.dof_awake_ind[idx];
            data.rk4_dof_asleep[v] = false;
        }
    }

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
        if hold {
            hold_asleep_dofs(data);
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

        // 2e. Activation trial state: act = act_saved + h * Σ A[(i-1)*3+j] * act_dot[j],
        // whatever the actuator's dynamics, and unclamped, as MuJoCo's stages
        // (engine_forward.c:1086-1089); the advance alone takes the exact
        // filter and the range. A disabled actuator's act_dot too: it acts
        // with no force, and the advance leaves its activation (S4.8).
        for act_i in 0..model.nu {
            let act_adr = model.actuator_act_adr[act_i];
            for k in 0..model.actuator_act_num[act_i] {
                let a = act_adr + k;
                let mut sum = 0.0;
                for j in 0..3 {
                    sum += RK4_A[(i - 1) * 3 + j] * data.rk4_act_dot[j][a];
                }
                data.act[a] = data.rk4_act_saved[a] + h * sum;
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
    if hold {
        hold_asleep_dofs(data);
    }

    // 4. ADVANCE from the step's start state, as MuJoCo's `mj_RungeKutta`
    // resets state and time and calls `mj_advance` (engine_forward.c:1113-1121):
    // the control history at the start time, with `ctrl` as the last stage's
    // control callback left it; the activations; the sleep step; velocity and
    // position; time; plugins; warmstart.
    data.time = t0;
    data.qpos.copy_from(&data.rk4_qpos_saved);
    data.qvel.copy_from(&data.rk4_qvel[0]);
    data.act.copy_from(&data.rk4_act_saved);
    crate::history::advance_ctrl(model, data, t0);

    // Advance activation from saved initial state. With actuation disabled
    // the activations stay where they were, not clamped either; a disabled
    // actuator's advance with act_dot = 0 (`mj_advance`, `engine_forward.c:886-895`).
    if !disabled(model, DISABLE_ACTUATION) {
        for act_i in 0..model.nu {
            let act_adr = model.actuator_act_adr[act_i];
            let is_disabled = actuator_disabled(model, act_i);
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
                        let tau = model.actuator_dynprm[act_i][0].max(MJ_MINVAL);
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
    }

    // The sleep step, at the start state (`rk4_dX_*` survive its forward
    // pass). Its forward pass starts from the position stage, so the poses
    // and all that follows from them are the start state's. MuJoCo's skips
    // that stage and keeps the last stage's poses, which the next step's
    // kinematics find changed, waking the tree (registry `D-RK4-SLEEP`).
    if crate::island::mj_sleep(model, data) > 0 {
        data.reforward_after_sleep(model, MjStage::None)?;
    }

    // Velocity over the awake dofs
    let use_dof_ind = model.enableflags & ENABLE_SLEEP != 0 && data.nv_awake < model.nv;
    let nv_adv = if use_dof_ind { data.nv_awake } else { nv };
    for idx in 0..nv_adv {
        let v = if use_dof_ind {
            data.dof_awake_ind[idx]
        } else {
            idx
        };
        data.qvel[v] += data.rk4_dX_acc[v] * h;
    }

    // Position on manifold from saved initial position, of the awake bodies:
    // a sleeping body's joints keep the start state (MuJoCo integrates over
    // `body_awake_ind`).
    mj_integrate_pos_explicit(
        model,
        &mut data.qpos,
        &data.rk4_qpos_saved,
        &data.rk4_dX_vel,
        h,
    );
    if model.enableflags & ENABLE_SLEEP != 0 {
        for jnt in 0..model.njnt {
            if data.body_sleep_state[model.jnt_body[jnt]] == SleepState::Asleep {
                let adr = model.jnt_qpos_adr[jnt];
                let n = model.jnt_type[jnt].nq();
                data.qpos.as_mut_slice()[adr..adr + n]
                    .copy_from_slice(&data.rk4_qpos_saved.as_slice()[adr..adr + n]);
            }
        }
    }

    // (§27F) Flex vertex positions now integrated by mj_integrate_pos_explicit above.

    data.time = t0 + h;
    crate::integrate::advance_plugins(model, data);

    // Save qacc for the next step's warmstart (§15.9), as `mj_advance` ends.
    data.qacc_warmstart.copy_from(&data.qacc);

    Ok(())
}

/// Zero the weighted acceleration of the dofs held asleep.
fn hold_asleep_dofs(data: &mut Data) {
    for (v, &asleep) in data.rk4_dof_asleep.iter().enumerate() {
        if asleep {
            data.rk4_dX_acc[v] = 0.0;
        }
    }
}
