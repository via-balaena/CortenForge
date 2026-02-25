//! Transmission and actuation pipeline.
//!
//! Computes actuator transmissions (site, body), actuator lengths/velocities,
//! activation dynamics, and actuator forces. Corresponds to MuJoCo's
//! actuation portion of `engine_forward.c`.

use super::muscle::muscle_activation_dynamics;
use crate::jacobian::mj_jac_site;
use crate::tendon::{accumulate_point_jacobian, apply_tendon_force, subquat};
use crate::types::{
    ActuatorDynamics, ActuatorTransmission, BiasType, Contact, Data, GainType, Model,
};
use nalgebra::{DVector, Vector3};

use crate::types::flags::{actuator_disabled, disabled};
use crate::types::validation::{MIN_VAL, is_bad};
use crate::types::warning::{Warning, mj_warning};
use crate::types::{DISABLE_ACTUATION, DISABLE_CLAMPCTRL, DISABLE_GRAVITY};

use super::muscle::{muscle_gain_length, muscle_gain_velocity, muscle_passive_force};

/// Compute actuator length and moment for site transmissions only.
///
/// For each Site-transmission actuator, computes `actuator_length` and
/// `actuator_moment` (nv-vector). Joint/Tendon transmissions are untouched.
/// Must run after `mj_fwd_position` (needs site poses) and before
/// `mj_sensor_pos` (which reads `actuator_length`).
pub fn mj_transmission_site(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] != ActuatorTransmission::Site {
            continue;
        }

        let sid = model.actuator_trnid[i][0];
        let refid = model.actuator_trnid[i][1];
        let gear = model.actuator_gear[i];

        let (jac_t, jac_r) = mj_jac_site(model, data, sid);

        if refid == usize::MAX {
            // Mode A — no refsite: length = 0, moment from wrench projection.
            data.actuator_length[i] = 0.0;

            // Wrench in world frame: rotate gear from site-local to world.
            let wrench_t = data.site_xmat[sid] * Vector3::new(gear[0], gear[1], gear[2]);
            let wrench_r = data.site_xmat[sid] * Vector3::new(gear[3], gear[4], gear[5]);

            // moment = J_trans^T @ wrench_t + J_rot^T @ wrench_r
            let moment = &mut data.actuator_moment[i];
            for dof in 0..model.nv {
                moment[dof] = jac_t.column(dof).dot(&wrench_t) + jac_r.column(dof).dot(&wrench_r);
            }
        } else {
            // Mode B — with refsite: length from position/quaternion differences,
            // moment from difference Jacobian.
            let (ref_jac_t, ref_jac_r) = mj_jac_site(model, data, refid);

            // Translational length: gear[0:3] · (R_ref^T @ (p_site - p_ref))
            let dp = data.site_xpos[sid] - data.site_xpos[refid];
            let dp_ref = data.site_xmat[refid].transpose() * dp;
            let len_trans = gear[0] * dp_ref.x + gear[1] * dp_ref.y + gear[2] * dp_ref.z;

            // Rotational length: gear[3:6] · subquat(q_site, q_ref)
            let dq = subquat(&data.site_xquat[sid], &data.site_xquat[refid]);
            let len_rot = gear[3] * dq.x + gear[4] * dq.y + gear[5] * dq.z;

            data.actuator_length[i] = len_trans + len_rot;

            // Difference Jacobian with common-ancestor DOF zeroing.
            let mut diff_jac_t = &jac_t - &ref_jac_t;
            let mut diff_jac_r = &jac_r - &ref_jac_r;

            // Zero common-ancestor DOF columns.
            let b0 = model.site_body[sid];
            let b1 = model.site_body[refid];
            // Find lowest common ancestor.
            let mut ancestors_b0 = Vec::new();
            {
                let mut b = b0;
                while b != 0 {
                    ancestors_b0.push(b);
                    b = model.body_parent[b];
                }
                ancestors_b0.push(0);
            }
            let bca = {
                let mut b = b1;
                loop {
                    if ancestors_b0.contains(&b) {
                        break b;
                    }
                    if b == 0 {
                        break 0;
                    }
                    b = model.body_parent[b];
                }
            };
            // Zero DOFs for bca and all its ancestors up to root.
            {
                let mut b = bca;
                loop {
                    let jnt_start = model.body_jnt_adr[b];
                    let jnt_end = jnt_start + model.body_jnt_num[b];
                    for jnt_id in jnt_start..jnt_end {
                        let dof_start = model.jnt_dof_adr[jnt_id];
                        let ndof = model.jnt_type[jnt_id].nv();
                        for d in dof_start..(dof_start + ndof) {
                            for k in 0..3 {
                                diff_jac_t[(k, d)] = 0.0;
                                diff_jac_r[(k, d)] = 0.0;
                            }
                        }
                    }
                    if b == 0 {
                        break;
                    }
                    b = model.body_parent[b];
                }
            }

            // Wrench in world frame: rotate gear by refsite frame.
            let wrench_t = data.site_xmat[refid] * Vector3::new(gear[0], gear[1], gear[2]);
            let wrench_r = data.site_xmat[refid] * Vector3::new(gear[3], gear[4], gear[5]);

            // moment = diff_J_trans^T @ wrench_t + diff_J_rot^T @ wrench_r
            let moment = &mut data.actuator_moment[i];
            for dof in 0..model.nv {
                moment[dof] =
                    diff_jac_t.column(dof).dot(&wrench_t) + diff_jac_r.column(dof).dot(&wrench_r);
            }
        }
    }
}

/// Compute contact normal Jacobian difference: `n^T · (J_p(b2) - J_p(b1))`.
///
/// `b1` is the body of `geom1`, `b2` is the body of `geom2`. Follows MuJoCo's
/// `mj_jacDifPair(b1, b2)` convention which computes `J(b2) - J(b1)` (second
/// argument minus first). The contact normal points from `geom[0]` toward
/// `geom[1]`.
fn compute_contact_normal_jacobian(model: &Model, data: &Data, contact: &Contact) -> DVector<f64> {
    let nv = model.nv;
    let b1 = model.geom_body[contact.geom1];
    let b2 = model.geom_body[contact.geom2];
    let point = &contact.pos;
    let normal = &contact.normal;

    let mut j_normal = DVector::zeros(nv);

    // Walk kinematic chain for body2 (+1 contribution)
    // MuJoCo convention: jacdifp = J(b2) - J(b1)
    accumulate_point_jacobian(
        model,
        &data.xpos,
        &data.xquat,
        &mut j_normal,
        b2,
        point,
        normal,
        1.0,
    );
    // Walk kinematic chain for body1 (-1 contribution)
    accumulate_point_jacobian(
        model,
        &data.xpos,
        &data.xquat,
        &mut j_normal,
        b1,
        point,
        normal,
        -1.0,
    );

    j_normal
}

/// Compute adhesion moment arm for a single body-transmission actuator.
///
/// Iterates all contacts involving the target body, accumulates the contact
/// normal Jacobians, negates and averages. The negation ensures positive ctrl
/// produces an attractive force (toward the contact surface).
///
/// Gear is NOT applied — MuJoCo's `mjTRN_BODY` case omits gear entirely.
/// Force magnitude is controlled by `gainprm[0]` (the `gain` attribute).
fn mj_transmission_body(model: &Model, data: &mut Data, actuator_id: usize) {
    let body_id = model.actuator_trnid[actuator_id][0];
    let nv = model.nv;
    let mut moment = DVector::zeros(nv);
    let mut count = 0usize;

    for c in 0..data.ncon {
        let contact = &data.contacts[c];

        // Skip flex contacts (MuJoCo: geom < 0)
        if contact.flex_vertex.is_some() {
            continue;
        }

        // Skip contacts not involving target body
        let b1 = model.geom_body[contact.geom1];
        let b2 = model.geom_body[contact.geom2];
        if b1 != body_id && b2 != body_id {
            continue;
        }

        // Compute normal^T · (J(b2, pos) - J(b1, pos))
        let j_normal = compute_contact_normal_jacobian(model, data, contact);
        moment += &j_normal;
        count += 1;
    }

    if count > 0 {
        // Negate and average. NO gear scaling (MuJoCo omits gear for body
        // transmission — force magnitude is controlled by gainprm, not gear).
        #[allow(clippy::cast_precision_loss)] // contact count is always small
        let inv_count = count as f64;
        moment *= -1.0 / inv_count;
    }

    data.actuator_moment[actuator_id] = moment;

    // Body transmission has no length concept
    data.actuator_length[actuator_id] = 0.0;
}

/// Dispatch body transmission computation for all body-transmission actuators.
///
/// Must run after `mj_collision()` (needs contacts) and before `mj_sensor_pos()`
/// (which reads `actuator_length`).
pub fn mj_transmission_body_dispatch(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if model.actuator_trntype[i] == ActuatorTransmission::Body {
            mj_transmission_body(model, data, i);
        }
    }
}

/// Compute actuator length and velocity from transmission state.
///
/// For each actuator, computes `actuator_length = gear * transmission_length`
/// and `actuator_velocity = gear * transmission_velocity`.
/// Called after `mj_fwd_velocity()` (which provides `ten_velocity`).
pub fn mj_actuator_length(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        let gear = model.actuator_gear[i][0];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let jid = model.actuator_trnid[i][0];
                if jid < model.njnt {
                    // Joint transmission only meaningful for Hinge/Slide (scalar qpos).
                    let nv = model.jnt_type[jid].nv();
                    if nv == 1 {
                        let qadr = model.jnt_qpos_adr[jid];
                        let dof_adr = model.jnt_dof_adr[jid];
                        data.actuator_length[i] = gear * data.qpos[qadr];
                        data.actuator_velocity[i] = gear * data.qvel[dof_adr];
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tid = model.actuator_trnid[i][0];
                if tid < model.ntendon {
                    data.actuator_length[i] = gear * data.ten_length[tid];
                    data.actuator_velocity[i] = gear * data.ten_velocity[tid];
                }
            }
            ActuatorTransmission::Site => {
                // Length already set by mj_transmission_site (position stage).
                // Velocity from cached moment:
                data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
            }
            ActuatorTransmission::Body => {
                // Length already set to 0 by mj_transmission_body (position stage).
                // Velocity from cached moment (same as Site):
                data.actuator_velocity[i] = data.actuator_moment[i].dot(&data.qvel);
            }
        }
    }
}

/// Compute next activation state: integrate act_dot and clamp to actrange.
///
/// Matches MuJoCo's `mj_nextActivation()` in `engine_forward.c`:
/// 1. Integrates activation using Euler (all types) or exact exponential (FilterExact).
/// 2. Clamps to `[actrange[0], actrange[1]]` when `actlimited` is true.
///
/// The `act_dot` input should be computed from the UNCLAMPED current activation
/// (MuJoCo computes act_dot before clamping). Clamping only applies after integration.
///
/// This function is called in two contexts:
/// - **`actearly` force computation**: predict next-step activation for force generation.
/// - **Integration step**: update the actual activation state.
pub fn mj_next_activation(
    model: &Model,
    actuator_id: usize,
    current_act: f64,
    act_dot: f64,
) -> f64 {
    let mut act = current_act;

    // Integration step
    if model.actuator_dyntype[actuator_id] == ActuatorDynamics::FilterExact {
        let tau = model.actuator_dynprm[actuator_id][0].max(1e-10);
        act += act_dot * tau * (1.0 - (-model.timestep / tau).exp());
    } else {
        act += act_dot * model.timestep;
    }

    // Activation clamping (§34)
    if model.actuator_actlimited[actuator_id] {
        let range = model.actuator_actrange[actuator_id];
        act = act.clamp(range.0, range.1);
    }

    act
}

/// Compute actuator forces from control inputs, activation dynamics, and muscle FLV curves.
///
/// This function:
/// 1. Computes activation derivatives (`data.act_dot`) without modifying `data.act`.
/// 2. Computes actuator force using gain/bias (muscle FLV for muscles, raw input for others).
/// 3. Clamps control inputs and output forces to their declared ranges.
/// 4. Maps actuator force to joint forces via the transmission.
pub fn mj_fwd_actuation(model: &Model, data: &mut Data) {
    // DT-79: Invoke user control callback (if set).
    // Called at the start of actuation, before force computation,
    // so the callback can set ctrl values (e.g., from an RL policy).
    if let Some(ref cb) = model.cb_control {
        (cb.0)(model, data);
    }

    // S4.8: Unconditional zero of per-actuator forces (matches MuJoCo).
    for i in 0..model.nu {
        data.actuator_force[i] = 0.0;
    }

    // S4.8: Guard — skip force computation when no actuators or actuation disabled.
    if model.nu == 0 || disabled(model, DISABLE_ACTUATION) {
        data.qfrc_actuator.fill(0.0);
        return;
    }

    data.qfrc_actuator.fill(0.0);

    // S8d: Bad ctrl validation — zero all ctrl on first bad value.
    for i in 0..model.nu {
        if is_bad(data.ctrl[i]) {
            #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
            mj_warning(data, Warning::BadCtrl, i as i32);
            for j in 0..model.nu {
                data.ctrl[j] = 0.0;
            }
            break;
        }
    }

    for i in 0..model.nu {
        // S7d: Skip force computation for per-group disabled actuators.
        if actuator_disabled(model, i) {
            continue;
        }

        // --- Phase 1: Activation dynamics (compute act_dot, do NOT integrate) ---
        // S4.9: Skip ctrl clamping when DISABLE_CLAMPCTRL is set.
        let ctrl = if disabled(model, DISABLE_CLAMPCTRL) {
            data.ctrl[i]
        } else {
            data.ctrl[i].clamp(model.actuator_ctrlrange[i].0, model.actuator_ctrlrange[i].1)
        };

        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => ctrl,
            ActuatorDynamics::Muscle => {
                let act_adr = model.actuator_act_adr[i];
                // act_dot computed from UNCLAMPED current activation (MuJoCo convention)
                data.act_dot[act_adr] =
                    muscle_activation_dynamics(ctrl, data.act[act_adr], &model.actuator_dynprm[i]);
                if model.actuator_actearly[i] {
                    // §34: predict next-step activation (integrated + clamped)
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
            }
            ActuatorDynamics::Filter | ActuatorDynamics::FilterExact => {
                // First-order filter: d(act)/dt = (ctrl - act) / tau
                // Filter uses Euler integration, FilterExact uses exact integration.
                // Both compute the same act_dot here; the difference is in integrate().
                let act_adr = model.actuator_act_adr[i];
                let tau = model.actuator_dynprm[i][0].max(1e-10);
                data.act_dot[act_adr] = (ctrl - data.act[act_adr]) / tau;
                if model.actuator_actearly[i] {
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
            }
            ActuatorDynamics::Integrator => {
                // Integrator: d(act)/dt = ctrl
                let act_adr = model.actuator_act_adr[i];
                data.act_dot[act_adr] = ctrl;
                if model.actuator_actearly[i] {
                    mj_next_activation(model, i, data.act[act_adr], data.act_dot[act_adr])
                } else {
                    data.act[act_adr]
                }
            }
        };

        // --- Phase 2: Force generation (gain * input + bias) ---
        let length = data.actuator_length[i];
        let velocity = data.actuator_velocity[i];

        let gain = match model.actuator_gaintype[i] {
            GainType::Fixed => model.actuator_gainprm[i][0],
            GainType::Affine => {
                model.actuator_gainprm[i][0]
                    + model.actuator_gainprm[i][1] * length
                    + model.actuator_gainprm[i][2] * velocity
            }
            GainType::Muscle => {
                // Muscle gain = -F0 * FL(L) * FV(V)
                let prm = &model.actuator_gainprm[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2]; // resolved by compute_muscle_params()

                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);
                let norm_vel = velocity / (l0 * prm[6]).max(1e-10);

                let fl = muscle_gain_length(norm_len, prm[4], prm[5]);
                let fv = muscle_gain_velocity(norm_vel, prm[8]);
                -f0 * fl * fv
            }
        };

        let bias = match model.actuator_biastype[i] {
            BiasType::None => 0.0,
            BiasType::Affine => {
                model.actuator_biasprm[i][0]
                    + model.actuator_biasprm[i][1] * length
                    + model.actuator_biasprm[i][2] * velocity
            }
            BiasType::Muscle => {
                let prm = &model.actuator_gainprm[i]; // muscle uses gainprm for both
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2];

                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);

                let fp = muscle_passive_force(norm_len, prm[5], prm[7]);
                -f0 * fp
            }
        };

        let force = gain * input + bias;

        // Clamp to force range
        let force = force.clamp(
            model.actuator_forcerange[i].0,
            model.actuator_forcerange[i].1,
        );
        data.actuator_force[i] = force;

        // --- Phase 3: Transmission (actuator_force → generalized forces) ---
        // qfrc_actuator += moment^T * actuator_force
        // where moment = gear * raw_Jacobian.
        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];
        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                if trnid < model.njnt {
                    let dof_adr = model.jnt_dof_adr[trnid];
                    let nv = model.jnt_type[trnid].nv();
                    if nv > 0 {
                        data.qfrc_actuator[dof_adr] += gear * force;
                    }
                }
            }
            ActuatorTransmission::Tendon => {
                let tendon_id = trnid;
                if tendon_id < model.ntendon {
                    apply_tendon_force(
                        model,
                        &data.ten_J[tendon_id],
                        model.tendon_type[tendon_id],
                        tendon_id,
                        gear * force,
                        &mut data.qfrc_actuator,
                    );
                }
            }
            ActuatorTransmission::Site | ActuatorTransmission::Body => {
                // Use cached moment vector from transmission function.
                for dof in 0..model.nv {
                    let m = data.actuator_moment[i][dof];
                    if m != 0.0 {
                        data.qfrc_actuator[dof] += m * force;
                    }
                }
            }
        }
    }
}

/// Route gravcomp forces to `qfrc_actuator` for joints with `jnt_actgravcomp`.
///
/// Must run AFTER `mj_fwd_passive()` (which computes `qfrc_gravcomp` via
/// `mj_gravcomp()`). Called from `forward_core()` in pipeline order.
/// Gated on `DISABLE_GRAVITY` and `DISABLE_ACTUATION` — in MuJoCo this
/// routing lives inside `mj_fwdActuation()`, so disabling actuation
/// implicitly skips it. We gate explicitly since it's a separate function.
pub fn mj_gravcomp_to_actuator(model: &Model, data: &mut Data) {
    if model.ngravcomp == 0
        || disabled(model, DISABLE_GRAVITY)
        || disabled(model, DISABLE_ACTUATION)
        || model.gravity.norm() < MIN_VAL
    {
        return;
    }
    for jnt in 0..model.njnt {
        if !model.jnt_actgravcomp[jnt] {
            continue;
        }
        let dofadr = model.jnt_dof_adr[jnt];
        let dofnum = model.jnt_type[jnt].nv();
        for i in 0..dofnum {
            data.qfrc_actuator[dofadr + i] += data.qfrc_gravcomp[dofadr + i];
        }
    }
}
