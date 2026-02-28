//! Muscle force-length-velocity curves and parameter precomputation.
//!
//! Implements MuJoCo's piecewise-quadratic muscle model: active FL curve,
//! FV curve, passive force, and activation dynamics. Also houses the
//! build-time `compute_actuator_params` that resolves `lengthrange`, `acc0`,
//! and `F0` for actuators. Corresponds to `engine_util_misc.c`.

use crate::dynamics::crba::mj_crba;
use crate::forward::position::mj_fwd_position;
use crate::jacobian::{mj_jac_point_axis, mj_jac_site};
use crate::linalg::mj_solve_sparse;
use crate::types::{
    ActuatorDynamics, ActuatorTransmission, BiasType, GainType, LengthRangeError, LengthRangeMode,
    LengthRangeOpt, Model, TendonType,
};
use nalgebra::{DVector, Vector3};

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
pub fn muscle_activation_dynamics(ctrl: f64, act: f64, dynprm: &[f64; 10]) -> f64 {
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
        // MuJoCo: x = 0.5*(dctrl/smoothing_width + 1)
        tau_deact + (tau_act - tau_deact) * sigmoid(0.5 * (dctrl / tausmooth + 1.0))
    };

    dctrl / tau.max(1e-10)
}

impl Model {
    /// Compute actuator parameters: lengthrange, acc0, dampratio, and F0.
    ///
    /// For each actuator:
    ///   1. Computes `actuator_lengthrange` from tendon/joint limits (gear-scaled).
    ///   2. Runs a forward pass at `qpos0` to get M, then computes
    ///      `acc0 = ||M^{-1} * moment||` for **every** actuator (not just muscles).
    ///   3. Converts positive `biasprm[2]` (dampratio) to negative damping for
    ///      position-like actuators using reflected inertia.
    ///   4. Resolves muscle `F0` (gainprm\[2\]) when `force < 0`: `F0 = scale / acc0`.
    ///
    /// Must be called after `compute_ancestors()` and `compute_implicit_params()`,
    /// and after all tendon/actuator fields are populated.
    pub fn compute_actuator_params(&mut self) {
        if self.nu == 0 {
            return;
        }

        // --- Phase 1: Compute actuator_lengthrange from limits ---
        // This applies to all actuator types (not just muscles) — limit-based
        // lengthrange is unconditional. (MuJoCo's mj_setLengthRange mode filtering
        // only applies to the simulation-based fallback, not the limit copy.)
        for i in 0..self.nu {
            let gear = self.actuator_gear[i][0];

            // ── Intentional MuJoCo deviation (DT-106) ──────────────────────
            // MuJoCo's `mj_setLengthRange` uselimit path copies raw `jnt_range`
            // directly to `actuator_lengthrange` WITHOUT gear scaling. But its
            // simulation-based path measures `actuator_length` at joint extremes,
            // and `actuator_length = gear * qpos`, so that path DOES produce
            // gear-scaled range. For gear != 1, these two MuJoCo paths disagree.
            //
            // We gear-scale in both paths. This is dimensionally correct:
            //   actuator_length   = gear * qpos
            //   actuator_lengthrange = gear * jnt_range
            // Both live in actuator-length units, which is required for muscle
            // normalization: `norm_len = prm[0] + (actuator_length - lr[0]) / L0`
            // where L0 = lr[1] - lr[0]. If lr were in joint-space but
            // actuator_length in actuator-space, the division would be
            // dimensionally wrong for gear != 1.
            //
            // Verified empirically with MuJoCo 3.5.0 — see verify_spec_a.py.
            // ─────────────────────────────────────────────────────────────────
            // actuator_length = gear * transmission_length,
            // so lengthrange = gear * transmission_lengthrange.
            // If gear < 0, min/max swap.
            let scale_range = |lo: f64, hi: f64| -> (f64, f64) {
                let a = gear * lo;
                let b = gear * hi;
                (a.min(b), a.max(b))
            };

            match self.actuator_trntype[i] {
                ActuatorTransmission::Tendon => {
                    let tid = self.actuator_trnid[i][0];
                    if tid < self.ntendon {
                        if self.tendon_limited[tid] {
                            let (lo, hi) = self.tendon_range[tid];
                            self.actuator_lengthrange[i] = scale_range(lo, hi);
                        } else {
                            // For unlimited spatial tendons, wrap-array DOF lookup is
                            // semantically wrong (wrap_objid holds site/geom IDs, not
                            // DOFs). Skip estimation and leave lengthrange = (0, 0).
                            if self.tendon_type[tid] == TendonType::Spatial {
                                eprintln!(
                                    "Warning: Unlimited spatial tendon {tid}: cannot \
                                     estimate actuator_lengthrange from joint ranges. \
                                     Specify explicit lengthrange in MJCF for muscle \
                                     actuators."
                                );
                                continue;
                            }
                            // For unlimited fixed tendons: estimate from joint ranges.
                            // Fixed tendon length = Σ coef_i * q_i, so extremes come
                            // from each joint at its range limit (sign-aware).
                            let adr = self.tendon_adr[tid];
                            let num = self.tendon_num[tid];
                            let (mut lmin, mut lmax) = (0.0, 0.0);
                            for w in adr..(adr + num) {
                                let dof = self.wrap_objid[w];
                                let coef = self.wrap_prm[w];
                                if let Some(jid) =
                                    (0..self.njnt).find(|&j| self.jnt_dof_adr[j] == dof)
                                {
                                    let (qlo, qhi) = self.jnt_range[jid];
                                    if coef >= 0.0 {
                                        lmin += coef * qlo;
                                        lmax += coef * qhi;
                                    } else {
                                        lmin += coef * qhi;
                                        lmax += coef * qlo;
                                    }
                                }
                            }
                            self.actuator_lengthrange[i] = scale_range(lmin, lmax);
                        }
                    }
                }
                ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
                    let jid = self.actuator_trnid[i][0];
                    if jid < self.njnt && self.jnt_limited[jid] {
                        let (lo, hi) = self.jnt_range[jid];
                        self.actuator_lengthrange[i] = scale_range(lo, hi);
                    }
                }
                ActuatorTransmission::Site
                | ActuatorTransmission::Body
                | ActuatorTransmission::SliderCrank => {
                    // Site/SliderCrank: configuration-dependent (full FK required).
                    // Body: no length concept. All: no-op, leave at (0, 0).
                }
            }
        }

        // --- Phase 2: Forward pass at qpos0 for acc0 (ALL actuators) ---
        // FK populates cinert (body spatial inertias) and tendon Jacobians (ten_J).
        // CRBA builds M and factors it (L^T D L). For fixed tendons, J is constructed
        // from wrap_objid/wrap_prm (constant). For spatial tendons, J is read from
        // data.ten_J[tid] (populated by mj_fwd_tendon_spatial during FK).
        let mut data = self.make_data(); // qpos = qpos0
        mj_fwd_position(self, &mut data); // FK: cinert from qpos0
        mj_crba(self, &mut data); // Mass matrix M + sparse factorization

        // Compute acc0 for ALL actuators (not just muscles).
        // MuJoCo's set0() in engine_setconst.c computes acc0 for every actuator.
        // Store moment vectors for reuse in the dampratio phase.
        let mut moment_vecs: Vec<DVector<f64>> = Vec::with_capacity(self.nu);

        for i in 0..self.nu {
            // NO muscle-only guard — compute acc0 for every actuator
            let j_vec = build_actuator_moment(self, &data, i);

            // Solve M * x = J using the sparse L^T D L factorization from CRBA.
            let mut x = j_vec.clone();
            let (rowadr, rownnz, colind) = self.qld_csr();
            mj_solve_sparse(
                rowadr,
                rownnz,
                colind,
                &data.qLD_data,
                &data.qLD_diag_inv,
                &mut x,
            );

            // acc0 = ||M^{-1} J||_2
            self.actuator_acc0[i] = x.norm().max(1e-10);

            moment_vecs.push(j_vec);
        }

        // --- Phase 3: dampratio → damping for position-like actuators ---
        // MuJoCo's set0() in engine_setconst.c converts positive biasprm[2]
        // (interpreted as dampratio) to negative damping using reflected inertia.

        for (i, j_vec) in moment_vecs.iter().enumerate() {
            let kp = self.actuator_gainprm[i][0];

            // Position-actuator fingerprint: gainprm[0] == -biasprm[1]
            // MuJoCo uses exact != comparison (both values derive from the same kp).
            #[allow(clippy::float_cmp)]
            if kp != -self.actuator_biasprm[i][1] {
                continue;
            }

            // Only positive biasprm[2] is interpreted as dampratio.
            // Zero or negative means explicit kv (already correct sign).
            if self.actuator_biasprm[i][2] <= 0.0 {
                continue;
            }

            // Compute reflected inertia: sum of dof_M0[dof] / trn^2
            // where dof_M0[dof] = qM[(dof,dof)] at qpos0.
            #[allow(clippy::items_after_statements)]
            const MJ_MINVAL: f64 = 1e-15;
            let mut mass = 0.0;
            for dof in 0..self.nv {
                let trn = j_vec[dof].abs();
                let trn2 = trn * trn;
                if trn2 > MJ_MINVAL {
                    let dof_m0 = data.qM[(dof, dof)];
                    mass += dof_m0 / trn2;
                }
            }

            // Critical damping formula: kv = dampratio * 2 * sqrt(kp * mass)
            let dampratio = self.actuator_biasprm[i][2];
            let damping = dampratio * 2.0 * (kp * mass).sqrt();

            // Store as negative (bias convention: b2 * velocity opposes motion)
            self.actuator_biasprm[i][2] = -damping;
        }

        // --- Phase 3b: Simulation-based lengthrange for actuators that need it ---
        // Must run AFTER Phase 2 (acc0) because the muscle force model in
        // mj_fwd_actuation uses F0 = scale / acc0 during the simulation.
        // With acc0 = 0, F0 would be ~2e17, causing massive passive muscle
        // forces and numerical instability.
        //
        // This matches MuJoCo's execution order: mj_setConst() → set0() computes
        // acc0 first, then the compiler calls mj_setLengthRange() separately.
        //
        // The `useexisting: true` default causes the LR simulation to skip any
        // actuator whose lengthrange was already set (lo < hi) by Phase 1.
        let lr_opt = LengthRangeOpt::default();
        // mj_set_length_range never fails — individual actuator convergence
        // failures are silently skipped (matching MuJoCo's behavior).
        mj_set_length_range(self, &lr_opt);

        // --- Phase 4: Resolve muscle F0 ---
        for i in 0..self.nu {
            if !matches!(
                self.actuator_dyntype[i],
                ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle
            ) {
                continue;
            }
            if self.actuator_gainprm[i][2] < 0.0 {
                // force < 0 means auto-compute: F0 = scale / acc0
                self.actuator_gainprm[i][2] = self.actuator_gainprm[i][3] / self.actuator_acc0[i];
                // Sync biasprm (MuJoCo layout: muscles share gain/bias parameters)
                self.actuator_biasprm[i][2] = self.actuator_gainprm[i][2];
            }
        }
    }
}

/// Build the actuator moment vector (transmission Jacobian) for a single actuator.
///
/// This is the dense equivalent of MuJoCo's sparse `actuator_moment` row. It maps
/// a unit actuator force to generalized forces: `qfrc = moment * force`.
///
/// Shared between `compute_actuator_params` (acc0, dampratio) and future
/// `mj_set_length_range` (simulation-based estimation).
fn build_actuator_moment(
    model: &Model,
    data: &crate::types::Data,
    actuator_idx: usize,
) -> DVector<f64> {
    let gear = model.actuator_gear[actuator_idx][0];
    let mut j_vec = DVector::zeros(model.nv);

    match model.actuator_trntype[actuator_idx] {
        ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
            let jid = model.actuator_trnid[actuator_idx][0];
            if jid < model.njnt {
                let dof_adr = model.jnt_dof_adr[jid];
                j_vec[dof_adr] = gear;
            }
        }
        ActuatorTransmission::Tendon => {
            let tid = model.actuator_trnid[actuator_idx][0];
            if tid < model.ntendon {
                match model.tendon_type[tid] {
                    TendonType::Fixed => {
                        let adr = model.tendon_adr[tid];
                        let num = model.tendon_num[tid];
                        for w in adr..(adr + num) {
                            let dof_adr = model.wrap_objid[w];
                            let coef = model.wrap_prm[w];
                            if dof_adr < model.nv {
                                j_vec[dof_adr] = gear * coef;
                            }
                        }
                    }
                    TendonType::Spatial => {
                        // Configuration-dependent J — use ten_J from FK at qpos0.
                        for dof in 0..model.nv {
                            j_vec[dof] = gear * data.ten_J[tid][dof];
                        }
                    }
                }
            }
        }
        ActuatorTransmission::Site => {
            let sid = model.actuator_trnid[actuator_idx][0];
            let refid = model.actuator_trnid[actuator_idx][1];
            let (jac_t, jac_r) = mj_jac_site(model, data, sid);
            let full_gear = model.actuator_gear[actuator_idx];

            if refid == usize::MAX {
                // Mode A: wrench in world frame via site rotation.
                let wrench_t =
                    data.site_xmat[sid] * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                let wrench_r =
                    data.site_xmat[sid] * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                for dof in 0..model.nv {
                    j_vec[dof] =
                        jac_t.column(dof).dot(&wrench_t) + jac_r.column(dof).dot(&wrench_r);
                }
            } else {
                // Mode B: difference Jacobian with common-ancestor zeroing.
                let (ref_jac_t, ref_jac_r) = mj_jac_site(model, data, refid);
                let mut diff_t = &jac_t - &ref_jac_t;
                let mut diff_r = &jac_r - &ref_jac_r;

                // Zero common-ancestor DOF columns.
                let b0 = model.site_body[sid];
                let b1 = model.site_body[refid];
                let mut ancestors = Vec::new();
                {
                    let mut b = b0;
                    while b != 0 {
                        ancestors.push(b);
                        b = model.body_parent[b];
                    }
                    ancestors.push(0);
                }
                let bca = {
                    let mut b = b1;
                    loop {
                        if ancestors.contains(&b) {
                            break b;
                        }
                        if b == 0 {
                            break 0;
                        }
                        b = model.body_parent[b];
                    }
                };
                {
                    let mut b = bca;
                    loop {
                        let js = model.body_jnt_adr[b];
                        let je = js + model.body_jnt_num[b];
                        for jid in js..je {
                            let ds = model.jnt_dof_adr[jid];
                            let nd = model.jnt_type[jid].nv();
                            for d in ds..(ds + nd) {
                                for k in 0..3 {
                                    diff_t[(k, d)] = 0.0;
                                    diff_r[(k, d)] = 0.0;
                                }
                            }
                        }
                        if b == 0 {
                            break;
                        }
                        b = model.body_parent[b];
                    }
                }

                let wrench_t =
                    data.site_xmat[refid] * Vector3::new(full_gear[0], full_gear[1], full_gear[2]);
                let wrench_r =
                    data.site_xmat[refid] * Vector3::new(full_gear[3], full_gear[4], full_gear[5]);
                for dof in 0..model.nv {
                    j_vec[dof] =
                        diff_t.column(dof).dot(&wrench_t) + diff_r.column(dof).dot(&wrench_r);
                }
            }
        }
        ActuatorTransmission::Body => {
            // At qpos0 there are no contacts, so J-vector is zero.
            // No-op: j_vec already initialized to zeros.
        }
        ActuatorTransmission::SliderCrank => {
            let crank_id = model.actuator_trnid[actuator_idx][0];
            let slider_id = model.actuator_trnid[actuator_idx][1];
            let rod = model.actuator_cranklength[actuator_idx];

            let axis: Vector3<f64> = data.site_xmat[slider_id].column(2).into_owned();
            let vec3 = data.site_xpos[crank_id] - data.site_xpos[slider_id];
            let av = vec3.dot(&axis);
            let det = av * av + rod * rod - vec3.dot(&vec3);

            let (dl_daxis, dl_dvec);
            if det <= 0.0 {
                dl_daxis = vec3;
                dl_dvec = axis.clone_owned();
            } else {
                let sdet = det.sqrt();
                let factor = 1.0 - av / sdet;
                dl_dvec = axis.scale(factor) + vec3.scale(1.0 / sdet);
                dl_daxis = vec3.scale(factor);
            }

            let slider_body = model.site_body[slider_id];
            let (jac_slider_point, jac_axis) =
                mj_jac_point_axis(model, data, slider_body, &data.site_xpos[slider_id], &axis);
            let (jac_crank_point, _) = mj_jac_site(model, data, crank_id);
            let jac_vec = &jac_crank_point - &jac_slider_point;

            for j in 0..model.nv {
                let mut m = 0.0;
                for k in 0..3 {
                    m += dl_daxis[k] * jac_axis[(k, j)] + dl_dvec[k] * jac_vec[(k, j)];
                }
                j_vec[j] = m * gear;
            }
        }
    }
    j_vec
}

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
fn mj_set_length_range(model: &mut Model, opt: &LengthRangeOpt) {
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

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant,
    clippy::uninlined_format_args,
    clippy::while_float
)]
mod muscle_tests {
    use super::*;
    use crate::types::{
        ActuatorDynamics, ActuatorTransmission, BiasType, GainType, Integrator, MjJointType, Model,
    };
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    // Helper: build a single-hinge model with a muscle actuator via joint transmission.
    // The muscle has the specified gear, inertia I from a box body.
    fn build_muscle_model_joint(gear: f64) -> Model {
        let mut model = Model::empty();

        // Add a body with a hinge joint
        model.nbody = 2; // world + body
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("arm".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        // Hinge joint
        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(-1.0, 1.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        // DOFs
        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None]; // root DOF
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        // Muscle actuator (joint transmission)
        model.nu = 1;
        model.na = 1;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::Muscle];
        model.actuator_trnid = vec![[0, usize::MAX]]; // joint 0
        model.actuator_gear = vec![[gear, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("muscle".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![1];
        model.actuator_gaintype = vec![GainType::Muscle];
        model.actuator_biastype = vec![BiasType::Muscle];
        model.actuator_dynprm = vec![[0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]; // default tau_act, tau_deact
        model.actuator_gainprm = vec![[0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]];
        model.actuator_biasprm = vec![[0.75, 1.05, -1.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]];
        model.actuator_lengthrange = vec![(0.0, 0.0)]; // will be computed
        model.actuator_acc0 = vec![0.0]; // will be computed
        // Muscle default: actlimited=true, actrange=[0,1] (§34 S5)
        model.actuator_actlimited = vec![true];
        model.actuator_actrange = vec![(0.0, 1.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        // Pre-compute
        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        model.compute_actuator_params();

        model
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

    // ---- AC #7: act_num fix ----
    #[test]
    fn test_muscle_act_num_is_one() {
        let model = build_muscle_model_joint(1.0);
        assert_eq!(model.actuator_act_num[0], 1);
        assert_eq!(model.na, 1);
    }

    // ---- AC #1: Activation dynamics correctness ----
    #[test]
    fn test_activation_dynamics_ramp_up() {
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        data.ctrl[0] = 1.0;

        for _ in 0..100 {
            data.step(&model).expect("step failed");
        }

        assert!(
            data.act[0] >= 0.95,
            "After 100 steps at dt=0.001 with ctrl=1, act should reach >= 0.95, got {}",
            data.act[0]
        );
    }

    #[test]
    fn test_activation_dynamics_ramp_down() {
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        // Start at act=1, ctrl=0
        data.act[0] = 1.0;
        data.ctrl[0] = 0.0;

        for _ in 0..200 {
            data.step(&model).expect("step failed");
        }

        assert!(
            data.act[0] <= 0.10,
            "After 200 steps at dt=0.001 with ctrl=0, act should fall to <= 0.10, got {}",
            data.act[0]
        );
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

    // ---- AC #6: End-to-end muscle force ----
    #[test]
    fn test_muscle_force_at_optimal_length() {
        // A muscle at optimal length (L=1.0), isometric (V=0), with act=1.0
        // should produce actuator_force = -F0 * (FL*FV*act + FP)
        // FL(1.0) = 1.0, FV(0.0) = 1.0, FP(1.0) = 0.0
        // so force = -F0 * (1*1*1 + 0) = -F0
        let model = build_muscle_model_joint(1.0);
        let mut data = model.make_data();
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        // Set qpos so that actuator_length maps to normalized L=1.0
        // lengthrange = gear * jnt_range = 1.0 * (-1.0, 1.0)
        // L0 = (1 - (-1)) / (1.05 - 0.75) = 2/0.3 = 6.667
        // norm_len = 0.75 + (len - (-1)) / L0
        // For norm_len = 1.0: len = (-1) + (1.0 - 0.75) * L0 = -1 + 0.25*6.667 = 0.667
        let l0 = 2.0 / 0.3;
        let target_len = -1.0 + (1.0 - 0.75) * l0;
        data.qpos[0] = target_len; // gear=1, so actuator_length = qpos

        // Run one forward pass to populate actuator_force
        data.forward(&model).expect("forward failed");

        let f0 = model.actuator_gainprm[0][2]; // resolved F0
        assert!(f0 > 0.0, "F0 should be positive, got {}", f0);

        let expected = -f0; // at optimal, FL=FV=1, FP=0, act=1 => force = -F0
        let actual = data.actuator_force[0];
        assert!(
            (actual - expected).abs() / f0.abs() < 0.05,
            "Force at optimal length should be ~-F0={}, got {}",
            expected,
            actual
        );
    }

    // ---- AC #8: Non-muscle DynType::None unchanged ----
    #[test]
    fn test_motor_actuator_unchanged() {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(); 2];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("body".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(-3.14, 3.14)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        // Motor actuator (DynType::None)
        model.nu = 1;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::None];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[2.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("motor".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![0];
        model.actuator_gaintype = vec![GainType::Fixed];
        model.actuator_biastype = vec![BiasType::None];
        model.actuator_dynprm = vec![[0.0; 10]];
        model.actuator_gainprm = vec![{
            let mut p = [0.0; 9];
            p[0] = 1.0; // Motor: unit gain
            p
        }];
        model.actuator_biasprm = vec![[0.0; 9]];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];
        model.actuator_actlimited = vec![false];
        model.actuator_actrange = vec![(0.0, 0.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.gravity = Vector3::zeros(); // no gravity for clean test

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        let mut data = model.make_data();
        data.ctrl[0] = 0.5;

        data.forward(&model).expect("forward failed");

        // Motor: force = ctrl = 0.5, qfrc = gear * force = 2.0 * 0.5 = 1.0
        assert!(
            (data.qfrc_actuator[0] - 1.0).abs() < 1e-10,
            "Motor qfrc should be gear * ctrl = 1.0, got {}",
            data.qfrc_actuator[0]
        );
        assert!(
            (data.actuator_force[0] - 0.5).abs() < 1e-10,
            "Motor actuator_force should be ctrl = 0.5, got {}",
            data.actuator_force[0]
        );
    }

    // ---- AC #9: Control clamping ----
    #[test]
    fn test_control_clamping() {
        let mut model = build_muscle_model_joint(1.0);
        model.actuator_ctrlrange[0] = (-1.0, 1.0);

        let mut data = model.make_data();
        data.ctrl[0] = 5.0; // should be clamped to 1.0

        data.forward(&model).expect("forward failed");

        // With ctrl clamped to 1.0 and act=0.0, act_dot should match ctrl=1.0 case
        let act_dot_clamped = data.act_dot[0];

        data.reset(&model);
        data.ctrl[0] = 1.0;
        data.forward(&model).expect("forward failed");

        let act_dot_normal = data.act_dot[0];

        assert!(
            (act_dot_clamped - act_dot_normal).abs() < 1e-10,
            "Clamped ctrl=5 should produce same act_dot as ctrl=1: {} vs {}",
            act_dot_clamped,
            act_dot_normal
        );
    }

    // ---- AC #10: Force clamping ----
    #[test]
    fn test_force_clamping() {
        let mut model = build_muscle_model_joint(1.0);
        model.actuator_forcerange[0] = (-100.0, 0.0);

        let mut data = model.make_data();
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        // Set qpos to optimal length (same as AC #6)
        let l0 = 2.0 / 0.3;
        let target_len = -1.0 + (1.0 - 0.75) * l0;
        data.qpos[0] = target_len;

        data.forward(&model).expect("forward failed");

        let f0 = model.actuator_gainprm[0][2];
        // Without clamping, force would be ~ -F0 (which is large negative)
        // With forcerange = (-100, 0), it should be clamped to -100 if |F0| > 100
        if f0 > 100.0 {
            assert!(
                (data.actuator_force[0] - (-100.0)).abs() < 1e-6,
                "Force should be clamped to -100, got {}",
                data.actuator_force[0]
            );
        } else {
            // F0 <= 100, force = -F0 which is within range
            assert!(
                data.actuator_force[0] >= -100.0,
                "Force should be >= -100, got {}",
                data.actuator_force[0]
            );
        }
    }

    // ---- AC #12: MJCF round-trip ----
    // (Tested in integration tests via sim-mjcf; here we test the builder output)
    #[test]
    fn test_muscle_params_transferred() {
        let model = build_muscle_model_joint(1.0);
        assert_eq!(
            model.actuator_dynprm[0],
            [0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
        assert!((model.actuator_gainprm[0][0] - 0.75).abs() < 1e-10); // range[0]
        assert!((model.actuator_gainprm[0][1] - 1.05).abs() < 1e-10); // range[1]
        assert!((model.actuator_gainprm[0][4] - 0.5).abs() < 1e-10); // lmin
        assert!((model.actuator_gainprm[0][5] - 1.6).abs() < 1e-10); // lmax
        assert!((model.actuator_gainprm[0][6] - 1.5).abs() < 1e-10); // vmax
        assert!((model.actuator_gainprm[0][7] - 1.3).abs() < 1e-10); // fpmax
        assert!((model.actuator_gainprm[0][8] - 1.2).abs() < 1e-10); // fvmax
    }

    // ---- AC #13: acc0 computation ----
    #[test]
    fn test_acc0_single_hinge() {
        // For a single-DOF hinge with inertia I and gear g:
        // M = I (scalar), J = g (scalar)
        // acc0 = ||M^{-1} * J|| = |g| / I
        let gear = 2.0;
        let model = build_muscle_model_joint(gear);

        // Inertia about Y axis (hinge axis) for a body with inertia (0.1, 0.1, 0.1)
        // Since it's a hinge about Y, the effective inertia is 0.1 (the Y component)
        // Plus mass * offset^2 (but body is at origin, so just 0.1)
        // Actually, CRBA computes the full mass matrix. For a single body at origin
        // with diagonal inertia, M[0,0] = I_yy = 0.1 (for Y-axis hinge)
        let expected_acc0 = gear.abs() / 0.1;
        let actual_acc0 = model.actuator_acc0[0];

        assert!(
            (actual_acc0 - expected_acc0).abs() / expected_acc0 < 0.01,
            "acc0 should be |gear|/I = {}, got {}",
            expected_acc0,
            actual_acc0
        );
    }

    // ---- AC #14: F0 auto-computation ----
    #[test]
    fn test_f0_auto_computation() {
        let model = build_muscle_model_joint(1.0);
        let acc0 = model.actuator_acc0[0];
        let f0 = model.actuator_gainprm[0][2]; // was -1.0, should be resolved
        let scale = model.actuator_gainprm[0][3]; // 200.0

        assert!(
            f0 > 0.0,
            "F0 should be resolved to positive value, got {}",
            f0
        );
        let expected_f0 = scale / acc0;
        assert!(
            (f0 - expected_f0).abs() < 1e-6,
            "F0 should be scale/acc0 = {}, got {}",
            expected_f0,
            f0
        );
    }

    #[test]
    fn test_f0_explicit_not_overridden() {
        let mut model = build_muscle_model_joint(1.0);
        // Re-set with explicit force (positive, so not auto-computed)
        model.actuator_gainprm[0][2] = 500.0;
        model.actuator_biasprm[0][2] = 500.0;

        // Re-run compute_actuator_params (it should NOT override explicit F0)
        model.compute_actuator_params();

        assert!(
            (model.actuator_gainprm[0][2] - 500.0).abs() < 1e-10,
            "Explicit F0=500 should not be overridden, got {}",
            model.actuator_gainprm[0][2]
        );
    }

    // ---- AC #15: RK4 activation integration ----
    #[test]
    fn test_rk4_activation_single_step() {
        let mut model = build_muscle_model_joint(1.0);
        model.integrator = Integrator::RungeKutta4;
        model.gravity = Vector3::zeros(); // isolate activation dynamics

        let mut data = model.make_data();
        data.ctrl[0] = 1.0;
        data.act[0] = 0.0;

        // Run 1 RK4 step
        data.step(&model).expect("step failed");

        // Manually compute RK4: 4 evaluations of muscle_activation_dynamics
        let dt = model.timestep;
        let dynprm = &model.actuator_dynprm[0];

        let k1 = muscle_activation_dynamics(1.0, 0.0, dynprm);
        let act1 = (0.0 + 0.5 * dt * k1).clamp(0.0, 1.0);
        let k2 = muscle_activation_dynamics(1.0, act1, dynprm);
        let act2 = (0.0 + 0.5 * dt * k2).clamp(0.0, 1.0);
        let k3 = muscle_activation_dynamics(1.0, act2, dynprm);
        let act3 = (0.0 + dt * k3).clamp(0.0, 1.0);
        let k4 = muscle_activation_dynamics(1.0, act3, dynprm);

        let act_expected = 0.0 + dt * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

        assert!(
            (data.act[0] - act_expected).abs() < 1e-6,
            "RK4 activation after 1 step: expected {}, got {}",
            act_expected,
            data.act[0]
        );
    }

    // ---- AC #11: Existing tests pass (verified by running full suite) ----
    // This is an implicit acceptance criterion verified at the integration test level.

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

    // =========================================================================
    // Spec A tests — acc0 for non-muscle actuators, dampratio, lengthrange
    // =========================================================================

    /// Build a single-hinge model with a non-muscle actuator (motor/position).
    ///
    /// Parameters:
    /// - `gear`: actuator gear ratio
    /// - `inertia_yy`: I_yy of the body (hinge about Y axis)
    /// - `dyntype`: actuator dynamics type
    /// - `gaintype`: actuator gain type
    /// - `biastype`: actuator bias type
    /// - `gainprm`: gain parameters (9 elements)
    /// - `biasprm`: bias parameters (9 elements)
    fn build_nonmuscle_model(
        gear: f64,
        inertia_yy: f64,
        dyntype: ActuatorDynamics,
        gaintype: GainType,
        biastype: BiasType,
        gainprm: [f64; 9],
        biasprm: [f64; 9],
    ) -> Model {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, inertia_yy, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("body".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![false];
        model.jnt_range = vec![(0.0, 0.0)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        model.nu = 1;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![dyntype];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[gear, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("act".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![0];
        model.actuator_gaintype = vec![gaintype];
        model.actuator_biastype = vec![biastype];
        model.actuator_dynprm = vec![[0.0; 10]];
        model.actuator_gainprm = vec![gainprm];
        model.actuator_biasprm = vec![biasprm];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];
        model.actuator_actlimited = vec![false];
        model.actuator_actrange = vec![(0.0, 0.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        model.compute_actuator_params();

        model
    }

    // ---- T1: acc0 for motor actuator → AC1 ----
    #[test]
    fn test_acc0_motor_actuator() {
        // Motor: gaintype=Fixed, biastype=None, gear=2.0, I_yy=0.1
        // For a single hinge: M = I_yy = 0.1, J = gear = 2.0
        // acc0 = ||M^{-1} J|| = |gear / I_yy| = 2.0 / 0.1 = 20.0
        let model = build_nonmuscle_model(
            2.0, // gear
            0.1, // I_yy
            ActuatorDynamics::None,
            GainType::Fixed,
            BiasType::None,
            {
                let mut gp = [0.0; 9];
                gp[0] = 1.0; // gain = 1
                gp
            },
            [0.0; 9],
        );

        assert!(
            (model.actuator_acc0[0] - 20.0).abs() < 1e-6,
            "Motor acc0 should be |gear|/I_yy = 20.0, got {}",
            model.actuator_acc0[0]
        );
    }

    // ---- T2: acc0 for position actuator → AC2 ----
    #[test]
    fn test_acc0_position_actuator() {
        // Position: kp=100, kv=10, gear=1.0, I_yy=0.1
        // acc0 = ||M^{-1} J|| = |gear / I_yy| = 1.0 / 0.1 = 10.0
        let kp = 100.0;
        let kv = 10.0;
        let model = build_nonmuscle_model(
            1.0,
            0.1,
            ActuatorDynamics::None,
            GainType::Fixed,
            BiasType::Affine,
            {
                let mut gp = [0.0; 9];
                gp[0] = kp;
                gp
            },
            {
                let mut bp = [0.0; 9];
                bp[1] = -kp;
                bp[2] = -kv;
                bp
            },
        );

        assert!(
            (model.actuator_acc0[0] - 10.0).abs() < 1e-6,
            "Position acc0 should be |gear|/I_yy = 10.0, got {}",
            model.actuator_acc0[0]
        );
    }

    // ---- T3: acc0 muscle regression → AC3 ----
    #[test]
    fn test_acc0_muscle_regression() {
        // Existing muscle model: gear=2.0, I_yy=0.1
        // acc0 should still be 20.0 ± 0.2
        let model = build_muscle_model_joint(2.0);
        assert!(
            (model.actuator_acc0[0] - 20.0).abs() < 0.2,
            "Muscle acc0 regression: expected 20.0 ± 0.2, got {}",
            model.actuator_acc0[0]
        );
    }

    // ---- T4: dampratio conversion → AC4 ----
    #[test]
    fn test_dampratio_conversion() {
        // Position actuator with kp=100, dampratio=1.0, gear=1.0, I_yy=0.1
        // reflected_mass = I_yy / gear^2 = 0.1
        // kv = dampratio * 2 * sqrt(kp * mass) = 1.0 * 2 * sqrt(100 * 0.1) = 2*sqrt(10)
        // biasprm[2] = -kv ≈ -6.3246
        let kp = 100.0;
        let model = build_nonmuscle_model(
            1.0,
            0.1,
            ActuatorDynamics::None,
            GainType::Fixed,
            BiasType::Affine,
            {
                let mut gp = [0.0; 9];
                gp[0] = kp;
                gp
            },
            {
                let mut bp = [0.0; 9];
                bp[1] = -kp; // position-actuator fingerprint
                bp[2] = 1.0; // positive = dampratio
                bp
            },
        );

        let expected = -2.0 * (100.0_f64 * 0.1).sqrt();
        assert!(
            (model.actuator_biasprm[0][2] - expected).abs() < 1e-4,
            "dampratio conversion: expected {}, got {}",
            expected,
            model.actuator_biasprm[0][2]
        );
    }

    // ---- T5: dampratio skip for motor → AC5 ----
    #[test]
    fn test_dampratio_skip_motor() {
        // Motor: gainprm[0]=1.0, biasprm[1]=0.0
        // Fingerprint fails: 1.0 != -0.0 → skip
        let model = build_nonmuscle_model(
            1.0,
            0.1,
            ActuatorDynamics::None,
            GainType::Fixed,
            BiasType::None,
            {
                let mut gp = [0.0; 9];
                gp[0] = 1.0;
                gp
            },
            [0.0; 9],
        );

        assert_eq!(
            model.actuator_biasprm[0][2], 0.0,
            "Motor biasprm[2] should remain 0.0"
        );
    }

    // ---- T6: dampratio skip for explicit kv → AC6 ----
    #[test]
    fn test_dampratio_skip_explicit_kv() {
        // Position actuator with explicit kv=10.0 → biasprm[2] = -10.0
        // Negative → already explicit, skip.
        let kp = 100.0;
        let model = build_nonmuscle_model(
            1.0,
            0.1,
            ActuatorDynamics::None,
            GainType::Fixed,
            BiasType::Affine,
            {
                let mut gp = [0.0; 9];
                gp[0] = kp;
                gp
            },
            {
                let mut bp = [0.0; 9];
                bp[1] = -kp;
                bp[2] = -10.0; // explicit kv (negative)
                bp
            },
        );

        assert_eq!(
            model.actuator_biasprm[0][2], -10.0,
            "Explicit kv should remain -10.0"
        );
    }

    // ---- S1 supplementary: acc0 with nv=0 ----
    #[test]
    fn test_acc0_no_dofs() {
        // Model with nu=1 but nv=0 → acc0 should not panic and remain 0 (or clamp)
        let mut model = Model::empty();
        model.nbody = 1;
        model.body_parent = vec![0];
        model.body_rootid = vec![0];
        model.body_jnt_adr = vec![0];
        model.body_jnt_num = vec![0];
        model.body_dof_adr = vec![0];
        model.body_dof_num = vec![0];
        model.body_geom_adr = vec![0];
        model.body_geom_num = vec![0];
        model.body_pos = vec![Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity()];
        model.body_ipos = vec![Vector3::zeros()];
        model.body_iquat = vec![UnitQuaternion::identity()];
        model.body_mass = vec![0.0];
        model.body_inertia = vec![Vector3::zeros()];
        model.body_name = vec![Some("world".to_string())];
        model.body_subtreemass = vec![0.0];

        model.njnt = 0;
        model.nq = 0;
        model.nv = 0;

        model.nu = 1;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::None];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("act".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![0];
        model.actuator_gaintype = vec![GainType::Fixed];
        model.actuator_biastype = vec![BiasType::None];
        model.actuator_dynprm = vec![[0.0; 10]];
        model.actuator_gainprm = vec![[0.0; 9]];
        model.actuator_biasprm = vec![[0.0; 9]];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];
        model.actuator_actlimited = vec![false];
        model.actuator_actrange = vec![(0.0, 0.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(0);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]];
        model.body_ancestor_mask = vec![vec![]];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        model.compute_actuator_params();

        // acc0 with nv=0: norm of empty vector = 0, clamped to 1e-10
        assert!(
            model.actuator_acc0[0] <= 1e-9,
            "acc0 with nv=0 should be ~0 (clamped to 1e-10), got {}",
            model.actuator_acc0[0]
        );
    }

    // ---- T13: Multi-body acc0 + dampratio conformance → AC14 ----

    /// Build a 2-link chain (world → link1 → link2) with 2 Y-axis hinges and
    /// 2 actuators: position on joint 0, motor on joint 1.
    ///
    /// Analytical mass matrix at qpos=0:
    ///   M = [[I1_yy + I2_yy, I2_yy], [I2_yy, I2_yy]]
    ///   with I1_yy=0.2, I2_yy=0.1 → M = [[0.3, 0.1], [0.1, 0.1]]
    ///
    /// M⁻¹ = (1/det) * [[0.1, -0.1], [-0.1, 0.3]]  where det = 0.02
    ///      = [[5, -5], [-5, 15]]
    ///
    /// Actuator 0 (position, joint 0, gear=1): J = [1, 0]
    ///   M⁻¹ J = [5, -5]  → acc0 = √50 = 5√2 ≈ 7.0710678
    ///   reflected_mass = M[0,0]/1² = 0.3
    ///   kv = 1.0 * 2 * √(200 * 0.3) = 2√60 → biasprm[2] = -2√60 ≈ -15.4919334
    ///
    /// Actuator 1 (motor, joint 1, gear=3): J = [0, 3]
    ///   M⁻¹ J = [-15, 45]  → acc0 = √(225+2025) = √2250 = 15√10 ≈ 47.4341649
    fn build_two_link_chain() -> Model {
        let mut model = Model::empty();

        // 3 bodies: world (0), link1 (1), link2 (2)
        model.nbody = 3;
        model.body_parent = vec![0, 0, 1]; // link1 → world, link2 → link1
        model.body_rootid = vec![0, 1, 1];
        model.body_jnt_adr = vec![0, 0, 1];
        model.body_jnt_num = vec![0, 1, 1];
        model.body_dof_adr = vec![0, 0, 1];
        model.body_dof_num = vec![0, 1, 1];
        model.body_geom_adr = vec![0, 0, 0];
        model.body_geom_num = vec![0, 0, 0];
        model.body_pos = vec![Vector3::zeros(); 3];
        model.body_quat = vec![UnitQuaternion::identity(); 3];
        model.body_ipos = vec![Vector3::zeros(); 3];
        model.body_iquat = vec![UnitQuaternion::identity(); 3];
        model.body_mass = vec![0.0, 2.0, 1.0];
        // I_yy: link1=0.2, link2=0.1 (only Y matters for Y-axis hinges)
        model.body_inertia = vec![
            Vector3::zeros(),
            Vector3::new(0.1, 0.2, 0.1),
            Vector3::new(0.05, 0.1, 0.05),
        ];
        model.body_name = vec![
            Some("world".to_string()),
            Some("link1".to_string()),
            Some("link2".to_string()),
        ];
        model.body_subtreemass = vec![3.0, 3.0, 1.0];

        // 2 hinge joints (both Y-axis)
        model.njnt = 2;
        model.nq = 2;
        model.nv = 2;
        model.jnt_type = vec![MjJointType::Hinge, MjJointType::Hinge];
        model.jnt_body = vec![1, 2];
        model.jnt_qpos_adr = vec![0, 1];
        model.jnt_dof_adr = vec![0, 1];
        model.jnt_pos = vec![Vector3::zeros(); 2];
        model.jnt_axis = vec![Vector3::y(), Vector3::y()];
        model.jnt_limited = vec![false, false];
        model.jnt_range = vec![(0.0, 0.0); 2];
        model.jnt_stiffness = vec![0.0; 2];
        model.jnt_springref = vec![0.0; 2];
        model.jnt_damping = vec![0.0; 2];
        model.jnt_armature = vec![0.0; 2];
        model.jnt_solref = vec![[0.02, 1.0]; 2];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]; 2];
        model.jnt_name = vec![Some("hinge0".to_string()), Some("hinge1".to_string())];

        // 2 DOFs
        model.dof_body = vec![1, 2];
        model.dof_jnt = vec![0, 1];
        model.dof_parent = vec![None, Some(0)]; // dof1 parent = dof0
        model.dof_armature = vec![0.0; 2];
        model.dof_damping = vec![0.0; 2];
        model.dof_frictionloss = vec![0.0; 2];

        // 2 actuators: position on joint 0, motor on joint 1
        let kp = 200.0;
        model.nu = 2;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint; 2];
        model.actuator_dyntype = vec![ActuatorDynamics::None; 2];
        model.actuator_trnid = vec![[0, usize::MAX], [1, usize::MAX]];
        model.actuator_gear = vec![
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], // position: gear=1
            [3.0, 0.0, 0.0, 0.0, 0.0, 0.0], // motor: gear=3
        ];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY); 2];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY); 2];
        model.actuator_name = vec![Some("pos_act".to_string()), Some("motor_act".to_string())];
        model.actuator_act_adr = vec![0, 0];
        model.actuator_act_num = vec![0, 0];
        model.actuator_gaintype = vec![GainType::Fixed, GainType::Fixed];
        model.actuator_biastype = vec![BiasType::Affine, BiasType::None];
        model.actuator_dynprm = vec![[0.0; 10]; 2];
        model.actuator_gainprm = vec![
            {
                let mut gp = [0.0; 9];
                gp[0] = kp; // kp=200
                gp
            },
            {
                let mut gp = [0.0; 9];
                gp[0] = 1.0; // motor gain=1
                gp
            },
        ];
        model.actuator_biasprm = vec![
            {
                let mut bp = [0.0; 9];
                bp[1] = -kp; // position-actuator fingerprint
                bp[2] = 1.0; // positive = dampratio
                bp
            },
            [0.0; 9], // motor: no bias
        ];
        model.actuator_lengthrange = vec![(0.0, 0.0); 2];
        model.actuator_acc0 = vec![0.0; 2];
        model.actuator_actlimited = vec![false; 2];
        model.actuator_actrange = vec![(0.0, 0.0); 2];
        model.actuator_actearly = vec![false; 2];

        model.qpos0 = DVector::zeros(2);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 3];
        model.body_ancestor_mask = vec![vec![]; 3];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        model.compute_actuator_params();

        model
    }

    #[test]
    fn test_acc0_dampratio_multibody() {
        let model = build_two_link_chain();

        // Actuator 0 (position, joint 0, gear=1):
        // acc0 = 5√2 ≈ 7.0710678
        let expected_acc0_0 = 5.0 * 2.0_f64.sqrt();
        assert!(
            (model.actuator_acc0[0] - expected_acc0_0).abs() < 1e-6,
            "acc0[0] should be 5√2 = {:.7}, got {}",
            expected_acc0_0,
            model.actuator_acc0[0]
        );

        // Actuator 1 (motor, joint 1, gear=3):
        // acc0 = 15√10 ≈ 47.4341649
        let expected_acc0_1 = 15.0 * 10.0_f64.sqrt();
        assert!(
            (model.actuator_acc0[1] - expected_acc0_1).abs() < 1e-6,
            "acc0[1] should be 15√10 = {:.7}, got {}",
            expected_acc0_1,
            model.actuator_acc0[1]
        );

        // Dampratio conversion on actuator 0:
        // reflected_mass = M[0,0]/gear² = 0.3/1 = 0.3
        // kv = 1.0 * 2 * √(200 * 0.3) = 2√60 ≈ 15.4919334
        // biasprm[2] = -kv
        let expected_kv = 2.0 * 60.0_f64.sqrt();
        assert!(
            (model.actuator_biasprm[0][2] + expected_kv).abs() < 1e-6,
            "biasprm[0][2] should be -2√60 = {:.7}, got {}",
            -expected_kv,
            model.actuator_biasprm[0][2]
        );

        // Motor actuator biasprm[2] should remain 0 (no dampratio conversion)
        assert_eq!(
            model.actuator_biasprm[1][2], 0.0,
            "Motor biasprm[2] should remain 0.0"
        );
    }

    // ---- LR simulation test: non-muscle with mode=All → AC8/AC9 coverage ----

    /// Motor actuator on a limited slide joint: mj_set_length_range with
    /// mode=All should produce nonzero lengthrange from the simulation.
    ///
    /// This tests the LR simulation infrastructure without the circular
    /// dependency that muscles have (muscle force needs lengthrange, but
    /// LR estimation runs muscle force). Using a motor with mode=All
    /// exercises the simulation path on a non-muscle actuator.
    #[test]
    fn test_lengthrange_simulation_motor_mode_all() {
        // Build a motor on a *limited* slide joint. Phase 1 will set
        // lengthrange from limits. Then we clear it and call
        // mj_set_length_range with mode=All + useexisting=false to force
        // the simulation path.
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 2.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("body".to_string())];
        model.body_subtreemass = vec![2.0, 2.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Slide];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::z()]; // slide along Z
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(-1.0, 1.0)]; // limited: -1 to 1 meter
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("slide".to_string())];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        model.nu = 1;
        model.na = 0;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::None];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("motor".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![0];
        model.actuator_gaintype = vec![GainType::Fixed];
        model.actuator_biastype = vec![BiasType::None];
        model.actuator_dynprm = vec![[0.0; 10]];
        model.actuator_gainprm = vec![{
            let mut gp = [0.0; 9];
            gp[0] = 1.0;
            gp
        }];
        model.actuator_biasprm = vec![[0.0; 9]];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];
        model.actuator_actlimited = vec![false];
        model.actuator_actrange = vec![(0.0, 0.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();

        // Run Phase 2 manually to get acc0 (needed by simulation)
        model.compute_actuator_params();

        // Phase 1 already set lengthrange from limits: (-1, 1)
        // Clear it to force the simulation path
        model.actuator_lengthrange[0] = (0.0, 0.0);

        // Call mj_set_length_range with mode=All, useexisting=false
        let opt = LengthRangeOpt {
            mode: LengthRangeMode::All,
            useexisting: false,
            uselimit: false, // skip limit copy — force simulation
            ..LengthRangeOpt::default()
        };
        mj_set_length_range(&mut model, &opt);

        let (lo, hi) = model.actuator_lengthrange[0];

        // The simulation should find range close to joint limits (-1, 1) * gear(1).
        // With 10s of simulation, ±2N force on 2kg body, velocity damping, and
        // joint limit constraints, the body converges to within a few percent of
        // the limits. Tolerance of 0.1 (10%) is conservative but catches real
        // regressions — a broken simulation would produce (0, 0) or wildly wrong values.
        assert!(
            lo < -0.9,
            "LR simulation lo should be near -1.0, got {}",
            lo
        );
        assert!(hi > 0.9, "LR simulation hi should be near 1.0, got {}", hi);
    }
}

// ── Hill-type muscle tests (Spec C) ──

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::approx_constant,
    clippy::uninlined_format_args
)]
mod hill_muscle_tests {
    use super::*;
    use crate::forward::actuation::{
        hill_active_fl, hill_force_velocity, hill_passive_fl, mj_fwd_actuation,
    };
    use crate::types::{
        ActuatorDynamics, ActuatorTransmission, BiasType, GainType, MjJointType, Model,
    };
    use nalgebra::{DVector, UnitQuaternion, Vector3};

    /// Build a single-hinge model with a HillMuscle actuator.
    fn build_hill_model(
        f0: f64,
        opt_len: f64,
        slack_len: f64,
        vmax: f64,
        pennation: f64,
        stiffness: f64,
        gear: f64,
    ) -> Model {
        let mut model = Model::empty();

        model.nbody = 2;
        model.body_parent = vec![0, 0];
        model.body_rootid = vec![0, 1];
        model.body_jnt_adr = vec![0, 0];
        model.body_jnt_num = vec![0, 1];
        model.body_dof_adr = vec![0, 0];
        model.body_dof_num = vec![0, 1];
        model.body_geom_adr = vec![0, 0];
        model.body_geom_num = vec![0, 0];
        model.body_pos = vec![Vector3::zeros(), Vector3::zeros()];
        model.body_quat = vec![UnitQuaternion::identity(); 2];
        model.body_ipos = vec![Vector3::zeros(); 2];
        model.body_iquat = vec![UnitQuaternion::identity(); 2];
        model.body_mass = vec![0.0, 1.0];
        model.body_inertia = vec![Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1)];
        model.body_name = vec![Some("world".to_string()), Some("arm".to_string())];
        model.body_subtreemass = vec![1.0, 1.0];

        model.njnt = 1;
        model.nq = 1;
        model.nv = 1;
        model.jnt_type = vec![MjJointType::Hinge];
        model.jnt_body = vec![1];
        model.jnt_qpos_adr = vec![0];
        model.jnt_dof_adr = vec![0];
        model.jnt_pos = vec![Vector3::zeros()];
        model.jnt_axis = vec![Vector3::y()];
        model.jnt_limited = vec![true];
        model.jnt_range = vec![(-0.5, 0.5)];
        model.jnt_stiffness = vec![0.0];
        model.jnt_springref = vec![0.0];
        model.jnt_damping = vec![0.0];
        model.jnt_armature = vec![0.0];
        model.jnt_solref = vec![[0.02, 1.0]];
        model.jnt_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.jnt_name = vec![Some("hinge".to_string())];

        model.dof_body = vec![1];
        model.dof_jnt = vec![0];
        model.dof_parent = vec![None];
        model.dof_armature = vec![0.0];
        model.dof_damping = vec![0.0];
        model.dof_frictionloss = vec![0.0];

        // HillMuscle actuator
        model.nu = 1;
        model.na = 1;
        model.actuator_trntype = vec![ActuatorTransmission::Joint];
        model.actuator_dyntype = vec![ActuatorDynamics::HillMuscle];
        model.actuator_trnid = vec![[0, usize::MAX]];
        model.actuator_gear = vec![[gear, 0.0, 0.0, 0.0, 0.0, 0.0]];
        model.actuator_ctrlrange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_forcerange = vec![(f64::NEG_INFINITY, f64::INFINITY)];
        model.actuator_name = vec![Some("hill".to_string())];
        model.actuator_act_adr = vec![0];
        model.actuator_act_num = vec![1];
        model.actuator_gaintype = vec![GainType::HillMuscle];
        model.actuator_biastype = vec![BiasType::HillMuscle];
        // dynprm: [tau_act, tau_deact, tausmooth, ...]
        model.actuator_dynprm = vec![[0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]];
        // gainprm: [range_lo, range_hi, F0, scale, opt_len, slack_len, vmax, penn, stiff]
        model.actuator_gainprm = vec![[
            0.75, 1.05, f0, 200.0, opt_len, slack_len, vmax, pennation, stiffness,
        ]];
        model.actuator_biasprm = vec![[0.0; 9]];
        model.actuator_lengthrange = vec![(0.0, 0.0)];
        model.actuator_acc0 = vec![0.0];
        model.actuator_actlimited = vec![true];
        model.actuator_actrange = vec![(0.0, 1.0)];
        model.actuator_actearly = vec![false];

        model.qpos0 = DVector::zeros(1);
        model.timestep = 0.001;

        model.body_ancestor_joints = vec![vec![]; 2];
        model.body_ancestor_mask = vec![vec![]; 2];
        model.compute_ancestors();
        model.compute_implicit_params();
        model.compute_qld_csr_metadata();
        model.compute_actuator_params();

        model
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
    #[test]
    fn test_hill_activation_smooth_blend() {
        let prm = [0.01, 0.04, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let act_dot = muscle_activation_dynamics(0.55, 0.5, &prm);
        // V7: act_clamped=0.5, tau_act=0.01×1.25=0.0125, tau_deact=0.04/1.25=0.032
        // dctrl=0.05, x=0.5*(0.05/0.2+1)=0.625
        // S(0.625)=0.625^3*(0.625*(6*0.625-15)+10)
        // tau=0.032+(0.0125-0.032)*S(0.625)
        // act_dot=0.05/tau
        assert!(
            (act_dot - 2.798_737).abs() < 1e-3,
            "smooth blend act_dot = {act_dot}, expected ≈ 2.798_737"
        );
    }

    // ---- Hill curve unit tests (supplementary) ----
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

    // ---- T3: Isometric at optimal length (AC3) ----
    #[test]
    fn test_hill_force_isometric_optimal() {
        let model = build_hill_model(
            1000.0, // F0
            0.10,   // optimal_fiber_length
            0.20,   // tendon_slack_length
            10.0,   // max_contraction_velocity
            0.0,    // pennation (0°)
            35.0,   // tendon_stiffness
            1.0,    // gear
        );
        let mut data = model.make_data();
        // Set actuator length = L_slack + L_opt = 0.30 (isometric at optimal)
        data.qpos[0] = 0.30; // joint = actuator_length / gear = 0.30
        data.actuator_length[0] = 0.30;
        data.actuator_velocity[0] = 0.0;
        data.act[0] = 1.0; // full activation
        data.ctrl[0] = 1.0;

        mj_fwd_actuation(&model, &mut data);

        // At optimal fiber length: FL(1.0)=1.0, FV(0)=1.0, cos(0)=1
        // gain = -1000 × 1 × 1 × 1 = -1000
        // bias = -1000 × FP(1.0) × 1 = 0 (FP=0 at norm_len=1)
        // force = -1000 × 1.0 + 0 = -1000
        assert!(
            (data.actuator_force[0] - (-1000.0)).abs() < 1e-8,
            "force at optimal length = {}, expected -1000",
            data.actuator_force[0]
        );
    }

    // ---- T4: Pennation angle effect (AC4) ----
    #[test]
    fn test_hill_force_pennation() {
        let pennation = 0.349_066; // 20° in radians
        let model = build_hill_model(1000.0, 0.10, 0.20, 10.0, pennation, 35.0, 1.0);
        let mut data = model.make_data();
        // Need to set actuator_length so fiber_length = L_opt
        // fiber_length = (mt_length - slack_len) / cos(α)
        // L_opt = (mt_length - 0.20) / cos(20°)
        // mt_length = L_opt × cos(20°) + 0.20
        let cos_penn = pennation.cos();
        let mt_length = 0.10 * cos_penn + 0.20;
        data.qpos[0] = mt_length;
        data.actuator_length[0] = mt_length;
        data.actuator_velocity[0] = 0.0;
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        mj_fwd_actuation(&model, &mut data);

        // gain = -1000 × FL(1.0) × FV(0) × cos(20°) = -1000 × cos(20°)
        let expected_gain = -1000.0 * cos_penn;
        // bias = -1000 × FP(1.0) × cos(20°) = 0
        // force = gain × act + bias = gain × 1.0 + 0
        assert!(
            (data.actuator_force[0] - expected_gain).abs() < 1e-4,
            "force with 20° pennation = {}, expected {expected_gain}",
            data.actuator_force[0]
        );
    }

    // ---- T5: F0 auto-computation (AC5) ----
    #[test]
    fn test_hill_f0_auto_computation() {
        let model = build_hill_model(
            -1.0, // auto-compute F0
            0.10, 0.20, 10.0, 0.0, 35.0, 1.0,
        );
        // compute_actuator_params() ran during build_hill_model
        assert!(
            model.actuator_gainprm[0][2] > 0.0,
            "F0 should be auto-computed to positive value, got {}",
            model.actuator_gainprm[0][2]
        );
        // biasprm[2] should be synced
        assert_eq!(
            model.actuator_biasprm[0][2], model.actuator_gainprm[0][2],
            "biasprm[2] should be synced with gainprm[2]"
        );
        // F0 = scale / acc0 = 200 / acc0
        let expected_f0 = 200.0 / model.actuator_acc0[0];
        assert!(
            (model.actuator_gainprm[0][2] - expected_f0).abs() < 1e-10,
            "F0 = {}, expected {} (200/acc0={})",
            model.actuator_gainprm[0][2],
            expected_f0,
            model.actuator_acc0[0]
        );
    }

    // ---- T6: Dynamics dispatch equivalence (AC6) ----
    #[test]
    fn test_hill_dynamics_matches_muscle() {
        // Build model with two actuators: one Muscle, one HillMuscle
        let mut model = build_hill_model(1000.0, 0.10, 0.20, 10.0, 0.0, 35.0, 1.0);

        // Add a second (Muscle) actuator on the same joint
        model.nu = 2;
        model.na = 2;
        model.actuator_trntype.push(ActuatorTransmission::Joint);
        model.actuator_dyntype.push(ActuatorDynamics::Muscle);
        model.actuator_trnid.push([0, usize::MAX]);
        model.actuator_gear.push([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model
            .actuator_ctrlrange
            .push((f64::NEG_INFINITY, f64::INFINITY));
        model
            .actuator_forcerange
            .push((f64::NEG_INFINITY, f64::INFINITY));
        model.actuator_name.push(Some("muscle_ref".to_string()));
        model.actuator_act_adr.push(1);
        model.actuator_act_num.push(1);
        model.actuator_gaintype.push(GainType::Muscle);
        model.actuator_biastype.push(BiasType::Muscle);
        model
            .actuator_dynprm
            .push([0.01, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        model
            .actuator_gainprm
            .push([0.75, 1.05, 1000.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]);
        model
            .actuator_biasprm
            .push([0.75, 1.05, 1000.0, 200.0, 0.5, 1.6, 1.5, 1.3, 1.2]);
        model.actuator_lengthrange.push((0.0, 0.0));
        model.actuator_acc0.push(0.0);
        model.actuator_actlimited.push(true);
        model.actuator_actrange.push((0.0, 1.0));
        model.actuator_actearly.push(false);

        let mut data = model.make_data();
        data.ctrl[0] = 0.8;
        data.ctrl[1] = 0.8;
        data.act[0] = 0.3;
        data.act[1] = 0.3;
        data.actuator_length[0] = 0.30;
        data.actuator_length[1] = 0.30;

        mj_fwd_actuation(&model, &mut data);

        // act_dot should be identical (same activation dynamics function)
        assert_eq!(
            data.act_dot[0], data.act_dot[1],
            "HillMuscle act_dot ({}) should equal Muscle act_dot ({})",
            data.act_dot[0], data.act_dot[1]
        );
    }

    // ---- T7: Actearly interaction (AC7) ----
    #[test]
    fn test_hill_actearly() {
        let mut model = build_hill_model(1000.0, 0.10, 0.20, 10.0, 0.0, 35.0, 1.0);
        model.actuator_actearly[0] = true;

        let mut data = model.make_data();
        data.ctrl[0] = 1.0;
        data.act[0] = 0.0; // start at zero activation
        data.actuator_length[0] = 0.30;
        data.actuator_velocity[0] = 0.0;

        mj_fwd_actuation(&model, &mut data);

        // With actearly=true and ctrl=1.0, act=0.0, the predicted next-step
        // activation should be > 0, leading to nonzero force.
        assert!(
            data.actuator_force[0] != 0.0,
            "force should be nonzero with actearly=true, got {}",
            data.actuator_force[0]
        );
    }

    // ---- T12: Length-range mode filtering (AC14) ----
    #[test]
    fn test_hill_lengthrange_mode_filter() {
        // Verify that HillMuscle actuators pass the is_muscle filter in
        // mj_set_length_range() when mode = LengthRangeMode::Muscle.
        //
        // Build a model with a HillMuscle actuator on a limited hinge joint.
        // Phase 1 of compute_actuator_params() copies joint limits to
        // lengthrange unconditionally. Phase 3b (mj_set_length_range) with
        // mode=Muscle, useexisting=true will skip it because Phase 1 already
        // set lr. To test the filter, we call mj_set_length_range directly
        // with useexisting=false, forcing re-estimation. The key assertion:
        // after re-estimation, lengthrange must still be nonzero (the filter
        // included HillMuscle, so estimation ran and produced a valid range).
        let mut model = build_hill_model(1000.0, 0.10, 0.20, 10.0, 0.0, 35.0, 1.0);

        // Phase 1 already set lengthrange from joint limits. Verify.
        assert!(
            model.actuator_lengthrange[0].0 < model.actuator_lengthrange[0].1,
            "Phase 1 should have set lengthrange from joint limits"
        );

        // Clear lengthrange to force re-estimation via simulation.
        model.actuator_lengthrange[0] = (0.0, 0.0);

        // Call mj_set_length_range with mode=Muscle, useexisting=false.
        let opt = LengthRangeOpt {
            mode: LengthRangeMode::Muscle,
            useexisting: false,
            ..LengthRangeOpt::default()
        };
        mj_set_length_range(&mut model, &opt);

        // If HillMuscle was excluded by the filter, lengthrange would remain (0, 0).
        let (lo, hi) = model.actuator_lengthrange[0];
        assert!(
            lo < hi,
            "HillMuscle should pass is_muscle filter in mode=Muscle, \
             but lengthrange is ({lo}, {hi}) — was the actuator skipped?"
        );

        // Contrast: mode=None should skip all actuators.
        model.actuator_lengthrange[0] = (0.0, 0.0);
        let opt_none = LengthRangeOpt {
            mode: LengthRangeMode::None,
            useexisting: false,
            ..LengthRangeOpt::default()
        };
        mj_set_length_range(&mut model, &opt_none);
        let (lo2, hi2) = model.actuator_lengthrange[0];
        assert_eq!(
            (lo2, hi2),
            (0.0, 0.0),
            "mode=None should skip all actuators, but lengthrange was set"
        );
    }

    // ---- T13: Zero lengthrange (AC15) ----
    #[test]
    fn test_hill_zero_lengthrange() {
        let model = build_hill_model(1000.0, 0.10, 0.20, 10.0, 0.0, 35.0, 1.0);
        // lengthrange is (0,0) or whatever compute_actuator_params set it to.
        // HillMuscle uses fiber_length/optimal_fiber_length, not MuJoCo's L0 normalization.
        let mut data = model.make_data();
        data.actuator_length[0] = 0.30;
        data.actuator_velocity[0] = 0.0;
        data.act[0] = 1.0;
        data.ctrl[0] = 1.0;

        mj_fwd_actuation(&model, &mut data);

        assert!(
            data.actuator_force[0].is_finite(),
            "force should be finite with zero lengthrange, got {}",
            data.actuator_force[0]
        );
        // Should match isometric optimal force
        assert!(
            (data.actuator_force[0] - (-1000.0)).abs() < 1e-8,
            "force = {}, expected -1000",
            data.actuator_force[0]
        );
    }
}
