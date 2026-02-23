//! Simulation transition derivatives (FD and analytical).
//!
//! This module provides derivative infrastructure for linearizing the simulation
//! step function `x_{t+1} = f(x_t, u_t)`. It implements a four-phase strategy:
//!
//! - **Phase A** (Step 2): Pure finite-difference transition derivatives via
//!   [`mjd_transition_fd`]. Black-box perturbation through `step()` — works with
//!   any integrator, captures contact transitions. Unblocks iLQR/DDP/MPC workflows.
//!
//! - **Phase B** (Steps 3–7): Analytical smooth-force velocity derivatives via
//!   [`mjd_smooth_vel`]. Computes `∂(qfrc_smooth)/∂qvel` analytically through
//!   passive forces, actuator forces, and bias (Coriolis) forces. Stored in
//!   `Data.qDeriv` (dense nv×nv).
//!
//! - **Phase C** (Step 8, Part 2): Analytical integration derivatives — quaternion
//!   chain rules for `∂qpos/∂qvel` and `∂qpos/∂qpos` through Ball/Free joints.
//!
//! - **Phase D** (Step 9, Part 2): Hybrid FD+analytical transition derivatives.
//!   Uses analytical `qDeriv` for velocity columns of A, FD only for position
//!   columns. ~2× speedup over pure FD.
//!
//! # Tangent-space convention
//!
//! All state vectors use tangent-space representation (dimension `nv`, not `nq`).
//! Position perturbations map to/from coordinates via
//! [`mj_integrate_pos_explicit`] and
//! [`mj_differentiate_pos`]. This avoids quaternion
//! singularities for Ball and Free joints.
//!
//! # MuJoCo correspondence
//!
//! | CortenForge | MuJoCo |
//! |-------------|--------|
//! | `mjd_transition_fd` | `mjd_transitionFD` (pure FD mode) |
//! | `mjd_smooth_vel` | `mjd_smooth_vel` |
//! | `mjd_passive_vel` | `mjd_passive_vel` |
//! | `mjd_actuator_vel` | `mjd_actuator_vel` |
//! | `mjd_rne_vel` | `mjd_rne_vel` |
//! | `Data.qDeriv` | `mjData.qDeriv` (sparse → dense) |
//!
//! # Quick start
//!
//! ```ignore
//! use sim_core::{Model, Data, DerivativeConfig, mjd_transition_fd};
//!
//! let model = Model::n_link_pendulum(3, 1.0, 0.1);
//! let mut data = model.make_data();
//! data.qpos[0] = 0.3;
//! data.forward(&model).unwrap();
//!
//! let config = DerivativeConfig::default();
//! let derivs = mjd_transition_fd(&model, &data, &config).unwrap();
//! // derivs.A is 6×6 (2*nv), derivs.B is 6×0 (nu=0)
//! ```

use crate::mujoco_pipeline::{
    // §40a fluid derivative infrastructure
    MJ_MINVAL,
    SpatialVector,
    // pub(crate) functions from mujoco_pipeline
    cholesky_solve_in_place,
    ellipsoid_moment,
    fluid_geom_semi_axes,
    joint_motion_subspace,
    lu_solve_factored,
    mj_differentiate_pos,
    mj_integrate_pos_explicit,
    mj_jac_body_com,
    mj_jac_geom,
    mj_solve_sparse,
    mj_solve_sparse_batch,
    norm3,
    object_velocity_local,
    spatial_cross_motion,
    tendon_all_dofs_sleeping,
};
use crate::types::{
    ActuatorDynamics, ActuatorTransmission, BiasType, Data, ENABLE_SLEEP, GainType, Integrator,
    MjJointType, Model, StepError,
};
use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, UnitQuaternion, Vector3};

// ============================================================================
// Step 0 — TransitionMatrices
// ============================================================================

/// Discrete-time transition Jacobians of the simulation step function.
///
/// Given the transition `x_{t+1} = f(x_t, u_t)` where:
/// - `x = [dq, qvel, act]` is the state in tangent space (dim `2*nv + na`)
/// - `u = ctrl` is the control input (dim `nu`)
///
/// The matrices encode first-order linearization:
/// `δx_{t+1} ≈ A · δx_t + B · δu_t`
///
/// # Tangent-space representation
///
/// Position perturbations `dq` live in the tangent space of the configuration
/// manifold (dimension `nv`), not in coordinate space (dimension `nq`). This
/// avoids singularities from quaternion constraints and matches MuJoCo's
/// `mjd_transitionFD` convention. The mapping between tangent and coordinate
/// space uses `mj_integrate_pos_explicit` (tangent → coordinate) and
/// `mj_differentiate_pos` (coordinate → tangent).
///
/// # State vector layout
///
/// ```text
/// x[0..nv]           = dq   (position tangent: δqpos mapped to velocity space)
/// x[nv..2*nv]        = qvel (joint velocities)
/// x[2*nv..2*nv+na]   = act  (actuator activations)
/// ```
#[derive(Debug, Clone)]
#[allow(non_snake_case)] // A, B, C, D are standard control theory notation
pub struct TransitionMatrices {
    /// State transition matrix `∂x_{t+1}/∂x_t`.
    /// Dimensions: `(2*nv + na) × (2*nv + na)`.
    pub A: DMatrix<f64>,

    /// Control influence matrix `∂x_{t+1}/∂u_t`.
    /// Dimensions: `(2*nv + na) × nu`.
    pub B: DMatrix<f64>,

    /// Sensor-state Jacobian `∂s_{t+1}/∂x_t`.
    /// `None` until sensor derivatives are implemented (see Scope Exclusions).
    pub C: Option<DMatrix<f64>>,

    /// Sensor-control Jacobian `∂s_{t+1}/∂u_t`.
    /// `None` until sensor derivatives are implemented.
    pub D: Option<DMatrix<f64>>,
}

// No Default impl — dimensions depend on the model. Construct via
// mjd_transition_fd() or mjd_transition_hybrid().

// ============================================================================
// Step 1 — DerivativeConfig
// ============================================================================

/// Configuration for derivative computation.
///
/// # Panics
///
/// `mjd_transition_fd` (and hybrid) will panic if `eps` is non-positive,
/// non-finite, or greater than `1e-2` (large perturbations invalidate the
/// linearization assumption). Use `DerivativeConfig::default()` unless you
/// have a specific reason to change epsilon.
#[derive(Debug, Clone)]
pub struct DerivativeConfig {
    /// Perturbation magnitude for finite differences.
    /// Default: `1e-6` (centered differences).
    /// Must be in `(0, 1e-2]`. Typical range: `1e-8` to `1e-4`.
    pub eps: f64,

    /// Use centered differences (more accurate, 2x cost) vs forward differences.
    /// Centered: O(ε²) error. Forward: O(ε) error.
    /// Default: `true`.
    pub centered: bool,

    /// Use hybrid analytical+FD method (Phase D) when available.
    ///
    /// When true and the integrator supports it (Euler or ImplicitSpringDamper),
    /// velocity columns of A and simple actuator columns of B use analytical
    /// derivatives from `qDeriv`. Position columns always use FD.
    /// Falls back to pure FD for RK4 or when analytical derivatives are
    /// unavailable.
    ///
    /// Default: `true`.
    pub use_analytical: bool,
}

impl Default for DerivativeConfig {
    fn default() -> Self {
        Self {
            eps: 1e-6,
            centered: true,
            use_analytical: true,
        }
    }
}

// ============================================================================
// Step 2 — mjd_transition_fd (Phase A: Pure FD)
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
#[allow(non_snake_case, clippy::similar_names)]
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

    let eps = config.eps;
    let nv = model.nv;
    let na = model.na;
    let nu = model.nu;
    let nx = 2 * nv + na;

    // Phase 0 — Save nominal state and clone scratch.
    let mut scratch = data.clone();
    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let act_0 = data.act.clone();
    let ctrl_0 = data.ctrl.clone();
    let time_0 = data.time;
    // For forward differences, compute nominal output by stepping unperturbed.
    let y_0 = if config.centered {
        None
    } else {
        scratch.step(model)?;
        let y = extract_state(model, &scratch, &qpos_0);
        // Restore scratch to nominal for subsequent perturbations.
        scratch.qpos.copy_from(&qpos_0);
        scratch.qvel.copy_from(&qvel_0);
        scratch.act.copy_from(&act_0);
        scratch.ctrl.copy_from(&ctrl_0);
        scratch.time = time_0;
        Some(y)
    };

    let mut A = DMatrix::zeros(nx, nx);
    let mut B = DMatrix::zeros(nx, nu);

    // Phase 1 — State perturbation (A matrix, nx columns).
    for i in 0..nx {
        // Apply +eps perturbation
        apply_state_perturbation(
            model,
            &mut scratch,
            &qpos_0,
            &qvel_0,
            &act_0,
            &ctrl_0,
            time_0,
            i,
            eps,
            nv,
            na,
        );
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);

        if config.centered {
            // Apply -eps perturbation
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                time_0,
                i,
                -eps,
                nv,
                na,
            );
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);

            // Central difference: (y+ - y-) / (2·eps)
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            A.column_mut(i).copy_from(&col);
        } else if let Some(ref y_ref) = y_0 {
            // Forward difference: (y+ - y0) / eps
            let col = (&y_plus - y_ref) / eps;
            A.column_mut(i).copy_from(&col);
        }
    }

    // Phase 2 — Control perturbation (B matrix, nu columns).
    for j in 0..nu {
        // Apply +eps control perturbation
        scratch.qpos.copy_from(&qpos_0);
        scratch.qvel.copy_from(&qvel_0);
        scratch.act.copy_from(&act_0);
        scratch.ctrl.copy_from(&ctrl_0);
        scratch.ctrl[j] += eps;
        scratch.time = time_0;
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);

        if config.centered {
            // Apply -eps control perturbation
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] -= eps;
            scratch.time = time_0;
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);

            let col = (&y_plus - &y_minus) / (2.0 * eps);
            B.column_mut(j).copy_from(&col);
        } else if let Some(ref y_ref) = y_0 {
            let col = (&y_plus - y_ref) / eps;
            B.column_mut(j).copy_from(&col);
        }
    }

    Ok(TransitionMatrices {
        A,
        B,
        C: None,
        D: None,
    })
}

/// Apply a state perturbation at index `i` with magnitude `delta`.
///
/// Restores scratch to nominal state first, then applies the perturbation:
/// - `i < nv`: position tangent via `mj_integrate_pos_explicit`
/// - `nv <= i < 2*nv`: velocity direct addition
/// - `2*nv <= i < 2*nv+na`: activation direct addition
#[allow(clippy::too_many_arguments)]
fn apply_state_perturbation(
    model: &Model,
    scratch: &mut Data,
    qpos_0: &DVector<f64>,
    qvel_0: &DVector<f64>,
    act_0: &DVector<f64>,
    ctrl_0: &DVector<f64>,
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
    scratch.time = time_0;
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
fn extract_state(model: &Model, data: &Data, qpos_ref: &DVector<f64>) -> DVector<f64> {
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
// Step 4 — mjd_passive_vel: Passive force velocity derivatives
// ============================================================================

/// Compute ∂(qfrc_passive)/∂qvel and add to data.qDeriv.
///
/// Per-DOF damping (all joint types):
///   qfrc_passive\[i\] -= damping\[i\] · qvel\[i\]
///   ⇒ ∂/∂qvel\[i\] = −damping\[i\]  (diagonal)
///
/// Tendon damping (explicit mode only):
///   qfrc_passive += J^T · (−b · J · qvel)
///   ⇒ ∂/∂qvel = −b · J^T · J   (rank-1 update per tendon)
///
/// Friction loss (tanh-smoothed) is NOT included (MuJoCo also omits it).
///
/// # Per-DOF damping source
///
/// Uses `model.implicit_damping[i]` — the canonical per-DOF damping vector
/// that merges `jnt_damping[jnt_id]` for Hinge/Slide joints and
/// `dof_damping[dof_idx]` for Ball/Free joints.
///
/// # Tendon damping
///
/// Tendon damping derivatives (−b · J^T · J) are included for all integrators.
/// In `ImplicitSpringDamper` mode, tendon damping is handled implicitly via
/// non-diagonal D matrices (DT-35), but the velocity derivative is still
/// physically present and must be captured here.
#[allow(non_snake_case)]
pub fn mjd_passive_vel(model: &Model, data: &mut Data) {
    // §40c: Sleep filtering — compute once, used by per-DOF and tendon loops.
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let use_dof_ind = sleep_enabled && data.nv_awake < model.nv;

    // 1. Fluid derivatives (has its own internal body-level sleep filtering)
    mjd_fluid_vel(model, data);

    // 2. Per-DOF damping: diagonal entries.
    // §40c: Use dof_awake_ind indirection to skip sleeping DOFs.
    let nv = if use_dof_ind { data.nv_awake } else { model.nv };
    for idx in 0..nv {
        let i = if use_dof_ind {
            data.dof_awake_ind[idx]
        } else {
            idx
        };
        data.qDeriv[(i, i)] += -model.implicit_damping[i];
    }

    // 3. Tendon damping: −b · J^T · J (rank-1 outer product per tendon).
    // DT-35: This runs for ALL integrators. In ImplicitSpringDamper mode the
    // tendon damping forces are folded into the implicit K/D matrices (not
    // skipped), so the velocity derivative is always physically present.
    for t in 0..model.ntendon {
        // §40c: Skip tendon if ALL target DOFs are sleeping.
        if sleep_enabled && tendon_all_dofs_sleeping(model, data, t) {
            continue;
        }
        let b = model.tendon_damping[t];
        if b <= 0.0 {
            continue;
        }
        let j = &data.ten_J[t];
        for r in 0..model.nv {
            if j[r] == 0.0 {
                continue;
            }
            for c in 0..model.nv {
                if j[c] == 0.0 {
                    continue;
                }
                data.qDeriv[(r, c)] += -b * j[r] * j[c];
            }
        }
    }
}

// ============================================================================
// Step 5 — mjd_actuator_vel: Actuator force velocity derivatives
// ============================================================================

/// Compute ∂(qfrc_actuator)/∂qvel and add to data.qDeriv.
///
/// For each actuator with GainType::Affine or BiasType::Affine:
///   force = gain(L, V) · input + bias(L, V)
///   ∂force/∂V = gainprm[2] · input + biasprm[2]
///
/// The velocity V maps to qvel through the transmission:
///   Joint:  V = gear · qvel[dof_adr]
///   Tendon: V = gear · J · qvel
///   Site:   V = moment^T · qvel
///
/// Combined: ∂qfrc/∂qvel += moment · ∂force/∂V · moment^T
///
/// Muscle actuators are SKIPPED (velocity derivatives involve piecewise
/// FLV curve gradients, captured via FD in Phase D).
#[allow(non_snake_case)]
pub(crate) fn mjd_actuator_vel(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        if matches!(model.actuator_gaintype[i], GainType::Muscle) {
            continue;
        }
        if matches!(model.actuator_biastype[i], BiasType::Muscle) {
            continue;
        }

        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => data.ctrl[i],
            _ => data.act[model.actuator_act_adr[i]],
        };

        let dgain_dv = match model.actuator_gaintype[i] {
            GainType::Fixed => 0.0,
            GainType::Affine => model.actuator_gainprm[i][2],
            GainType::Muscle => continue,
        };

        let dbias_dv = match model.actuator_biastype[i] {
            BiasType::None => 0.0,
            BiasType::Affine => model.actuator_biasprm[i][2],
            BiasType::Muscle => continue,
        };

        let dforce_dv = dgain_dv * input + dbias_dv;
        if dforce_dv.abs() < 1e-30 {
            continue;
        }

        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];

        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint => {
                let dof_adr = model.jnt_dof_adr[trnid];
                // moment = gear, ∂V/∂qvel[dof] = gear
                // ∂qfrc[dof]/∂qvel[dof] += gear² · ∂force/∂V
                data.qDeriv[(dof_adr, dof_adr)] += gear * gear * dforce_dv;
            }
            ActuatorTransmission::Tendon => {
                let j = &data.ten_J[trnid];
                // moment = gear · J^T, ∂V/∂qvel = gear · J
                // ∂qfrc/∂qvel += gear² · ∂force/∂V · J^T · J
                let scale = gear * gear * dforce_dv;
                for r in 0..model.nv {
                    if j[r] == 0.0 {
                        continue;
                    }
                    for c in 0..model.nv {
                        if j[c] == 0.0 {
                            continue;
                        }
                        data.qDeriv[(r, c)] += scale * j[r] * j[c];
                    }
                }
            }
            ActuatorTransmission::Site | ActuatorTransmission::Body => {
                let moment = &data.actuator_moment[i];
                // moment is nv-dim, ∂V/∂qvel = moment
                // ∂qfrc/∂qvel += ∂force/∂V · moment · moment^T
                for r in 0..model.nv {
                    if moment[r] == 0.0 {
                        continue;
                    }
                    for c in 0..model.nv {
                        if moment[c] == 0.0 {
                            continue;
                        }
                        data.qDeriv[(r, c)] += dforce_dv * moment[r] * moment[c];
                    }
                }
            }
        }
    }
}

// ============================================================================
// Step 6 — mjd_rne_vel: Bias force velocity derivative via chain-rule RNE
// ============================================================================

/// Derivative of spatial_cross_motion(v, s) w.r.t. v.
/// Returns 6×6 matrix M such that d(v ×_m s)/dv = M · dv.
///
/// spatial_cross_motion(v, s):
///   result_ang = w × s_ang
///   result_lin = w × s_lin + v_lin × s_ang
///
/// Derivative w.r.t. v = [w; v_lin] with s fixed:
///   d(result_ang)/d(w) = -[s_ang]×  (3×3)
///   d(result_ang)/d(v_lin) = 0
///   d(result_lin)/d(w) = -[s_lin]×  (3×3)
///   d(result_lin)/d(v_lin) = -[s_ang]× (3×3)
fn mjd_cross_motion_vel(s: &SpatialVector) -> Matrix6<f64> {
    let s_ang = Vector3::new(s[0], s[1], s[2]);
    let s_lin = Vector3::new(s[3], s[4], s[5]);

    let neg_skew_ang = neg_skew(&s_ang);
    let neg_skew_lin = neg_skew(&s_lin);

    let mut m = Matrix6::zeros();
    // Top-left 3×3: d(result_ang)/d(w) = -[s_ang]×
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&neg_skew_ang);
    // Bottom-left 3×3: d(result_lin)/d(w) = -[s_lin]×
    m.fixed_view_mut::<3, 3>(3, 0).copy_from(&neg_skew_lin);
    // Bottom-right 3×3: d(result_lin)/d(v_lin) = -[s_ang]×
    m.fixed_view_mut::<3, 3>(3, 3).copy_from(&neg_skew_ang);

    m
}

/// Derivative of spatial_cross_force(v, f) w.r.t. v.
/// Returns 6×6 matrix M such that d(v ×_f f)/dv = M · dv.
///
/// spatial_cross_force(v, f):
///   result_ang = w × f_ang + v_lin × f_lin
///   result_lin = w × f_lin
///
/// Derivative w.r.t. v = [w; v_lin] with f fixed:
///   d(result_ang)/d(w) = -[f_ang]×
///   d(result_ang)/d(v_lin) = -[f_lin]×
///   d(result_lin)/d(w) = -[f_lin]×
///   d(result_lin)/d(v_lin) = 0
fn mjd_cross_force_vel(f: &SpatialVector) -> Matrix6<f64> {
    let f_ang = Vector3::new(f[0], f[1], f[2]);
    let f_lin = Vector3::new(f[3], f[4], f[5]);

    let neg_skew_fang = neg_skew(&f_ang);
    let neg_skew_flin = neg_skew(&f_lin);

    let mut m = Matrix6::zeros();
    // Top-left 3×3: d(result_ang)/d(w) = -[f_ang]×
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&neg_skew_fang);
    // Top-right 3×3: d(result_ang)/d(v_lin) = -[f_lin]×
    m.fixed_view_mut::<3, 3>(0, 3).copy_from(&neg_skew_flin);
    // Bottom-left 3×3: d(result_lin)/d(w) = -[f_lin]×
    m.fixed_view_mut::<3, 3>(3, 0).copy_from(&neg_skew_flin);

    m
}

/// Derivative of spatial_cross_force(v, f) w.r.t. f.
/// Returns 6×6 matrix M such that d(v ×_f f)/df = M · df.
///
/// spatial_cross_force(v, f):
///   result_ang = w × f_ang + v_lin × f_lin
///   result_lin = w × f_lin
///
/// Derivative w.r.t. f = [f_ang; f_lin] with v fixed:
///   d(result_ang)/d(f_ang) = [w]×
///   d(result_ang)/d(f_lin) = [v_lin]×
///   d(result_lin)/d(f_ang) = 0
///   d(result_lin)/d(f_lin) = [w]×
///
/// Note: `d(a × b)/d(b) = [a]×` (positive skew), whereas
/// `d(a × b)/d(a) = -[b]×` (negative skew). This function differentiates
/// w.r.t. the second argument `f`, so it uses the positive skew `[v]×`.
fn mjd_cross_force_frc(v: &SpatialVector) -> Matrix6<f64> {
    let w = Vector3::new(v[0], v[1], v[2]);
    let v_lin = Vector3::new(v[3], v[4], v[5]);

    let skew_w = skew(&w);
    let skew_vlin = skew(&v_lin);

    let mut m = Matrix6::zeros();
    // Top-left 3×3: d(result_ang)/d(f_ang) = [w]×
    m.fixed_view_mut::<3, 3>(0, 0).copy_from(&skew_w);
    // Top-right 3×3: d(result_ang)/d(f_lin) = [v_lin]×
    m.fixed_view_mut::<3, 3>(0, 3).copy_from(&skew_vlin);
    // Bottom-right 3×3: d(result_lin)/d(f_lin) = [w]×
    m.fixed_view_mut::<3, 3>(3, 3).copy_from(&skew_w);

    m
}

/// Negative skew-symmetric matrix: -[a]×.
///
/// `[a]× · b = a × b`, so `-[a]× · b = -(a × b) = b × a`.
///
/// This is the Jacobian of `a × b` w.r.t. `a` (first argument, with b fixed):
/// `d(a × b)/da = -[b]×`, so `neg_skew(b)` gives the correct derivative.
fn neg_skew(a: &Vector3<f64>) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(0.0, a[2], -a[1], -a[2], 0.0, a[0], a[1], -a[0], 0.0)
}

/// Positive skew-symmetric matrix: [a]×.
///
/// `[a]× · b = a × b`.
///
/// This is the Jacobian of `a × b` w.r.t. `b` (second argument, with a fixed):
/// `d(a × b)/db = [a]×`, so `skew(a)` gives the correct derivative.
fn skew(a: &Vector3<f64>) -> nalgebra::Matrix3<f64> {
    nalgebra::Matrix3::new(0.0, -a[2], a[1], a[2], 0.0, -a[0], -a[1], a[0], 0.0)
}

/// Compute ∂(qfrc_bias)/∂qvel and SUBTRACT from data.qDeriv.
///
/// Uses chain-rule propagation through the kinematic tree to compute the
/// Coriolis matrix derivative analytically. Single-pass O(nbody · nv) algorithm.
///
/// The bias force `qfrc_bias = C(q)·v + g(q)` has:
/// - Gravity `g(q)`: ∂/∂qvel = 0 (position-only)
/// - Coriolis/centrifugal: quadratic in velocity, differentiated here
#[allow(non_snake_case, clippy::similar_names, clippy::needless_range_loop)]
pub(crate) fn mjd_rne_vel(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nbody = model.nbody;

    if nv == 0 {
        return;
    }

    // Zero scratch Jacobians
    for b in 0..nbody {
        data.deriv_Dcvel[b].fill(0.0);
        data.deriv_Dcacc[b].fill(0.0);
        data.deriv_Dcfrc[b].fill(0.0);
    }

    // Pre-compute joint motion subspaces (same optimization as mj_rne)
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    // ========== Forward pass (root to leaves) ==========
    for body_id in 1..nbody {
        let parent_id = model.body_parent[body_id];

        // Propagate velocity Jacobian from parent: Dcvel[b] = Dcvel[parent]
        // (clone needed because we borrow data.deriv_Dcvel mutably below)
        let parent_dcvel = data.deriv_Dcvel[parent_id].clone();
        data.deriv_Dcvel[body_id].copy_from(&parent_dcvel);

        // For each joint of this body
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();

            // Direct contribution: ∂(cvel)/∂(v_dof) += S[:, d]
            for d in 0..ndof {
                for row in 0..6 {
                    data.deriv_Dcvel[body_id][(row, dof_adr + d)] += s[(row, d)];
                }
            }
        }

        // Propagate acceleration Jacobian from parent: Dcacc[b] = Dcacc[parent]
        let parent_dcacc = data.deriv_Dcacc[parent_id].clone();
        data.deriv_Dcacc[body_id].copy_from(&parent_dcacc);

        // For each joint of this body
        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let s = &joint_subspaces[jnt_id];
            let ndof = model.jnt_type[jnt_id].nv();

            // Compute v_joint = S · qvel[dof_adr..dof_adr+ndof]
            let mut v_joint = SpatialVector::zeros();
            for d in 0..ndof {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[dof_adr + d];
                }
            }

            // Term 1 (direct): crossMotion(cvel[parent], S[:, d])
            let cvel_parent = data.cvel[parent_id];
            for d in 0..ndof {
                let s_col = SpatialVector::new(
                    s[(0, d)],
                    s[(1, d)],
                    s[(2, d)],
                    s[(3, d)],
                    s[(4, d)],
                    s[(5, d)],
                );
                let cross = spatial_cross_motion(cvel_parent, s_col);
                for row in 0..6 {
                    data.deriv_Dcacc[body_id][(row, dof_adr + d)] += cross[row];
                }
            }

            // Term 2 (chain rule): mjd_cross_motion_vel(v_joint) · Dcvel[parent]
            let mat = mjd_cross_motion_vel(&v_joint); // 6×6
            let parent_dcvel_ref = data.deriv_Dcvel[parent_id].clone();
            // Dcacc[b] += mat · Dcvel[parent]  (6×nv += 6×6 · 6×nv)
            // Column-by-column to avoid mixed static/dynamic type
            for c in 0..nv {
                let col = mat * parent_dcvel_ref.column(c);
                for r in 0..6 {
                    data.deriv_Dcacc[body_id][(r, c)] += col[r];
                }
            }
        }
    }

    // ========== Backward pass — Phase 1: Compute local body force derivatives ==========
    for body_id in 1..nbody {
        let inertia = &data.cinert[body_id];
        let cvel = data.cvel[body_id];
        let i_v = inertia * cvel; // I · v (momentum vector)

        // Dcfrc[b] = I · Dcacc[b]
        let dcacc = data.deriv_Dcacc[body_id].clone();
        // Matrix6 * DMatrix produces a mixed static/dynamic type;
        // multiply column-by-column into the pre-allocated DMatrix.
        for c in 0..nv {
            let col = inertia * dcacc.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc[body_id][(r, c)] = col[r];
            }
        }

        // + crossForce_vel(I·v) · Dcvel[b]
        let cross_vel_mat = mjd_cross_force_vel(&i_v); // 6×6
        let dcvel = data.deriv_Dcvel[body_id].clone();
        for c in 0..nv {
            let col = cross_vel_mat * dcvel.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc[body_id][(r, c)] += col[r];
            }
        }

        // + crossForce_frc(v) · I · Dcvel[b]
        let cross_frc_mat = mjd_cross_force_frc(&cvel); // 6×6
        for c in 0..nv {
            let i_col = inertia * dcvel.column(c); // 6×1
            let result = cross_frc_mat * i_col; // 6×1
            for r in 0..6 {
                data.deriv_Dcfrc[body_id][(r, c)] += result[r];
            }
        }
    }

    // ========== Backward pass — Phase 2: Accumulate to parent (leaves to root) ==========
    for body_id in (1..nbody).rev() {
        let parent_id = model.body_parent[body_id];
        if parent_id != 0 {
            let child_dcfrc = data.deriv_Dcfrc[body_id].clone();
            data.deriv_Dcfrc[parent_id] += child_dcfrc;
        }
    }

    // ========== Projection: qDeriv[dof, :] -= S[:, d]^T · Dcfrc[body] ==========
    for jnt_id in 0..model.njnt {
        let body_id = model.jnt_body[jnt_id];
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id];
        let ndof = model.jnt_type[jnt_id].nv();

        for d in 0..ndof {
            // qDeriv[dof_adr + d, :] -= S[:, d]^T · Dcfrc[body_id]
            // Each column c: qDeriv[dof_adr+d, c] -= Σ_row S[row,d] · Dcfrc[body_id][row, c]
            for c in 0..nv {
                let mut val = 0.0;
                for row in 0..6 {
                    val += s[(row, d)] * data.deriv_Dcfrc[body_id][(row, c)];
                }
                data.qDeriv[(dof_adr + d, c)] -= val;
            }
        }
    }

    // ========== Direct gyroscopic derivative for Ball/Free joints ==========
    // The Featherstone backward pass above captures spatial Coriolis via
    // crossForce_vel(I·v) and crossForce_frc(v)·I terms, but body-frame
    // gyroscopic torques ω × (I·ω) have an additional derivative term
    // not covered by the spatial chain-rule propagation.
    //
    // For Ball joint: qfrc_bias[dof..dof+3] -= ω_body × (I · ω_body)
    //   where ω_body = qvel[dof..dof+3] and I = diag(body_inertia)
    //   d(ω × I·ω)/dω_j = e_j × (I·ω) + ω × (I·e_j)
    //
    // For Free joint: same but angular DOFs are at dof+3..dof+6
    for body_id in 1..nbody {
        let jnt_start = model.body_jnt_adr[body_id];
        let jnt_end = jnt_start + model.body_jnt_num[body_id];

        for jnt_id in jnt_start..jnt_end {
            let dof_adr = model.jnt_dof_adr[jnt_id];

            let (ang_offset, has_gyro) = match model.jnt_type[jnt_id] {
                MjJointType::Ball => (0, true),
                MjJointType::Free => (3, true),
                _ => (0, false),
            };

            if !has_gyro {
                continue;
            }

            let omega = Vector3::new(
                data.qvel[dof_adr + ang_offset],
                data.qvel[dof_adr + ang_offset + 1],
                data.qvel[dof_adr + ang_offset + 2],
            );
            let inertia = model.body_inertia[body_id];
            let i_omega = Vector3::new(
                inertia.x * omega.x,
                inertia.y * omega.y,
                inertia.z * omega.z,
            );

            // Compute 3×3 Jacobian: d(ω × I·ω)/dω
            // Column j: e_j × (I·ω) + ω × (I·e_j)
            for j in 0..3 {
                let mut e_j = Vector3::zeros();
                e_j[j] = 1.0;

                let term1 = e_j.cross(&i_omega);
                let i_ej = Vector3::new(inertia.x * e_j.x, inertia.y * e_j.y, inertia.z * e_j.z);
                let term2 = omega.cross(&i_ej);
                let deriv = term1 + term2;

                // Subtract from qDeriv (bias is subtracted in qfrc_smooth)
                for i in 0..3 {
                    data.qDeriv[(dof_adr + ang_offset + i, dof_adr + ang_offset + j)] -= deriv[i];
                }
            }
        }
    }
}

// ============================================================================
// Step 7 — mjd_smooth_vel: Combined analytical dispatch
// ============================================================================

/// Compute ∂(qfrc_smooth)/∂qvel analytically and store in data.qDeriv.
///
/// `qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias`
///
/// Populates `data.qDeriv` with three analytical contributions:
///   1. ∂(passive)/∂qvel via `mjd_passive_vel()`
///   2. ∂(actuator)/∂qvel via `mjd_actuator_vel()`
///   3. −∂(bias)/∂qvel via `mjd_rne_vel()`
///
/// # Preconditions
///
/// `data.forward(model)` must have been called (FK, velocity FK, actuation,
/// CRBA, RNE outputs needed).
#[allow(non_snake_case)]
pub fn mjd_smooth_vel(model: &Model, data: &mut Data) {
    data.qDeriv.fill(0.0);
    mjd_passive_vel(model, data);
    mjd_actuator_vel(model, data);
    mjd_rne_vel(model, data);
}

// ============================================================================
// Step 8 — Phase C: Analytical integration derivatives
// ============================================================================

/// Quaternion integration derivatives on SO(3).
///
/// Given `q_new = q_old · exp(ω·h/2)`, computes two 3×3 tangent-space Jacobians:
///
/// - `dqnew_dqold = exp(−[ω·h]×)` — adjoint representation (rotation by −ω·h)
/// - `dqnew_domega = h · J_r(ω·h)` — h times the right Jacobian of SO(3)
///
/// When `θ = ‖ω‖·h < 1e-8`, both Jacobians use small-angle limits (I and h·I).
#[must_use]
#[allow(non_snake_case)]
pub fn mjd_quat_integrate(
    _quat: &UnitQuaternion<f64>,
    omega: &Vector3<f64>,
    h: f64,
) -> (Matrix3<f64>, Matrix3<f64>) {
    let theta_vec = omega * h; // ω·h
    let theta = theta_vec.norm(); // ‖ω‖·h

    if theta < 1e-8 {
        // Small-angle limits
        return (Matrix3::identity(), Matrix3::identity() * h);
    }

    let theta2 = theta * theta;
    let theta3 = theta2 * theta;

    // Skew-symmetric matrix [θ̂]×  where θ̂ = ω·h
    let skew_theta = Matrix3::new(
        0.0,
        -theta_vec.z,
        theta_vec.y,
        theta_vec.z,
        0.0,
        -theta_vec.x,
        -theta_vec.y,
        theta_vec.x,
        0.0,
    );
    let skew_theta_sq = skew_theta * skew_theta;

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();

    // dqnew_dqold = exp(−[ω·h]×) = I − sin(θ)/θ · [θ̂]× + (1−cos(θ))/θ² · [θ̂]×²
    let dqnew_dqold = Matrix3::identity() - skew_theta * (sin_theta / theta)
        + skew_theta_sq * ((1.0 - cos_theta) / theta2);

    // dqnew_domega = h · J_r(ω·h)
    //             = h · (I − (1−cos(θ))/θ² · [θ̂]× + (θ−sin(θ))/θ³ · [θ̂]×²)
    let jr = Matrix3::identity() - skew_theta * ((1.0 - cos_theta) / theta2)
        + skew_theta_sq * ((theta - sin_theta) / theta3);
    let dqnew_domega = jr * h;

    (dqnew_dqold, dqnew_domega)
}

/// Integration Jacobian blocks for one Euler or ImplicitSpringDamper step.
///
/// Contains `∂qpos/∂qpos`, `∂qpos/∂qvel`, `∂act/∂act`, and `∂act/∂act_dot`
/// from the integration stage only (not the force-velocity coupling).
#[allow(non_snake_case)]
struct IntegrationDerivatives {
    /// ∂qpos_{t+1}/∂qpos_t in tangent space. nv × nv, block-diagonal per joint.
    /// Currently unused — position columns use FD which captures this implicitly.
    /// Retained for potential future fully-analytical position columns.
    #[allow(dead_code)]
    dqpos_dqpos: DMatrix<f64>,
    /// ∂qpos_{t+1}/∂qvel_{t+1} in tangent space. nv × nv, block-diagonal.
    dqpos_dqvel: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_t. na × na, diagonal.
    dact_dact: DMatrix<f64>,
    /// ∂act_{t+1}/∂act_dot. na × na, diagonal.
    /// Currently unused by the hybrid path — act_dot does not depend on qvel
    /// for any existing dyntype. Retained for future dynamics types.
    #[allow(dead_code)]
    dact_dactdot: DMatrix<f64>,
}

/// Compute integration Jacobians (pure function, no mutation).
#[allow(non_snake_case)]
fn compute_integration_derivatives(model: &Model, data: &Data) -> IntegrationDerivatives {
    let nv = model.nv;
    let na = model.na;
    let h = model.timestep;

    let mut dqpos_dqpos = DMatrix::zeros(nv, nv);
    let mut dqpos_dqvel = DMatrix::zeros(nv, nv);
    let mut dact_dact = DMatrix::zeros(na, na);
    let mut dact_dactdot = DMatrix::zeros(na, na);

    // Joint dispatch
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let qpos_adr = model.jnt_qpos_adr[jnt_id];

        match model.jnt_type[jnt_id] {
            MjJointType::Hinge | MjJointType::Slide => {
                dqpos_dqpos[(dof_adr, dof_adr)] = 1.0;
                dqpos_dqvel[(dof_adr, dof_adr)] = h;
            }
            MjJointType::Ball => {
                let omega = Vector3::new(
                    data.qvel[dof_adr],
                    data.qvel[dof_adr + 1],
                    data.qvel[dof_adr + 2],
                );
                let quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    data.qpos[qpos_adr],
                    data.qpos[qpos_adr + 1],
                    data.qpos[qpos_adr + 2],
                    data.qpos[qpos_adr + 3],
                ));
                let (dpos_dpos, dpos_dvel) = mjd_quat_integrate(&quat, &omega, h);
                dqpos_dqpos
                    .view_mut((dof_adr, dof_adr), (3, 3))
                    .copy_from(&dpos_dpos);
                dqpos_dqvel
                    .view_mut((dof_adr, dof_adr), (3, 3))
                    .copy_from(&dpos_dvel);
            }
            MjJointType::Free => {
                // Linear part (dof_adr..dof_adr+3)
                for i in 0..3 {
                    dqpos_dqpos[(dof_adr + i, dof_adr + i)] = 1.0;
                    dqpos_dqvel[(dof_adr + i, dof_adr + i)] = h;
                }
                // Angular part (dof_adr+3..dof_adr+6)
                let omega = Vector3::new(
                    data.qvel[dof_adr + 3],
                    data.qvel[dof_adr + 4],
                    data.qvel[dof_adr + 5],
                );
                let quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                    data.qpos[qpos_adr + 3],
                    data.qpos[qpos_adr + 4],
                    data.qpos[qpos_adr + 5],
                    data.qpos[qpos_adr + 6],
                ));
                let (dpos_dpos, dpos_dvel) = mjd_quat_integrate(&quat, &omega, h);
                dqpos_dqpos
                    .view_mut((dof_adr + 3, dof_adr + 3), (3, 3))
                    .copy_from(&dpos_dpos);
                dqpos_dqvel
                    .view_mut((dof_adr + 3, dof_adr + 3), (3, 3))
                    .copy_from(&dpos_dvel);
            }
        }
    }

    // Activation dispatch
    for i in 0..model.nu {
        let act_adr = model.actuator_act_adr[i];
        let act_num = model.actuator_act_num[i];
        for k in 0..act_num {
            let j = act_adr + k;
            match model.actuator_dyntype[i] {
                ActuatorDynamics::Filter => {
                    let tau = model.actuator_dynprm[i][0].max(1e-10);
                    dact_dact[(j, j)] = 1.0 - h / tau;
                    dact_dactdot[(j, j)] = h;
                }
                ActuatorDynamics::FilterExact => {
                    let tau = model.actuator_dynprm[i][0].max(1e-10);
                    dact_dact[(j, j)] = (-h / tau).exp();
                    dact_dactdot[(j, j)] = tau * (1.0 - (-h / tau).exp());
                }
                ActuatorDynamics::Integrator => {
                    dact_dact[(j, j)] = 1.0;
                    dact_dactdot[(j, j)] = h;
                }
                ActuatorDynamics::Muscle => {
                    // Approximate: ignores ∂act_dot/∂act from act-dependent
                    // time constants. Not consumed by hybrid (Muscle uses FD).
                    let at_boundary = data.act[j] <= 0.0 || data.act[j] >= 1.0;
                    dact_dact[(j, j)] = if at_boundary { 0.0 } else { 1.0 };
                    dact_dactdot[(j, j)] = if at_boundary { 0.0 } else { h };
                }
                ActuatorDynamics::None => {
                    // No activation state
                }
            }
        }
    }

    IntegrationDerivatives {
        dqpos_dqpos,
        dqpos_dqvel,
        dact_dact,
        dact_dactdot,
    }
}

// ============================================================================
// Step 9 — Phase D: mjd_transition_hybrid (hybrid analytical+FD)
// ============================================================================

/// Compute hybrid analytical+FD transition derivatives.
///
/// Uses analytical `qDeriv` for velocity columns of A, FD for position columns.
/// Falls back to pure FD for RK4 or when `config.use_analytical == false`.
///
/// See module-level docs for the four-phase strategy.
///
/// # Panics
///
/// Panics if `config.eps` is not finite, not positive, or exceeds 1e-2.
///
/// # Errors
///
/// Returns `StepError` if any simulation step during FD perturbation fails.
#[allow(non_snake_case, clippy::similar_names)]
pub fn mjd_transition_hybrid(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    assert!(
        config.eps.is_finite() && config.eps > 0.0 && config.eps <= 1e-2,
        "DerivativeConfig::eps must be in (0, 1e-2], got {}",
        config.eps
    );

    let eps = config.eps;
    let h = model.timestep;
    let nv = model.nv;
    let na = model.na;
    let nu = model.nu;
    let nx = 2 * nv + na;

    // 1. Populate qDeriv analytically on a cloned working copy.
    //    The clone preserves qLD_data/qLD_diag_inv, scratch_m_impl, qM — all
    //    unmodified by mjd_smooth_vel.
    let mut data_work = data.clone();
    mjd_smooth_vel(model, &mut data_work);

    // 2. Compute integration derivatives (pure function).
    let integ = compute_integration_derivatives(model, data);

    // 3. Allocate output.
    let mut a_mat = DMatrix::zeros(nx, nx);
    let mut b_mat = DMatrix::zeros(nx, nu);

    // === 3a/3b. Velocity columns of A (analytical) ===
    let dvdv = match model.integrator {
        Integrator::Euler => {
            // ∂v⁺/∂v = I + h · M⁻¹ · qDeriv
            // Batch solve: solve M⁻¹ · qDeriv for all nv columns at once.
            // O(1) CSR metadata sweeps vs O(nv) for separate solves.
            let mut minv_qderiv = data_work.qDeriv.clone();
            let (rowadr, rownnz, colind) = model.qld_csr();
            mj_solve_sparse_batch(
                rowadr,
                rownnz,
                colind,
                &data_work.qLD_data,
                &data_work.qLD_diag_inv,
                &mut minv_qderiv,
            );
            let mut dvdv = DMatrix::identity(nv, nv);
            dvdv += h * &minv_qderiv;
            dvdv
        }
        Integrator::ImplicitSpringDamper => {
            // ∂v⁺/∂v = (M+hD+h²K)⁻¹ · (M + h·(qDeriv + D))
            let d = &model.implicit_damping;
            let mut dvdv = DMatrix::zeros(nv, nv);
            for j in 0..nv {
                let mut rhs = DVector::zeros(nv);
                for i in 0..nv {
                    rhs[i] = data_work.qM[(i, j)]
                        + h * data_work.qDeriv[(i, j)]
                        + if i == j { h * d[i] } else { 0.0 };
                }
                cholesky_solve_in_place(&data_work.scratch_m_impl, &mut rhs);
                dvdv.column_mut(j).copy_from(&rhs);
            }
            dvdv
        }
        Integrator::ImplicitFast => {
            // ∂v⁺/∂v = I + h · (M − h·D)⁻¹ · qDeriv
            // scratch_m_impl holds Cholesky factors of (M − h·D) from forward pass
            let mut dvdv = DMatrix::identity(nv, nv);
            for j in 0..nv {
                let mut col = data_work.qDeriv.column(j).clone_owned();
                cholesky_solve_in_place(&data_work.scratch_m_impl, &mut col);
                for i in 0..nv {
                    dvdv[(i, j)] += h * col[i];
                }
            }
            dvdv
        }
        Integrator::Implicit => {
            // ∂v⁺/∂v = I + h · (M − h·D)⁻¹ · qDeriv
            // scratch_m_impl holds LU factors, scratch_lu_piv holds pivots
            let mut dvdv = DMatrix::identity(nv, nv);
            for j in 0..nv {
                let mut col = data_work.qDeriv.column(j).clone_owned();
                lu_solve_factored(
                    &data_work.scratch_m_impl,
                    &data_work.scratch_lu_piv,
                    &mut col,
                );
                for i in 0..nv {
                    dvdv[(i, j)] += h * col[i];
                }
            }
            dvdv
        }
        Integrator::RungeKutta4 => {
            // Should not reach here — caller should use mjd_transition_fd
            return mjd_transition_fd(model, data, config);
        }
    };

    // Chain rule: ∂q⁺/∂v = dqpos_dqvel · dvdv
    let dqdv = &integ.dqpos_dqvel * &dvdv;

    // Fill velocity columns of A
    a_mat.view_mut((0, nv), (nv, nv)).copy_from(&dqdv);
    a_mat.view_mut((nv, nv), (nv, nv)).copy_from(&dvdv);
    // activation-velocity block is zero (no coupling)

    // === 4. Activation columns of A ===
    // Track which activation indices use FD fallback
    let mut act_fd_indices: Vec<usize> = Vec::new();

    for actuator_idx in 0..model.nu {
        let act_adr = model.actuator_act_adr[actuator_idx];
        let act_num = model.actuator_act_num[actuator_idx];
        if act_num == 0 {
            continue;
        }

        let is_muscle = matches!(
            model.actuator_dyntype[actuator_idx],
            ActuatorDynamics::Muscle
        );

        for k in 0..act_num {
            let j = act_adr + k;
            let state_col = 2 * nv + j;

            if is_muscle {
                // FD fallback for this column
                act_fd_indices.push(state_col);
                continue;
            }

            // Analytical: ∂force/∂act = gain
            let gain = match model.actuator_gaintype[actuator_idx] {
                GainType::Fixed => model.actuator_gainprm[actuator_idx][0],
                GainType::Affine => {
                    model.actuator_gainprm[actuator_idx][0]
                        + model.actuator_gainprm[actuator_idx][1]
                            * data.actuator_length[actuator_idx]
                        + model.actuator_gainprm[actuator_idx][2]
                            * data.actuator_velocity[actuator_idx]
                }
                GainType::Muscle => {
                    // Should not reach — guarded by is_muscle check above
                    act_fd_indices.push(state_col);
                    continue;
                }
            };

            // ∂qfrc/∂act = moment · gain
            let moment = &data.actuator_moment[actuator_idx];
            let mut dvdact = DVector::zeros(nv);
            for dof in 0..nv {
                dvdact[dof] = h * gain * moment[dof];
            }

            // Solve: M⁻¹ or (M−hD)⁻¹ or (M+hD+h²K)⁻¹
            match model.integrator {
                Integrator::Euler => {
                    let (rowadr, rownnz, colind) = model.qld_csr();
                    mj_solve_sparse(
                        rowadr,
                        rownnz,
                        colind,
                        &data_work.qLD_data,
                        &data_work.qLD_diag_inv,
                        &mut dvdact,
                    );
                }
                Integrator::ImplicitSpringDamper | Integrator::ImplicitFast => {
                    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut dvdact);
                }
                Integrator::Implicit => {
                    lu_solve_factored(
                        &data_work.scratch_m_impl,
                        &data_work.scratch_lu_piv,
                        &mut dvdact,
                    );
                }
                Integrator::RungeKutta4 => unreachable!(),
            }

            // Fill activation column
            let dqdact = &integ.dqpos_dqvel * &dvdact;
            for r in 0..nv {
                a_mat[(r, state_col)] = dqdact[r];
            }
            for r in 0..nv {
                a_mat[(nv + r, state_col)] = dvdact[r];
            }
            a_mat[(state_col, state_col)] = integ.dact_dact[(j, j)];
        }
    }

    // === 5. Position columns of A (FD) ===
    // Save nominal state for FD perturbation loop
    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let act_0 = data.act.clone();
    let ctrl_0 = data.ctrl.clone();
    let time_0 = data.time;
    let mut scratch = data.clone();

    // Compute nominal output for forward differences
    let y_0 = if config.centered {
        None
    } else {
        scratch.step(model)?;
        let y = extract_state(model, &scratch, &qpos_0);
        scratch.qpos.copy_from(&qpos_0);
        scratch.qvel.copy_from(&qvel_0);
        scratch.act.copy_from(&act_0);
        scratch.ctrl.copy_from(&ctrl_0);
        scratch.time = time_0;
        Some(y)
    };

    // Position FD columns (0..nv)
    for i in 0..nv {
        apply_state_perturbation(
            model,
            &mut scratch,
            &qpos_0,
            &qvel_0,
            &act_0,
            &ctrl_0,
            time_0,
            i,
            eps,
            nv,
            na,
        );
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);

        if config.centered {
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                time_0,
                i,
                -eps,
                nv,
                na,
            );
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            a_mat.column_mut(i).copy_from(&col);
        } else if let Some(ref y_ref) = y_0 {
            let col = (&y_plus - y_ref) / eps;
            a_mat.column_mut(i).copy_from(&col);
        }
    }

    // Muscle activation FD fallback columns
    for &state_col in &act_fd_indices {
        apply_state_perturbation(
            model,
            &mut scratch,
            &qpos_0,
            &qvel_0,
            &act_0,
            &ctrl_0,
            time_0,
            state_col,
            eps,
            nv,
            na,
        );
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);

        if config.centered {
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                time_0,
                state_col,
                -eps,
                nv,
                na,
            );
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            a_mat.column_mut(state_col).copy_from(&col);
        } else if let Some(ref y_ref) = y_0 {
            let col = (&y_plus - y_ref) / eps;
            a_mat.column_mut(state_col).copy_from(&col);
        }
    }

    // === 6. B matrix ===
    // Track which ctrl columns need FD
    let mut ctrl_fd_indices: Vec<usize> = Vec::new();

    for actuator_idx in 0..nu {
        let is_direct = matches!(model.actuator_dyntype[actuator_idx], ActuatorDynamics::None);

        if is_direct {
            // Analytical: ∂v⁺/∂ctrl = h · M⁻¹ · moment · gain
            let gain = match model.actuator_gaintype[actuator_idx] {
                GainType::Fixed => model.actuator_gainprm[actuator_idx][0],
                GainType::Affine => {
                    model.actuator_gainprm[actuator_idx][0]
                        + model.actuator_gainprm[actuator_idx][1]
                            * data.actuator_length[actuator_idx]
                        + model.actuator_gainprm[actuator_idx][2]
                            * data.actuator_velocity[actuator_idx]
                }
                GainType::Muscle => {
                    ctrl_fd_indices.push(actuator_idx);
                    continue;
                }
            };

            let moment = &data.actuator_moment[actuator_idx];
            let mut dvdctrl = DVector::zeros(nv);
            for dof in 0..nv {
                dvdctrl[dof] = h * gain * moment[dof];
            }

            match model.integrator {
                Integrator::Euler => {
                    let (rowadr, rownnz, colind) = model.qld_csr();
                    mj_solve_sparse(
                        rowadr,
                        rownnz,
                        colind,
                        &data_work.qLD_data,
                        &data_work.qLD_diag_inv,
                        &mut dvdctrl,
                    );
                }
                Integrator::ImplicitSpringDamper | Integrator::ImplicitFast => {
                    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut dvdctrl);
                }
                Integrator::Implicit => {
                    lu_solve_factored(
                        &data_work.scratch_m_impl,
                        &data_work.scratch_lu_piv,
                        &mut dvdctrl,
                    );
                }
                Integrator::RungeKutta4 => unreachable!(),
            }

            let dqdctrl = &integ.dqpos_dqvel * &dvdctrl;
            for r in 0..nv {
                b_mat[(r, actuator_idx)] = dqdctrl[r];
            }
            for r in 0..nv {
                b_mat[(nv + r, actuator_idx)] = dvdctrl[r];
            }
            // activation rows zero for DynType::None
        } else {
            ctrl_fd_indices.push(actuator_idx);
        }
    }

    // FD for non-analytical B columns
    for &j in &ctrl_fd_indices {
        scratch.qpos.copy_from(&qpos_0);
        scratch.qvel.copy_from(&qvel_0);
        scratch.act.copy_from(&act_0);
        scratch.ctrl.copy_from(&ctrl_0);
        scratch.ctrl[j] += eps;
        scratch.time = time_0;
        scratch.step(model)?;
        let y_plus = extract_state(model, &scratch, &qpos_0);

        if config.centered {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.ctrl[j] -= eps;
            scratch.time = time_0;
            scratch.step(model)?;
            let y_minus = extract_state(model, &scratch, &qpos_0);
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            b_mat.column_mut(j).copy_from(&col);
        } else if let Some(ref y_ref) = y_0 {
            let col = (&y_plus - y_ref) / eps;
            b_mat.column_mut(j).copy_from(&col);
        }
    }

    Ok(TransitionMatrices {
        A: a_mat,
        B: b_mat,
        C: None,
        D: None,
    })
}

// ============================================================================
// Step 10 — Public API dispatch
// ============================================================================

/// Compute transition derivatives using the best available method.
///
/// When `config.use_analytical == true` and the integrator is Euler or
/// ImplicitSpringDamper, uses hybrid analytical+FD (Phase D). Otherwise
/// falls back to pure FD (Phase A).
///
/// # Errors
///
/// Returns `StepError` if any simulation step during derivative computation fails.
pub fn mjd_transition(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    let can_analytical =
        config.use_analytical && !matches!(model.integrator, Integrator::RungeKutta4);
    if can_analytical {
        mjd_transition_hybrid(model, data, config)
    } else {
        mjd_transition_fd(model, data, config)
    }
}

/// Convenience method: compute transition derivatives at the current state.
impl Data {
    /// Compute transition derivatives at the current state.
    ///
    /// Equivalent to `mjd_transition(model, self, config)`.
    ///
    /// # Errors
    ///
    /// Returns `StepError` if any simulation step during derivative computation fails.
    pub fn transition_derivatives(
        &self,
        model: &Model,
        config: &DerivativeConfig,
    ) -> Result<TransitionMatrices, StepError> {
        mjd_transition(model, self, config)
    }
}

// ============================================================================
// Step 11 — Validation utilities
// ============================================================================

/// Compare matrices element-wise, returning max relative error and location.
///
/// Uses `floor` to prevent division-by-zero for near-zero entries:
/// `rel_error(i,j) = |a(i,j) − b(i,j)| / max(|a(i,j)|, |b(i,j)|, floor)`
///
/// # Panics
///
/// Panics if `a` and `b` have different dimensions.
#[must_use]
#[allow(non_snake_case)]
pub fn max_relative_error(a: &DMatrix<f64>, b: &DMatrix<f64>, floor: f64) -> (f64, (usize, usize)) {
    assert_eq!(a.nrows(), b.nrows());
    assert_eq!(a.ncols(), b.ncols());
    let mut max_err = 0.0_f64;
    let mut max_loc = (0, 0);
    for r in 0..a.nrows() {
        for c in 0..a.ncols() {
            let va = a[(r, c)];
            let vb = b[(r, c)];
            let denom = va.abs().max(vb.abs()).max(floor);
            let err = (va - vb).abs() / denom;
            if err > max_err {
                max_err = err;
                max_loc = (r, c);
            }
        }
    }
    (max_err, max_loc)
}

/// Compare FD and hybrid derivatives to validate analytical implementation.
///
/// Returns `(max_error_A, max_error_B)` — the max relative errors between
/// pure-FD and hybrid A/B matrices. The caller checks against desired tolerance.
///
/// # Errors
///
/// Returns `StepError` if any simulation step during derivative computation fails.
#[allow(non_snake_case)]
pub fn validate_analytical_vs_fd(model: &Model, data: &Data) -> Result<(f64, f64), StepError> {
    let fd = mjd_transition(
        model,
        data,
        &DerivativeConfig {
            use_analytical: false,
            ..Default::default()
        },
    )?;
    let hybrid = mjd_transition(
        model,
        data,
        &DerivativeConfig {
            use_analytical: true,
            ..Default::default()
        },
    )?;
    let (err_a, _) = max_relative_error(&fd.A, &hybrid.A, 1e-10);
    let (err_b, _) = max_relative_error(&fd.B, &hybrid.B, 1e-10);
    Ok((err_a, err_b))
}

/// Check FD convergence by comparing derivatives at two epsilon scales.
///
/// Computes `mjd_transition_fd` at `eps` and `eps/10` (both centered),
/// then compares A matrices via `max_relative_error`. Returns `true`
/// if the max relative error is below `tol`.
///
/// # Errors
///
/// Returns `StepError` if any simulation step during FD computation fails.
pub fn fd_convergence_check(
    model: &Model,
    data: &Data,
    eps: f64,
    tol: f64,
) -> Result<bool, StepError> {
    let c1 = DerivativeConfig {
        eps,
        centered: true,
        use_analytical: false,
    };
    let c2 = DerivativeConfig {
        eps: eps / 10.0,
        centered: true,
        use_analytical: false,
    };
    let d1 = mjd_transition_fd(model, data, &c1)?;
    let d2 = mjd_transition_fd(model, data, &c2)?;
    let (err, _) = max_relative_error(&d1.A, &d2.A, 1e-10);
    Ok(err < tol)
}

// ============================================================================
// §40a — Fluid Force Velocity Derivatives
// ============================================================================

/// Cross-product Jacobians: `Da = ∂(a×b)/∂a`, `Db = ∂(a×b)/∂b`.
/// Both are 3×3 row-major. `Da = [b]×^T`, `Db = −[a]×^T`.
#[inline]
fn mjd_cross(a: &[f64; 3], b: &[f64; 3]) -> ([f64; 9], [f64; 9]) {
    let da = [
        0.0, b[2], -b[1], // row 0
        -b[2], 0.0, b[0], // row 1
        b[1], -b[0], 0.0, // row 2
    ];
    let db = [
        0.0, -a[2], a[1], // row 0
        a[2], 0.0, -a[0], // row 1
        -a[1], a[0], 0.0, // row 2
    ];
    (da, db)
}

/// Rotate both 3-row blocks of a 6×nv Jacobian to local frame: `J_local = R^T · J_world`.
#[allow(non_snake_case)]
fn rotate_jac_to_local(j: &mut DMatrix<f64>, r: &Matrix3<f64>) {
    let rt = r.transpose();
    let nv = j.ncols();
    // Rotate angular rows (0–2)
    for c in 0..nv {
        let v = Vector3::new(j[(0, c)], j[(1, c)], j[(2, c)]);
        let rv = rt * v;
        j[(0, c)] = rv.x;
        j[(1, c)] = rv.y;
        j[(2, c)] = rv.z;
    }
    // Rotate linear rows (3–5)
    for c in 0..nv {
        let v = Vector3::new(j[(3, c)], j[(4, c)], j[(5, c)]);
        let rv = rt * v;
        j[(3, c)] = rv.x;
        j[(4, c)] = rv.y;
        j[(5, c)] = rv.z;
    }
}

/// Accumulate `J^T · B · J` into `q_deriv`. B is 6×6 as `[[f64; 6]; 6]`.
/// Zero-skips on J entries for efficiency (body Jacobians are sparse).
#[allow(non_snake_case)]
fn add_jtbj(q_deriv: &mut DMatrix<f64>, j: &DMatrix<f64>, b: &[[f64; 6]; 6]) {
    let nv = j.ncols();
    let b_mat = Matrix6::from_fn(|i, k| b[i][k]);
    // Compute BJ = B · J (6×nv)
    // Column-by-column to avoid allocating a full 6×nv temporary
    for c in 0..nv {
        let jcol = nalgebra::Vector6::new(
            j[(0, c)],
            j[(1, c)],
            j[(2, c)],
            j[(3, c)],
            j[(4, c)],
            j[(5, c)],
        );
        let bj_col = b_mat * jcol;
        // Accumulate J^T[:,r] · BJ[:,c] for each r
        for r in 0..nv {
            let mut dot = 0.0;
            for k in 0..6 {
                let jkr = j[(k, r)];
                if jkr != 0.0 {
                    dot += jkr * bj_col[k];
                }
            }
            if dot != 0.0 {
                q_deriv[(r, c)] += dot;
            }
        }
    }
}

/// Rank-1 update: `q_deriv += b · row^T · row`. Zero-skips for efficiency.
#[allow(non_snake_case)]
fn add_rank1(q_deriv: &mut DMatrix<f64>, b: f64, j: &DMatrix<f64>, row_idx: usize) {
    let nv = j.ncols();
    for r in 0..nv {
        let jr = j[(row_idx, r)];
        if jr == 0.0 {
            continue;
        }
        let br = b * jr;
        for c in 0..nv {
            let jc = j[(row_idx, c)];
            if jc != 0.0 {
                q_deriv[(r, c)] += br * jc;
            }
        }
    }
}

/// Add 3×3 matrix D (row-major `[f64; 9]`) to B at block position `(row_block, col_block)`.
fn add_to_quadrant(b: &mut [[f64; 6]; 6], d: &[f64; 9], row_block: usize, col_block: usize) {
    let r0 = 3 * row_block;
    let c0 = 3 * col_block;
    for i in 0..3 {
        for j in 0..3 {
            b[r0 + i][c0 + j] += d[3 * i + j];
        }
    }
}

// ── Inertia-box derivatives (S4) ──

/// Compute inertia-box fluid velocity derivatives for a single body.
/// 6 diagonal B scalars → 6 rank-1 updates.
#[allow(non_snake_case, clippy::needless_range_loop)]
fn mjd_inertia_box_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let mass = model.body_mass[body_id];
    let inertia = model.body_inertia[body_id];

    // Equivalent box dimensions (same as forward computation)
    let bx = ((inertia.y + inertia.z - inertia.x).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let by = ((inertia.x + inertia.z - inertia.y).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let bz = ((inertia.x + inertia.y - inertia.z).max(MJ_MINVAL) / mass * 6.0).sqrt();
    let box_dims = [bx, by, bz];

    // Local velocity at body CoM in inertia frame (same as forward)
    let mut lvel = object_velocity_local(
        model,
        data,
        body_id,
        &data.xipos[body_id],
        &data.ximat[body_id],
    );
    let wind_local = data.ximat[body_id].transpose() * model.wind;
    lvel[3] -= wind_local.x;
    lvel[4] -= wind_local.y;
    lvel[5] -= wind_local.z;

    let diam = (bx + by + bz) / 3.0;
    let pi = std::f64::consts::PI;

    // Compute 6 diagonal B entries: b_k = viscous_k + quadratic_k
    let mut b = [0.0f64; 6];

    // Angular axes (k = 0,1,2)
    for k in 0..3 {
        let j = (k + 1) % 3;
        let l = (k + 2) % 3;
        if beta > 0.0 {
            b[k] += -pi * diam.powi(3) * beta;
        }
        if rho > 0.0 {
            b[k] += -2.0 * rho * box_dims[k] * (box_dims[j].powi(4) + box_dims[l].powi(4)) / 64.0
                * lvel[k].abs();
        }
    }

    // Linear axes (k = 3,4,5), mapping to box axes (0,1,2)
    for k in 0..3 {
        let j = (k + 1) % 3;
        let l = (k + 2) % 3;
        if beta > 0.0 {
            b[3 + k] += -3.0 * pi * diam * beta;
        }
        if rho > 0.0 {
            b[3 + k] += -2.0 * 0.5 * rho * box_dims[j] * box_dims[l] * lvel[3 + k].abs();
        }
    }

    // Jacobian at body CoM, rotated to inertia frame
    let mut jac = mj_jac_body_com(model, data, body_id);
    rotate_jac_to_local(&mut jac, &data.ximat[body_id]);

    // 6 rank-1 updates: qDeriv += b_k · J[k,:]^T · J[k,:]
    for k in 0..6 {
        if b[k] != 0.0 {
            add_rank1(&mut data.qDeriv, b[k], &jac, k);
        }
    }
}

// ── Ellipsoid component derivatives (S5) ──

/// Component 1: Added mass (gyroscopic) derivatives.
/// Fills all four quadrants of B.
fn mjd_added_mass_forces(
    b: &mut [[f64; 6]; 6],
    rho: f64,
    vmass: &[f64; 3],
    vinertia: &[f64; 3],
    w: &[f64; 3],
    v: &[f64; 3],
) {
    // p_ang = ρ·[vi₀·ω₀, vi₁·ω₁, vi₂·ω₂]
    let p_ang = [
        rho * vinertia[0] * w[0],
        rho * vinertia[1] * w[1],
        rho * vinertia[2] * w[2],
    ];
    // p_lin = ρ·[vm₀·v₀, vm₁·v₁, vm₂·v₂]
    let p_lin = [
        rho * vmass[0] * v[0],
        rho * vmass[1] * v[1],
        rho * vmass[2] * v[2],
    ];

    // Term 1: torque += p_ang × ω → Q(0,0)
    {
        let (mut da, db) = mjd_cross(&p_ang, w);
        // Chain rule: ∂p_ang/∂ω = diag(ρ·vi) → scale columns of Da
        for i in 0..9 {
            da[i] *= rho * vinertia[i % 3];
        }
        add_to_quadrant(b, &db, 0, 0);
        add_to_quadrant(b, &da, 0, 0);
    }

    // Term 2: torque += p_lin × v → Q(0,1)
    {
        let (mut da, db) = mjd_cross(&p_lin, v);
        // Chain rule: ∂p_lin/∂v = diag(ρ·vm) → scale columns of Da
        for i in 0..9 {
            da[i] *= rho * vmass[i % 3];
        }
        add_to_quadrant(b, &db, 0, 1);
        add_to_quadrant(b, &da, 0, 1);
    }

    // Term 3: force += p_lin × ω → Q(1,0) for Db, Q(1,1) for Da
    {
        let (mut da, db) = mjd_cross(&p_lin, w);
        // Chain rule: ∂p_lin/∂v = diag(ρ·vm) → scale columns of Da
        for i in 0..9 {
            da[i] *= rho * vmass[i % 3];
        }
        add_to_quadrant(b, &db, 1, 0); // ∂(p_lin × ω)/∂ω → force/ω
        add_to_quadrant(b, &da, 1, 1); // ∂(p_lin × ω)/∂p_lin · ∂p_lin/∂v → force/v
    }
}

/// Component 2: Magnus lift derivative.
/// Fills Q(1,0) and Q(1,1).
fn mjd_magnus_force(
    b: &mut [[f64; 6]; 6],
    c_mag: f64,
    rho: f64,
    vol: f64,
    w: &[f64; 3],
    v: &[f64; 3],
) {
    let c = c_mag * rho * vol;
    let sw = [c * w[0], c * w[1], c * w[2]];
    let sv = [c * v[0], c * v[1], c * v[2]];
    let (da, db) = mjd_cross(&sw, &sv);
    add_to_quadrant(b, &da, 1, 0); // ∂F/∂ω
    add_to_quadrant(b, &db, 1, 1); // ∂F/∂v
}

/// Component 3: Kutta lift derivative → Q(1,1).
/// Returns 3×3 row-major. Guard: zero at `|v| < MJ_MINVAL`.
#[allow(
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::tuple_array_conversions
)]
fn mjd_kutta_lift(coef: &[f64; 3], c_k: f64, rho: f64, v: &[f64; 3]) -> [f64; 9] {
    let (x, y, z) = (v[0], v[1], v[2]);
    let (xx, yy, zz) = (x * x, y * y, z * z);
    let norm2 = xx + yy + zz;
    if norm2.sqrt() < MJ_MINVAL {
        return [0.0; 9];
    }

    let (a, b_c, c) = (coef[0], coef[1], coef[2]);
    let (aa, bb, cc) = (a * a, b_c * b_c, c * c);

    let proj_denom = aa * xx + bb * yy + cc * zz;
    let proj_num = a * xx + b_c * yy + c * zz;

    let denom_val = (proj_denom * proj_num * norm2).sqrt().max(MJ_MINVAL);
    let df_denom = std::f64::consts::PI * c_k * rho / denom_val;

    // Per-axis lift coefficients
    let dfx_coef = yy * (a - b_c) + zz * (a - c);
    let dfy_coef = xx * (b_c - a) + zz * (b_c - c);
    let dfz_coef = xx * (c - a) + yy * (c - b_c);
    let df_coef = [dfx_coef, dfy_coef, dfz_coef];

    let proj_term = proj_num / proj_denom.max(MJ_MINVAL);
    let cos_term = proj_num / norm2.max(MJ_MINVAL);

    let vel = [x, y, z];
    let mut d = [0.0f64; 9];

    // Step 1: D[i][j] = 2·proj_num·(coef[j] − coef[i])
    let coef_arr = [a, b_c, c];
    for i in 0..3 {
        for j in 0..3 {
            d[3 * i + j] = 2.0 * proj_num * (coef_arr[j] - coef_arr[i]);
        }
    }

    // Step 2: inner[k] = coef[k]²·proj_term − coef[k] + cos_term
    //         D[i][:] += df_coef[i] · inner[:]
    for k in 0..3 {
        let inner_k = coef_arr[k] * coef_arr[k] * proj_term - coef_arr[k] + cos_term;
        for i in 0..3 {
            d[3 * i + k] += df_coef[i] * inner_k;
        }
    }

    // Step 3: D[i][j] *= vel[i]·vel[j]
    for i in 0..3 {
        for j in 0..3 {
            d[3 * i + j] *= vel[i] * vel[j];
        }
    }

    // Step 4: D[i][i] -= df_coef[i]·proj_num
    for i in 0..3 {
        d[3 * i + i] -= df_coef[i] * proj_num;
    }

    // Step 5: D *= df_denom
    for val in &mut d {
        *val *= df_denom;
    }

    d
}

/// Component 4: Combined linear drag derivative → Q(1,1).
/// Three contributions: quadratic drag, area gradient, Stokes drag.
#[allow(
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::tuple_array_conversions
)]
fn mjd_viscous_drag(
    s: &[f64; 3],
    coef: &[f64; 3],
    c_b: f64,
    c_s: f64,
    beta: f64,
    rho: f64,
    v: &[f64; 3],
) -> [f64; 9] {
    let pi = std::f64::consts::PI;
    let (x, y, z) = (v[0], v[1], v[2]);
    let (xx, yy, zz) = (x * x, y * y, z * z);
    let norm2 = xx + yy + zz;
    let norm = norm2.sqrt();

    let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
    let d_max = s[0].max(s[1]).max(s[2]);
    let d_min = s[0].min(s[1]).min(s[2]);
    let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
    let a_max = pi * d_max * d_mid;

    let (a, b_c, c) = (coef[0], coef[1], coef[2]);
    let (aa, bb, cc) = (a * a, b_c * b_c, c * c);

    let proj_denom = aa * xx + bb * yy + cc * zz;
    let proj_num = a * xx + b_c * yy + c * zz;
    let a_proj = pi * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();

    let lin_coef = beta * 3.0 * pi * eq_d;

    let mut d = [0.0f64; 9];

    // Stokes drag diagonal: always contributed
    for i in 0..3 {
        d[3 * i + i] -= lin_coef;
    }

    // Quadratic + area gradient terms: guarded by speed
    if norm >= MJ_MINVAL {
        let inv_norm = 1.0 / norm;
        let quad_coef = rho * (a_proj * c_b + c_s * (a_max - a_proj));

        // Quadratic drag: d(−quad_coef·|v|·v)/dv = −quad_coef · (vvᵀ + |v|²I)/|v|
        for i in 0..3 {
            for j in 0..3 {
                let vi_vj = [x, y, z][i] * [x, y, z][j];
                d[3 * i + j] +=
                    -quad_coef * inv_norm * (vi_vj + norm2 * if i == j { 1.0 } else { 0.0 });
            }
        }

        // Area gradient contribution
        let aproj_coef = rho * norm * (c_b - c_s);
        let da_coef_val = pi / (proj_num.powi(3) * proj_denom).sqrt().max(MJ_MINVAL);
        let coef_arr = [a, b_c, c];
        let vel = [x, y, z];

        let mut da_proj_dv = [0.0f64; 3];
        for k in 0..3 {
            let j = (k + 1) % 3;
            let l = (k + 2) % 3;
            da_proj_dv[k] = aproj_coef
                * da_coef_val
                * coef_arr[k]
                * vel[k]
                * (coef_arr[j] * vel[j] * vel[j] * (coef_arr[k] - coef_arr[j])
                    + coef_arr[l] * vel[l] * vel[l] * (coef_arr[k] - coef_arr[l]));
        }

        // Area gradient outer product: D[i][j] += −v[i] · dAproj_dv[j]
        for i in 0..3 {
            for j in 0..3 {
                d[3 * i + j] += -vel[i] * da_proj_dv[j];
            }
        }
    }

    d
}

/// Component 5: Combined angular drag derivative → Q(0,0).
/// Anisotropic: guarded by `|mom_visc| < MJ_MINVAL` for quadratic term.
#[allow(clippy::similar_names)]
fn mjd_viscous_torque(
    s: &[f64; 3],
    c_ang: f64,
    c_s: f64,
    beta: f64,
    rho: f64,
    w: &[f64; 3],
) -> [f64; 9] {
    let pi = std::f64::consts::PI;
    let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
    let d_max = s[0].max(s[1]).max(s[2]);
    let d_min = s[0].min(s[1]).min(s[2]);
    let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
    let i_max = (8.0 / 15.0) * pi * d_mid * d_max.powi(4);

    let ii = [
        ellipsoid_moment(s, 0),
        ellipsoid_moment(s, 1),
        ellipsoid_moment(s, 2),
    ];

    let mom_coef = [
        c_ang * ii[0] + c_s * (i_max - ii[0]),
        c_ang * ii[1] + c_s * (i_max - ii[1]),
        c_ang * ii[2] + c_s * (i_max - ii[2]),
    ];

    let mom_visc = [w[0] * mom_coef[0], w[1] * mom_coef[1], w[2] * mom_coef[2]];

    let lin_coef = beta * pi * eq_d.powi(3);

    let mut d = [0.0f64; 9];

    // Stokes torque diagonal: always contributed
    for i in 0..3 {
        d[3 * i + i] -= lin_coef;
    }

    // Anisotropic quadratic term: guarded by |mom_visc|
    let mom_norm = norm3(&mom_visc);
    if mom_norm >= MJ_MINVAL {
        let density = rho / mom_norm;
        let mom_sq = [
            -density * w[0] * mom_coef[0] * mom_coef[0],
            -density * w[1] * mom_coef[1] * mom_coef[1],
            -density * w[2] * mom_coef[2] * mom_coef[2],
        ];

        let diag_val: f64 = (0..3).map(|k| w[k] * mom_sq[k]).sum();

        for i in 0..3 {
            for j in 0..3 {
                d[3 * i + j] += if i == j { diag_val } else { 0.0 } + w[i] * mom_sq[j];
            }
        }
    }

    d
}

// ── Ellipsoid assembly (S5 + S6 + S7) ──

/// Compute ellipsoid fluid velocity derivatives for a single body.
/// Per-geom loop: assemble B from 5 components, project via J^T·B·J.
#[allow(non_snake_case, clippy::similar_names, clippy::needless_range_loop)]
fn mjd_ellipsoid_fluid(model: &Model, data: &mut Data, body_id: usize) {
    let rho = model.density;
    let beta = model.viscosity;
    let pi = std::f64::consts::PI;
    let is_implicitfast = model.integrator == Integrator::ImplicitFast;

    let geom_adr = model.body_geom_adr[body_id];
    let geom_num = model.body_geom_num[body_id];

    for gid in geom_adr..geom_adr + geom_num {
        let fluid = &model.geom_fluid[gid];
        let interaction_coef = fluid[0];
        if interaction_coef == 0.0 {
            continue;
        }

        // Unpack coefficients (same as forward)
        let (c_blunt, c_slender, c_ang) = (fluid[1], fluid[2], fluid[3]);
        let (c_kutta, c_magnus) = (fluid[4], fluid[5]);
        let vmass = [fluid[6], fluid[7], fluid[8]];
        let vinertia = [fluid[9], fluid[10], fluid[11]];

        // Semi-axes
        let s = fluid_geom_semi_axes(model.geom_type[gid], &model.geom_size[gid]);

        // Local velocity at geom center (same as forward)
        let geom_body = model.geom_body[gid];
        let mut lvel = object_velocity_local(
            model,
            data,
            geom_body,
            &data.geom_xpos[gid],
            &data.geom_xmat[gid],
        );
        let wind_local = data.geom_xmat[gid].transpose() * model.wind;
        lvel[3] -= wind_local.x;
        lvel[4] -= wind_local.y;
        lvel[5] -= wind_local.z;

        let w = [lvel[0], lvel[1], lvel[2]];
        let v = [lvel[3], lvel[4], lvel[5]];

        // Assemble 6×6 B matrix from 5 components
        let mut big_b = [[0.0f64; 6]; 6];

        // Component 1: Added mass
        mjd_added_mass_forces(&mut big_b, rho, &vmass, &vinertia, &w, &v);

        // Component 2: Magnus lift
        let vol = (4.0 / 3.0) * pi * s[0] * s[1] * s[2];
        mjd_magnus_force(&mut big_b, c_magnus, rho, vol, &w, &v);

        // Component 3: Kutta lift
        let kutta_coef = [
            (s[1] * s[2]).powi(2),
            (s[2] * s[0]).powi(2),
            (s[0] * s[1]).powi(2),
        ];
        let kutta_d = mjd_kutta_lift(&kutta_coef, c_kutta, rho, &v);
        add_to_quadrant(&mut big_b, &kutta_d, 1, 1);

        // Component 4: Viscous drag
        let drag_coef = kutta_coef; // same projected area coefficients
        let drag_d = mjd_viscous_drag(&s, &drag_coef, c_blunt, c_slender, beta, rho, &v);
        add_to_quadrant(&mut big_b, &drag_d, 1, 1);

        // Component 5: Viscous torque
        let torque_d = mjd_viscous_torque(&s, c_ang, c_slender, beta, rho, &w);
        add_to_quadrant(&mut big_b, &torque_d, 0, 0);

        // Symmetrize for ImplicitFast (Cholesky requires SPD)
        if is_implicitfast {
            for i in 0..6 {
                for j in (i + 1)..6 {
                    let avg = 0.5 * (big_b[i][j] + big_b[j][i]);
                    big_b[i][j] = avg;
                    big_b[j][i] = avg;
                }
            }
        }

        // Scale by interaction coefficient
        for row in &mut big_b {
            for val in row.iter_mut() {
                *val *= interaction_coef;
            }
        }

        // Jacobian at geom center, rotated to geom frame
        let mut jac = mj_jac_geom(model, data, gid);
        rotate_jac_to_local(&mut jac, &data.geom_xmat[gid]);

        // Project: qDeriv += J^T · B · J
        add_jtbj(&mut data.qDeriv, &jac, &big_b);
    }
}

// ── Top-level dispatch (S7) ──

/// Compute ∂(qfrc_fluid)/∂qvel and add to data.qDeriv.
///
/// Dispatches to inertia-box or ellipsoid derivative computation per body,
/// matching the forward `mj_fluid()` dispatch logic.
#[allow(non_snake_case)]
fn mjd_fluid_vel(model: &Model, data: &mut Data) {
    if model.density == 0.0 && model.viscosity == 0.0 {
        return;
    }

    // §40c: Sleep filtering — skip sleeping bodies (MuJoCo engine_derivative.c pattern)
    let sleep_enabled = model.enableflags & ENABLE_SLEEP != 0;
    let sleep_filter = sleep_enabled && data.nbody_awake < model.nbody;
    let nbody = if sleep_filter {
        data.nbody_awake
    } else {
        model.nbody
    };

    for idx in 0..nbody {
        let body_id = if sleep_filter {
            data.body_awake_ind[idx]
        } else {
            idx
        };

        if model.body_mass[body_id] < MJ_MINVAL {
            continue;
        }

        let geom_adr = model.body_geom_adr[body_id];
        let geom_num = model.body_geom_num[body_id];
        let use_ellipsoid =
            (geom_adr..geom_adr + geom_num).any(|gid| model.geom_fluid[gid][0] > 0.0);

        if use_ellipsoid {
            mjd_ellipsoid_fluid(model, data, body_id);
        } else {
            mjd_inertia_box_fluid(model, data, body_id);
        }
    }
}

// ============================================================================
// Unit tests — per-component FD validation (T7–T11) + Jacobian (T32)
// ============================================================================

#[cfg(test)]
#[allow(
    clippy::similar_names,
    clippy::many_single_char_names,
    clippy::needless_range_loop
)]
mod fluid_derivative_unit_tests {
    use super::*;
    use crate::mujoco_pipeline::{ellipsoid_moment, norm3};

    const EPS: f64 = 1e-6;
    const FD_TOL: f64 = 1e-5;
    const PI: f64 = std::f64::consts::PI;

    // ── Forward force helpers (pure-math reimplementations for FD) ──

    fn cross3(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    }

    /// Forward added-mass forces: torque = p_ang×ω + p_lin×v, force = p_lin×ω
    fn fwd_added_mass(
        rho: f64,
        vmass: &[f64; 3],
        vinertia: &[f64; 3],
        w: &[f64; 3],
        v: &[f64; 3],
    ) -> [f64; 6] {
        let p_ang = [
            rho * vinertia[0] * w[0],
            rho * vinertia[1] * w[1],
            rho * vinertia[2] * w[2],
        ];
        let p_lin = [
            rho * vmass[0] * v[0],
            rho * vmass[1] * v[1],
            rho * vmass[2] * v[2],
        ];
        let t1 = cross3(&p_ang, w);
        let t2 = cross3(&p_lin, v);
        let f1 = cross3(&p_lin, w);
        [
            t1[0] + t2[0],
            t1[1] + t2[1],
            t1[2] + t2[2],
            f1[0],
            f1[1],
            f1[2],
        ]
    }

    /// Forward Magnus lift: F = c_mag·ρ·V·(ω×v)
    fn fwd_magnus(c_mag: f64, rho: f64, vol: f64, w: &[f64; 3], v: &[f64; 3]) -> [f64; 6] {
        let c = cross3(w, v);
        let s = c_mag * rho * vol;
        [0.0, 0.0, 0.0, s * c[0], s * c[1], s * c[2]]
    }

    /// Forward Kutta lift: F = c_K·ρ·cos_α·A_proj · ((n×v)×v)
    fn fwd_kutta(coef: &[f64; 3], c_k: f64, rho: f64, v: &[f64; 3]) -> [f64; 3] {
        let speed = norm3(v);
        if speed < MJ_MINVAL {
            return [0.0; 3];
        }
        let (a, b, c) = (coef[0], coef[1], coef[2]);
        let norm_vec = [a * v[0], b * v[1], c * v[2]];
        let proj_denom = a * a * v[0] * v[0] + b * b * v[1] * v[1] + c * c * v[2] * v[2];
        let proj_num = a * v[0] * v[0] + b * v[1] * v[1] + c * v[2] * v[2];
        let a_proj = PI * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();
        let cos_alpha = proj_num / (speed * proj_denom).max(MJ_MINVAL);
        let circ = cross3(&norm_vec, v);
        let scale = c_k * rho * cos_alpha * a_proj;
        let scaled_circ = [circ[0] * scale, circ[1] * scale, circ[2] * scale];
        cross3(&scaled_circ, v)
    }

    /// Forward viscous drag: F_i = -(lin_coef + quad_coef·|v|)·v_i
    fn fwd_viscous_drag(
        s: &[f64; 3],
        coef: &[f64; 3],
        c_b: f64,
        c_s: f64,
        beta: f64,
        rho: f64,
        v: &[f64; 3],
    ) -> [f64; 3] {
        let speed = norm3(v);
        let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
        let d_max = s[0].max(s[1]).max(s[2]);
        let d_min = s[0].min(s[1]).min(s[2]);
        let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
        let a_max = PI * d_max * d_mid;
        let (a, b, c) = (coef[0], coef[1], coef[2]);
        let proj_denom = a * a * v[0] * v[0] + b * b * v[1] * v[1] + c * c * v[2] * v[2];
        let proj_num = a * v[0] * v[0] + b * v[1] * v[1] + c * v[2] * v[2];
        let a_proj = PI * (proj_denom / proj_num.max(MJ_MINVAL)).sqrt();
        let drag_lin =
            beta * 3.0 * PI * eq_d + rho * speed * (a_proj * c_b + c_s * (a_max - a_proj));
        [-drag_lin * v[0], -drag_lin * v[1], -drag_lin * v[2]]
    }

    /// Forward viscous torque: τ_i = -(lin_coef + ρ·|mom_visc|)·ω_i
    fn fwd_viscous_torque(
        s: &[f64; 3],
        c_ang: f64,
        c_s: f64,
        beta: f64,
        rho: f64,
        w: &[f64; 3],
    ) -> [f64; 3] {
        let eq_d = (2.0 / 3.0) * (s[0] + s[1] + s[2]);
        let d_max = s[0].max(s[1]).max(s[2]);
        let d_min = s[0].min(s[1]).min(s[2]);
        let d_mid = s[0] + s[1] + s[2] - d_max - d_min;
        let i_max = (8.0 / 15.0) * PI * d_mid * d_max.powi(4);
        let ii = [
            ellipsoid_moment(s, 0),
            ellipsoid_moment(s, 1),
            ellipsoid_moment(s, 2),
        ];
        let mom_visc = [
            w[0] * (c_ang * ii[0] + c_s * (i_max - ii[0])),
            w[1] * (c_ang * ii[1] + c_s * (i_max - ii[1])),
            w[2] * (c_ang * ii[2] + c_s * (i_max - ii[2])),
        ];
        let drag_ang = beta * PI * eq_d.powi(3) + rho * norm3(&mom_visc);
        [-drag_ang * w[0], -drag_ang * w[1], -drag_ang * w[2]]
    }

    /// Compute centered FD of a 6→6 function w.r.t. 6D velocity.
    fn fd_6x6(
        f: impl Fn(&[f64; 3], &[f64; 3]) -> [f64; 6],
        w: &[f64; 3],
        v: &[f64; 3],
    ) -> [[f64; 6]; 6] {
        let mut jac = [[0.0f64; 6]; 6];
        for j in 0..6 {
            let mut w_p = *w;
            let mut v_p = *v;
            let mut w_m = *w;
            let mut v_m = *v;
            if j < 3 {
                w_p[j] += EPS;
                w_m[j] -= EPS;
            } else {
                v_p[j - 3] += EPS;
                v_m[j - 3] -= EPS;
            }
            let f_p = f(&w_p, &v_p);
            let f_m = f(&w_m, &v_m);
            for i in 0..6 {
                jac[i][j] = (f_p[i] - f_m[i]) / (2.0 * EPS);
            }
        }
        jac
    }

    /// Compute centered FD of a 3→3 function w.r.t. 3D input.
    fn fd_3x3(f: impl Fn(&[f64; 3]) -> [f64; 3], x: &[f64; 3]) -> [[f64; 3]; 3] {
        let mut jac = [[0.0f64; 3]; 3];
        for j in 0..3 {
            let mut x_p = *x;
            let mut x_m = *x;
            x_p[j] += EPS;
            x_m[j] -= EPS;
            let f_p = f(&x_p);
            let f_m = f(&x_m);
            for i in 0..3 {
                jac[i][j] = (f_p[i] - f_m[i]) / (2.0 * EPS);
            }
        }
        jac
    }

    // ── T7: Added mass per-component FD ──

    #[test]
    fn t07_added_mass_per_component_fd() {
        let rho = 1.2;
        let vmass = [0.05, 0.08, 0.12];
        let vinertia = [0.002, 0.005, 0.008];
        let w = [0.5, -1.1, 0.8];
        let v = [1.2, -0.7, 0.3];

        // Analytical B from production code
        let mut b_anal = [[0.0f64; 6]; 6];
        mjd_added_mass_forces(&mut b_anal, rho, &vmass, &vinertia, &w, &v);

        // FD of forward added-mass forces
        let fd = fd_6x6(
            |ww, vv| fwd_added_mass(rho, &vmass, &vinertia, ww, vv),
            &w,
            &v,
        );

        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (b_anal[i][j] - fd[i][j]).abs() < FD_TOL,
                    "T7 added mass B[{}][{}]: analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                    i,
                    j,
                    b_anal[i][j],
                    fd[i][j],
                    (b_anal[i][j] - fd[i][j]).abs()
                );
            }
        }
    }

    // ── T8: Magnus per-component FD ──

    #[test]
    fn t08_magnus_per_component_fd() {
        let c_mag = 0.5;
        let rho = 1.2;
        let vol = (4.0 / 3.0) * PI * 0.1 * 0.05 * 0.02;
        let w = [0.5, -1.1, 0.8];
        let v = [1.2, -0.7, 0.3];

        // Analytical B from production code
        let mut b_anal = [[0.0f64; 6]; 6];
        mjd_magnus_force(&mut b_anal, c_mag, rho, vol, &w, &v);

        // FD of forward Magnus force
        let fd = fd_6x6(|ww, vv| fwd_magnus(c_mag, rho, vol, ww, vv), &w, &v);

        for i in 0..6 {
            for j in 0..6 {
                assert!(
                    (b_anal[i][j] - fd[i][j]).abs() < FD_TOL,
                    "T8 Magnus B[{}][{}]: analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                    i,
                    j,
                    b_anal[i][j],
                    fd[i][j],
                    (b_anal[i][j] - fd[i][j]).abs()
                );
            }
        }
    }

    // ── T9: Kutta lift per-component FD ──

    #[test]
    fn t09_kutta_per_component_fd() {
        let s: [f64; 3] = [0.1, 0.05, 0.02];
        let coef: [f64; 3] = [
            (s[1] * s[2]).powi(2),
            (s[2] * s[0]).powi(2),
            (s[0] * s[1]).powi(2),
        ];
        let c_k = 1.0;
        let rho = 1.2;
        let v = [1.2, -0.7, 0.3];

        // Analytical derivative from production code
        let d_anal = mjd_kutta_lift(&coef, c_k, rho, &v);

        // FD of forward Kutta force
        let fd = fd_3x3(|vv| fwd_kutta(&coef, c_k, rho, vv), &v);

        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (d_anal[3 * i + j] - fd[i][j]).abs() < FD_TOL,
                    "T9 Kutta D[{}][{}]: analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                    i,
                    j,
                    d_anal[3 * i + j],
                    fd[i][j],
                    (d_anal[3 * i + j] - fd[i][j]).abs()
                );
            }
        }
    }

    // ── T10: Viscous drag per-component FD ──

    #[test]
    fn t10_viscous_drag_per_component_fd() {
        let s: [f64; 3] = [0.1, 0.05, 0.02];
        let coef: [f64; 3] = [
            (s[1] * s[2]).powi(2),
            (s[2] * s[0]).powi(2),
            (s[0] * s[1]).powi(2),
        ];
        let c_b = 0.5;
        let c_s = 0.1;
        let beta = 0.001;
        let rho = 1.2;
        let v = [1.2, -0.7, 0.3];

        // Analytical derivative from production code
        let d_anal = mjd_viscous_drag(&s, &coef, c_b, c_s, beta, rho, &v);

        // FD of forward viscous drag
        let fd = fd_3x3(
            |vv| fwd_viscous_drag(&s, &coef, c_b, c_s, beta, rho, vv),
            &v,
        );

        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (d_anal[3 * i + j] - fd[i][j]).abs() < FD_TOL,
                    "T10 viscous drag D[{}][{}]: analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                    i,
                    j,
                    d_anal[3 * i + j],
                    fd[i][j],
                    (d_anal[3 * i + j] - fd[i][j]).abs()
                );
            }
        }
    }

    // ── T11: Viscous torque per-component FD ──

    #[test]
    fn t11_viscous_torque_per_component_fd() {
        let s = [0.1, 0.05, 0.02];
        let c_ang = 1.0;
        let c_s = 0.1;
        let beta = 0.001;
        let rho = 1.2;
        let w = [0.5, -1.1, 0.8];

        // Analytical derivative from production code
        let d_anal = mjd_viscous_torque(&s, c_ang, c_s, beta, rho, &w);

        // FD of forward viscous torque
        let fd = fd_3x3(|ww| fwd_viscous_torque(&s, c_ang, c_s, beta, rho, ww), &w);

        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (d_anal[3 * i + j] - fd[i][j]).abs() < FD_TOL,
                    "T11 viscous torque D[{}][{}]: analytical={:.10e}, fd={:.10e}, diff={:.3e}",
                    i,
                    j,
                    d_anal[3 * i + j],
                    fd[i][j],
                    (d_anal[3 * i + j] - fd[i][j]).abs()
                );
            }
        }
    }
}
