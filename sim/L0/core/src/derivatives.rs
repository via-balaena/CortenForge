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

use crate::constraint::impedance::MJ_MINVAL;
use crate::dynamics::object_velocity_local;
use crate::dynamics::spatial::{SpatialVector, spatial_cross_force, spatial_cross_motion};
use crate::forward::{
    MjStage, ellipsoid_moment, fluid_geom_semi_axes, hill_active_fl, hill_force_velocity,
    muscle_gain_length, muscle_gain_velocity, norm3,
};
use crate::integrate::implicit::tendon_all_dofs_sleeping;
use crate::jacobian::{
    mj_differentiate_pos, mj_integrate_pos_explicit, mj_jac_body_com, mj_jac_geom,
};
use crate::joint_visitor::joint_motion_subspace;
use crate::linalg::{
    cholesky_solve_in_place, lu_solve_factored, mj_solve_sparse, mj_solve_sparse_batch,
};
use crate::types::{
    ActuatorDynamics, ActuatorTransmission, BiasType, DISABLE_SPRING, Data, ENABLE_SLEEP, GainType,
    Integrator, MjJointType, Model, StepError,
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

    /// Sensor-state Jacobian `∂sensordata_{t+1}/∂x_t`.
    /// Dimensions: `nsensordata × (2*nv + na)`.
    /// `None` when `DerivativeConfig.compute_sensor_derivatives` is false (default).
    /// `Some(matrix)` when sensor derivatives are computed — even if
    /// `nsensordata == 0` (in which case the matrix has 0 rows).
    pub C: Option<DMatrix<f64>>,

    /// Sensor-control Jacobian `∂sensordata_{t+1}/∂u_t`.
    /// Dimensions: `nsensordata × nu`.
    /// `None` when `DerivativeConfig.compute_sensor_derivatives` is false (default).
    /// `Some(matrix)` when sensor derivatives are computed.
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

    /// When true, compute sensor derivatives (C, D matrices) alongside
    /// state transition derivatives (A, B). Sensor outputs are captured
    /// via finite differencing of `sensordata` during the same perturbation
    /// loop that computes A/B.
    ///
    /// When false (default), C and D remain `None` in `TransitionMatrices`
    /// and sensor evaluation is skipped during FD perturbation steps,
    /// reducing computation cost.
    ///
    /// MuJoCo equivalent: passing non-NULL `C`/`D` pointers to
    /// `mjd_transitionFD()`.
    pub compute_sensor_derivatives: bool,
}

impl Default for DerivativeConfig {
    fn default() -> Self {
        Self {
            eps: 1e-6,
            centered: true,
            use_analytical: true,
            compute_sensor_derivatives: false,
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
        if let Some(d_mat) = &mut D {
            let s0 = sensor_0.as_ref().unwrap();
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
fn apply_state_perturbation(
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
fn in_ctrl_range(x1: f64, x2: f64, range: (f64, f64)) -> bool {
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
/// For each actuator:
///   force = gain(L, V) · input + bias(L, V)
///   ∂force/∂V = (∂gain/∂V) · input + (∂bias/∂V)
///
/// The velocity V maps to qvel through the transmission:
///   Joint:  V = gear · qvel[dof_adr]
///   Tendon: V = gear · J · qvel
///   Site:   V = moment^T · qvel
///
/// Combined: ∂qfrc/∂qvel += moment · ∂force/∂V · moment^T
///
/// Supports all gain/bias types including Muscle and HillMuscle (DT-54).
/// For muscle actuators, the velocity derivative is computed from the
/// piecewise FLV curve gradient (force-velocity derivative).
#[allow(non_snake_case)]
pub(crate) fn mjd_actuator_vel(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => data.ctrl[i],
            _ => data.act[model.actuator_act_adr[i]],
        };

        let length = data.actuator_length[i];
        let velocity = data.actuator_velocity[i];

        // ∂gain/∂V for this actuator
        let dgain_dv = match model.actuator_gaintype[i] {
            GainType::Fixed => 0.0,
            GainType::Affine => model.actuator_gainprm[i][2],
            GainType::Muscle => {
                // gain = -F0 * FL(L_norm) * FV(V_norm)
                // ∂gain/∂V = -F0 * FL(L_norm) * dFV/dV_norm * dV_norm/dV
                let prm = &model.actuator_gainprm[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2];

                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);
                let norm_vel = velocity / (l0 * prm[6]).max(1e-10);

                let fl = muscle_gain_length(norm_len, prm[4], prm[5]);
                let dfv_dnv = muscle_gain_velocity_deriv(norm_vel, prm[8]);
                let dnv_dv = 1.0 / (l0 * prm[6]).max(1e-10);

                -f0 * fl * dfv_dnv * dnv_dv
            }
            GainType::HillMuscle => {
                // gain = -F0 * FL(L_norm) * FV(V_norm) * cos(α)
                // ∂gain/∂V = -F0 * FL * dFV/dV_norm * dV_norm/dV * cos(α)
                let prm = &model.actuator_gainprm[i];
                let f0 = prm[2];
                let optimal_fiber_length = prm[4];
                let tendon_slack_length = prm[5];
                let max_contraction_velocity = prm[6];
                let pennation_angle = prm[7];

                let cos_penn = pennation_angle.cos().max(1e-10);
                let fiber_length = (length - tendon_slack_length) / cos_penn;
                let norm_len = fiber_length / optimal_fiber_length.max(1e-10);
                let norm_vel = velocity
                    / (cos_penn * optimal_fiber_length * max_contraction_velocity).max(1e-10);

                let fl = hill_active_fl(norm_len);
                let dfv_dnv = hill_force_velocity_deriv(norm_vel);
                // dV_norm/dV: V_fiber = V/cos(α), V_norm = V_fiber/(L_opt*vmax)
                // so dV_norm/dV = 1/(cos(α) * L_opt * vmax)
                let dnv_dv =
                    1.0 / (cos_penn * optimal_fiber_length * max_contraction_velocity).max(1e-10);

                -f0 * fl * dfv_dnv * dnv_dv * cos_penn
            }
            GainType::User => continue,
        };

        // ∂bias/∂V for this actuator
        // Muscle/HillMuscle bias (passive force) depends on length only, not velocity → 0.
        let dbias_dv = match model.actuator_biastype[i] {
            BiasType::Affine => model.actuator_biasprm[i][2],
            BiasType::None | BiasType::Muscle | BiasType::HillMuscle => 0.0,
            BiasType::User => continue,
        };

        let dforce_dv = dgain_dv * input + dbias_dv;
        if dforce_dv.abs() < 1e-30 {
            continue;
        }

        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];

        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
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
            ActuatorTransmission::Site
            | ActuatorTransmission::Body
            | ActuatorTransmission::SliderCrank => {
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
// Step 7b — mjd_smooth_pos: Analytical position derivative dispatch
// ============================================================================

/// Compute ∂(qfrc_smooth)/∂qpos analytically and store in data.qDeriv_pos.
///
/// `qfrc_smooth = qfrc_passive + qfrc_actuator − qfrc_bias`
///
/// Populates position derivative storage with three analytical contributions:
///   1. ∂qfrc_passive/∂qpos via spring chain rules (joint + tendon)
///   2. ∂qfrc_actuator/∂qpos via gain/bias length derivatives
///   3. −∂RNEA(q,v,qacc)/∂qpos via RNE position chain rule
///      (includes −(∂M/∂q)·qacc from evaluating RNEA at actual acceleration)
///
/// **CortenForge extension** — MuJoCo has no analytical position derivatives.
/// Validated by comparing against FD position columns.
///
/// # Precondition
/// Forward pass must have completed (data.qacc populated).
#[allow(non_snake_case)]
pub fn mjd_smooth_pos(model: &Model, data: &mut Data) {
    // Ensure cacc/cfrc_int are populated (lazy — may not have been computed yet).
    // mjd_rne_pos reads cacc[] to compute the acceleration transform derivatives.
    if !data.flg_rnepost {
        crate::forward::acceleration::mj_body_accumulators(model, data);
    }

    data.qDeriv_pos.fill(0.0);
    mjd_passive_pos(model, data);
    mjd_actuator_pos(model, data);
    mjd_rne_pos(model, data);
}

/// Compute ∂(qfrc_passive)/∂qpos and add to data.qDeriv_pos.
///
/// Components:
/// 1. Joint spring stiffness: −k for hinge/slide (diagonal), −k·∂subquat/∂q for ball/free
/// 2. Tendon spring stiffness: −k · J^T · J
///
/// Fluid, gravcomp, and flex passive forces are NOT included (deferred — AD-1).
#[allow(non_snake_case)]
pub(crate) fn mjd_passive_pos(model: &Model, data: &mut Data) {
    let nv = model.nv;
    if nv == 0 {
        return;
    }

    // 1. Joint spring stiffness: ∂qfrc_spring/∂qpos
    if model.disableflags & DISABLE_SPRING == 0 {
        for jnt_id in 0..model.njnt {
            let dof_adr = model.jnt_dof_adr[jnt_id];
            let stiffness = model.jnt_stiffness[jnt_id];
            if stiffness == 0.0 {
                continue;
            }

            match model.jnt_type[jnt_id] {
                MjJointType::Hinge | MjJointType::Slide => {
                    // qfrc_spring[dof] -= k * (q - q_ref)
                    // ∂/∂qpos[dof] = -k (diagonal)
                    data.qDeriv_pos[(dof_adr, dof_adr)] += -stiffness;
                }
                MjJointType::Ball => {
                    // qfrc_spring[dof..dof+3] -= k * subquat(q, q_ref)
                    // ∂/∂qpos = -k * ∂subquat/∂qa (3×3 tangent-space Jacobian)
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    let qa = [
                        data.qpos[qpos_adr],
                        data.qpos[qpos_adr + 1],
                        data.qpos[qpos_adr + 2],
                        data.qpos[qpos_adr + 3],
                    ];
                    let qb = [
                        model.qpos_spring[qpos_adr],
                        model.qpos_spring[qpos_adr + 1],
                        model.qpos_spring[qpos_adr + 2],
                        model.qpos_spring[qpos_adr + 3],
                    ];
                    let (d_dqa, _d_dqb) = mjd_sub_quat(&qa, &qb);
                    for r in 0..3 {
                        for c in 0..3 {
                            data.qDeriv_pos[(dof_adr + r, dof_adr + c)] +=
                                -stiffness * d_dqa[(r, c)];
                        }
                    }
                }
                MjJointType::Free => {
                    // Translational part (dof..dof+3): same as Hinge/Slide (diagonal)
                    for i in 0..3 {
                        data.qDeriv_pos[(dof_adr + i, dof_adr + i)] += -stiffness;
                    }
                    // Rotational part (dof+3..dof+6): same as Ball
                    let qpos_adr = model.jnt_qpos_adr[jnt_id];
                    let qa = [
                        data.qpos[qpos_adr + 3],
                        data.qpos[qpos_adr + 4],
                        data.qpos[qpos_adr + 5],
                        data.qpos[qpos_adr + 6],
                    ];
                    let qb = [
                        model.qpos_spring[qpos_adr + 3],
                        model.qpos_spring[qpos_adr + 4],
                        model.qpos_spring[qpos_adr + 5],
                        model.qpos_spring[qpos_adr + 6],
                    ];
                    let (d_dqa, _d_dqb) = mjd_sub_quat(&qa, &qb);
                    for r in 0..3 {
                        for c in 0..3 {
                            data.qDeriv_pos[(dof_adr + 3 + r, dof_adr + 3 + c)] +=
                                -stiffness * d_dqa[(r, c)];
                        }
                    }
                }
            }
        }
    }

    // 2. Tendon spring stiffness: ∂(J^T * k * (bound - length))/∂qpos ≈ -k · J^T · J
    // This ignores the cross-term (∂J^T/∂qpos) · force — which is zero for
    // fixed-point tendons (constant J) and small for spatial tendons.
    for t in 0..model.ntendon {
        let stiffness = model.tendon_stiffness[t];
        if stiffness == 0.0 {
            continue;
        }

        let length = data.ten_length[t];
        let [lower, upper] = model.tendon_lengthspring[t];

        // Only active when outside deadband
        let active = (length < lower) || (length > upper);
        if !active {
            continue;
        }

        let j = &data.ten_J[t];
        let scale = -stiffness;
        for r in 0..nv {
            if j[r] == 0.0 {
                continue;
            }
            for c in 0..nv {
                if j[c] == 0.0 {
                    continue;
                }
                data.qDeriv_pos[(r, c)] += scale * j[r] * j[c];
            }
        }
    }
}

// ============================================================================
// Step 7c — mjd_actuator_pos: Actuator force position derivatives
// ============================================================================

/// Derivative of the MuJoCo piecewise-quadratic active FL curve w.r.t.
/// normalized length. Returns `dFL/dL_norm`.
fn muscle_active_fl_deriv(length: f64, lmin: f64, lmax: f64) -> f64 {
    const EPS: f64 = 1e-10;
    if length < lmin || length > lmax {
        return 0.0;
    }
    let a = 0.5 * (lmin + 1.0);
    let b = 0.5 * (1.0 + lmax);

    if length <= a {
        let x = (length - lmin) / (a - lmin).max(EPS);
        // FL = 0.5 * x^2, dFL/dL = x / (a - lmin)
        x / (a - lmin).max(EPS)
    } else if length <= 1.0 {
        let x = (1.0 - length) / (1.0 - a).max(EPS);
        // FL = 1 - 0.5*x^2, dFL/dL = x / (1 - a) = (1-L)/((1-a)^2)
        // dFL/dL_norm = d/dL[1 - 0.5*((1-L)/(1-a))^2] = (1-L)/((1-a)^2)
        // but x = (1-L)/(1-a), dFL = -x * dx/dL = -x * (-1/(1-a)) = x/(1-a)
        x / (1.0 - a).max(EPS)
    } else if length <= b {
        let x = (length - 1.0) / (b - 1.0).max(EPS);
        // FL = 1 - 0.5*x^2, dFL/dL = -x / (b-1)
        -x / (b - 1.0).max(EPS)
    } else {
        let x = (lmax - length) / (lmax - b).max(EPS);
        // FL = 0.5*x^2, dFL/dL = -x / (lmax-b)
        -x / (lmax - b).max(EPS)
    }
}

/// Derivative of the Hill-type Gaussian active FL curve w.r.t. normalized length.
/// FL(x) = exp(−((x−1)/w)²) where w = w_asc (x≤1) or w_desc (x>1).
/// Returns `dFL/dL_norm`.
fn hill_active_fl_deriv(norm_len: f64) -> f64 {
    let (w_asc, w_desc) = (0.45, 0.56);
    let (l_min, l_max) = (0.5, 1.6);
    if norm_len < l_min || norm_len > l_max {
        return 0.0;
    }
    let w = if norm_len <= 1.0 { w_asc } else { w_desc };
    let x = (norm_len - 1.0) / w;
    // dFL/dL = FL * d(−x²)/dL = FL * (−2x/w)
    let fl = (-x * x).exp();
    fl * (-2.0 * x / w)
}

/// Compute ∂(qfrc_actuator)/∂qpos and add to data.qDeriv_pos.
///
/// For each actuator:
///   force = gain(L, V) · input + bias(L, V)
///   ∂force/∂qpos = (∂gain/∂L · input + ∂bias/∂L) · ∂L/∂qpos
///
/// where ∂L/∂qpos = moment (the moment arm IS the length Jacobian).
///
/// Only the force chain-rule term is computed — the moment-arm cross-term
/// `(∂moment/∂qpos) · force` is deferred for non-Joint transmissions (AD-1).
#[allow(non_snake_case)]
pub(crate) fn mjd_actuator_pos(model: &Model, data: &mut Data) {
    for i in 0..model.nu {
        let input = match model.actuator_dyntype[i] {
            ActuatorDynamics::None => data.ctrl[i],
            _ => data.act[model.actuator_act_adr[i]],
        };
        let length = data.actuator_length[i];

        // Compute ∂gain/∂L
        let dgain_dl = match model.actuator_gaintype[i] {
            GainType::Fixed => 0.0,
            GainType::Affine => {
                // gain = prm[0] + prm[1]*L + prm[2]*V
                // ∂gain/∂L = prm[1]
                model.actuator_gainprm[i][1]
            }
            GainType::Muscle => {
                // gain = -F0 * FL(L_norm) * FV(V_norm)
                // ∂gain/∂L = -F0 * dFL/dL_norm * dL_norm/dL * FV(V_norm)
                let prm = &model.actuator_gainprm[i];
                let lengthrange = model.actuator_lengthrange[i];
                let f0 = prm[2];
                let l0 = (lengthrange.1 - lengthrange.0) / (prm[1] - prm[0]).max(1e-10);
                let norm_len = prm[0] + (length - lengthrange.0) / l0.max(1e-10);
                let norm_vel = data.actuator_velocity[i] / (l0 * prm[6]).max(1e-10);
                let dfl_dnl = muscle_active_fl_deriv(norm_len, prm[4], prm[5]);
                let fv = muscle_gain_velocity(norm_vel, prm[8]);
                let dnl_dl = 1.0 / l0.max(1e-10);
                -f0 * dfl_dnl * dnl_dl * fv
            }
            GainType::HillMuscle => {
                let prm = &model.actuator_gainprm[i];
                let f0 = prm[2];
                let optimal_fiber_length = prm[4];
                let tendon_slack_length = prm[5];
                let max_contraction_velocity = prm[6];
                let pennation_angle = prm[7];
                let cos_penn = pennation_angle.cos().max(1e-10);
                let fiber_length = (length - tendon_slack_length) / cos_penn;
                let norm_len = fiber_length / optimal_fiber_length.max(1e-10);
                let norm_vel = data.actuator_velocity[i]
                    / (cos_penn * optimal_fiber_length * max_contraction_velocity).max(1e-10);
                let dfl_dnl = hill_active_fl_deriv(norm_len);
                let fv = hill_force_velocity(norm_vel);
                let dnl_dl = 1.0 / (cos_penn * optimal_fiber_length).max(1e-10);
                -f0 * dfl_dnl * dnl_dl * fv * cos_penn
            }
            GainType::User => continue,
        };

        // Compute ∂bias/∂L
        let dbias_dl = match model.actuator_biastype[i] {
            BiasType::Affine => {
                // bias = prm[0] + prm[1]*L + prm[2]*V
                // ∂bias/∂L = prm[1]
                model.actuator_biasprm[i][1]
            }
            // Passive FL derivative deferred for Muscle/HillMuscle — models with
            // these bias types are excluded from analytical position columns via
            // the eligibility check. BiasType::None has no length dependence.
            BiasType::None | BiasType::Muscle | BiasType::HillMuscle => 0.0,
            BiasType::User => continue,
        };

        let dforce_dl = dgain_dl * input + dbias_dl;
        if dforce_dl.abs() < 1e-30 {
            continue;
        }

        // Transmission dispatch (same structure as mjd_actuator_vel)
        // ∂qfrc/∂qpos += moment · ∂force/∂L · moment^T
        //   (because ∂L/∂qpos = moment^T for joint/tendon transmissions)
        let gear = model.actuator_gear[i][0];
        let trnid = model.actuator_trnid[i][0];

        match model.actuator_trntype[i] {
            ActuatorTransmission::Joint | ActuatorTransmission::JointInParent => {
                let dof_adr = model.jnt_dof_adr[trnid];
                data.qDeriv_pos[(dof_adr, dof_adr)] += gear * gear * dforce_dl;
            }
            ActuatorTransmission::Tendon => {
                let j = &data.ten_J[trnid];
                let scale = gear * gear * dforce_dl;
                for r in 0..model.nv {
                    if j[r] == 0.0 {
                        continue;
                    }
                    for c in 0..model.nv {
                        if j[c] == 0.0 {
                            continue;
                        }
                        data.qDeriv_pos[(r, c)] += scale * j[r] * j[c];
                    }
                }
            }
            ActuatorTransmission::Site
            | ActuatorTransmission::Body
            | ActuatorTransmission::SliderCrank => {
                // Force chain-rule term only; moment-arm cross-term deferred (AD-1).
                let moment = &data.actuator_moment[i];
                for r in 0..model.nv {
                    if moment[r] == 0.0 {
                        continue;
                    }
                    for c in 0..model.nv {
                        if moment[c] == 0.0 {
                            continue;
                        }
                        data.qDeriv_pos[(r, c)] += dforce_dl * moment[r] * moment[c];
                    }
                }
            }
        }
    }
}

// ============================================================================
// Step 7d — mjd_rne_pos: Bias force position derivatives via chain-rule RNE
// ============================================================================

/// Compute −∂RNEA(q, v, qacc)/∂qpos and SUBTRACT from data.qDeriv_pos.
///
/// Uses a corrected RNEA with proper spatial transport (X_b for acceleration
/// forward, X_b^T for force backward) to ensure the chain-rule derivatives
/// correctly capture how position changes affect the moment arms and inertia
/// projections throughout the kinematic chain.
///
/// The operating-point RNEA (cacc_rne, cfrc_rne) is recomputed internally
/// with proper transport; this gives `S^T · cfrc_rne = M·qacc + qfrc_bias`.
///
/// Three-phase structure:
/// 1. Forward: propagate ∂cvel/∂q (X_b) and ∂cacc_rne/∂q (X_b + transport deriv)
/// 2. Backward: compute ∂cfrc_rne/∂q then accumulate with X_b^T + transport deriv
/// 3. Projection: S^T · ∂cfrc/∂q → qDeriv_pos rows
///
/// Critical difference from `mjd_rne_vel`: position derivatives are evaluated
/// at ACTUAL acceleration qacc (not zero), capturing the (∂M/∂q)·qacc term.
#[allow(
    non_snake_case,
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::too_many_lines
)]
pub(crate) fn mjd_rne_pos(model: &Model, data: &mut Data) {
    let nv = model.nv;
    let nbody = model.nbody;

    if nv == 0 {
        return;
    }

    // Zero scratch Jacobians
    for b in 0..nbody {
        data.deriv_Dcvel_pos[b].fill(0.0);
        data.deriv_Dcacc_pos[b].fill(0.0);
        data.deriv_Dcfrc_pos[b].fill(0.0);
    }

    // Pre-compute joint motion subspaces
    let joint_subspaces: Vec<_> = (0..model.njnt)
        .map(|jnt_id| joint_motion_subspace(model, data, jnt_id))
        .collect();

    // Pre-compute DOF→axis mappings for ancestor transport derivatives
    let mut dof_axis = vec![Vector3::zeros(); nv];
    for jnt_id in 0..model.njnt {
        let dof_adr = model.jnt_dof_adr[jnt_id];
        let s = &joint_subspaces[jnt_id];
        let ndof = model.jnt_type[jnt_id].nv();
        for d in 0..ndof {
            dof_axis[dof_adr + d] = Vector3::new(s[(0, d)], s[(1, d)], s[(2, d)]);
        }
    }

    // ========== Split RNEA: two separate operating points ==========
    //
    // A combined X_b/X_b^T RNEA for all terms (gravity + M·qacc + Coriolis/gyroscopic)
    // breaks the RNEA identity S^T·cfrc = M·qacc + qfrc_bias because X_b^T backward
    // transport of gyroscopic forces creates spurious lever-arm torques (r × f_gyro_lin).
    //
    // Solution: split into two RNEAs, each using a transport convention that makes
    // its own RNEA identity exact, and differentiate each with its own convention.
    //
    // Part A — gravity + M·qacc (zero velocity):
    //   Forward:  X_b transport,  a_A[0] = grav,  a_A[b] = X_b(a_A[parent]) + S·qacc
    //   Backward: X_b^T transport, f_A[b] = I·a_A[b] (no gyroscopic)
    //   Identity: S^T · cfrc_A = g(q) + M·qacc  ← exact (verified)
    //
    // Part B — Coriolis/gyroscopic:
    //   Forward:  simple copy,  a_B[0] = 0,  a_B[b] = a_B[parent] + cvel[parent] ×_m S·qvel
    //   Backward: simple add,   f_B[b] = I·a_B[b] + v ×* (I·v)
    //   Identity: S^T · cfrc_B = C(q,v)  ← exact (verified, matches MuJoCo convention)

    let grav = if model.disableflags & crate::types::DISABLE_GRAVITY != 0 {
        SpatialVector::zeros()
    } else {
        let mut sv = SpatialVector::zeros();
        sv[3] = -model.gravity[0];
        sv[4] = -model.gravity[1];
        sv[5] = -model.gravity[2];
        sv
    };

    // ==================== Part A operating point ====================
    // Zero-velocity RNEA: gravity + M·qacc
    let mut cacc_a = vec![SpatialVector::zeros(); nbody];
    cacc_a[0] = grav;
    for b in 1..nbody {
        let pid = model.body_parent[b];
        let r = data.xpos[b] - data.xpos[pid];
        // X_b transport: [ω; v + ω×r]
        let mut acc = cacc_a[pid];
        let omega_a = Vector3::new(acc[0], acc[1], acc[2]);
        let cross = omega_a.cross(&r);
        acc[3] += cross.x;
        acc[4] += cross.y;
        acc[5] += cross.z;
        // Add S·qacc (no Coriolis — zero velocity)
        let js = model.body_jnt_adr[b];
        let je = js + model.body_jnt_num[b];
        for jid in js..je {
            let da = model.jnt_dof_adr[jid];
            let s = &joint_subspaces[jid];
            let nd = model.jnt_type[jid].nv();
            for d in 0..nd {
                for row in 0..6 {
                    acc[row] += s[(row, d)] * data.qacc[da + d];
                }
            }
        }
        cacc_a[b] = acc;
    }
    // Backward: f_A = I·a_A (no gyroscopic), then X_b^T accumulation
    let mut cfrc_a = vec![SpatialVector::zeros(); nbody];
    for b in 1..nbody {
        cfrc_a[b] = data.cinert[b] * cacc_a[b];
    }
    for b in (1..nbody).rev() {
        let pid = model.body_parent[b];
        if pid > 0 {
            let r = data.xpos[b] - data.xpos[pid];
            let f = cfrc_a[b];
            let f_lin = Vector3::new(f[3], f[4], f[5]);
            let torque = r.cross(&f_lin);
            cfrc_a[pid][0] += f[0] + torque.x;
            cfrc_a[pid][1] += f[1] + torque.y;
            cfrc_a[pid][2] += f[2] + torque.z;
            cfrc_a[pid][3] += f[3];
            cfrc_a[pid][4] += f[4];
            cfrc_a[pid][5] += f[5];
        }
    }

    // ==================== Part B operating point ====================
    // Coriolis/gyroscopic RNEA (MuJoCo convention)
    let mut cacc_b = vec![SpatialVector::zeros(); nbody];
    for b in 1..nbody {
        let pid = model.body_parent[b];
        cacc_b[b] = cacc_b[pid]; // simple copy (no X_b transport)
        let js = model.body_jnt_adr[b];
        let je = js + model.body_jnt_num[b];
        for jid in js..je {
            let da = model.jnt_dof_adr[jid];
            let s = &joint_subspaces[jid];
            let nd = model.jnt_type[jid].nv();
            let mut vj = SpatialVector::zeros();
            for d in 0..nd {
                for row in 0..6 {
                    vj[row] += s[(row, d)] * data.qvel[da + d];
                }
            }
            cacc_b[b] += spatial_cross_motion(data.cvel[pid], vj);
        }
    }
    // Backward: f_B = I·a_B + gyroscopic, then simple-add accumulation
    let mut cfrc_b = vec![SpatialVector::zeros(); nbody];
    for b in 1..nbody {
        let i_a = data.cinert[b] * cacc_b[b];
        let v = data.cvel[b];
        let i_v = data.cinert[b] * v;
        cfrc_b[b] = i_a + spatial_cross_force(v, i_v);
    }
    for b in (1..nbody).rev() {
        let pid = model.body_parent[b];
        if pid > 0 {
            let fc = cfrc_b[b];
            for row in 0..6 {
                cfrc_b[pid][row] += fc[row]; // simple add (no X_b^T)
            }
        }
    }

    // ==================== Velocity derivatives (shared) ====================
    // cvel uses X_b transport: cvel[b] = X_b(cvel[parent]) + S·qvel
    // These derivatives are needed by Part B's Term C.
    for body_id in 1..nbody {
        let parent_id = model.body_parent[body_id];
        let r = data.xpos[body_id] - data.xpos[parent_id];

        // X_b transport of parent Dcvel
        let parent_dcvel = data.deriv_Dcvel_pos[parent_id].clone();
        data.deriv_Dcvel_pos[body_id].copy_from(&parent_dcvel);
        for col in 0..nv {
            let ox = parent_dcvel[(0, col)];
            let oy = parent_dcvel[(1, col)];
            let oz = parent_dcvel[(2, col)];
            data.deriv_Dcvel_pos[body_id][(3, col)] += oy * r.z - oz * r.y;
            data.deriv_Dcvel_pos[body_id][(4, col)] += oz * r.x - ox * r.z;
            data.deriv_Dcvel_pos[body_id][(5, col)] += ox * r.y - oy * r.x;
        }

        // Velocity transport derivative: cvel[parent]_ang × (axis × r)
        let omega_p = Vector3::new(
            data.cvel[parent_id][0],
            data.cvel[parent_id][1],
            data.cvel[parent_id][2],
        );
        if omega_p.norm_squared() > 0.0 {
            let mut anc = parent_id;
            while anc != 0 {
                let anc_js = model.body_jnt_adr[anc];
                let anc_je = anc_js + model.body_jnt_num[anc];
                for jid in anc_js..anc_je {
                    let da = model.jnt_dof_adr[jid];
                    let nd = model.jnt_type[jid].nv();
                    for d in 0..nd {
                        let axis = dof_axis[da + d];
                        let dr = axis.cross(&r);
                        let contrib = omega_p.cross(&dr);
                        data.deriv_Dcvel_pos[body_id][(3, da + d)] += contrib.x;
                        data.deriv_Dcvel_pos[body_id][(4, da + d)] += contrib.y;
                        data.deriv_Dcvel_pos[body_id][(5, da + d)] += contrib.z;
                    }
                }
                anc = model.body_parent[anc];
            }
        }

        // Joint subspace position derivative: ∂(vJ_total)/∂q_k
        //
        // vJ_total = Σ_j S_j · qvel_j is the total joint velocity from all joints
        // on this body. When DOF d of joint k has an angular component, perturbing
        // q_kd rotates the body, which rotates ALL joint axes on the body.
        // The derivative is: [s_ang_kd; 0] ×_m vJ_total
        //
        // This handles multi-joint bodies (cross-derivatives between joints on the
        // same body) and also single-joint cases correctly. Ball and Free joints
        // use world-frame axes so ∂S/∂q = 0.
        let js = model.body_jnt_adr[body_id];
        let je = js + model.body_jnt_num[body_id];

        // Pre-compute velocity from orientation-dependent joints only (Hinge/Slide).
        // Ball/Free use world-frame axes that don't change with body rotation.
        let mut vj_rotating = SpatialVector::zeros();
        for jid in js..je {
            if matches!(
                model.jnt_type[jid],
                crate::types::MjJointType::Ball | crate::types::MjJointType::Free
            ) {
                continue; // World-frame axes: ∂S/∂q = 0
            }
            let da = model.jnt_dof_adr[jid];
            let s = &joint_subspaces[jid];
            let nd = model.jnt_type[jid].nv();
            for d in 0..nd {
                for row in 0..6 {
                    vj_rotating[row] += s[(row, d)] * data.qvel[da + d];
                }
            }
        }

        // Derivative w.r.t. each perturbing angular DOF on this body.
        // Any angular DOF (including Ball/Free) can rotate the body, changing
        // the orientation-dependent joint axes (Hinge/Slide).
        for jid in js..je {
            let da = model.jnt_dof_adr[jid];
            let nd = model.jnt_type[jid].nv();
            for d in 0..nd {
                let axis = dof_axis[da + d];
                if axis.norm_squared() == 0.0 {
                    continue; // Pure translational DOF — no body rotation
                }
                let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                let cross = spatial_cross_motion(s_ang, vj_rotating);
                for row in 0..6 {
                    data.deriv_Dcvel_pos[body_id][(row, da + d)] += cross[row];
                }
            }
        }

        // Ancestor joint subspace derivative: ∂(vJ_rotating)/∂q_k for ancestor DOFs.
        // When an ancestor angular DOF rotates, it changes this body's orientation,
        // affecting Hinge/Slide joint axes (not Ball/Free world-frame axes).
        if vj_rotating.norm_squared() > 0.0 {
            let mut anc = parent_id;
            while anc != 0 {
                let anc_js = model.body_jnt_adr[anc];
                let anc_je = anc_js + model.body_jnt_num[anc];
                for jid in anc_js..anc_je {
                    let da = model.jnt_dof_adr[jid];
                    let nd = model.jnt_type[jid].nv();
                    for d in 0..nd {
                        let axis = dof_axis[da + d];
                        if axis.norm_squared() == 0.0 {
                            continue;
                        }
                        let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                        let cross = spatial_cross_motion(s_ang, vj_rotating);
                        for row in 0..6 {
                            data.deriv_Dcvel_pos[body_id][(row, da + d)] += cross[row];
                        }
                    }
                }
                anc = model.body_parent[anc];
            }
        }
    }

    // ==================== Part A derivatives ====================
    // Part A acceleration: a_A[b] = X_b(a_A[parent]) + S·qacc
    // ∂a_A/∂q = X_b(∂a_A[parent]/∂q) + a_A[parent]_ang × (axis × r) + (∂S/∂q)·qacc
    // Part A force: f_A = I·a_A (no gyroscopic)
    // ∂f_A/∂q = I·∂a_A/∂q + (∂I/∂q)·a_A
    // Part A backward: X_b^T transport + transport derivative

    // Forward: acceleration derivatives
    for b in 1..nbody {
        let pid = model.body_parent[b];
        let r = data.xpos[b] - data.xpos[pid];

        // X_b transport of parent dcacc_A
        let parent_dcacc = data.deriv_Dcacc_pos[pid].clone();
        data.deriv_Dcacc_pos[b].copy_from(&parent_dcacc);
        for col in 0..nv {
            let ox = parent_dcacc[(0, col)];
            let oy = parent_dcacc[(1, col)];
            let oz = parent_dcacc[(2, col)];
            data.deriv_Dcacc_pos[b][(3, col)] += oy * r.z - oz * r.y;
            data.deriv_Dcacc_pos[b][(4, col)] += oz * r.x - ox * r.z;
            data.deriv_Dcacc_pos[b][(5, col)] += ox * r.y - oy * r.x;
        }

        // X_b transport derivative: a_A[parent]_ang × (axis × r)
        let omega_acc_p = Vector3::new(cacc_a[pid][0], cacc_a[pid][1], cacc_a[pid][2]);
        if omega_acc_p.norm_squared() > 0.0 {
            let mut anc = pid;
            while anc != 0 {
                let anc_js = model.body_jnt_adr[anc];
                let anc_je = anc_js + model.body_jnt_num[anc];
                for jid in anc_js..anc_je {
                    let da = model.jnt_dof_adr[jid];
                    let nd = model.jnt_type[jid].nv();
                    for d in 0..nd {
                        let axis = dof_axis[da + d];
                        let dr = axis.cross(&r);
                        let contrib = omega_acc_p.cross(&dr);
                        data.deriv_Dcacc_pos[b][(3, da + d)] += contrib.x;
                        data.deriv_Dcacc_pos[b][(4, da + d)] += contrib.y;
                        data.deriv_Dcacc_pos[b][(5, da + d)] += contrib.z;
                    }
                }
                anc = model.body_parent[anc];
            }
        }

        // Term D: ∂(Σ_j S_j · qacc_j)/∂q — joint acceleration subspace derivative.
        // When an angular DOF on this body is perturbed, it rotates all joint axes,
        // changing the total S·qacc contribution. Uses angular-only perturbing axis
        // with total joint acceleration (handles multi-joint bodies correctly).
        let js = model.body_jnt_adr[b];
        let je = js + model.body_jnt_num[b];

        // Pre-compute S·qacc from orientation-dependent joints only (Hinge/Slide).
        // Ball/Free use world-frame axes: ∂S/∂q = 0.
        let mut sj_qacc_rotating = SpatialVector::zeros();
        for jid in js..je {
            if matches!(
                model.jnt_type[jid],
                crate::types::MjJointType::Ball | crate::types::MjJointType::Free
            ) {
                continue;
            }
            let da = model.jnt_dof_adr[jid];
            let s = &joint_subspaces[jid];
            let nd = model.jnt_type[jid].nv();
            for d in 0..nd {
                for row in 0..6 {
                    sj_qacc_rotating[row] += s[(row, d)] * data.qacc[da + d];
                }
            }
        }

        // Same-body angular DOFs
        for jid in js..je {
            let da = model.jnt_dof_adr[jid];
            let nd = model.jnt_type[jid].nv();
            for d in 0..nd {
                let axis = dof_axis[da + d];
                if axis.norm_squared() == 0.0 {
                    continue;
                }
                let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                let cross_d = spatial_cross_motion(s_ang, sj_qacc_rotating);
                for row in 0..6 {
                    data.deriv_Dcacc_pos[b][(row, da + d)] += cross_d[row];
                }
            }
        }

        // Ancestor DOFs: ancestor angular rotations also change this body's joint axes
        if sj_qacc_rotating.norm_squared() > 0.0 {
            let mut anc = pid;
            while anc != 0 {
                let anc_js = model.body_jnt_adr[anc];
                let anc_je = anc_js + model.body_jnt_num[anc];
                for jid in anc_js..anc_je {
                    let da = model.jnt_dof_adr[jid];
                    let nd = model.jnt_type[jid].nv();
                    for d in 0..nd {
                        let axis = dof_axis[da + d];
                        if axis.norm_squared() == 0.0 {
                            continue;
                        }
                        let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                        let cross_d = spatial_cross_motion(s_ang, sj_qacc_rotating);
                        for row in 0..6 {
                            data.deriv_Dcacc_pos[b][(row, da + d)] += cross_d[row];
                        }
                    }
                }
                anc = model.body_parent[anc];
            }
        }
    }

    // Local force derivatives for Part A: I · dcacc_A (no gyroscopic)
    for b in 1..nbody {
        let inertia = &data.cinert[b];
        let dcacc = data.deriv_Dcacc_pos[b].clone();
        for c in 0..nv {
            let col = inertia * dcacc.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc_pos[b][(r, c)] = col[r];
            }
        }
    }

    // Inertia rotation derivatives for Part A: (∂I/∂q) · a_A (no velocity coupling)
    // Only the ANGULAR part of the motion subspace affects inertia rotation.
    // Translation DOFs (slide, free linear) don't rotate the body, so ∂I/∂q = 0.
    for b in 1..nbody {
        let inertia = &data.cinert[b];
        let i_cacc = inertia * cacc_a[b];

        let mut anc = b;
        while anc != 0 {
            let anc_js = model.body_jnt_adr[anc];
            let anc_je = anc_js + model.body_jnt_num[anc];
            for jid in anc_js..anc_je {
                let da = model.jnt_dof_adr[jid];
                let s = &joint_subspaces[jid];
                let nd = model.jnt_type[jid].nv();
                for d in 0..nd {
                    // Use angular part only: rotation is what changes inertia
                    let s_ang = SpatialVector::new(s[(0, d)], s[(1, d)], s[(2, d)], 0.0, 0.0, 0.0);
                    if s_ang[0] == 0.0 && s_ang[1] == 0.0 && s_ang[2] == 0.0 {
                        continue; // Pure translational DOF — no inertia rotation
                    }
                    // (∂I/∂q_d) · a = s_ang ×* (I·a) - I · (s_ang ×_m a)
                    let sf_i_a = spatial_cross_force(s_ang, i_cacc);
                    let i_sm_a = inertia * spatial_cross_motion(s_ang, cacc_a[b]);
                    for row in 0..6 {
                        data.deriv_Dcfrc_pos[b][(row, da + d)] += sf_i_a[row] - i_sm_a[row];
                    }
                }
            }
            anc = model.body_parent[anc];
        }
    }

    // Backward pass for Part A: X_b^T transport + transport derivative
    for b in (1..nbody).rev() {
        let pid = model.body_parent[b];
        if pid == 0 {
            continue;
        }
        let r = data.xpos[b] - data.xpos[pid];

        // X_b^T transport: [τ + r×f; f]
        let child_dcfrc = data.deriv_Dcfrc_pos[b].clone();
        for col in 0..nv {
            let fx = child_dcfrc[(3, col)];
            let fy = child_dcfrc[(4, col)];
            let fz = child_dcfrc[(5, col)];
            data.deriv_Dcfrc_pos[pid][(0, col)] += child_dcfrc[(0, col)] + r.y * fz - r.z * fy;
            data.deriv_Dcfrc_pos[pid][(1, col)] += child_dcfrc[(1, col)] + r.z * fx - r.x * fz;
            data.deriv_Dcfrc_pos[pid][(2, col)] += child_dcfrc[(2, col)] + r.x * fy - r.y * fx;
            data.deriv_Dcfrc_pos[pid][(3, col)] += fx;
            data.deriv_Dcfrc_pos[pid][(4, col)] += fy;
            data.deriv_Dcfrc_pos[pid][(5, col)] += fz;
        }

        // Transport derivative: (axis × r) × cfrc_A[child]_lin
        let f_lin = Vector3::new(cfrc_a[b][3], cfrc_a[b][4], cfrc_a[b][5]);
        if f_lin.norm_squared() > 0.0 {
            let mut anc = pid;
            while anc != 0 {
                let anc_js = model.body_jnt_adr[anc];
                let anc_je = anc_js + model.body_jnt_num[anc];
                for jid in anc_js..anc_je {
                    let da = model.jnt_dof_adr[jid];
                    let nd = model.jnt_type[jid].nv();
                    for d in 0..nd {
                        let axis = dof_axis[da + d];
                        let dr = axis.cross(&r);
                        let torque = dr.cross(&f_lin);
                        data.deriv_Dcfrc_pos[pid][(0, da + d)] += torque.x;
                        data.deriv_Dcfrc_pos[pid][(1, da + d)] += torque.y;
                        data.deriv_Dcfrc_pos[pid][(2, da + d)] += torque.z;
                    }
                }
                anc = model.body_parent[anc];
            }
        }
    }

    // Part A projection: S^T · Dcfrc_A → qDeriv_pos
    for jid in 0..model.njnt {
        let bid = model.jnt_body[jid];
        let da = model.jnt_dof_adr[jid];
        let s = &joint_subspaces[jid];
        let nd = model.jnt_type[jid].nv();
        for d in 0..nd {
            for c in 0..nv {
                let mut val = 0.0;
                for row in 0..6 {
                    val += s[(row, d)] * data.deriv_Dcfrc_pos[bid][(row, c)];
                }
                data.qDeriv_pos[(da + d, c)] -= val;
            }
        }
    }

    // Part A projection derivative: (∂S/∂q)^T · cfrc_A
    // When an angular DOF on the same body or any ancestor body is perturbed,
    // it rotates the body, changing the joint subspace S. Unified loop over
    // all perturbing angular DOFs (same-body + ancestors).
    // Skip Ball/Free TARGET joints: their world-frame S doesn't depend on q.
    for jid in 0..model.njnt {
        if matches!(
            model.jnt_type[jid],
            crate::types::MjJointType::Ball | crate::types::MjJointType::Free
        ) {
            continue; // World-frame subspace: ∂S/∂q = 0
        }
        let bid = model.jnt_body[jid];
        let da_j = model.jnt_dof_adr[jid];
        let s_j = &joint_subspaces[jid];
        let nd_j = model.jnt_type[jid].nv();
        let cfrc = cfrc_a[bid];

        // Iterate over all angular DOFs on this body and ancestors
        let mut anc = bid;
        while anc != 0 {
            let anc_js = model.body_jnt_adr[anc];
            let anc_je = anc_js + model.body_jnt_num[anc];
            for kid in anc_js..anc_je {
                let da_k = model.jnt_dof_adr[kid];
                let nd_k = model.jnt_type[kid].nv();
                for d in 0..nd_k {
                    let axis = dof_axis[da_k + d];
                    if axis.norm_squared() == 0.0 {
                        continue;
                    }
                    let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                    for dd in 0..nd_j {
                        let s_jdd = SpatialVector::new(
                            s_j[(0, dd)],
                            s_j[(1, dd)],
                            s_j[(2, dd)],
                            s_j[(3, dd)],
                            s_j[(4, dd)],
                            s_j[(5, dd)],
                        );
                        let ds = spatial_cross_motion(s_ang, s_jdd);
                        let proj: f64 = (0..6).map(|row| ds[row] * cfrc[row]).sum();
                        data.qDeriv_pos[(da_j + dd, da_k + d)] -= proj;
                    }
                }
            }
            anc = model.body_parent[anc];
        }
    }

    // ==================== Part B derivatives ====================
    // Zero scratch for reuse
    for b in 0..nbody {
        data.deriv_Dcacc_pos[b].fill(0.0);
        data.deriv_Dcfrc_pos[b].fill(0.0);
    }

    // Part B acceleration: a_B[b] = a_B[parent] + cvel[parent] ×_m S·qvel
    // ∂a_B/∂q = ∂a_B[parent]/∂q (simple copy, no transport derivative!)
    //         + (∂cvel[parent]/∂q) ×_m v_joint  (Term C)
    //         + cvel[parent] ×_m (∂S/∂q · qvel)  (zero for hinges)
    for b in 1..nbody {
        let pid = model.body_parent[b];

        // Simple copy of parent dcacc_B (no X_b transport, no transport derivative)
        let parent_dcacc = data.deriv_Dcacc_pos[pid].clone();
        data.deriv_Dcacc_pos[b].copy_from(&parent_dcacc);

        let js = model.body_jnt_adr[b];
        let je = js + model.body_jnt_num[b];
        for jid in js..je {
            let da = model.jnt_dof_adr[jid];
            let s = &joint_subspaces[jid];
            let nd = model.jnt_type[jid].nv();

            // v_joint = S · qvel
            let mut v_joint = SpatialVector::zeros();
            for d in 0..nd {
                for row in 0..6 {
                    v_joint[row] += s[(row, d)] * data.qvel[da + d];
                }
            }

            // Term C: (∂cvel[parent]/∂q) ×_m v_joint
            let mat = mjd_cross_motion_vel(&v_joint);
            let parent_dcvel = data.deriv_Dcvel_pos[pid].clone();
            for c in 0..nv {
                let col = mat * parent_dcvel.column(c);
                for row in 0..6 {
                    data.deriv_Dcacc_pos[b][(row, c)] += col[row];
                }
            }

            // Velocity subspace derivative: cvel[parent] ×_m ((∂S/∂q)·qvel)
            // For hinges, ∂S/∂q = 0. For other joint types:
            // ∂(S·qvel)/∂q_d = (S_d ×_m S)·qvel
            // Not needed for hinges; included for generality.
            // (Already captured by the self-cross term in Dcvel which feeds Term C.)
        }
    }

    // Local force derivatives for Part B:
    //   I · dcacc_B + gyroscopic velocity derivatives + (∂I/∂q) terms
    for b in 1..nbody {
        let inertia = &data.cinert[b];
        let cvel_b = data.cvel[b];
        let i_v = inertia * cvel_b;

        // I · dcacc_B
        let dcacc = data.deriv_Dcacc_pos[b].clone();
        for c in 0..nv {
            let col = inertia * dcacc.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc_pos[b][(r, c)] = col[r];
            }
        }

        // Gyroscopic velocity derivatives: ∂(v ×* (I·v))/∂q
        // = crossForce_vel(I·v) · Dcvel + crossForce_frc(v) · I · Dcvel
        let cross_vel_mat = mjd_cross_force_vel(&i_v);
        let dcvel = data.deriv_Dcvel_pos[b].clone();
        for c in 0..nv {
            let col = cross_vel_mat * dcvel.column(c);
            for r in 0..6 {
                data.deriv_Dcfrc_pos[b][(r, c)] += col[r];
            }
        }
        let cross_frc_mat = mjd_cross_force_frc(&cvel_b);
        for c in 0..nv {
            let i_col = inertia * dcvel.column(c);
            let result = cross_frc_mat * i_col;
            for r in 0..6 {
                data.deriv_Dcfrc_pos[b][(r, c)] += result[r];
            }
        }
    }

    // Inertia rotation derivatives for Part B:
    //   (∂I/∂q) · a_B + v ×* ((∂I/∂q) · v)
    // Only angular part of motion subspace affects inertia rotation.
    for b in 1..nbody {
        let inertia = &data.cinert[b];
        let i_cacc = inertia * cacc_b[b];
        let cvel_b = data.cvel[b];
        let i_cvel = inertia * cvel_b;

        let mut anc = b;
        while anc != 0 {
            let anc_js = model.body_jnt_adr[anc];
            let anc_je = anc_js + model.body_jnt_num[anc];
            for jid in anc_js..anc_je {
                let da = model.jnt_dof_adr[jid];
                let s = &joint_subspaces[jid];
                let nd = model.jnt_type[jid].nv();
                for d in 0..nd {
                    let s_ang = SpatialVector::new(s[(0, d)], s[(1, d)], s[(2, d)], 0.0, 0.0, 0.0);
                    if s_ang[0] == 0.0 && s_ang[1] == 0.0 && s_ang[2] == 0.0 {
                        continue; // Pure translational DOF — no inertia rotation
                    }
                    // (∂I/∂q_d) · a_B
                    let sf_i_a = spatial_cross_force(s_ang, i_cacc);
                    let i_sm_a = inertia * spatial_cross_motion(s_ang, cacc_b[b]);
                    // v ×* ((∂I/∂q_d) · v)
                    let sf_i_v = spatial_cross_force(s_ang, i_cvel);
                    let i_sm_v = inertia * spatial_cross_motion(s_ang, cvel_b);
                    let di_v = sf_i_v - i_sm_v;
                    let vel_coupling = spatial_cross_force(cvel_b, di_v);
                    for row in 0..6 {
                        data.deriv_Dcfrc_pos[b][(row, da + d)] +=
                            (sf_i_a[row] - i_sm_a[row]) + vel_coupling[row];
                    }
                }
            }
            anc = model.body_parent[anc];
        }
    }

    // Backward pass for Part B: simple add (NO transport derivative)
    for b in (1..nbody).rev() {
        let pid = model.body_parent[b];
        if pid == 0 {
            continue;
        }
        // Simple add: just accumulate derivatives (no X_b^T, no transport derivative)
        let child_dcfrc = data.deriv_Dcfrc_pos[b].clone();
        for col in 0..nv {
            for row in 0..6 {
                data.deriv_Dcfrc_pos[pid][(row, col)] += child_dcfrc[(row, col)];
            }
        }
    }

    // Part B projection: S^T · Dcfrc_B → qDeriv_pos (add to existing Part A)
    for jid in 0..model.njnt {
        let bid = model.jnt_body[jid];
        let da = model.jnt_dof_adr[jid];
        let s = &joint_subspaces[jid];
        let nd = model.jnt_type[jid].nv();
        for d in 0..nd {
            for c in 0..nv {
                let mut val = 0.0;
                for row in 0..6 {
                    val += s[(row, d)] * data.deriv_Dcfrc_pos[bid][(row, c)];
                }
                data.qDeriv_pos[(da + d, c)] -= val;
            }
        }
    }

    // Part B projection derivative: (∂S/∂q)^T · cfrc_B
    // Same unified ancestor structure as Part A projection derivative.
    // Skip Ball/Free TARGET joints: their world-frame S doesn't depend on q.
    for jid in 0..model.njnt {
        if matches!(
            model.jnt_type[jid],
            crate::types::MjJointType::Ball | crate::types::MjJointType::Free
        ) {
            continue;
        }
        let bid = model.jnt_body[jid];
        let da_j = model.jnt_dof_adr[jid];
        let s_j = &joint_subspaces[jid];
        let nd_j = model.jnt_type[jid].nv();
        let cfrc = cfrc_b[bid];

        let mut anc = bid;
        while anc != 0 {
            let anc_js = model.body_jnt_adr[anc];
            let anc_je = anc_js + model.body_jnt_num[anc];
            for kid in anc_js..anc_je {
                let da_k = model.jnt_dof_adr[kid];
                let nd_k = model.jnt_type[kid].nv();
                for d in 0..nd_k {
                    let axis = dof_axis[da_k + d];
                    if axis.norm_squared() == 0.0 {
                        continue;
                    }
                    let s_ang = SpatialVector::new(axis[0], axis[1], axis[2], 0.0, 0.0, 0.0);
                    for dd in 0..nd_j {
                        let s_jdd = SpatialVector::new(
                            s_j[(0, dd)],
                            s_j[(1, dd)],
                            s_j[(2, dd)],
                            s_j[(3, dd)],
                            s_j[(4, dd)],
                            s_j[(5, dd)],
                        );
                        let ds = spatial_cross_motion(s_ang, s_jdd);
                        let proj: f64 = (0..6).map(|row| ds[row] * cfrc[row]).sum();
                        data.qDeriv_pos[(da_j + dd, da_k + d)] -= proj;
                    }
                }
            }
            anc = model.body_parent[anc];
        }
    }
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
    let theta_vec = omega * h; // ω·h = θ
    let theta = theta_vec.norm(); // ‖ω‖·h

    if theta < 1e-8 {
        // Small-angle limits: J_l^{-1}(0) = I, velocity = h·I
        return (Matrix3::identity(), Matrix3::identity() * h);
    }

    let theta2 = theta * theta;

    // Skew-symmetric matrix [θ]×  where θ = ω·h
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

    // Position Jacobian: J_l^{-1}(θ) — the inverse left Jacobian of SO(3).
    //
    // Right-multiply integration: q_new = q_old * exp(θ)
    // FD tangent convention: both input and output tangent measured from q_0.
    //   η = log(q_0^{-1} · q_next) = log(exp(ξ) · exp(θ)) using BCH
    //   ∂η/∂ξ|_{ξ=0} = J_l^{-1}(θ)
    //
    // J_l^{-1}(θ) = I − ½·[θ]× + α·[θ]×²
    //   where α = 1/θ² − (1+cosθ)/(2θ·sinθ)  →  1/12 as θ → 0
    let alpha = 1.0 / theta2 - (1.0 + cos_theta) / (2.0 * theta * sin_theta);
    let dqnew_dqold = Matrix3::identity() - skew_theta * 0.5 + skew_theta_sq * alpha;

    // Velocity Jacobian: h·I for the FD tangent convention.
    //   η = log(exp(0) · exp(h·ω)) = h·ω  (exact)
    //   ∂η/∂ω = h·I
    let dqnew_domega = Matrix3::identity() * h;

    (dqnew_dqold, dqnew_domega)
}

/// Integration Jacobian blocks for one Euler or ImplicitSpringDamper step.
///
/// Contains `∂qpos/∂qpos`, `∂qpos/∂qvel`, `∂act/∂act`, and `∂act/∂act_dot`
/// from the integration stage only (not the force-velocity coupling).
#[allow(non_snake_case)]
struct IntegrationDerivatives {
    /// ∂qpos_{t+1}/∂qpos_t in tangent space. nv × nv, block-diagonal per joint.
    /// Consumed by `mjd_transition_hybrid` for analytical position columns.
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
                ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle => {
                    // Approximate: ignores ∂act_dot/∂act from act-dependent
                    // time constants. Not consumed by hybrid (Muscle uses FD).
                    let at_boundary = data.act[j] <= 0.0 || data.act[j] >= 1.0;
                    dact_dact[(j, j)] = if at_boundary { 0.0 } else { 1.0 };
                    dact_dactdot[(j, j)] = if at_boundary { 0.0 } else { h };
                }
                ActuatorDynamics::None => {
                    // No activation state
                }
                ActuatorDynamics::User => {
                    // User-defined dynamics: no analytical derivative available.
                    // FD fallback handles this via the hybrid path.
                    dact_dact[(j, j)] = 1.0;
                    dact_dactdot[(j, j)] = h;
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
#[allow(non_snake_case, clippy::similar_names, clippy::unwrap_used)]
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
    // MuJoCo rejects models with history (delays) for FD derivatives.
    assert!(
        model.nhistory == 0,
        "FD derivatives not supported with nhistory > 0 (delays)"
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
            ActuatorDynamics::Muscle | ActuatorDynamics::HillMuscle
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
                GainType::Muscle | GainType::HillMuscle | GainType::User => {
                    // Muscle/HillMuscle: guarded by is_muscle check above
                    // User: no analytical derivative — fall back to FD
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

    // === 5. Position columns of A ===
    // Check eligibility for analytical position columns.
    // Ineligible models fall back to FD for position columns.
    let use_analytical_pos = model.density == 0.0
        && model.viscosity == 0.0
        && !model.body_gravcomp.iter().any(|g| *g != 0.0)
        && model.nflex == 0
        && !model.actuator_trntype.iter().any(|t| {
            matches!(
                t,
                ActuatorTransmission::Site
                    | ActuatorTransmission::Body
                    | ActuatorTransmission::SliderCrank
            )
        })
        && !model
            .actuator_biastype
            .iter()
            .any(|t| matches!(t, BiasType::Muscle | BiasType::HillMuscle));

    // Save nominal state for FD perturbation loop (needed by activation/B matrix FD too)
    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let act_0 = data.act.clone();
    let ctrl_0 = data.ctrl.clone();
    let warmstart_0 = data.qacc_warmstart.clone();
    let time_0 = data.time;
    let mut scratch = data.clone();
    let ns = model.nsensordata;
    let compute_sensors = config.compute_sensor_derivatives && ns > 0;

    // Compute nominal next state by stepping unperturbed.
    // MuJoCo always computes this unconditionally. We need it for:
    // - forward differencing (non-centered A/B columns)
    // - clamped control differencing fallback (centered mode where one
    //   direction is infeasible due to actuator_ctrlrange boundary)
    scratch.qacc_warmstart.copy_from(&warmstart_0);
    scratch.step(model)?;
    let y_0 = extract_state(model, &scratch, &qpos_0);
    let sensor_0 = if compute_sensors {
        Some(scratch.sensordata.clone())
    } else {
        None
    };
    scratch.qpos.copy_from(&qpos_0);
    scratch.qvel.copy_from(&qvel_0);
    scratch.act.copy_from(&act_0);
    scratch.ctrl.copy_from(&ctrl_0);
    scratch.qacc_warmstart.copy_from(&warmstart_0);
    scratch.time = time_0;

    let mut c_mat = if compute_sensors {
        Some(DMatrix::zeros(ns, nx))
    } else {
        None
    };
    let mut d_mat = if compute_sensors {
        Some(DMatrix::zeros(ns, nu))
    } else {
        None
    };

    if use_analytical_pos {
        // Analytical position columns via mjd_smooth_pos + chain rule
        mjd_smooth_pos(model, &mut data_work);

        let dvdq = match model.integrator {
            Integrator::Euler => {
                // dvdq = h · M⁻¹ · qDeriv_pos
                let mut dvdq = data_work.qDeriv_pos.clone();
                let (rowadr, rownnz, colind) = model.qld_csr();
                mj_solve_sparse_batch(
                    rowadr,
                    rownnz,
                    colind,
                    &data_work.qLD_data,
                    &data_work.qLD_diag_inv,
                    &mut dvdq,
                );
                dvdq *= h;
                dvdq
            }
            Integrator::ImplicitSpringDamper => {
                // dvdq = (M+hD+h²K)⁻¹ · h · qDeriv_pos
                let mut dvdq = DMatrix::zeros(nv, nv);
                for j in 0..nv {
                    let mut rhs = DVector::zeros(nv);
                    for i in 0..nv {
                        rhs[i] = h * data_work.qDeriv_pos[(i, j)];
                    }
                    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut rhs);
                    dvdq.column_mut(j).copy_from(&rhs);
                }
                dvdq
            }
            Integrator::ImplicitFast => {
                // dvdq = h · (M − h·D)⁻¹ · qDeriv_pos
                let mut dvdq = DMatrix::zeros(nv, nv);
                for j in 0..nv {
                    let mut col = data_work.qDeriv_pos.column(j).clone_owned();
                    cholesky_solve_in_place(&data_work.scratch_m_impl, &mut col);
                    for i in 0..nv {
                        dvdq[(i, j)] = h * col[i];
                    }
                }
                dvdq
            }
            Integrator::Implicit => {
                // dvdq = h · LU⁻¹ · qDeriv_pos
                let mut dvdq = DMatrix::zeros(nv, nv);
                for j in 0..nv {
                    let mut col = data_work.qDeriv_pos.column(j).clone_owned();
                    lu_solve_factored(
                        &data_work.scratch_m_impl,
                        &data_work.scratch_lu_piv,
                        &mut col,
                    );
                    for i in 0..nv {
                        dvdq[(i, j)] = h * col[i];
                    }
                }
                dvdq
            }
            Integrator::RungeKutta4 => unreachable!(),
        };

        // Chain rule: ∂q⁺/∂q = dqpos_dqpos + dqpos_dqvel · dvdq
        let dqdq = &integ.dqpos_dqpos + &integ.dqpos_dqvel * &dvdq;

        // Fill position columns of A
        a_mat.view_mut((0, 0), (nv, nv)).copy_from(&dqdq);
        a_mat.view_mut((nv, 0), (nv, nv)).copy_from(&dvdq);
        // ∂act/∂qpos = 0 (already zero from allocation)

        // Sensor C position columns — need FD since analytical doesn't capture sensor outputs.
        if compute_sensors {
            for i in 0..nv {
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
                scratch.forward_skip(model, MjStage::None, false)?;
                scratch.integrate(model);
                let s_plus = scratch.sensordata.clone();

                if config.centered {
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
                    scratch.forward_skip(model, MjStage::None, false)?;
                    scratch.integrate(model);
                    let s_minus = scratch.sensordata.clone();
                    let scol = (&s_plus - &s_minus) / (2.0 * eps);
                    c_mat.as_mut().unwrap().column_mut(i).copy_from(&scol);
                } else {
                    let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                    c_mat.as_mut().unwrap().column_mut(i).copy_from(&scol);
                }
            }
        }
    } else {
        // Position FD columns (0..nv) — piggyback sensor recording
        for i in 0..nv {
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
                let col = (&y_plus - &y_minus) / (2.0 * eps);
                a_mat.column_mut(i).copy_from(&col);

                if let (Some(c), Some(sp), Some(sm)) = (&mut c_mat, &s_plus, &s_minus) {
                    let scol = (sp - sm) / (2.0 * eps);
                    c.column_mut(i).copy_from(&scol);
                }
            } else {
                let col = (&y_plus - &y_0) / eps;
                a_mat.column_mut(i).copy_from(&col);

                if let (Some(c), Some(sp), Some(s0)) = (&mut c_mat, &s_plus, &sensor_0) {
                    let scol = (sp - s0) / eps;
                    c.column_mut(i).copy_from(&scol);
                }
            }
        }
    }

    // === Velocity columns: sensor-only FD ===
    // A velocity columns are analytical (no FD step). Sensor C velocity
    // columns need FD passes with skipstage=MjStage::Pos.
    if compute_sensors {
        for i in 0..nv {
            let state_col = nv + i;
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                &warmstart_0,
                time_0,
                state_col,
                eps,
                nv,
                na,
            );
            scratch.forward_skip(model, MjStage::Pos, false)?;
            scratch.integrate(model);
            let s_plus = scratch.sensordata.clone();

            if config.centered {
                apply_state_perturbation(
                    model,
                    &mut scratch,
                    &qpos_0,
                    &qvel_0,
                    &act_0,
                    &ctrl_0,
                    &warmstart_0,
                    time_0,
                    state_col,
                    -eps,
                    nv,
                    na,
                );
                scratch.forward_skip(model, MjStage::Pos, false)?;
                scratch.integrate(model);
                let s_minus = scratch.sensordata.clone();
                let scol = (&s_plus - &s_minus) / (2.0 * eps);
                c_mat
                    .as_mut()
                    .unwrap()
                    .column_mut(state_col)
                    .copy_from(&scol);
            } else {
                let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                c_mat
                    .as_mut()
                    .unwrap()
                    .column_mut(state_col)
                    .copy_from(&scol);
            }
        }
    }

    // === Activation columns: sensor-only FD for analytically-computed activation columns ===
    if compute_sensors {
        // Determine which activation columns were computed analytically
        // (i.e., NOT in act_fd_indices) and run sensor-only FD for those.
        let act_fd_set: std::collections::HashSet<usize> = act_fd_indices.iter().copied().collect();
        for actuator_idx in 0..model.nu {
            let act_adr = model.actuator_act_adr[actuator_idx];
            let act_num = model.actuator_act_num[actuator_idx];
            if act_num == 0 {
                continue;
            }
            for k in 0..act_num {
                let j = act_adr + k;
                let state_col = 2 * nv + j;
                if act_fd_set.contains(&state_col) {
                    continue; // FD column — sensor recording handled in piggybacked FD below
                }
                // Sensor-only FD for analytical activation column
                apply_state_perturbation(
                    model,
                    &mut scratch,
                    &qpos_0,
                    &qvel_0,
                    &act_0,
                    &ctrl_0,
                    &warmstart_0,
                    time_0,
                    state_col,
                    eps,
                    nv,
                    na,
                );
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                let s_plus = scratch.sensordata.clone();

                if config.centered {
                    apply_state_perturbation(
                        model,
                        &mut scratch,
                        &qpos_0,
                        &qvel_0,
                        &act_0,
                        &ctrl_0,
                        &warmstart_0,
                        time_0,
                        state_col,
                        -eps,
                        nv,
                        na,
                    );
                    scratch.forward_skip(model, MjStage::Vel, false)?;
                    scratch.integrate(model);
                    let s_minus = scratch.sensordata.clone();
                    let scol = (&s_plus - &s_minus) / (2.0 * eps);
                    c_mat
                        .as_mut()
                        .unwrap()
                        .column_mut(state_col)
                        .copy_from(&scol);
                } else {
                    let scol = (&s_plus - sensor_0.as_ref().unwrap()) / eps;
                    c_mat
                        .as_mut()
                        .unwrap()
                        .column_mut(state_col)
                        .copy_from(&scol);
                }
            }
        }
    }

    // Muscle activation FD fallback columns — piggyback sensor recording
    for &state_col in &act_fd_indices {
        apply_state_perturbation(
            model,
            &mut scratch,
            &qpos_0,
            &qvel_0,
            &act_0,
            &ctrl_0,
            &warmstart_0,
            time_0,
            state_col,
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
            apply_state_perturbation(
                model,
                &mut scratch,
                &qpos_0,
                &qvel_0,
                &act_0,
                &ctrl_0,
                &warmstart_0,
                time_0,
                state_col,
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
            let col = (&y_plus - &y_minus) / (2.0 * eps);
            a_mat.column_mut(state_col).copy_from(&col);

            if let (Some(c), Some(sp), Some(sm)) = (&mut c_mat, &s_plus, &s_minus) {
                let scol = (sp - sm) / (2.0 * eps);
                c.column_mut(state_col).copy_from(&scol);
            }
        } else {
            let col = (&y_plus - &y_0) / eps;
            a_mat.column_mut(state_col).copy_from(&col);

            if let (Some(c), Some(sp), Some(s0)) = (&mut c_mat, &s_plus, &sensor_0) {
                let scol = (sp - s0) / eps;
                c.column_mut(state_col).copy_from(&scol);
            }
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
                GainType::Muscle | GainType::HillMuscle | GainType::User => {
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

    // === Sensor-only FD for analytical B columns ===
    if compute_sensors {
        let ctrl_fd_set: std::collections::HashSet<usize> =
            ctrl_fd_indices.iter().copied().collect();
        for actuator_idx in 0..nu {
            if ctrl_fd_set.contains(&actuator_idx) {
                continue; // Handled in piggybacked FD below
            }
            // Sensor-only FD for this analytical B column
            let range = model.actuator_ctrlrange[actuator_idx];
            let nudge_fwd = in_ctrl_range(ctrl_0[actuator_idx], ctrl_0[actuator_idx] + eps, range);
            let nudge_back = (config.centered || !nudge_fwd)
                && in_ctrl_range(ctrl_0[actuator_idx] - eps, ctrl_0[actuator_idx], range);

            let s_plus = if nudge_fwd {
                scratch.qpos.copy_from(&qpos_0);
                scratch.qvel.copy_from(&qvel_0);
                scratch.act.copy_from(&act_0);
                scratch.ctrl.copy_from(&ctrl_0);
                scratch.ctrl[actuator_idx] += eps;
                scratch.qacc_warmstart.copy_from(&warmstart_0);
                scratch.time = time_0;
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            let s_minus = if nudge_back {
                scratch.qpos.copy_from(&qpos_0);
                scratch.qvel.copy_from(&qvel_0);
                scratch.act.copy_from(&act_0);
                scratch.ctrl.copy_from(&ctrl_0);
                scratch.ctrl[actuator_idx] -= eps;
                scratch.qacc_warmstart.copy_from(&warmstart_0);
                scratch.time = time_0;
                scratch.forward_skip(model, MjStage::Vel, false)?;
                scratch.integrate(model);
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            let s0 = sensor_0.as_ref().unwrap();
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d_mat
                .as_mut()
                .unwrap()
                .column_mut(actuator_idx)
                .copy_from(&scol);
        }
    }

    // FD for non-analytical B columns — piggyback sensor recording
    let nx = 2 * nv + na;
    for &j in &ctrl_fd_indices {
        let range = model.actuator_ctrlrange[j];
        let nudge_fwd = in_ctrl_range(ctrl_0[j], ctrl_0[j] + eps, range);
        let nudge_back =
            (config.centered || !nudge_fwd) && in_ctrl_range(ctrl_0[j] - eps, ctrl_0[j], range);

        let (y_plus, s_plus) = if nudge_fwd {
            scratch.qpos.copy_from(&qpos_0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.act.copy_from(&act_0);
            scratch.ctrl.copy_from(&ctrl_0);
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.ctrl[j] += eps;
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
            scratch.qacc_warmstart.copy_from(&warmstart_0);
            scratch.ctrl[j] -= eps;
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

        let col = match (&y_plus, &y_minus) {
            (Some(yp), Some(ym)) => (yp - ym) / (2.0 * eps),
            (Some(yp), None) => (yp - &y_0) / eps,
            (None, Some(ym)) => (&y_0 - ym) / eps,
            (None, None) => DVector::zeros(nx),
        };
        b_mat.column_mut(j).copy_from(&col);

        // Sensor D column (piggybacked — same clamped differencing)
        if let Some(d) = &mut d_mat {
            let s0 = sensor_0.as_ref().unwrap();
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => (sp - sm) / (2.0 * eps),
                (Some(sp), None) => (sp - s0) / eps,
                (None, Some(sm)) => (s0 - sm) / eps,
                (None, None) => DVector::zeros(ns),
            };
            d.column_mut(j).copy_from(&scol);
        }
    }

    // Handle nsensordata == 0 with compute_sensor_derivatives == true:
    // Return empty Some matrices (AD-2).
    let (c_result, d_result) = if config.compute_sensor_derivatives {
        if ns > 0 {
            (c_mat, d_mat)
        } else {
            (Some(DMatrix::zeros(0, nx)), Some(DMatrix::zeros(0, nu)))
        }
    } else {
        (None, None)
    };

    Ok(TransitionMatrices {
        A: a_mat,
        B: b_mat,
        C: c_result,
        D: d_result,
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
        compute_sensor_derivatives: false,
    };
    let c2 = DerivativeConfig {
        eps: eps / 10.0,
        centered: true,
        use_analytical: false,
        compute_sensor_derivatives: false,
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
// Step 12 — mjd_sub_quat: Quaternion subtraction Jacobians (DT-52)
// ============================================================================

/// Compute the 3×3 Jacobians of quaternion subtraction with respect to each
/// input quaternion, in tangent space.
///
/// Given `res = subquat(qa, qb)` — the axis-angle 3-vector representing the
/// rotation from `qb` to `qa` — this function computes:
/// - `d(res)/d(qa)` — 3×3 Jacobian w.r.t. perturbations of `qa` in its tangent space
/// - `d(res)/d(qb)` — 3×3 Jacobian w.r.t. perturbations of `qb` in its tangent space
///
/// # Tangent-space convention
///
/// Perturbations are in the SO(3) tangent space (3D axis-angle), not in raw
/// 4D quaternion coordinates. A perturbation `δ` of `qa` means
/// `qa' = qa · exp(δ/2)`, and the Jacobian `d(res)/d(qa)` maps `δ` to the
/// change in the subtraction result.
///
/// # MuJoCo correspondence
///
/// Matches `mjd_subQuat()` in `engine_derivative.c`. The result quaternion
/// `dq = qb^{-1} · qa` is decomposed as `res = log(dq)` (axis × angle),
/// and the Jacobians are computed via the inverse right Jacobian of SO(3).
///
/// # Edge cases
///
/// - Identity (`qa == qb`): both Jacobians are `±I` (small-angle limit).
/// - Antipodal (`qa == -qb`): angle ≈ π, inverse Jacobian has a singularity
///   at θ = 2π but `subquat` wraps to `[-π, π]`, so this returns the
///   limiting form at θ → π.
#[must_use]
#[allow(non_snake_case, clippy::similar_names)]
pub fn mjd_sub_quat(qa: &[f64; 4], qb: &[f64; 4]) -> (Matrix3<f64>, Matrix3<f64>) {
    // Compute the relative quaternion: dq = qb^{-1} * qa
    let qa_uq =
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qa[0], qa[1], qa[2], qa[3]));
    let qb_uq =
        UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qb[0], qb[1], qb[2], qb[3]));
    let dq = qb_uq.conjugate() * qa_uq;

    // Extract axis-angle from dq
    let xyz = Vector3::new(dq.i, dq.j, dq.k);
    let sin_half = xyz.norm();

    if sin_half < 1e-14 {
        // Small-angle limit: res ≈ 0, both Jacobians are ±I.
        // d(res)/d(qa) perturbation: qa' = qa·exp(δ/2) → dq' = qb⁻¹·qa·exp(δ/2)
        //   → res' ≈ res + δ → d(res)/d(qa) = I
        // d(res)/d(qb) perturbation: qb' = qb·exp(δ/2) → dq' = exp(-δ/2)·qb⁻¹·qa
        //   → res' ≈ res - δ → d(res)/d(qb) = -I
        return (Matrix3::identity(), -Matrix3::identity());
    }

    let axis = xyz / sin_half;
    let mut angle = 2.0 * sin_half.atan2(dq.w);
    // Shortest path wrapping
    if angle > std::f64::consts::PI {
        angle -= 2.0 * std::f64::consts::PI;
    }

    let theta = angle; // signed angle
    let theta_abs = theta.abs();

    // Compute the inverse right Jacobian of SO(3): J_r^{-1}(θ)
    // J_r^{-1}(v) = I + 0.5·[v]× + (1/θ² − (1+cos θ)/(2θ sin θ))·[v]×²
    // where v = axis * angle (the expmap vector)
    let v = axis * theta;
    let skew_v = Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0);
    let skew_v_sq = skew_v * skew_v;

    let jr_inv = if theta_abs < 1e-8 {
        // Small-angle: J_r^{-1} ≈ I + 0.5·[v]× + 1/12·[v]×²
        Matrix3::identity() + skew_v * 0.5 + skew_v_sq * (1.0 / 12.0)
    } else {
        let coeff = 1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
        Matrix3::identity() + skew_v * 0.5 + skew_v_sq * coeff
    };

    // d(res)/d(qa): perturb qa → qa·exp(δ/2)
    // dq' = qb⁻¹ · qa · exp(δ/2) = dq · exp(δ/2)
    // res' = log(dq · exp(δ/2)) ≈ res + J_r^{-1}(res) · δ
    // So: d(res)/d(qa) = J_r^{-1}(res)
    let d_dqa = jr_inv;

    // d(res)/d(qb): perturb qb → qb·exp(δ/2)
    // dq' = (qb·exp(δ/2))⁻¹ · qa = exp(-δ/2) · qb⁻¹ · qa = exp(-δ/2) · dq
    // res' = log(exp(-δ/2) · dq)
    //
    // Using the BCH / adjoint formula:
    // log(exp(-δ/2)·exp(v)) ≈ v − J_l^{-1}(v)·δ   (for small δ)
    // where J_l^{-1}(v) = J_r^{-1}(-v)
    //
    // J_l^{-1}(v) = I − 0.5·[v]× + coeff·[v]×²
    // So: d(res)/d(qb) = −J_l^{-1}(res)
    let jl_inv = if theta_abs < 1e-8 {
        Matrix3::identity() - skew_v * 0.5 + skew_v_sq * (1.0 / 12.0)
    } else {
        let coeff = 1.0 / (theta * theta) - (1.0 + theta.cos()) / (2.0 * theta * theta.sin());
        Matrix3::identity() - skew_v * 0.5 + skew_v_sq * coeff
    };
    let d_dqb = -jl_inv;

    (d_dqa, d_dqb)
}

// ============================================================================
// Step 13 — DT-54: Muscle actuator velocity derivative helpers
// ============================================================================

/// Derivative of the MuJoCo piecewise-quadratic force-velocity curve w.r.t.
/// normalized velocity.
///
/// `velocity` is normalized: `V_norm = V / (l0 * vmax)`, so `V_norm = -1` is
/// max shortening. `fvmax` is the eccentric force plateau.
///
/// Returns `dFV/dV_norm`.
fn muscle_gain_velocity_deriv(velocity: f64, fvmax: f64) -> f64 {
    let y = fvmax - 1.0;
    if velocity <= -1.0 {
        // FV = 0, flat
        0.0
    } else if velocity <= 0.0 {
        // FV = (v+1)^2 → dFV/dv = 2(v+1)
        2.0 * (velocity + 1.0)
    } else if velocity <= y {
        // FV = fvmax - (y-v)^2/y → dFV/dv = 2(y-v)/y
        let y_safe = y.max(1e-10);
        2.0 * (y - velocity) / y_safe
    } else {
        // FV = fvmax, flat
        0.0
    }
}

/// Derivative of the Hill-type hyperbolic force-velocity curve w.r.t.
/// normalized velocity.
///
/// Returns `dFV/dV_norm`.
fn hill_force_velocity_deriv(norm_vel: f64) -> f64 {
    let a = 0.25; // curvature parameter
    let fv_max = 1.5; // eccentric plateau
    if norm_vel <= -1.0 {
        // FV = 0, flat
        0.0
    } else if norm_vel <= 0.0 {
        // Concentric: FV = (1+v)/(1-v/a)
        // dFV/dv = (1/a + 1) / (1-v/a)^2 = (1+a) / (a*(1-v/a)^2)
        //        = (1+a)/a · 1/(1-v/a)^2
        let denom = 1.0 - norm_vel / a;
        (1.0 + a) / (a * denom * denom)
    } else {
        // Eccentric: FV = 1 + (fv_max-1)*v/(v+a)
        // dFV/dv = (fv_max-1)*a/(v+a)^2
        let denom = norm_vel + a;
        (fv_max - 1.0) * a / (denom * denom)
    }
}

// ============================================================================
// Step 14 — DT-51: mjd_inverse_fd (Inverse Dynamics Derivatives)
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
        scratch.forward_skip(model, MjStage::None, true)?;
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
        scratch.forward_skip(model, MjStage::None, true)?;
        scratch.qacc.copy_from(&qacc_0);
        scratch.inverse(model);
        let f_plus = scratch.qfrc_inverse.clone();

        if config.centered {
            // -eps perturbation
            dq[i] = -eps;
            mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
            scratch.qvel.copy_from(&qvel_0);
            scratch.forward_skip(model, MjStage::None, true)?;
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
    scratch.forward_skip(model, MjStage::None, true)?;

    for i in 0..nv {
        // +eps perturbation
        scratch.qvel.copy_from(&qvel_0);
        scratch.qvel[i] += eps;
        scratch.forward_skip(model, MjStage::Pos, true)?;
        scratch.qacc.copy_from(&qacc_0);
        scratch.inverse(model);
        let f_plus = scratch.qfrc_inverse.clone();

        if config.centered {
            // -eps perturbation
            scratch.qvel.copy_from(&qvel_0);
            scratch.qvel[i] -= eps;
            scratch.forward_skip(model, MjStage::Pos, true)?;
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
    use crate::forward::{ellipsoid_moment, norm3};

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

// ============================================================================
// DT-52 Unit Tests — mjd_sub_quat FD validation
// ============================================================================

#[cfg(test)]
#[allow(
    clippy::similar_names,
    clippy::needless_range_loop,
    clippy::uninlined_format_args
)]
mod sub_quat_tests {
    use super::*;
    use crate::tendon::subquat;

    const EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    /// Apply a tangent-space perturbation to a quaternion:
    /// q' = q · exp(δ/2)
    fn perturb_quat(q: &[f64; 4], delta: &Vector3<f64>) -> [f64; 4] {
        let q_uq = UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]));
        let half_delta = delta * 0.5;
        let angle = half_delta.norm();
        let exp_half = if angle < 1e-14 {
            UnitQuaternion::identity()
        } else {
            let axis = nalgebra::Unit::new_normalize(half_delta / angle);
            UnitQuaternion::from_axis_angle(&axis, 2.0 * angle)
        };
        let q_new = q_uq * exp_half;
        [q_new.w, q_new.i, q_new.j, q_new.k]
    }

    /// Evaluate subquat as a function of raw quaternion arrays.
    fn eval_subquat(qa: &[f64; 4], qb: &[f64; 4]) -> Vector3<f64> {
        let qa_uq =
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qa[0], qa[1], qa[2], qa[3]));
        let qb_uq =
            UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qb[0], qb[1], qb[2], qb[3]));
        subquat(&qa_uq, &qb_uq)
    }

    /// Compute FD Jacobian of subquat w.r.t. qa (tangent-space perturbation).
    fn fd_d_dqa(qa: &[f64; 4], qb: &[f64; 4]) -> Matrix3<f64> {
        let mut jac = Matrix3::zeros();
        for j in 0..3 {
            let mut delta_p = Vector3::zeros();
            delta_p[j] = EPS;
            let mut delta_m = Vector3::zeros();
            delta_m[j] = -EPS;

            let qa_p = perturb_quat(qa, &delta_p);
            let qa_m = perturb_quat(qa, &delta_m);

            let res_p = eval_subquat(&qa_p, qb);
            let res_m = eval_subquat(&qa_m, qb);

            let col = (res_p - res_m) / (2.0 * EPS);
            jac.set_column(j, &col);
        }
        jac
    }

    /// Compute FD Jacobian of subquat w.r.t. qb (tangent-space perturbation).
    fn fd_d_dqb(qa: &[f64; 4], qb: &[f64; 4]) -> Matrix3<f64> {
        let mut jac = Matrix3::zeros();
        for j in 0..3 {
            let mut delta_p = Vector3::zeros();
            delta_p[j] = EPS;
            let mut delta_m = Vector3::zeros();
            delta_m[j] = -EPS;

            let qb_p = perturb_quat(qb, &delta_p);
            let qb_m = perturb_quat(qb, &delta_m);

            let res_p = eval_subquat(qa, &qb_p);
            let res_m = eval_subquat(qa, &qb_m);

            let col = (res_p - res_m) / (2.0 * EPS);
            jac.set_column(j, &col);
        }
        jac
    }

    fn assert_mat_close(name: &str, anal: &Matrix3<f64>, fd: &Matrix3<f64>, tol: f64) {
        for i in 0..3 {
            for j in 0..3 {
                let diff = (anal[(i, j)] - fd[(i, j)]).abs();
                let scale = anal[(i, j)].abs().max(fd[(i, j)].abs()).max(1e-10);
                assert!(
                    diff / scale < tol,
                    "{} [{},{}]: analytical={:.10e}, fd={:.10e}, rel_diff={:.3e}",
                    name,
                    i,
                    j,
                    anal[(i, j)],
                    fd[(i, j)],
                    diff / scale,
                );
            }
        }
    }

    #[test]
    fn dt52_identity_case() {
        // qa == qb → res = 0, d/dqa = I, d/dqb = -I
        let q = [1.0, 0.0, 0.0, 0.0];
        let (d_dqa, d_dqb) = mjd_sub_quat(&q, &q);

        assert_mat_close("d_dqa (identity)", &d_dqa, &Matrix3::identity(), 1e-10);
        assert_mat_close("d_dqb (identity)", &d_dqb, &-Matrix3::identity(), 1e-10);
    }

    #[test]
    fn dt52_small_angle() {
        // Small rotation: qa ≈ qb + small perturbation
        let qa = [1.0, 0.0, 0.0, 0.0];
        // Small rotation about z-axis
        let angle: f64 = 0.01;
        let qb = [(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()];
        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (small angle)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (small angle)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_moderate_rotation() {
        // 90-degree rotation about x-axis
        let angle = std::f64::consts::FRAC_PI_2;
        let qa = [(angle / 2.0).cos(), (angle / 2.0).sin(), 0.0, 0.0];
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (90 deg)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (90 deg)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_arbitrary_quaternion_pair() {
        // Two arbitrary quaternions
        let qa_raw: [f64; 4] = [0.5, 0.5, 0.5, 0.5]; // 120° about [1,1,1]
        let n = (qa_raw[0] * qa_raw[0]
            + qa_raw[1] * qa_raw[1]
            + qa_raw[2] * qa_raw[2]
            + qa_raw[3] * qa_raw[3])
            .sqrt();
        let qa = [qa_raw[0] / n, qa_raw[1] / n, qa_raw[2] / n, qa_raw[3] / n];

        let angle_b: f64 = 1.2;
        let axis_b = Vector3::new(0.3, -0.7, 0.6).normalize();
        let half: f64 = angle_b / 2.0;
        let qb = [
            half.cos(),
            axis_b.x * half.sin(),
            axis_b.y * half.sin(),
            axis_b.z * half.sin(),
        ];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (arbitrary)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (arbitrary)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_near_pi_rotation() {
        // Near-pi rotation (large angle, tests numerical stability)
        let angle: f64 = 3.0; // 3 radians ≈ 172 degrees
        let half: f64 = angle / 2.0;
        let qa = [half.cos(), 0.0, half.sin(), 0.0];
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (near pi)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (near pi)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_both_rotated() {
        // Both quaternions non-identity, with non-trivial difference
        let angle_a: f64 = 0.8;
        let axis_a = Vector3::new(1.0, 0.0, 0.0);
        let ha: f64 = angle_a / 2.0;
        let qa = [
            ha.cos(),
            ha.sin() * axis_a.x,
            ha.sin() * axis_a.y,
            ha.sin() * axis_a.z,
        ];

        let angle_b: f64 = 1.5;
        let axis_b = Vector3::new(0.0, 0.0, 1.0);
        let hb: f64 = angle_b / 2.0;
        let qb = [
            hb.cos(),
            hb.sin() * axis_b.x,
            hb.sin() * axis_b.y,
            hb.sin() * axis_b.z,
        ];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);
        let fd_dqa = fd_d_dqa(&qa, &qb);
        let fd_dqb = fd_d_dqb(&qa, &qb);

        assert_mat_close("d_dqa (both rotated)", &d_dqa, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (both rotated)", &d_dqb, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_negative_w_quaternion() {
        // Antipodal representative: qa with w < 0. Both [w,x,y,z] and [-w,-x,-y,-z]
        // represent the same rotation. Verify mjd_sub_quat produces identical Jacobians.
        let angle_a: f64 = 1.2;
        let ha: f64 = angle_a / 2.0;
        let qa_pos = [ha.cos(), ha.sin(), 0.0, 0.0];
        let qa_neg = [-ha.cos(), -ha.sin(), 0.0, 0.0]; // antipodal
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa_pos, d_dqb_pos) = mjd_sub_quat(&qa_pos, &qb);
        let (d_dqa_neg, d_dqb_neg) = mjd_sub_quat(&qa_neg, &qb);

        assert_mat_close("d_dqa (neg-w)", &d_dqa_pos, &d_dqa_neg, 1e-10);
        assert_mat_close("d_dqb (neg-w)", &d_dqb_pos, &d_dqb_neg, 1e-10);

        // Also verify FD validation works with negative-w
        let fd_dqa = fd_d_dqa(&qa_neg, &qb);
        let fd_dqb = fd_d_dqb(&qa_neg, &qb);
        assert_mat_close("d_dqa (neg-w vs FD)", &d_dqa_neg, &fd_dqa, FD_TOL);
        assert_mat_close("d_dqb (neg-w vs FD)", &d_dqb_neg, &fd_dqb, FD_TOL);
    }

    #[test]
    fn dt52_antipodal_pair() {
        // qa == -qb: both represent the same rotation, difference should be zero.
        // dq = conj(-qb) * qa = -(conj(qb) * qa). If qa == qb, dq is antipodal
        // identity [-1, 0, 0, 0], which has sin_half ≈ 0 → early return (I, -I).
        let angle: f64 = 0.8;
        let half: f64 = angle / 2.0;
        let qa = [half.cos(), half.sin(), 0.0, 0.0];
        let qb = [-half.cos(), -half.sin(), 0.0, 0.0]; // -qa

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        // Difference is zero (identity), so Jacobians should be (I, -I)
        assert_mat_close("d_dqa (antipodal)", &d_dqa, &Matrix3::identity(), 1e-10);
        assert_mat_close("d_dqb (antipodal)", &d_dqb, &-Matrix3::identity(), 1e-10);
    }

    #[test]
    fn dt52_exactly_pi_rotation() {
        // θ = π exactly. The axis-angle representation has a topological
        // singularity at θ = π (the wrapping ±π creates a discontinuity),
        // so FD is invalid here. This test verifies:
        // 1. No NaN/Inf from the 0/0 form in J_r^{-1} coefficient
        // 2. The coefficient evaluates to the correct limit (1/π²)
        let qa = [0.0, 1.0, 0.0, 0.0]; // 180° about x: [cos(π/2), sin(π/2), 0, 0]
        let qb = [1.0, 0.0, 0.0, 0.0];

        let (d_dqa, d_dqb) = mjd_sub_quat(&qa, &qb);

        // Verify no NaN/Inf
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    d_dqa[(i, j)].is_finite(),
                    "d_dqa[{},{}] is not finite: {}",
                    i,
                    j,
                    d_dqa[(i, j)]
                );
                assert!(
                    d_dqb[(i, j)].is_finite(),
                    "d_dqb[{},{}] is not finite: {}",
                    i,
                    j,
                    d_dqb[(i, j)]
                );
            }
        }

        // At θ = π about x-axis: v = [π, 0, 0].
        // J_r^{-1} coeff = 1/π² - (1+cos(π))/(2π·sin(π)) = 1/π² (IEEE 754: 0/0 → 0).
        // [v]×² for v = [π,0,0] is diag(0, -π², -π²), so coeff·[v]×² adds
        // diag(0, -1, -1) — meaning the yy and zz entries of J_r^{-1} are 0
        // (identity cancels with -1 from coeff·[v]×²).
        // J_r^{-1} should be: diag(1, 0, 0) + off-diag from skew terms.
        // Verify d_dqa[0,0] = 1 (the axis direction is preserved).
        assert!(
            (d_dqa[(0, 0)] - 1.0).abs() < 1e-10,
            "d_dqa[0,0] at θ=π should be 1.0, got {}",
            d_dqa[(0, 0)]
        );
    }

    #[test]
    fn dt52_negative_w_on_qb() {
        // Negative w on qb (the second argument). Verifies symmetry of
        // antipodal invariance for both inputs.
        let qa = [1.0, 0.0, 0.0, 0.0];
        let angle_b: f64 = 2.0;
        let hb: f64 = angle_b / 2.0;
        let qb_pos = [hb.cos(), 0.0, hb.sin(), 0.0];
        let qb_neg = [-hb.cos(), 0.0, -hb.sin(), 0.0];

        let (d_dqa_pos, d_dqb_pos) = mjd_sub_quat(&qa, &qb_pos);
        let (d_dqa_neg, d_dqb_neg) = mjd_sub_quat(&qa, &qb_neg);

        assert_mat_close("d_dqa (neg-w qb)", &d_dqa_pos, &d_dqa_neg, 1e-10);
        assert_mat_close("d_dqb (neg-w qb)", &d_dqb_pos, &d_dqb_neg, 1e-10);
    }
}

// ============================================================================
// DT-54 Unit Tests — Muscle velocity derivative curve FD validation
// ============================================================================

#[cfg(test)]
#[allow(clippy::similar_names, clippy::uninlined_format_args)]
mod muscle_vel_deriv_tests {
    use super::*;
    use crate::forward::{hill_force_velocity, muscle_gain_velocity};

    const EPS: f64 = 1e-7;
    const FD_TOL: f64 = 1e-5;

    // ── MuJoCo piecewise-quadratic FV curve derivative ──

    #[test]
    fn dt54_mujoco_fv_deriv_concentric() {
        // V_norm in [-1, 0]: FV = (v+1)^2, dFV/dv = 2(v+1)
        let fvmax = 1.2;
        for &v in &[-0.9, -0.5, -0.1] {
            let d_anal = muscle_gain_velocity_deriv(v, fvmax);
            let fv_p = muscle_gain_velocity(v + EPS, fvmax);
            let fv_m = muscle_gain_velocity(v - EPS, fvmax);
            let d_fd = (fv_p - fv_m) / (2.0 * EPS);
            assert!(
                (d_anal - d_fd).abs() < FD_TOL,
                "MuJoCo FV deriv at v={}: anal={:.8e}, fd={:.8e}, diff={:.3e}",
                v,
                d_anal,
                d_fd,
                (d_anal - d_fd).abs()
            );
        }
    }

    #[test]
    fn dt54_mujoco_fv_deriv_eccentric() {
        // V_norm in [0, fvmax-1]: FV = fvmax - (y-v)^2/y, dFV/dv = 2(y-v)/y
        let fvmax = 1.2;
        for &v in &[0.01, 0.05, 0.1] {
            let d_anal = muscle_gain_velocity_deriv(v, fvmax);
            let fv_p = muscle_gain_velocity(v + EPS, fvmax);
            let fv_m = muscle_gain_velocity(v - EPS, fvmax);
            let d_fd = (fv_p - fv_m) / (2.0 * EPS);
            assert!(
                (d_anal - d_fd).abs() < FD_TOL,
                "MuJoCo FV deriv (ecc) at v={}: anal={:.8e}, fd={:.8e}, diff={:.3e}",
                v,
                d_anal,
                d_fd,
                (d_anal - d_fd).abs()
            );
        }
    }

    #[test]
    fn dt54_mujoco_fv_deriv_plateau() {
        // V_norm > fvmax-1: FV = fvmax, flat
        let fvmax = 1.2;
        let v = 0.3; // > 0.2 = fvmax-1
        let d_anal = muscle_gain_velocity_deriv(v, fvmax);
        assert!(
            d_anal.abs() < 1e-14,
            "MuJoCo FV deriv (plateau) should be 0, got {}",
            d_anal
        );
    }

    #[test]
    fn dt54_mujoco_fv_deriv_max_shortening() {
        // V_norm <= -1: FV = 0, flat
        let fvmax = 1.2;
        let d_anal = muscle_gain_velocity_deriv(-1.5, fvmax);
        assert!(
            d_anal.abs() < 1e-14,
            "MuJoCo FV deriv (max shortening) should be 0, got {}",
            d_anal
        );
    }

    // ── Hill-type FV curve derivative ──

    #[test]
    fn dt54_hill_fv_deriv_concentric() {
        // V_norm in [-1, 0]: Hill hyperbola
        for &v in &[-0.9, -0.5, -0.1] {
            let d_anal = hill_force_velocity_deriv(v);
            let fv_p = hill_force_velocity(v + EPS);
            let fv_m = hill_force_velocity(v - EPS);
            let d_fd = (fv_p - fv_m) / (2.0 * EPS);
            assert!(
                (d_anal - d_fd).abs() < FD_TOL,
                "Hill FV deriv (conc) at v={}: anal={:.8e}, fd={:.8e}, diff={:.3e}",
                v,
                d_anal,
                d_fd,
                (d_anal - d_fd).abs()
            );
        }
    }

    #[test]
    fn dt54_hill_fv_deriv_eccentric() {
        // V_norm > 0: eccentric plateau approach
        for &v in &[0.01, 0.1, 0.5, 2.0] {
            let d_anal = hill_force_velocity_deriv(v);
            let fv_p = hill_force_velocity(v + EPS);
            let fv_m = hill_force_velocity(v - EPS);
            let d_fd = (fv_p - fv_m) / (2.0 * EPS);
            assert!(
                (d_anal - d_fd).abs() < FD_TOL,
                "Hill FV deriv (ecc) at v={}: anal={:.8e}, fd={:.8e}, diff={:.3e}",
                v,
                d_anal,
                d_fd,
                (d_anal - d_fd).abs()
            );
        }
    }

    #[test]
    fn dt54_hill_fv_deriv_isometric() {
        // V_norm = 0 is a slope discontinuity: concentric side gives dFV/dv = 5.0,
        // eccentric side gives dFV/dv = 2.0. The analytical function uses the
        // concentric formula (v <= 0 branch), matching MuJoCo's behavior.
        let d_anal = hill_force_velocity_deriv(0.0);
        let a: f64 = 0.25;

        // Concentric side derivative at v=0: (1+a)/(a·1²) = 5.0
        let expected_conc = (1.0 + a) / a;
        assert!(
            (d_anal - expected_conc).abs() < 1e-10,
            "Hill FV deriv at v=0 (concentric): expected {}, got {}",
            expected_conc,
            d_anal
        );

        // Verify one-sided FD matches on each side
        let v_conc = -1e-4; // slightly concentric
        let d_conc = hill_force_velocity_deriv(v_conc);
        let fv_p = hill_force_velocity(v_conc + EPS);
        let fv_m = hill_force_velocity(v_conc - EPS);
        let d_fd_conc = (fv_p - fv_m) / (2.0 * EPS);
        assert!(
            (d_conc - d_fd_conc).abs() < FD_TOL,
            "Hill FV deriv near v=0 (concentric): anal={:.8e}, fd={:.8e}",
            d_conc,
            d_fd_conc
        );

        let v_ecc = 1e-4; // slightly eccentric
        let d_ecc = hill_force_velocity_deriv(v_ecc);
        let fv_p = hill_force_velocity(v_ecc + EPS);
        let fv_m = hill_force_velocity(v_ecc - EPS);
        let d_fd_ecc = (fv_p - fv_m) / (2.0 * EPS);
        assert!(
            (d_ecc - d_fd_ecc).abs() < FD_TOL,
            "Hill FV deriv near v=0 (eccentric): anal={:.8e}, fd={:.8e}",
            d_ecc,
            d_fd_ecc
        );
    }

    #[test]
    fn dt54_mujoco_fv_deriv_at_v_zero() {
        // v=0 is a branch boundary (concentric → eccentric). FV is C1 here:
        // left: dFV/dv = 2(0+1) = 2, right: 2(y-0)/y = 2. Both agree.
        let fvmax = 1.2;
        let d_anal = muscle_gain_velocity_deriv(0.0, fvmax);
        assert!(
            (d_anal - 2.0).abs() < 1e-10,
            "MuJoCo FV deriv at v=0: expected 2.0, got {}",
            d_anal
        );
        // Verify via centered FD straddling the boundary
        let fv_p = muscle_gain_velocity(EPS, fvmax);
        let fv_m = muscle_gain_velocity(-EPS, fvmax);
        let d_fd = (fv_p - fv_m) / (2.0 * EPS);
        assert!(
            (d_anal - d_fd).abs() < FD_TOL,
            "MuJoCo FV deriv at v=0: anal={:.8e}, fd={:.8e}",
            d_anal,
            d_fd
        );
    }

    #[test]
    fn dt54_mujoco_fv_deriv_at_v_minus_one() {
        // v=-1 is a branch boundary (flat zero → concentric parabola).
        // One-sided derivative from right: 2(-1+1) = 0. Function is C1 here.
        let fvmax = 1.2;
        let d_anal = muscle_gain_velocity_deriv(-1.0, fvmax);
        assert!(
            d_anal.abs() < 1e-14,
            "MuJoCo FV deriv at v=-1: expected 0, got {}",
            d_anal
        );
    }

    #[test]
    fn dt54_mujoco_fv_deriv_fvmax_one() {
        // fvmax = 1.0 → y = 0. Eccentric region has zero width.
        // At v=0: concentric branch gives dFV/dv = 2(0+1) = 2.
        // At v>0: falls through to plateau (0).
        let fvmax: f64 = 1.0;
        assert!(
            (muscle_gain_velocity_deriv(0.0, fvmax) - 2.0).abs() < 1e-10,
            "fvmax=1: deriv at v=0 should be 2"
        );
        assert!(
            muscle_gain_velocity_deriv(0.01, fvmax).abs() < 1e-10,
            "fvmax=1: deriv at v=0.01 should be 0 (plateau)"
        );
        assert!(
            (muscle_gain_velocity_deriv(-0.5, fvmax) - 1.0).abs() < 1e-10,
            "fvmax=1: deriv at v=-0.5 should be 2*(-0.5+1)=1.0"
        );
    }

    #[test]
    fn dt54_hill_fv_deriv_at_v_minus_one() {
        // v=-1 boundary: FV = 0 (left), FV → 0 (right). Derivative jumps from 0
        // to 0.2 (C0-not-C1 discontinuity inherent in Hill model).
        // Code returns 0 for v <= -1, matching MuJoCo convention.
        let d_anal = hill_force_velocity_deriv(-1.0);
        assert!(
            d_anal.abs() < 1e-14,
            "Hill FV deriv at v=-1: expected 0, got {}",
            d_anal
        );
        // Just above -1: derivative should be ≈ a/(a+1) = 0.2
        let d_above = hill_force_velocity_deriv(-1.0 + 1e-6);
        let expected: f64 = 0.25 / 1.25; // a/(a+1) = 0.2
        assert!(
            (d_above - expected).abs() < 1e-3,
            "Hill FV deriv just above v=-1: expected ≈ {}, got {}",
            expected,
            d_above
        );
    }
}

// ============================================================================
// DT-53 tests: forward_skip regression
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

// ============================================================================
// DT-51 tests: mjd_inverse_fd
// ============================================================================

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
    use crate::types::Model;

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
