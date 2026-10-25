//! Finite-difference perturbation methods for transition derivatives.

use super::{DerivativeConfig, TransitionMatrices};
use crate::forward::MjStage;
use crate::jacobian::{mj_differentiate_pos, mj_integrate_pos_explicit};
use crate::types::{Data, ENABLE_SLEEP, Model, StepError};
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
/// - Centered: `1 + 2 · (2·nv + na + nu)` calls to `step()` (the `1`, the
///   nominal step, runs either way).
/// - Forward:  `1 + (2·nv + na + nu)` calls to `step()` (the `1` is for
///   the nominal `y_0 = f(x)` evaluation).
/// - With sleep enabled, each perturbed step also starts from a copy of the
///   whole `Data`.
///
/// # Quaternion handling
///
/// Position perturbations operate in tangent space (dimension `nv`) via
/// `mj_integrate_pos_explicit()`. The perturbed next states are differenced
/// with each other, as MuJoCo's `mjd_stepFD` does (its `stateDiff`), so a
/// quaternion joint's position rows are tangents at the next state.
///
/// # Contact handling
///
/// FD naturally captures contact transitions. If a perturbation causes a
/// contact to activate/deactivate, the derivative reflects this discontinuity.
///
/// # Errors
///
/// Before any work: the step inputs `step` checks
/// ([`StepError::InvalidTimestep`], [`StepError::DataShapeMismatch`]), then
/// [`StepError::UnsupportedIntegrator`] for RK4 and
/// [`StepError::UnsupportedHistory`] for a model with history buffers, as
/// MuJoCo's `mjd_transitionFD` refuses them. Then a `StepError` from any
/// perturbed `step()`.
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
// Mathematical symbols (A, B, C, D, J, M) follow MuJoCo's analytical derivatives notation; paired identifiers (q/qpos, v/qvel) are intentionally similar; the unwrap is a defensive guard on a length-known-at-construction slice.
#[allow(non_snake_case, clippy::similar_names, clippy::unwrap_used)]
pub fn mjd_transition_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<TransitionMatrices, StepError> {
    super::check_fd_transition_inputs(model, data)?;
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
    let ns = model.nsensordata;
    let compute_sensors = config.compute_sensor_derivatives && ns > 0;

    // Phase 0 — Save nominal state and clone scratch.
    let mut scratch = data.clone();
    // Compute nominal next state by stepping unperturbed.
    // MuJoCo always computes this unconditionally. We need it for:
    // - forward differencing (non-centered A/B columns)
    // - clamped control differencing fallback (centered mode where one
    //   direction is infeasible due to actuator_ctrlrange boundary)
    // step() computes the sensors in its forward pass, before integrating:
    // sensordata is then at the step's (here unperturbed) current state, the
    // state C and D differentiate, as MuJoCo's mjd_stepFD reads it.
    scratch.step(model)?;
    let y_0 = NextState::of(&scratch);
    let factor = NominalFactor::of(&scratch);
    let sensor_0 = if compute_sensors {
        Some(scratch.sensordata.clone())
    } else {
        None
    };

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

    // Phase 1 — State perturbation (A matrix + C sensor-state columns). An
    // activation column steps with the nominal factor.
    let step = |scratch: &mut Data, i: usize| {
        if i < 2 * nv {
            scratch.step(model)
        } else {
            factor.step(model, scratch)
        }
    };
    for i in 0..nx {
        // Apply +eps perturbation
        perturb_state(model, &mut scratch, data, i, eps);
        step(&mut scratch, i)?;
        let y_plus = NextState::of(&scratch);
        let s_plus = if compute_sensors {
            Some(scratch.sensordata.clone())
        } else {
            None
        };

        if config.centered {
            // Apply -eps perturbation
            perturb_state(model, &mut scratch, data, i, -eps);
            step(&mut scratch, i)?;
            let y_minus = NextState::of(&scratch);
            let s_minus = if compute_sensors {
                Some(scratch.sensordata.clone())
            } else {
                None
            };

            // Centred: (y+ − y−) / (2·eps)
            A.column_mut(i)
                .copy_from(&state_diff(model, &y_minus, &y_plus, 2.0 * eps));
            if let (Some(c_mat), Some(sp), Some(sm)) = (&mut C, &s_plus, &s_minus) {
                c_mat.column_mut(i).copy_from(&diff(sm, sp, 2.0 * eps));
            }
        } else {
            // Forward: (y+ − y0) / eps
            A.column_mut(i)
                .copy_from(&state_diff(model, &y_0, &y_plus, eps));
            if let (Some(c_mat), Some(sp), Some(s0)) = (&mut C, &s_plus, &sensor_0) {
                c_mat.column_mut(i).copy_from(&diff(s0, sp, eps));
            }
        }
    }

    // Phase 2 — Control perturbation (B matrix + D sensor-control columns),
    // each step with the nominal factor.
    // Matches MuJoCo's mjd_stepFD control clamping: only nudge within
    // actuator_ctrlrange. Uses forward-only, backward-only, or centered
    // differencing based on which nudges are feasible.
    for j in 0..nu {
        let range = model.actuator_ctrlrange[j];
        let nudge_fwd = in_ctrl_range(data.ctrl[j], data.ctrl[j] + eps, range);
        let nudge_back = (config.centered || !nudge_fwd)
            && in_ctrl_range(data.ctrl[j] - eps, data.ctrl[j], range);

        let (y_plus, s_plus) = if nudge_fwd {
            perturb_ctrl(model, &mut scratch, data, j, eps);
            factor.step(model, &mut scratch)?;
            let yp = NextState::of(&scratch);
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
            perturb_ctrl(model, &mut scratch, data, j, -eps);
            factor.step(model, &mut scratch)?;
            let ym = NextState::of(&scratch);
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
        B.column_mut(j).copy_from(&clamped_state_diff(
            model,
            &y_0,
            y_plus.as_ref(),
            y_minus.as_ref(),
            eps,
        ));

        // Sensor D column (the same clamping; centred, the difference MuJoCo
        // takes reversed, registry D-FD-CENTERED-D-SIGN)
        if let (Some(d_mat), Some(s0)) = (&mut D, &sensor_0) {
            let scol = match (&s_plus, &s_minus) {
                (Some(sp), Some(sm)) => diff(sm, sp, 2.0 * eps),
                (Some(sp), None) => diff(s0, sp, eps),
                (None, Some(sm)) => diff(sm, s0, eps),
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

/// The implicit integrators' factor the nominal step left (`qH`,
/// `qHDiagInv`, `qLU`). MuJoCo's transition steps its control and activation
/// columns from the velocity stage (`mj_stepSkip(m, d, mjSTAGE_VEL, …)`,
/// `engine_derivative_fd.c:350`, `:365`, `:393`, `:405`), whose
/// `mj_implicitSkip` solves with the factor in `d` (`:140`): the nominal
/// step's, since those columns come before every column that factors. So a
/// velocity derivative that moves with the control is not in MuJoCo's B,
/// nor one that moves with the activation in its A.
pub(super) struct NominalFactor {
    qh: Vec<f64>,
    qh_diag_inv: Vec<f64>,
    qlu: Vec<f64>,
}

impl NominalFactor {
    pub(super) fn of(data: &Data) -> Self {
        Self {
            qh: data.qH.clone(),
            qh_diag_inv: data.qHDiagInv.clone(),
            qlu: data.qLU.clone(),
        }
    }

    /// Steps `scratch` with this factor.
    pub(super) fn step(&self, model: &Model, scratch: &mut Data) -> Result<(), StepError> {
        scratch.qH.copy_from_slice(&self.qh);
        scratch.qHDiagInv.copy_from_slice(&self.qh_diag_inv);
        scratch.qLU.copy_from_slice(&self.qlu);
        scratch.step_skip_factor(model, true)
    }
}

/// Puts `scratch` back at the caller's state `nominal` before a
/// finite-difference step.
/// MuJoCo's `mjd_stepFD` restores `mjSTATE_FULLPHYSICS | mjSTATE_CTRL` and the
/// warm start (`engine_derivative_fd.c:307`); with sleep disabled this
/// restores qpos, qvel, act, ctrl, the warm start, the time and the plugin
/// state (`transition_derivatives_take_each_column_from_the_plugin_state`).
/// With sleep enabled a step also reads what a sleeping tree keeps from the
/// step before it, its stored pose among them (a pose that differs wakes the
/// tree, `forward/position.rs`), so the whole state is restored: from less,
/// one column's wake reaches the next (registry `D-FD-SLEEP`;
/// `transition_derivatives_take_each_column_from_the_sleep_state`).
fn restore(model: &Model, scratch: &mut Data, nominal: &Data) {
    if model.enableflags & ENABLE_SLEEP != 0 {
        scratch.clone_from(nominal);
    } else {
        scratch.qpos.copy_from(&nominal.qpos);
        scratch.qvel.copy_from(&nominal.qvel);
        scratch.act.copy_from(&nominal.act);
        scratch.ctrl.copy_from(&nominal.ctrl);
        scratch.qacc_warmstart.copy_from(&nominal.qacc_warmstart);
        scratch.time = nominal.time;
        scratch.plugin_state.copy_from_slice(&nominal.plugin_state);
    }
}

/// Puts `scratch` at `nominal` with state coordinate `i` moved by `delta`: a
/// position (`i < nv`) along its tangent through `mj_integrate_pos_explicit`,
/// a velocity (`nv <= i < 2*nv`) or an activation (`2*nv <= i`) by addition.
pub(super) fn perturb_state(
    model: &Model,
    scratch: &mut Data,
    nominal: &Data,
    i: usize,
    delta: f64,
) {
    restore(model, scratch, nominal);
    let nv = model.nv;
    if i < nv {
        let mut dq = DVector::zeros(nv);
        dq[i] = delta;
        mj_integrate_pos_explicit(model, &mut scratch.qpos, &nominal.qpos, &dq, 1.0);
    } else if i < 2 * nv {
        scratch.qvel[i - nv] += delta;
    } else {
        scratch.act[i - 2 * nv] += delta;
    }
}

/// Puts `scratch` at `nominal` with control `j` moved by `delta`.
pub(super) fn perturb_ctrl(
    model: &Model,
    scratch: &mut Data,
    nominal: &Data,
    j: usize,
    delta: f64,
) {
    restore(model, scratch, nominal);
    scratch.ctrl[j] += delta;
}

/// Check if both values are within the given range.
/// Matches MuJoCo's `inRange()` in `engine_derivative_fd.c`.
pub(super) fn in_ctrl_range(x1: f64, x2: f64, range: (f64, f64)) -> bool {
    x1 >= range.0 && x1 <= range.1 && x2 >= range.0 && x2 <= range.1
}

/// A step's output whole, as MuJoCo's `mjd_stepFD` reads it
/// (`getState(mjSTATE_PHYSICS)`, `engine_derivative_fd.c:37-42`).
pub(super) struct NextState {
    qpos: DVector<f64>,
    qvel: DVector<f64>,
    act: DVector<f64>,
}

impl NextState {
    pub(super) fn of(data: &Data) -> Self {
        Self {
            qpos: data.qpos.clone(),
            qvel: data.qvel.clone(),
            act: data.act.clone(),
        }
    }
}

/// `(x2 − x1)/h` as MuJoCo's `diff` computes it, `(1/h)·(x2 − x1)`
/// (`engine_derivative_fd.c:46-51`).
pub(super) fn diff(x1: &DVector<f64>, x2: &DVector<f64>, h: f64) -> DVector<f64> {
    let inv_h = 1.0 / h;
    x2.zip_map(x1, |b, a| inv_h * (b - a))
}

/// `(s2 − s1)/h` in the state's tangent space, as MuJoCo's `stateDiff`
/// (`engine_derivative_fd.c:55-64`): with a quaternion joint (`nq ≠ nv`) the
/// positions through [`mj_differentiate_pos`], each a tangent at `s1`'s
/// position, the rest by [`diff`]; without one, all by [`diff`].
pub(super) fn state_diff(model: &Model, s1: &NextState, s2: &NextState, h: f64) -> DVector<f64> {
    let (nv, na) = (model.nv, model.na);
    let mut ds = DVector::zeros(2 * nv + na);
    if model.nq == nv {
        ds.rows_mut(0, nv).copy_from(&diff(&s1.qpos, &s2.qpos, h));
    } else {
        let mut dq = DVector::zeros(nv);
        mj_differentiate_pos(model, &mut dq, &s1.qpos, &s2.qpos, h);
        ds.rows_mut(0, nv).copy_from(&dq);
    }
    ds.rows_mut(nv, nv).copy_from(&diff(&s1.qvel, &s2.qvel, h));
    if na > 0 {
        ds.rows_mut(2 * nv, na)
            .copy_from(&diff(&s1.act, &s2.act, h));
    }
    ds
}

/// A control column's state difference, as MuJoCo's `clampedStateDiff`
/// (`engine_derivative_fd.c:87-102`): forward from the nominal output `s`, or
/// backward to it, where only one nudge stays in the control range;
/// centred where both do; zeros where neither does.
pub(super) fn clamped_state_diff(
    model: &Model,
    s: &NextState,
    plus: Option<&NextState>,
    minus: Option<&NextState>,
    h: f64,
) -> DVector<f64> {
    match (plus, minus) {
        (Some(p), None) => state_diff(model, s, p, h),
        (None, Some(m)) => state_diff(model, m, s, h),
        (Some(p), Some(m)) => state_diff(model, m, p, 2.0 * h),
        (None, None) => DVector::zeros(2 * model.nv + model.na),
    }
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
/// velocity, and acceleration. `DfDa` is the mass matrix `M` less the
/// derivative of `qfrc_constraint`, which `inverse()` remakes from `qacc`
/// (`qfrc_inverse = M*qacc + bias - passive - constraint`): a row in its
/// quadratic zone adds its `J'·D·J`.
///
/// # MuJoCo Equivalence
///
/// MuJoCo's `mjd_inverseFD` outputs (`engine_derivative_fd.c`), without its
/// sensor Jacobians and `DmDq`, transposed: column `i` here is the derivative
/// with respect to input `i`, which MuJoCo stores as row `i`.
#[derive(Debug, Clone)]
#[allow(non_snake_case)]
pub struct InverseDynamicsDerivatives {
    /// `∂qfrc_inverse/∂qpos` (nv × nv).
    /// Position perturbations in tangent space.
    pub DfDq: DMatrix<f64>,
    /// `∂qfrc_inverse/∂qvel` (nv × nv).
    pub DfDv: DMatrix<f64>,
    /// `∂qfrc_inverse/∂qacc` (nv × nv): the mass matrix M less the
    /// constraint force's derivative (above).
    pub DfDa: DMatrix<f64>,
}

/// Compute finite-difference derivatives of inverse dynamics.
///
/// Perturbs `qacc`, `qvel` and `qpos` around the current state, as MuJoCo's
/// `mjd_inverseFD` (`engine_derivative_fd.c:608-710`), and measures the
/// change of `qfrc_inverse`. Produces three nv×nv Jacobian matrices.
///
/// # Algorithm
///
/// Each point is MuJoCo's inverse dynamics (`mj_inverseSkip`,
/// `engine_inverse.c:184-250`, as [`Data::inverse`] without its sensors)
/// from the stage its nudge changes, with the nominal `qacc` but where it is
/// nudged. The centre point runs every stage; the columns follow in
/// MuJoCo's order:
///
/// 1. **DfDa**: `qacc[i] ± ε`, the constraint force and the sum only.
/// 2. **DfDv**: `qvel[i] ± ε`, from the velocity stage on.
/// 3. **DfDq**: `qpos` moved by `± ε` in tangent direction `i`, every stage.
///
/// Forward differences (`centered: false`) take each column against the
/// centre point; MuJoCo's `mjd_inverseFD` takes forward differences only.
/// Sensors are skipped. No column fires `cb_control`, and none runs the
/// actuation or the constraint solve, as MuJoCo's `mj_inverseSkip`, so
/// `ctrl` is read as the caller left it.
///
/// Each point remakes the constraint force from its `qacc`, as MuJoCo's
/// `mj_inverseSkip` does (`mj_invConstraint`, `engine_inverse.c:223`).
///
/// # Cost
///
/// - Centered: `1 + 2·nv` position and velocity stages, `2·nv` velocity
///   stages, and `1 + 6·nv` constraint forces and sums.
/// - Forward: `1 + nv` position and velocity stages, `nv` velocity stages,
///   and `1 + 3·nv` constraint forces and sums.
///
/// # Panics
///
/// Panics if `config.eps` is non-positive, non-finite, or greater than `1e-2`.
///
/// # Errors
///
/// Before any work: the step inputs `step` checks
/// ([`StepError::InvalidTimestep`], [`StepError::DataShapeMismatch`]), then
/// [`StepError::UnsupportedIntegrator`] for RK4 and
/// [`StepError::UnsupportedNoslip`] for a model with noslip iterations, as
/// MuJoCo's `mjd_inverseFD` refuses them, then
/// [`StepError::UnsupportedInverseDiscrete`] where
/// [`Data::inverse`](crate::Data::inverse) refuses it. From the inverse at
/// each point, the errors `inverse` returns after collision
/// ([`StepError::UndefinedContactFrame`],
/// [`StepError::ContactWithSleepingGeom`]).
// Mathematical symbols (J, M, K, qfrc) follow MuJoCo's inverse-dynamics-derivatives notation.
#[allow(non_snake_case, clippy::similar_names)]
pub fn mjd_inverse_fd(
    model: &Model,
    data: &Data,
    config: &DerivativeConfig,
) -> Result<InverseDynamicsDerivatives, StepError> {
    super::check_fd_inverse_inputs(model, data)?;
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

    let qpos_0 = data.qpos.clone();
    let qvel_0 = data.qvel.clone();
    let qacc_0 = data.qacc.clone();
    let mut scratch = data.clone();

    // The force at the perturbed input; `stage` is the last stage the nudge
    // leaves as it was (None: a qpos nudge, Pos: a qvel nudge, Vel: qacc).
    let force = |scratch: &mut Data, stage: MjStage| -> Result<DVector<f64>, StepError> {
        scratch.inverse_skip_unchecked(model, stage, true)?;
        Ok(scratch.qfrc_inverse.clone())
    };

    // Centre point.
    let f_0 = force(&mut scratch, MjStage::None)?;

    // With sleep enabled every point starts from the centre's whole state,
    // its sleep state included: from less, a nudge that wakes a tree would
    // reach the later points (registry `D-INVERSE-SLEEP`).
    let centre = (model.enableflags & ENABLE_SLEEP != 0).then(|| scratch.clone());
    let fresh = |scratch: &mut Data| {
        if let Some(centre) = &centre {
            scratch.clone_from(centre);
        }
    };

    let mut DfDq = DMatrix::zeros(nv, nv);
    let mut DfDv = DMatrix::zeros(nv, nv);
    let mut DfDa = DMatrix::zeros(nv, nv);
    let difference = |plus: &DVector<f64>, minus: Option<&DVector<f64>>| match minus {
        Some(minus) => (plus - minus) / (2.0 * eps),
        None => (plus - &f_0) / eps,
    };

    // --- DfDa: perturb qacc, at the centre point ---
    for i in 0..nv {
        fresh(&mut scratch);
        scratch.qacc[i] = qacc_0[i] + eps;
        let f_plus = force(&mut scratch, MjStage::Vel)?;
        let f_minus = if config.centered {
            fresh(&mut scratch);
            scratch.qacc[i] = qacc_0[i] - eps;
            Some(force(&mut scratch, MjStage::Vel)?)
        } else {
            None
        };
        scratch.qacc[i] = qacc_0[i];
        DfDa.column_mut(i)
            .copy_from(&difference(&f_plus, f_minus.as_ref()));
    }

    // --- DfDv: perturb qvel; positions are the centre point's ---
    for i in 0..nv {
        fresh(&mut scratch);
        scratch.qvel[i] = qvel_0[i] + eps;
        let f_plus = force(&mut scratch, MjStage::Pos)?;
        let f_minus = if config.centered {
            fresh(&mut scratch);
            scratch.qvel[i] = qvel_0[i] - eps;
            Some(force(&mut scratch, MjStage::Pos)?)
        } else {
            None
        };
        scratch.qvel[i] = qvel_0[i];
        DfDv.column_mut(i)
            .copy_from(&difference(&f_plus, f_minus.as_ref()));
    }

    // --- DfDq: perturb qpos in tangent space ---
    let mut dq = DVector::zeros(nv);
    for i in 0..nv {
        dq[i] = eps;
        fresh(&mut scratch);
        mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
        let f_plus = force(&mut scratch, MjStage::None)?;
        let f_minus = if config.centered {
            dq[i] = -eps;
            fresh(&mut scratch);
            mj_integrate_pos_explicit(model, &mut scratch.qpos, &qpos_0, &dq, 1.0);
            Some(force(&mut scratch, MjStage::None)?)
        } else {
            None
        };
        dq[i] = 0.0;
        DfDq.column_mut(i)
            .copy_from(&difference(&f_plus, f_minus.as_ref()));
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
        data_skip.integrate(&model).expect("integrate");

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

    /// Verify that after a velocity nudge `forward_skip(Pos)` gives the full
    /// forward's bias force and acceleration, skipping the position stage.
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

        // Reference: use full forward() for velocity FD
        let mut d_full = data.clone();
        d_full.qvel[0] += eps;
        d_full.forward(&model).expect("ref fwd+ failed");

        // Test: use forward_skip(Pos) for velocity FD
        let mut d_skip = data.clone();
        // Pre-populate position data with full forward
        d_skip.forward(&model).expect("init fwd failed");
        // Now perturb velocity and use skip(Pos)
        d_skip.qvel[0] += eps;
        d_skip
            .forward_skip(&model, MjStage::Pos, true)
            .expect("skip fwd+ failed");

        // The velocity stage's bias force and the acceleration should match
        // (inverse() runs its own stages, so it cannot tell them apart).
        for (what, full, skip) in [
            ("qfrc_bias", &d_full.qfrc_bias, &d_skip.qfrc_bias),
            ("qacc", &d_full.qacc, &d_skip.qacc),
        ] {
            for i in 0..model.nv {
                assert!(
                    (full[i] - skip[i]).abs() < 1e-12,
                    "{}[{}] mismatch: full={:.16e}, skip={:.16e}",
                    what,
                    i,
                    full[i],
                    skip[i]
                );
            }
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
    use crate::validation::max_abs_or_nan;
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
        d_plus.inverse(&model).expect("inverse");
        d_minus.forward(&model).expect("fwd- failed");
        d_minus.qacc.copy_from(&qacc_0);
        d_minus.inverse(&model).expect("inverse");

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

        let max_abs = max_abs_or_nan(&derivs.DfDq);
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
        d_plus.inverse(&model).expect("inverse");
        d_minus.forward(&model).expect("fwd- failed");
        d_minus.qacc.copy_from(&qacc_0);
        d_minus.inverse(&model).expect("inverse");

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

    /// Verify mjd_inverse_fd's skip stages: position columns run the whole
    /// pipeline and velocity columns start at the velocity stage (skipping
    /// FK and collision). Test this indirectly by verifying the output
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
            scratch.inverse(&model).expect("inverse");
            let f_plus = scratch.qfrc_inverse.clone();

            // -eps
            scratch.qacc.copy_from(&qacc_0);
            scratch.qacc[j] -= eps;
            scratch.flg_rnepost = false;
            scratch.inverse(&model).expect("inverse");

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
