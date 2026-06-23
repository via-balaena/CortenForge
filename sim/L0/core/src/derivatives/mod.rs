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
//! [`mj_integrate_pos_explicit`](crate::jacobian::mj_integrate_pos_explicit) and
//! [`mj_differentiate_pos`](crate::jacobian::mj_differentiate_pos). This avoids quaternion
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

mod fd;
mod hybrid;
mod integration;
mod param;

#[cfg(test)]
mod transition_matrix_harness;

// Re-export public items so they remain accessible at `crate::derivatives::*`
pub use fd::{InverseDynamicsDerivatives, mjd_inverse_fd, mjd_transition_fd};
pub use hybrid::{
    mass_directional_derivative, mjd_passive_vel, mjd_smooth_pos, mjd_smooth_vel,
    mjd_transition_hybrid,
};
pub use integration::{mjd_quat_integrate, mjd_sub_quat};
pub use param::{
    DampingJacobian, MassJacobian, TrajectoryDampingJacobian, TrajectoryMassJacobian,
    mjd_damping_jacobian, mjd_damping_trajectory_jacobian, mjd_mass_jacobian,
    mjd_mass_trajectory_jacobian,
};

// Crate-internal re-exports
pub(crate) use hybrid::{mjd_actuator_vel, mjd_rne_vel};

use crate::types::{BiasType, Data, GainType, Integrator, Model, StepError};
use nalgebra::DMatrix;

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
    // MillardMuscle's analytic actuator Jacobian (the Bézier FV-curve derivative +
    // the β·v̄ damping derivative) is not yet implemented, so a model containing one
    // takes the full finite-difference path for an EXACT transition matrix rather than
    // an incomplete analytic one (R-implicit-deriv). The forward simulation is
    // unaffected; only this derivative output is routed.
    let has_millard = model
        .actuator_gaintype
        .iter()
        .any(|g| matches!(g, GainType::MillardMuscle))
        || model
            .actuator_biastype
            .iter()
            .any(|b| matches!(b, BiasType::MillardMuscle));
    let can_analytical = config.use_analytical
        && !matches!(model.integrator, Integrator::RungeKutta4)
        && !has_millard;
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
