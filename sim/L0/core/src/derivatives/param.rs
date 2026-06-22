//! Parameter Jacobians of the simulation step (rigid *model* parameters).
//!
//! Where [`transition_derivatives`](crate::Data::transition_derivatives) gives
//! `A = ∂x⁺/∂x` (state) and `B = ∂x⁺/∂u` (control), this module provides the
//! third sibling: `∂x⁺/∂θ` for rigid **model** parameters `θ` — the gradient
//! channel that sim-to-real / system-ID calibration needs (mass, joint damping,
//! inertia, friction). The first parameter family implemented here is per-DOF
//! **joint damping**.
//!
//! # Math (single semi-implicit Euler step, eulerdamp)
//!
//! Joint damping `D` (a diagonal of per-DOF coefficients) enters the Euler step
//! through **two** paths — verified against the forward code, not assumed:
//!
//! - **Explicit** passive force: `qfrc_damper = −D·v` (`passive.rs`), which feeds
//!   the total force `rhs = qfrc_smooth + qfrc_constraint`.
//! - **Implicit** eulerdamp solve (`integrate`): `M_impl = M + h·D`, then
//!   `qacc_new = M_impl⁻¹·rhs` and `v⁺ = v + h·qacc_new`.
//!
//! Writing `rhs = G − D·v` with `G` damping-independent,
//! `v⁺ = v + h·M_impl⁻¹·(G − D·v)`. Differentiating w.r.t. the coefficient on
//! DOF `i` (which moves *both* the `M_impl` term and the `−D·v` term):
//!
//! ```text
//! ∂v⁺/∂D_i = −h · M_impl⁻¹ · E_ii · v⁺   ( = −h · v⁺_i · (M_impl⁻¹ column i) )
//! ```
//!
//! The two paths sum exactly: `−h²·qacc_new_i` (implicit) plus `−h·v_i`
//! (explicit) combine to `−h·v⁺_i` since `v⁺_i = v_i + h·qacc_new_i`. The driver
//! is the **post-step velocity `v⁺`**, not the eulerdamp acceleration `qacc` —
//! using `qacc` (the implicit path alone) is wrong by the explicit term.
//!
//! For semi-implicit Euler with hinge/slide joints the position is integrated as
//! `qpos⁺ = qpos + h·v⁺`, so the position-tangent row is simply `h · ∂v⁺/∂D`.

#![allow(non_snake_case)] // E_ii / M_impl follow the derivative-module notation.

use super::fd::extract_state;
use super::{DerivativeConfig, mjd_transition};
use crate::linalg::{cholesky_in_place, cholesky_solve_in_place};
use crate::types::{Integrator, MjJointType, StepError};
use crate::{Data, Model};
use nalgebra::{DMatrix, DVector};

/// Analytic single-step Jacobian of the next state w.r.t. per-DOF joint damping.
///
/// Column `j` of [`dxdD`](Self::dxdD) is `∂x⁺/∂D_j`, the sensitivity of the next
/// state to the joint-damping coefficient on DOF `j`. `D_j` is the *physical*
/// parameter: it drives both the explicit passive force `−D_j·v_j` and the
/// implicit eulerdamp matrix `M_impl`. For hinge/slide joints
/// `implicit_damping[dof] == jnt_damping[joint]` one-to-one, so this is also
/// `∂x⁺/∂jnt_damping[joint]`.
pub struct DampingJacobian {
    /// `∂x⁺/∂D`, dimensions `(2*nv + na) × nv`.
    ///
    /// Row layout matches [`TransitionMatrices`](super::TransitionMatrices):
    /// `[0..nv]` position tangent, `[nv..2*nv]` velocity, `[2*nv..2*nv+na]`
    /// activation (always zero — damping does not affect activation in one step).
    /// Column `j` is the derivative w.r.t. `implicit_damping[j]`.
    pub dxdD: DMatrix<f64>,
}

/// Hard-assert a model is within the supported scope of the analytic damping
/// channel (Euler integrator, hinge/slide joints, no tendons). Out-of-scope
/// models would yield a silently-wrong gradient, so this panics in all profiles.
fn assert_damping_scope(model: &Model) {
    assert_eq!(
        model.integrator,
        Integrator::Euler,
        "analytic damping Jacobian requires the Euler (eulerdamp) integrator",
    );
    assert!(
        model
            .jnt_type
            .iter()
            .all(|t| matches!(t, MjJointType::Hinge | MjJointType::Slide)),
        "analytic damping Jacobian position row is hinge/slide-only",
    );
    assert_eq!(
        model.ntendon, 0,
        "analytic damping Jacobian does not model tendon spring/damper M_impl terms",
    );
}

/// Compute the analytic single-step damping Jacobian `∂x⁺/∂D` at the current state.
///
/// Reuses the same `M_impl = M + h·D` factor as the eulerdamp step and the
/// Tier-1/2 transition derivatives.
///
/// # Panics
///
/// Panics (in **all** build profiles, not just debug) unless the model is within
/// the supported scope — silently returning a wrong gradient on an out-of-scope
/// model would be a contract violation, so these are hard asserts:
///
/// - **Integrator must be [`Integrator::Euler`]** (the eulerdamp path). Other
///   integrators are a follow-on.
/// - **All joints must be hinge or slide.** The velocity row is general, but the
///   position-tangent row uses the hinge/slide map `∂qpos⁺/∂D = h·∂v⁺/∂D`; the
///   ball/free right-Jacobian factor is a follow-on.
/// - **No tendon spring/damper** (`M_impl` reconstruction here is `M + h·D`
///   only; tendon `JᵀJ` terms are not joint-damping parameters).
///
/// # Errors
///
/// Returns [`StepError::CholeskyFailed`] if `M_impl` is not positive definite,
/// or any step error encountered while evaluating the operating point.
pub fn mjd_damping_jacobian(model: &Model, data: &Data) -> Result<DampingJacobian, StepError> {
    assert_damping_scope(model);

    let nv = model.nv;
    let na = model.na;
    let nx = 2 * nv + na;
    let h = model.timestep;

    // Operating point: post-step velocity v⁺ from a real step (captures the
    // exact eulerdamp solve), and the pre-step mass matrix M from forward()
    // (pose-dependent only; the eulerdamp builds M_impl from this same qM).
    let mut d_post = data.clone();
    d_post.step(model)?;
    let v_plus = &d_post.qvel;

    let mut d_pre = data.clone();
    d_pre.forward(model)?;

    // M_impl = M + h·D (same diagonal modification as mj_fwd_acceleration_implicit).
    let mut m_impl = d_pre.qM.clone();
    for i in 0..nv {
        m_impl[(i, i)] += h * model.implicit_damping[i];
    }
    cholesky_in_place(&mut m_impl)?;

    let mut dxdD = DMatrix::zeros(nx, nv);
    let mut e_j = DVector::zeros(nv);
    for j in 0..nv {
        // col_j = M_impl⁻¹ · e_j  (j-th column of the inverse).
        e_j.fill(0.0);
        e_j[j] = 1.0;
        cholesky_solve_in_place(&m_impl, &mut e_j);

        // Velocity row: ∂v⁺/∂D_j = −h · v⁺_j · (M_impl⁻¹ column j).
        let scale = -h * v_plus[j];
        for r in 0..nv {
            let dv = scale * e_j[r];
            dxdD[(nv + r, j)] = dv;
            // Position-tangent row (hinge/slide semi-implicit Euler): h · ∂v⁺/∂D.
            dxdD[(r, j)] = h * dv;
        }
        // Activation rows stay zero.
    }

    Ok(DampingJacobian { dxdD })
}

/// Terminal-state sensitivity of an `n_steps` pure-rigid rollout w.r.t. joint
/// damping — `∂x_N/∂D` accumulated analytically in one forward pass.
pub struct TrajectoryDampingJacobian {
    /// Terminal state `x_N = [dq_N, qvel_N, act_N]` in tangent space, taken
    /// relative to the *initial* `qpos` (length `2*nv + na`).
    pub terminal_state: DVector<f64>,
    /// `∂x_N/∂D`, dimensions `(2*nv + na) × nv`. Column `j` is the sensitivity
    /// of the terminal state to the joint-damping coefficient on DOF `j`.
    pub dterminal_dD: DMatrix<f64>,
}

/// Analytic terminal-state Jacobian of a damped rollout w.r.t. per-DOF joint
/// damping, via the forward-sensitivity recursion.
///
/// Let `x_{t+1} = f(x_t; D)`. The total sensitivity `s_t = ∂x_t/∂D` obeys
///
/// ```text
/// s_0     = 0
/// s_{t+1} = A_t · s_t + P_t
/// ```
///
/// where `A_t = ∂f/∂x_t` is the [transition matrix](mjd_transition) (Tier-1/2,
/// eulerdamp-correct under damping) and `P_t = ∂f/∂D` is the single-step
/// [`mjd_damping_jacobian`] (the *direct* parameter Jacobian, holding `x_t`
/// fixed). This routine returns `s_N` and the terminal state `x_N`.
///
/// # Panics
///
/// Same scope as [`mjd_damping_jacobian`] (Euler integrator, hinge/slide joints,
/// no tendons) — the flat hinge/slide tangent space lets the per-step
/// linearizations compose in a common frame without parallel transport. Panics in
/// all build profiles if the model is out of scope.
///
/// # Errors
///
/// Propagates any [`StepError`] from the rollout or per-step derivatives.
pub fn mjd_damping_trajectory_jacobian(
    model: &Model,
    data0: &Data,
    n_steps: usize,
    config: &DerivativeConfig,
) -> Result<TrajectoryDampingJacobian, StepError> {
    assert_damping_scope(model);

    let nv = model.nv;
    let na = model.na;
    let nx = 2 * nv + na;
    let qpos_0 = data0.qpos.clone();

    let mut data = data0.clone();
    let mut s = DMatrix::zeros(nx, nv); // s_0 = ∂x_0/∂D = 0
    for _ in 0..n_steps {
        // Re-derive the operating point so the per-step linearizations read a
        // forward-consistent state (both helpers also re-forward internally).
        data.forward(model)?;
        let a_t = mjd_transition(model, &data, config)?.A;
        let p_t = mjd_damping_jacobian(model, &data)?.dxdD;
        // s_{t+1} = A_t · s_t + P_t
        s = &a_t * &s + &p_t;
        data.step(model)?;
    }

    Ok(TrajectoryDampingJacobian {
        terminal_state: extract_state(model, &data, &qpos_0),
        dterminal_dD: s,
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::similar_names)]
mod tests {
    use super::super::fd::extract_state;
    use super::*;
    use crate::derivatives::max_relative_error;

    /// Central-FD reference for `∂x⁺/∂D` — perturbs the *physical* damping knob
    /// `jnt_damping[joint]` and recomputes the cached `implicit_damping`, so both
    /// the explicit passive-force path and the implicit eulerdamp path move
    /// together. For the hinge pendulum joint `j` ↔ DOF `j` one-to-one.
    fn fd_damping_jacobian(model: &Model, data: &Data, eps: f64) -> DMatrix<f64> {
        let nv = model.nv;
        let na = model.na;
        let nx = 2 * nv + na;
        let qpos_0 = data.qpos.clone();
        let mut jac = DMatrix::zeros(nx, nv);
        for j in 0..nv {
            let mut mp = model.clone();
            mp.jnt_damping[j] += eps;
            mp.compute_implicit_params();
            let mut dp = data.clone();
            dp.step(&mp).unwrap();
            let y_plus = extract_state(&mp, &dp, &qpos_0);

            let mut mm = model.clone();
            mm.jnt_damping[j] -= eps;
            mm.compute_implicit_params();
            let mut dm = data.clone();
            dm.step(&mm).unwrap();
            let y_minus = extract_state(&mm, &dm, &qpos_0);

            let col = (&y_plus - &y_minus) / (2.0 * eps);
            jac.column_mut(j).copy_from(&col);
        }
        jac
    }

    /// 2-link hinge pendulum with per-joint damping (the spike's regime).
    fn damped_pendulum() -> (Model, Data) {
        let mut model = Model::n_link_pendulum(2, 1.0, 0.1);
        model.jnt_damping = vec![0.5, 0.8];
        model.compute_implicit_params();
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qpos[1] = -0.2;
        data.qvel[0] = 0.7;
        data.qvel[1] = -0.4;
        data.forward(&model).unwrap();
        (model, data)
    }

    /// Central-FD reference for `∂x_N/∂D` over a full `n_steps` rollout:
    /// perturb the physical knob `jnt_damping[j]`, roll the whole trajectory,
    /// and difference the terminal state (tangent vs the initial `qpos`).
    fn fd_trajectory_jacobian(
        model: &Model,
        data0: &Data,
        n_steps: usize,
        eps: f64,
    ) -> (DVector<f64>, DMatrix<f64>) {
        let nv = model.nv;
        let nx = 2 * nv + model.na;
        let qpos_0 = data0.qpos.clone();

        let rollout = |damp_j: usize, delta: f64| -> DVector<f64> {
            let mut m = model.clone();
            m.jnt_damping[damp_j] += delta;
            m.compute_implicit_params();
            let mut d = data0.clone();
            for _ in 0..n_steps {
                d.step(&m).unwrap();
            }
            extract_state(&m, &d, &qpos_0)
        };

        let mut nominal = data0.clone();
        for _ in 0..n_steps {
            nominal.step(model).unwrap();
        }
        let terminal = extract_state(model, &nominal, &qpos_0);

        let mut jac = DMatrix::zeros(nx, nv);
        for j in 0..nv {
            let col = (rollout(j, eps) - rollout(j, -eps)) / (2.0 * eps);
            jac.column_mut(j).copy_from(&col);
        }
        (terminal, jac)
    }

    #[test]
    fn analytic_trajectory_jacobian_matches_fd_2link() {
        let (model, data) = damped_pendulum();
        let n_steps = 60;
        let cfg = DerivativeConfig::default();
        let analytic = mjd_damping_trajectory_jacobian(&model, &data, n_steps, &cfg).unwrap();
        let (fd_terminal, fd_jac) = fd_trajectory_jacobian(&model, &data, n_steps, 1e-6);

        // Terminal state must agree (sanity that both roll out the same dynamics).
        let state_err = (&analytic.terminal_state - &fd_terminal).amax();
        assert!(state_err < 1e-9, "terminal state mismatch: {state_err:.3e}");

        // Gradient: analytic recursion (hybrid A, FD position columns) vs full
        // central-FD of the rollout. Compounding over 60 steps; floor matches the
        // transition-harness convention.
        // Observed ~6e-9; 1e-6 is a meaningful regression guard with wide margin.
        let (err, loc) = max_relative_error(&analytic.dterminal_dD, &fd_jac, 1e-3);
        assert!(
            err < 1e-6,
            "analytic ∂x_N/∂D disagrees with rollout FD: max_rel_err={err:.3e} at {loc:?}"
        );

        // The compounded gradient is non-trivial (not a degenerate ~0 match).
        assert!(
            analytic.dterminal_dD.amax() > 1e-3,
            "expected a non-negligible terminal damping sensitivity"
        );
    }

    /// The scope contract is enforced in all build profiles (not just debug):
    /// an out-of-scope integrator panics loudly rather than silently returning a
    /// wrong gradient.
    #[test]
    #[should_panic(expected = "Euler")]
    fn rejects_non_euler_integrator() {
        let (mut model, data) = damped_pendulum();
        model.integrator = Integrator::RungeKutta4;
        mjd_damping_jacobian(&model, &data).unwrap(); // panics in the scope assert first
    }

    /// Same hard-scope contract on the trajectory entry point.
    #[test]
    #[should_panic(expected = "hinge/slide")]
    fn trajectory_rejects_ball_joint() {
        let (model, data) = damped_pendulum();
        // Force an out-of-scope joint type to confirm the loud panic.
        let mut m = model;
        m.jnt_type[1] = MjJointType::Ball;
        mjd_damping_trajectory_jacobian(&m, &data, 4, &DerivativeConfig::default()).unwrap();
    }

    #[test]
    fn analytic_damping_jacobian_matches_fd_2link() {
        let (model, data) = damped_pendulum();
        let analytic = mjd_damping_jacobian(&model, &data).unwrap();
        let fd = fd_damping_jacobian(&model, &data, 1e-6);

        // Near-zero entries (~h²·small) are dominated by FD noise; floor matches
        // the transition-harness convention.
        let (err, loc) = max_relative_error(&analytic.dxdD, &fd, 1e-3);
        assert!(
            err < 1e-5,
            "analytic ∂x⁺/∂D disagrees with FD: max_rel_err={err:.3e} at {loc:?}\n\
             analytic=\n{:.6}\nfd=\n{:.6}",
            analytic.dxdD,
            fd
        );
    }

    /// Damping enters through two paths (explicit `−D·v` force + implicit
    /// `M_impl`). The analytic `−h·v⁺·M_impl⁻¹` form combines both; the
    /// implicit-only `qacc` form (what perturbing just `implicit_damping` would
    /// measure) is materially different — this test pins that the *physical*
    /// parameter needs the combined `v⁺` form.
    #[test]
    fn velocity_row_needs_both_damping_paths() {
        let (model, data) = damped_pendulum();
        let analytic = mjd_damping_jacobian(&model, &data).unwrap();
        let fd_full = fd_damping_jacobian(&model, &data, 1e-6);
        let nv = model.nv;
        let h = model.timestep;

        // Implicit-only FD: perturb ONLY the cached eulerdamp field, leaving the
        // explicit passive force fixed. This is the `qacc`-path partial.
        let qpos_0 = data.qpos.clone();
        let mut fd_implicit_only = DMatrix::zeros(2 * nv, nv);
        for j in 0..nv {
            let mut mp = model.clone();
            mp.implicit_damping[j] += 1e-6;
            let mut dp = data.clone();
            dp.step(&mp).unwrap();
            let yp = extract_state(&mp, &dp, &qpos_0);
            let mut mm = model.clone();
            mm.implicit_damping[j] -= 1e-6;
            let mut dm = data.clone();
            dm.step(&mm).unwrap();
            let ym = extract_state(&mm, &dm, &qpos_0);
            fd_implicit_only
                .column_mut(j)
                .copy_from(&((&yp - &ym) / 2e-6).rows(0, 2 * nv));
        }

        let analytic_vel = analytic.dxdD.view((nv, 0), (nv, nv)).into_owned();
        let full_vel = fd_full.view((nv, 0), (nv, nv)).into_owned();
        let implicit_vel = fd_implicit_only.view((nv, 0), (nv, nv)).into_owned();

        // Analytic matches the full (both-path) physical sensitivity.
        let (err_full, _) = max_relative_error(&analytic_vel, &full_vel, 1e-3);
        assert!(
            err_full < 1e-5,
            "analytic must match full-knob FD: {err_full:.3e}"
        );

        // …and the implicit-only partial is genuinely different (≈ factor
        // h·qacc/v⁺ smaller) — confirming both paths are load-bearing, and h > 0.
        let _ = h;
        let (err_partial, _) = max_relative_error(&analytic_vel, &implicit_vel, 1e-3);
        assert!(
            err_partial > 1e-1,
            "implicit-only path must differ from the combined form: {err_partial:.3e}"
        );
    }
}
