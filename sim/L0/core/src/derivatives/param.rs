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
use crate::dynamics::compute_body_spatial_inertia;
use crate::dynamics::crba::mj_crba;
use crate::dynamics::rne::mj_rne;
use crate::linalg::{cholesky_in_place, cholesky_solve_in_place};
use crate::types::{Integrator, MjJointType, StepError};
use crate::{Data, Model};
use nalgebra::{DMatrix, DVector, Matrix6, Vector3};

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

/// Analytic single-step Jacobian of the next state w.r.t. per-body mass.
///
/// Column `b` of [`dxdm`](Self::dxdm) is `∂x⁺/∂body_mass[b]`. Column `0` (the
/// world body) is always zero. Body mass enters the step through three coupled
/// paths — the mass matrix `M` (CRBA), the gravity force, and the Coriolis/bias
/// force (RNE) — but they collapse to a single inverse-dynamics term (see
/// [`mjd_mass_jacobian`]).
pub struct MassJacobian {
    /// `∂x⁺/∂m`, dimensions `(2*nv + na) × nbody`.
    ///
    /// Row layout matches [`TransitionMatrices`](super::TransitionMatrices):
    /// `[0..nv]` position tangent, `[nv..2*nv]` velocity, `[2*nv..2*nv+na]`
    /// activation (always zero — mass does not affect activation in one step).
    /// Column `b` is the derivative w.r.t. `body_mass[b]`; column `0` is zero.
    pub dxdm: DMatrix<f64>,
}

/// Hard-assert a model is within the supported scope of the analytic mass channel
/// (Euler integrator, hinge/slide joints, no tendons) — identical scope to the
/// damping channel. Out-of-scope models would yield a silently-wrong gradient, so
/// this panics in all profiles.
fn assert_mass_scope(model: &Model) {
    assert_eq!(
        model.integrator,
        Integrator::Euler,
        "analytic mass Jacobian requires the Euler (eulerdamp) integrator",
    );
    assert!(
        model
            .jnt_type
            .iter()
            .all(|t| matches!(t, MjJointType::Hinge | MjJointType::Slide)),
        "analytic mass Jacobian position row is hinge/slide-only",
    );
    assert_eq!(
        model.ntendon, 0,
        "analytic mass Jacobian does not model tendon spring/damper M_impl terms",
    );
    // Unlike damping, body mass enters two further passive forces the channel does
    // NOT differentiate, so they must be absent (mass-specific guards):
    // - gravity compensation: qfrc_gravcomp = −g·(body_mass·gravcomp), linear in mass.
    // - inertia-box fluid drag: equivalent box dims ∝ 1/mass (see mj_inertia_box_fluid).
    assert_eq!(
        model.ngravcomp, 0,
        "analytic mass Jacobian does not differentiate the mass-proportional gravcomp force",
    );
    // Exact zero is the disabled sentinel for both fluid coefficients.
    #[allow(clippy::float_cmp)]
    {
        assert!(
            model.density == 0.0 && model.viscosity == 0.0,
            "analytic mass Jacobian does not model mass-dependent fluid drag (density/viscosity must be 0)",
        );
    }
}

/// Compute the analytic single-step mass Jacobian `∂x⁺/∂body_mass` at the current
/// state.
///
/// # Math
///
/// The Euler eulerdamp step (`integrate`) is `v⁺ = v + h·M_impl⁻¹·F` with
/// `M_impl = M + h·D` (`D = implicit_damping`; spring `K` enters Euler only as a
/// mass-independent explicit force, never `M_impl`) and `F = qfrc_smooth +
/// qfrc_constraint`. Equivalently `v⁺ = M_impl⁻¹·(M·v + h·F)`, with
/// `qacc = (v⁺ − v)/h`. Body mass `m_b` enters `M` (so `M·v` and `M_impl`) and
/// `qfrc_bias` ⊂ `qfrc_smooth` (gravity + Coriolis). The `M·v` and `M_impl`
/// mass-dependences combine exactly, leaving
///
/// ```text
/// ∂v⁺/∂m_b = h · M_impl⁻¹ · [ ∂f/∂m_b − (∂M/∂m_b)·qacc ]
///          = −h · M_impl⁻¹ · ∂τ_ID/∂m_b
/// ```
///
/// where `τ_ID = M·qacc + qfrc_bias` is the inverse-dynamics torque at the
/// operating point `(q, v, qacc)`. The bracket is `−∂τ_ID/∂m_b`, and since both
/// CRBA and RNE are **linear in the body spatial inertias**, the two pieces
/// `(∂M/∂m_b)·qacc` and `∂qfrc_bias/∂m_b` are obtained by re-running the existing
/// `mj_crba` and `mj_rne` on a perturbation inertia distribution:
///
/// - `∂cinert[b]/∂m_b = compute_body_spatial_inertia(1, 0, R_b, h_b)` (the
///   unit-mass, zero-rotational-inertia spatial inertia of body `b`), zero for
///   all other bodies — feeds the CRBA composite-inertia and RNE Coriolis passes.
/// - For gravity, the joint torque is built from the mass-moment
///   `subtree_mass·subtree_com` (and a `subtree_mass·jpos` lever), so the exact
///   `∂/∂m_b` is reproduced by "a unit point mass at body `b`'s COM in every
///   subtree containing `b`": `subtree_mass[k] ← [b ∈ subtree(k)]`,
///   `subtree_com[k] ← xipos[b]`.
///
/// Reusing the production CRBA/RNE inherits their conformance fixes (the spatial
/// transport / `Xᵀ` moment lever) rather than re-deriving them.
///
/// The position-tangent row uses the hinge/slide semi-implicit map
/// `∂qpos⁺/∂m = h·∂v⁺/∂m`.
///
/// # Scope
///
/// Beyond the hard-asserted scope below, the operating point must be
/// **constraint-free** (no active contacts/joint limits): `qfrc_constraint`
/// depends on mass through the constraint solve and is not differentiated here
/// (shared with the damping channel). The pure-rigid `n_link_pendulum` regime
/// satisfies this.
///
/// # Performance
///
/// `O(nbody)` per call: one operating-point step/forward plus, for each body, a
/// `Data` clone and a full `mj_rne` + `mj_crba` pass (the latter's factorization
/// is unused). The trajectory driver calls this every step, so the trajectory
/// cost is `O(nbody · n_steps · cost(rne+crba))` — fine for system-ID on small
/// chains; a perturbation-CRBA without factorization is a future optimization.
///
/// # Panics
///
/// Hard scope (panics in all profiles): Euler integrator, hinge/slide joints, no
/// tendons, no gravity compensation (`ngravcomp == 0`), no fluid drag
/// (`density == viscosity == 0`). The last two are mass-specific: those passive
/// forces are mass-dependent but not modelled, so an out-of-scope model would get
/// a silently-wrong gradient.
///
/// # Errors
///
/// Returns [`StepError::CholeskyFailed`] if `M_impl` is not positive definite, or
/// any step error encountered while evaluating the operating point.
pub fn mjd_mass_jacobian(model: &Model, data: &Data) -> Result<MassJacobian, StepError> {
    assert_mass_scope(model);

    let nv = model.nv;
    let na = model.na;
    let nx = 2 * nv + na;
    let nbody = model.nbody;
    let h = model.timestep;

    // Operating point: eulerdamp qacc from a real step, and the pose-dependent
    // forward state (cinert, cvel, subtree_*, qM) the perturbation passes reuse.
    let mut d_post = data.clone();
    d_post.step(model)?;
    let qacc: DVector<f64> = (&d_post.qvel - &data.qvel) / h;

    let mut d_op = data.clone();
    d_op.forward(model)?;

    // M_impl = M + h·D (same factor as the eulerdamp step and Tier-1/2).
    let mut m_impl = d_op.qM.clone();
    for i in 0..nv {
        m_impl[(i, i)] += h * model.implicit_damping[i];
    }
    cholesky_in_place(&mut m_impl)?;

    // Armature is mass-independent, so the ∂M/∂m CRBA pass must exclude it.
    let mut model_no_arm = model.clone();
    model_no_arm.jnt_armature.iter_mut().for_each(|a| *a = 0.0);

    let mut dxdm = DMatrix::zeros(nx, nbody);
    for b in 1..nbody {
        let mut d_pert = d_op.clone();

        // ∂cinert/∂m_b: unit-mass, zero-rotational-inertia spatial inertia at body
        // b's COM offset; zero for every other body.
        let h_b = d_op.xipos[b] - d_op.xpos[b];
        let d_inertia = compute_body_spatial_inertia(1.0, Vector3::zeros(), &d_op.ximat[b], h_b);
        for i in 0..nbody {
            d_pert.cinert[i] = if i == b { d_inertia } else { Matrix6::zeros() };
        }

        // ∂(subtree mass-moment)/∂m_b: a unit point mass at body b's COM in every
        // subtree containing b (b and its ancestors). The gravity block reads only
        // `subtree_mass` and `subtree_com`, and its torque depends on the product
        // `subtree_mass·subtree_com`, so these substitutions give the exact
        // gravity-torque derivative.
        let xipos_b = d_op.xipos[b];
        for k in 0..nbody {
            d_pert.subtree_mass[k] = 0.0;
            d_pert.subtree_com[k] = xipos_b;
        }
        let mut k = b;
        loop {
            d_pert.subtree_mass[k] = 1.0;
            let parent = model.body_parent[k];
            if parent == k {
                break; // reached the world root (its own parent)
            }
            k = parent;
        }

        // ∂qfrc_bias/∂m_b (gravity + Coriolis), linear in the perturbation inertia.
        mj_rne(model, &mut d_pert);
        let d_bias = d_pert.qfrc_bias.clone();

        // ∂M/∂m_b (no armature), linear in the perturbation inertia; only qM is read
        // (the bogus factorization of this non-PD derivative matrix is unused).
        mj_crba(&model_no_arm, &mut d_pert);
        let d_mass_qacc = &d_pert.qM * &qacc;

        // ∂v⁺/∂m_b = h · M_impl⁻¹ · [ −∂qfrc_bias/∂m_b − (∂M/∂m_b)·qacc ].
        let mut col = -(&d_bias) - d_mass_qacc;
        col *= h;
        cholesky_solve_in_place(&m_impl, &mut col);
        for r in 0..nv {
            dxdm[(nv + r, b)] = col[r];
            dxdm[(r, b)] = h * col[r]; // hinge/slide position-tangent row
        }
    }

    Ok(MassJacobian { dxdm })
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

/// Terminal-state sensitivity of an `n_steps` pure-rigid rollout w.r.t. per-body
/// mass — `∂x_N/∂m` accumulated analytically in one forward pass.
pub struct TrajectoryMassJacobian {
    /// Terminal state `x_N = [dq_N, qvel_N, act_N]` in tangent space, taken
    /// relative to the *initial* `qpos` (length `2*nv + na`).
    pub terminal_state: DVector<f64>,
    /// `∂x_N/∂m`, dimensions `(2*nv + na) × nbody`. Column `b` is the sensitivity
    /// of the terminal state to `body_mass[b]`; column `0` (world) is zero.
    pub dterminal_dm: DMatrix<f64>,
}

/// Analytic terminal-state Jacobian of a rollout w.r.t. per-body mass, via the
/// forward-sensitivity recursion.
///
/// Identical recursion to [`mjd_damping_trajectory_jacobian`] — the recursion
/// `s_0 = 0`, `s_{t+1} = A_t·s_t + P_t` is **parameter-agnostic** and reuses the
/// same Tier-1/2 transition matrix `A_t`. Only the single-step parameter Jacobian
/// `P_t` differs: here it is [`mjd_mass_jacobian`] (`∂f/∂m`) instead of
/// [`mjd_damping_jacobian`]. Returns `s_N` (with `nbody` columns) and `x_N`.
///
/// # Panics
///
/// Same scope as [`mjd_mass_jacobian`] (Euler integrator, hinge/slide joints, no
/// tendons). Panics in all build profiles if the model is out of scope.
///
/// # Errors
///
/// Propagates any [`StepError`] from the rollout or per-step derivatives.
pub fn mjd_mass_trajectory_jacobian(
    model: &Model,
    data0: &Data,
    n_steps: usize,
    config: &DerivativeConfig,
) -> Result<TrajectoryMassJacobian, StepError> {
    assert_mass_scope(model);

    let nv = model.nv;
    let na = model.na;
    let nx = 2 * nv + na;
    let qpos_0 = data0.qpos.clone();

    let mut data = data0.clone();
    let mut s = DMatrix::zeros(nx, model.nbody); // s_0 = ∂x_0/∂m = 0
    for _ in 0..n_steps {
        // Re-derive the operating point so the per-step linearizations read a
        // forward-consistent state (both helpers also re-forward internally).
        data.forward(model)?;
        let a_t = mjd_transition(model, &data, config)?.A;
        let p_t = mjd_mass_jacobian(model, &data)?.dxdm;
        // s_{t+1} = A_t · s_t + P_t
        s = &a_t * &s + &p_t;
        data.step(model)?;
    }

    Ok(TrajectoryMassJacobian {
        terminal_state: extract_state(model, &data, &qpos_0),
        dterminal_dm: s,
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

    /// Central-FD reference for `∂x⁺/∂body_mass` — perturbs the physical knob
    /// `body_mass[b]` and re-steps. Column `b` is `∂x⁺/∂m_b`; column `0` (world)
    /// is left zero to match the analytic layout.
    fn fd_mass_jacobian(model: &Model, data: &Data, eps: f64) -> DMatrix<f64> {
        let nv = model.nv;
        let nx = 2 * nv + model.na;
        let qpos_0 = data.qpos.clone();
        let mut jac = DMatrix::zeros(nx, model.nbody);
        for b in 1..model.nbody {
            let mut mp = model.clone();
            mp.body_mass[b] += eps;
            let mut dp = data.clone();
            dp.step(&mp).unwrap();
            let yp = extract_state(&mp, &dp, &qpos_0);

            let mut mm = model.clone();
            mm.body_mass[b] -= eps;
            let mut dm = data.clone();
            dm.step(&mm).unwrap();
            let ym = extract_state(&mm, &dm, &qpos_0);

            jac.column_mut(b).copy_from(&((yp - ym) / (2.0 * eps)));
        }
        jac
    }

    #[test]
    fn analytic_mass_jacobian_matches_fd_2link() {
        let (model, data) = damped_pendulum();
        let analytic = mjd_mass_jacobian(&model, &data).unwrap();
        let fd = fd_mass_jacobian(&model, &data, 1e-6);

        let (err, loc) = max_relative_error(&analytic.dxdm, &fd, 1e-3);
        assert!(
            err < 1e-5,
            "analytic ∂x⁺/∂m disagrees with FD: max_rel_err={err:.3e} at {loc:?}\n\
             analytic=\n{:.6}\nfd=\n{:.6}",
            analytic.dxdm,
            fd
        );

        // Non-trivial sensitivity (not a degenerate ~0 match), and the world
        // column is exactly zero.
        assert!(
            analytic.dxdm.amax() > 1e-3,
            "expected a non-negligible mass sensitivity"
        );
        // The world-body column is never written (the loop starts at b=1), so it
        // is exactly the zero it was initialized to — an exact compare is correct.
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(
                analytic.dxdm.column(0).amax(),
                0.0,
                "world-body mass column must be zero"
            );
        }
    }

    #[test]
    #[should_panic(expected = "Euler")]
    fn mass_rejects_non_euler_integrator() {
        let (mut model, data) = damped_pendulum();
        model.integrator = Integrator::RungeKutta4;
        mjd_mass_jacobian(&model, &data).unwrap();
    }

    /// Gravity compensation is mass-proportional (`qfrc_gravcomp = −g·m·gravcomp`)
    /// but not differentiated — an in-scope-by-other-asserts model with gravcomp
    /// must panic rather than return a silently-wrong gradient.
    #[test]
    #[should_panic(expected = "gravcomp")]
    fn mass_rejects_gravcomp() {
        let (mut model, data) = damped_pendulum();
        model.ngravcomp = 1; // signals at least one gravcomp body present
        mjd_mass_jacobian(&model, &data).unwrap();
    }

    /// 3-link variant of the single-step FD gate. The 2-link fixture cannot
    /// exercise 3-deep ancestor transport in the gravity subtree-mass walk or the
    /// CRBA composite-inertia accumulation; this catches a transport/sign bug that
    /// vanishes for ≤2-link chains (a recurring failure mode in this codebase).
    #[test]
    fn analytic_mass_jacobian_matches_fd_3link() {
        let mut model = Model::n_link_pendulum(3, 1.0, 0.1);
        model.jnt_damping = vec![0.5, 0.3, 0.7];
        model.compute_implicit_params();
        let mut data = model.make_data();
        data.qpos[0] = 0.3;
        data.qpos[1] = -0.2;
        data.qpos[2] = 0.4;
        data.qvel[0] = 0.7;
        data.qvel[1] = -0.4;
        data.qvel[2] = 0.5;
        data.forward(&model).unwrap();

        let analytic = mjd_mass_jacobian(&model, &data).unwrap();
        let fd = fd_mass_jacobian(&model, &data, 1e-6);
        let (err, loc) = max_relative_error(&analytic.dxdm, &fd, 1e-3);
        assert!(
            err < 1e-5,
            "3-link analytic ∂x⁺/∂m disagrees with FD: max_rel_err={err:.3e} at {loc:?}\n\
             analytic=\n{:.6}\nfd=\n{:.6}",
            analytic.dxdm,
            fd
        );
        assert!(
            analytic.dxdm.amax() > 1e-3,
            "expected non-negligible sensitivity"
        );
    }

    /// Central-FD reference for `∂x_N/∂m` over a full `n_steps` rollout: perturb
    /// the physical knob `body_mass[b]`, roll the whole trajectory, and difference
    /// the terminal state (tangent vs the initial `qpos`).
    fn fd_mass_trajectory_jacobian(
        model: &Model,
        data0: &Data,
        n_steps: usize,
        eps: f64,
    ) -> (DVector<f64>, DMatrix<f64>) {
        let nv = model.nv;
        let nx = 2 * nv + model.na;
        let qpos_0 = data0.qpos.clone();

        let rollout = |body: usize, delta: f64| -> DVector<f64> {
            let mut m = model.clone();
            m.body_mass[body] += delta;
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

        let mut jac = DMatrix::zeros(nx, model.nbody);
        for b in 1..model.nbody {
            let col = (rollout(b, eps) - rollout(b, -eps)) / (2.0 * eps);
            jac.column_mut(b).copy_from(&col);
        }
        (terminal, jac)
    }

    #[test]
    fn analytic_mass_trajectory_jacobian_matches_fd_2link() {
        let (model, data) = damped_pendulum();
        let n_steps = 60;
        let cfg = DerivativeConfig::default();
        let analytic = mjd_mass_trajectory_jacobian(&model, &data, n_steps, &cfg).unwrap();
        let (fd_terminal, fd_jac) = fd_mass_trajectory_jacobian(&model, &data, n_steps, 1e-6);

        let state_err = (&analytic.terminal_state - &fd_terminal).amax();
        assert!(state_err < 1e-9, "terminal state mismatch: {state_err:.3e}");

        let (err, loc) = max_relative_error(&analytic.dterminal_dm, &fd_jac, 1e-3);
        assert!(
            err < 1e-6,
            "analytic ∂x_N/∂m disagrees with rollout FD: max_rel_err={err:.3e} at {loc:?}"
        );

        assert!(
            analytic.dterminal_dm.amax() > 1e-3,
            "expected a non-negligible terminal mass sensitivity"
        );
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
