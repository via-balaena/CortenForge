//! Differentiable closed-loop feedback policy: the [`DiffPolicy`] trait and its
//! smallest impl [`LinearFeedback`] (`u = w_z·z + w_vz·vz + b`).

use crate::types::PolicyState;
use sim_ml_chassis::{Tape, Var};

/// A differentiable closed-loop feedback policy `u_k = π_θ(state_k)`: it maps the
/// per-step platen state to a vertical **control force** on the platen, with
/// parameters θ shared across every step.
///
/// Two views of the same function are required, and **must agree**:
/// - [`eval`](Self::eval) — a plain (non-tape) `f64` evaluation, used by the
///   forward oracle [`StaggeredCoupling::coupled_trajectory_policy_z`](crate::StaggeredCoupling::coupled_trajectory_policy_z) and to drive
///   the real physics.
/// - [`emit`](Self::emit) — builds the policy as a sub-expression on the chassis
///   tape from the θ parameter leaves and the [`PolicyState`](crate::PolicyState) vars, returning the
///   control output var. The chassis autograd carries both `∂u/∂θ` and `∂u/∂state`,
///   so [`StaggeredCoupling::coupled_trajectory_policy_gradient`](crate::StaggeredCoupling::coupled_trajectory_policy_gradient) gets the
///   closed-loop gradient `∂z_N/∂θ` (backprop-through-time across the
///   state→control recurrence) from one `tape.backward` — no hand-rolled adjoint.
///
/// The two views must compute the identical function of `(θ, z, vz)`; the gate's
/// forward-match assertion (tape `z_N` == oracle `z_N`) catches any divergence.
pub trait DiffPolicy {
    /// Number of policy parameters θ (the gradient length).
    fn n_params(&self) -> usize;

    /// Plain evaluation `u = π_θ(z, vz)` from real state — drives the physics and
    /// the forward oracle.
    fn eval(&self, params: &[f64], z: f64, vz: f64) -> f64;

    /// Emit the control output var onto `tape` from the parameter leaves `params`
    /// and the current platen `state`. The returned var is `[1]`-shaped.
    fn emit(&self, tape: &mut Tape, params: &[Var], state: PolicyState) -> Var;
}

/// A linear state-feedback policy `u = w_z·z + w_vz·vz + b`, parameters
/// θ = `[w_z, w_vz, b]` (all **signed**). A genuine feedback law — the control
/// depends on the platen state — physically a PD-style controller about the
/// implicit setpoint `z_ref = −b/w_z` (with `w_z` the proportional and `w_vz` the
/// derivative gain, both as signed weights). The smallest closed-loop policy that
/// exercises the recurrence; a multilayer policy is a follow-on `DiffPolicy` impl
/// (e.g. via the chassis `affine`/`tanh` primitives).
#[derive(Clone, Copy, Debug, Default)]
pub struct LinearFeedback;

impl DiffPolicy for LinearFeedback {
    fn n_params(&self) -> usize {
        3
    }

    fn eval(&self, params: &[f64], z: f64, vz: f64) -> f64 {
        params[0] * z + params[1] * vz + params[2]
    }

    fn emit(&self, tape: &mut Tape, params: &[Var], state: PolicyState) -> Var {
        let t1 = tape.mul(params[0], state.z);
        let t2 = tape.mul(params[1], state.vz);
        let s = tape.add(t1, t2);
        tape.add(s, params[2])
    }
}
