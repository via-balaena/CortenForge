//! `Differentiable` trait — VJP registry surface.
//!
//! Four items: `register_vjp`, `ift_adjoint`, `time_adjoint`,
//! `fd_wrapper`. Not a hot-path trait. Skeleton ships
//! `CpuDifferentiable` (stub).
//!
//! BF-4: `register_vjp` is stubbed — chassis's shipped `Tape::push_custom`
//! (PR #213) bundles the VJP with the node at forward-pass time rather
//! than using a key-indexed registry. Skeleton writes VJPs via
//! `push_custom` inline, with `register_vjp` kept as trait-contract-
//! compliance only. Logged as Pass 2/3 book finding in spec §13.

use sim_ml_chassis::{Tensor, autograd::VjpOp};

use crate::readout::GradientEstimate;
use crate::solver::NewtonStep;

pub mod newton_vjp;

pub use newton_vjp::CpuDifferentiable;

/// Lookup key for a forward-pass tape node. Skeleton-local alias;
/// chassis uses `Var(u32)` and does not surface this name. BF-4 flags
/// the book's `register_vjp(TapeNodeKey, ...)` API for Pass 2/3 edit.
pub type TapeNodeKey = u32;

/// VJP-registry surface. Paired with `Solver::Tape` so forward and
/// backward share one tape representation.
pub trait Differentiable {
    /// Tape representation this registry writes against.
    type Tape;

    /// Register a VJP for a forward-pass node.
    ///
    /// **BF-4 stub:** chassis's `Tape::push_custom` bundles the VJP at
    /// forward time. Skeleton keeps this method for trait compliance
    /// but never wires it up. Book-edit queue: `project_soft_body_bug_audit.md`.
    fn register_vjp(&mut self, forward_key: TapeNodeKey, vjp: Box<dyn VjpOp>);

    /// Implicit-function-theorem adjoint for one Newton step. Seeds
    /// `upstream` on `step`'s output var and reads back the cotangent
    /// of θ via `tape.backward`.
    fn ift_adjoint(
        &self,
        tape: &Self::Tape,
        step: &NewtonStep<Self::Tape>,
        upstream: &Tensor<f64>,
    ) -> Tensor<f64>;

    /// Multi-step (time-) adjoint over a rollout. Phase E+.
    fn time_adjoint(
        &self,
        tape: &Self::Tape,
        rollout: &[NewtonStep<Self::Tape>],
        upstream: &Tensor<f64>,
    ) -> Tensor<f64>;

    /// Finite-difference wrapper returning a gradient estimate with
    /// variance. Phase G stochastic-adjoint path.
    fn fd_wrapper(
        &self,
        forward: &dyn Fn(&Tensor<f64>) -> Tensor<f64>,
        theta: &Tensor<f64>,
    ) -> (Tensor<f64>, GradientEstimate);
}
