//! `CpuDifferentiable` — CPU-backend Differentiable impl.
//!
//! Phase B ships `NewtonStepVjp: VjpOp` that stashes the converged
//! `x*`, θ, faer `Llt<I, f64>` factor of \(A = \partial r / \partial x\),
//! and sparse \(\partial r / \partial \theta\). `ift_adjoint` seeds
//! `upstream`, calls `tape.backward`, and reads back the θ cotangent.
//! Form A per spec §5.

use sim_ml_chassis::{Tensor, autograd::VjpOp};

use super::{Differentiable, TapeNodeKey};
use crate::readout::GradientEstimate;
use crate::solver::{CpuTape, NewtonStep};

/// CPU-backend `Differentiable` impl. Stateless at type level;
/// `NewtonStepVjp` holds primal data per tape node.
#[derive(Clone, Copy, Debug, Default)]
pub struct CpuDifferentiable;

impl Differentiable for CpuDifferentiable {
    type Tape = CpuTape;

    fn register_vjp(&mut self, _forward_key: TapeNodeKey, _vjp: Box<dyn VjpOp>) {
        unimplemented!(
            "skeleton phase 2 — BF-4 stub: chassis uses push_custom, not a key-indexed registry"
        )
    }

    fn ift_adjoint(
        &self,
        _tape: &Self::Tape,
        _step: &NewtonStep<Self::Tape>,
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2")
    }

    fn time_adjoint(
        &self,
        _tape: &Self::Tape,
        _rollout: &[NewtonStep<Self::Tape>],
        _upstream: &Tensor<f64>,
    ) -> Tensor<f64> {
        unimplemented!("skeleton phase 2 — time-adjoint is Phase E+")
    }

    fn fd_wrapper(
        &self,
        _forward: &dyn Fn(&Tensor<f64>) -> Tensor<f64>,
        _theta: &Tensor<f64>,
    ) -> (Tensor<f64>, GradientEstimate) {
        unimplemented!("skeleton phase 2 — fd_wrapper is Phase G stochastic-adjoint")
    }
}
