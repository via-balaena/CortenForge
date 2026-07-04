//! `impl Solver for` [`CpuNewtonSolver`](super::CpuNewtonSolver).

use sim_ml_chassis::{Tensor, Var};

use crate::contact::{ActivePairsFor, ContactModel};
use crate::element::Element;
use crate::material::Material;
use crate::mesh::Mesh;
use crate::solver::{CpuTape, NewtonStep, Solver, SolverFailure};

use super::CpuNewtonSolver;

impl<E, Msh, C, M, const N: usize, const G: usize> Solver for CpuNewtonSolver<E, Msh, C, M, N, G>
where
    E: Element<N, G>,
    Msh: Mesh<M>,
    M: Material,
    C: ContactModel + ActivePairsFor<M>,
{
    type Tape = CpuTape;

    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Snapshot θ's value off the tape so the forward Newton loop can
        // proceed in pure-tensor space. The tape's role is reverse-mode
        // bookkeeping; the primal solve is tape-free.
        let theta_tensor = tape.value_tensor(theta_var).clone();
        let (mut step, lm_final_lambda) = self.solve_impl(x_prev, v_prev, &theta_tensor, dt);

        // IFT adjoint factor: re-assemble A at x_final (post-convergence)
        // and factor it via `factor_free_tangent` — Llt happy path or
        // A2 Lu fallback per `FactoredFreeTangent`. F3: pass the
        // Newton-final λ as the LM seed (per spec §2.1) so late-iter
        // LM activity warm-starts the adjoint factor; on the disabled
        // path the seed is ignored. Factor ownership pattern (I-3)
        // verified for Llt in tests/invariant_3_factor.rs; the same
        // Arc-internal ownership shape holds for Lu.
        let factor = self.factor_at_position(&step.x_final, None, dt, lm_final_lambda);
        self.push_newton_step_vjp(tape, theta_var, &mut step, factor);
        step
    }

    fn replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape> {
        // Pure-function counterpart; no tape mutation. At skeleton scale
        // this is the same primal solve — Phase-E checkpoint replay will
        // diverge by reading a stored primal instead of re-solving.
        // `x_final_var` stays `None` — no tape means no Var. Drops the
        // F3 Newton-final λ: replay path doesn't run the IFT adjoint
        // factor (which is the only consumer of the seed).
        let (step, _lm_final_lambda) = self.solve_impl(x_prev, v_prev, theta, dt);
        step
    }

    // Graceful API: every fail-close (ArmijoStall included) surfaces as
    // `Err`, never a panic — `try_solve_impl` already returns all four
    // variants. The panic-on-fail-close mirror is `step` (a separate
    // `solve_impl` path), which is unaffected by this.
    fn try_step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure> {
        let theta_tensor = tape.value_tensor(theta_var).clone();
        let (mut step, lm_final_lambda) = self.try_solve_impl(x_prev, v_prev, &theta_tensor, dt)?;
        let factor = self.try_factor_at_position(&step.x_final, None, dt, lm_final_lambda)?;
        self.push_newton_step_vjp(tape, theta_var, &mut step, factor);
        Ok(step)
    }

    // Replay mirror of `try_step` — same unconditional-`Err` contract.
    fn try_replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure> {
        let (step, _lm_final_lambda) = self.try_solve_impl(x_prev, v_prev, theta, dt)?;
        Ok(step)
    }

    fn current_dt(&self) -> f64 {
        self.config.dt
    }

    fn convergence_tol(&self) -> f64 {
        self.config.tol
    }
}
