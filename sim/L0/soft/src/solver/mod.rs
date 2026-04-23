//! `Solver` trait — the only `dyn`-safe public boundary.
//!
//! One runtime dispatch per `step` via `Box<dyn Solver<Tape = CpuTape>>`.
//! Per-Newton-iteration work monomorphizes behind the concrete solver
//! (skeleton: `CpuNewtonSolver<NeoHookean, Tet4, SingleTetMesh,
//! NullContact, 4, 1>`). Part 11 Ch 01 01-composition §Solver commits
//! to this split.
//!
//! The associated `Tape` type pins CPU / GPU VJPs to their own tape
//! representation. Skeleton is CPU-only (`CpuTape`).

use std::marker::PhantomData;

use sim_ml_chassis::{Tape, Tensor};

pub mod backward_euler;

pub use backward_euler::{CpuNewtonSolver, SolverConfig};

/// CPU tape — alias for the chassis `Tape` so VJPs register against a
/// named backend type. `GpuTape` (Phase E) lands as a separate alias.
pub type CpuTape = Tape;

/// The artifact of one Newton step: primal outputs, the converged
/// factor, and any VJP-primal state the adjoint back-substitutes
/// against. `T` is the tape type (`CpuTape` / `GpuTape`).
///
/// Skeleton carries no fields — the VJP's primal stash lives on the
/// tape via `push_custom` per BF-4. Phase B expands this to a full
/// record per Part 6 Ch 02 / Ch 04.
#[derive(Debug)]
pub struct NewtonStep<T> {
    _tape: PhantomData<T>,
}

/// Mechanical solver surface. Two concrete impls: `CpuNewtonSolver`
/// (Phase B) and `GpuNewtonSolver` (Phase E).
pub trait Solver: Send + Sync {
    /// Tape representation the solver's VJPs register against.
    type Tape;

    /// Advance one time step. Writes the forward trajectory and its
    /// adjoint hooks onto `tape`. Must be deterministic in `(x_prev,
    /// v_prev, theta, dt)` — no cross-call output-affecting state.
    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape>;

    /// Re-solve the same step for backward-pass replay. Pure-function
    /// counterpart to `step` — no `&mut tape`. Must be bit-reproducible
    /// given the stored primal `(x, v, dt)` per Part 6 Ch 04.
    fn replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> NewtonStep<Self::Tape>;

    /// Current integration time-step (seconds).
    fn current_dt(&self) -> f64;

    /// Newton convergence tolerance (residual norm).
    fn convergence_tol(&self) -> f64;
}
