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

/// The artifact of one converged Newton step: primal outputs, Newton
/// iteration count, and final residual norm. `T` is the tape type
/// (`CpuTape` / `GpuTape`).
///
/// Step 4 exposes `x_final`, `iter_count`, `final_residual_norm` so
/// integration tests can assert convergence behavior. Step 5 adds the
/// VJP's primal stash (factor, `∂r/∂θ`) via `Tape::push_custom` per
/// BF-4 — those fields land when the VJP first needs them.
#[derive(Debug)]
pub struct NewtonStep<T> {
    // Phantom tape — VJP registrations (step 5) use `T` to pin
    // CPU-tape / GPU-tape adjoint paths to the matching backend.
    _tape: PhantomData<T>,
    /// Converged primal position in 12-entry vertex-major + xyz-inner
    /// DOF layout.
    pub x_final: Vec<f64>,
    /// Number of Newton iterations taken to reach `convergence_tol`.
    pub iter_count: usize,
    /// Final free-DOF residual norm at convergence.
    pub final_residual_norm: f64,
}

impl<T> NewtonStep<T> {
    /// Construct a converged-step record. Solver impls call this only
    /// from within the Newton-convergence branch; non-convergence paths
    /// panic rather than return partial data (scope §3 R-2 SPD contract).
    #[must_use]
    pub const fn new_converged(
        x_final: Vec<f64>,
        iter_count: usize,
        final_residual_norm: f64,
    ) -> Self {
        Self {
            _tape: PhantomData,
            x_final,
            iter_count,
            final_residual_norm,
        }
    }
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
