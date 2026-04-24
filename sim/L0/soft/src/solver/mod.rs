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

use sim_ml_chassis::{Tape, Tensor, Var};

pub mod backward_euler;

pub use backward_euler::{CpuNewtonSolver, SolverConfig};

/// CPU tape — alias for the chassis `Tape` so VJPs register against a
/// named backend type. `GpuTape` (Phase E) lands as a separate alias.
pub type CpuTape = Tape;

/// The artifact of one converged Newton step.
///
/// Carries primal outputs (`x_final`, `iter_count`, `final_residual_norm`)
/// plus — for tape-aware calls — the `Var` handle of `x_final` on the
/// tape. `T` is the tape type (`CpuTape` / `GpuTape`).
///
/// Step 4 introduced `x_final`, `iter_count`, `final_residual_norm` so
/// integration tests can assert convergence behavior. Step 5 adds
/// `x_final_var`: `Some` when produced by `Solver::step` (which pushes
/// `NewtonStepVjp` onto the tape), `None` when produced by
/// `Solver::replay_step` (pure-function counterpart, tape-free).
#[derive(Debug)]
pub struct NewtonStep<T> {
    // Phantom tape — pins CPU-tape / GPU-tape adjoint paths to the
    // matching backend at the type level.
    _tape: PhantomData<T>,
    /// Converged primal position in 12-entry vertex-major + xyz-inner
    /// DOF layout.
    pub x_final: Vec<f64>,
    /// Number of Newton iterations taken to reach `convergence_tol`.
    pub iter_count: usize,
    /// Final free-DOF residual norm at convergence.
    pub final_residual_norm: f64,
    /// Tape handle of `x_final` when this step was produced by
    /// `Solver::step` (which pushes `NewtonStepVjp` onto the tape with
    /// `theta_var` as parent). `None` when produced by `replay_step`
    /// (pure-function counterpart — no tape mutation).
    pub x_final_var: Option<Var>,
}

impl<T> NewtonStep<T> {
    /// Construct a converged-step record without a tape Var — the
    /// `replay_step` / tape-free path. Solver impls call this only from
    /// within the Newton-convergence branch; non-convergence paths panic
    /// rather than return partial data (scope §3 R-2 SPD contract).
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
            x_final_var: None,
        }
    }
}

/// Mechanical solver surface. Two concrete impls: `CpuNewtonSolver`
/// (Phase B) and `GpuNewtonSolver` (Phase E).
pub trait Solver: Send + Sync {
    /// Tape representation the solver's VJPs register against.
    type Tape;

    /// Advance one time step. Reads θ's current value from `tape` via
    /// `theta_var`, runs Newton + Armijo to convergence, re-factors the
    /// tangent at `x_final`, and pushes `NewtonStepVjp` onto the tape
    /// with `theta_var` as parent. Deterministic in `(x_prev, v_prev,
    /// theta_value, dt)` — no cross-call output-affecting state.
    fn step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> NewtonStep<Self::Tape>;

    /// Re-solve the same step for backward-pass replay. Pure-function
    /// counterpart to `step` — no `&mut tape`. Must be bit-reproducible
    /// given the stored primal `(x, v, dt)` per Part 6 Ch 04. Takes θ
    /// as a bare tensor (not a `Var`) because replay does not compose
    /// onto any tape.
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
