//! `Solver` trait — the only `dyn`-safe public boundary.
//!
//! One runtime dispatch per `step` via `Box<dyn Solver<Tape = CpuTape>>`.
//! Per-Newton-iteration work monomorphizes behind the concrete solver
//! (skeleton: `CpuNewtonSolver<Tet4, SingleTetMesh, NullContact,
//! NeoHookean, 4, 1>`). Phase 4 Decision G's NH-only monomorphization
//! was widened by the Yeoh arc's F4.0 commit to a generic `M:
//! Material` parameter (default [`crate::material::NeoHookean`]); see
//! [`crate::CpuNewtonSolver`] for the bound details. Part 11 Ch 01
//! 01-composition §Solver commits to the dyn-vs-monomorphized split.
//!
//! The associated `Tape` type pins CPU / GPU VJPs to their own tape
//! representation. Skeleton is CPU-only (`CpuTape`).

use std::marker::PhantomData;

use sim_ml_chassis::{Tape, Tensor, Var};

pub mod backward_euler;
pub mod lm;

pub(crate) use backward_euler::FactoredFreeTangent;
pub use backward_euler::{CpuNewtonSolver, FrictionReactionGradients, SolverConfig};
pub use lm::{LmConfig, SaturationPolicy};
// `SolverFailure` is re-exported from this module's own definition
// below — no `use` needed.

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
    /// rather than return partial data. Remaining panic surfaces post-A2
    /// are Newton iter cap, Armijo line-search stall, and the
    /// doubly-failed Llt-then-Lu factor case (`factor_free_tangent`);
    /// single-failure SPD trips are no longer panics — A2 routes them to
    /// the Lu fallback per `FactoredFreeTangent`.
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

/// Failure variants surfaced by [`Solver::try_step`] /
/// [`Solver::try_replay_step`] (F3.3 graceful-failure API per
/// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5).
///
/// Each variant carries `x_partial` — the Newton iterate at which the
/// failure was detected — so the caller can decide whether to accept
/// the partial solve (Fork-B `tol = 1e-1` is loose enough to
/// physically interpret some non-converged states; see the docstring
/// on `insertion_solver_config()` in
/// `tools/cf-device-design/src/insertion_sim.rs`).
///
/// Variant semantics per [`crate::SaturationPolicy`]:
/// - `ArmijoStall`: only this variant is governed by the
///   [`LmConfig::on_saturation`](crate::LmConfig::on_saturation)
///   policy. `Solver::step` always panics on this; `try_step`
///   dispatches on the policy (`PanicOnStall` → forward to `step`'s
///   panic; `ReturnFailed` → return `Err(ArmijoStall)`).
/// - `NewtonIterCap` and `DoublyFailedFactor`: `try_step` ALWAYS
///   returns `Err`; `Solver::step` ALWAYS panics — no per-variant
///   policy.
#[derive(Debug)]
pub enum SolverFailure {
    /// Armijo line-search stalled (under F3 LM, this means: retries
    /// exhausted at `λ_max` AND the un-regularized LU fallback step
    /// still failed to satisfy the Armijo decrease condition within
    /// `max_line_search_backtracks`; with LM disabled, this is the
    /// first Armijo stall on the pre-F3 LU-fallback step).
    ///
    /// `x_partial` is the `x_curr` at the START of the failed Newton
    /// iter — the best-known position from which Armijo found no
    /// descent step. NOT the most-recent `trial_x` from backtracking
    /// (that's a worse-residual rejected candidate); NOT a
    /// partial-Armijo-accepted x (no such thing — Armijo either
    /// accepts a step or stalls).
    ArmijoStall {
        /// `x_curr` at the START of the failed Newton iter (per
        /// variant docstring — committed-iterate semantics).
        x_partial: Vec<f64>,
        /// Newton iter number at which the stall was detected.
        last_iter: usize,
        /// Free-DOF residual norm at the start of the failed iter
        /// (the value Armijo was trying to decrease).
        last_r_norm: f64,
    },
    /// Newton iter cap (`SolverConfig::max_newton_iter`) reached
    /// without `r_norm < config.tol`. `x_partial` is the most-recent
    /// accepted Newton iterate (`x_curr` at the moment the loop
    /// exited via cap, i.e., after the last accepted Armijo step).
    /// The new `Result` variant of the pre-F3 `panic!` at
    /// `solve_impl`'s iter-cap branch.
    NewtonIterCap {
        /// Most-recent armijo-accepted iterate (one full iter past
        /// the last failed convergence check).
        x_partial: Vec<f64>,
        /// The configured `SolverConfig::max_newton_iter` cap.
        max_iter: usize,
        /// Free-DOF residual norm at the most-recent failed
        /// convergence check.
        last_r_norm: f64,
    },
    /// Doubly-failed factor: Llt non-PD (with LM retry budget
    /// exhausted under F3, or first non-PD with LM disabled) AND the
    /// LU fallback factor also failed. Pre-F3 this was a `panic!`;
    /// F3.3 surfaces it as `Err` for graceful-failure consumers.
    ///
    /// `x_partial` is the position at which the factor failed —
    /// `x_curr` at the start of the failed Newton iter for the
    /// in-Newton-loop case (`factor_and_solve_free`), or `x_final`
    /// for the post-Newton IFT-adjoint case (`factor_at_position`).
    /// `last_iter` is the Newton iter number for the in-loop case,
    /// or `0` for the IFT-adjoint case (no iter number applies
    /// post-Newton — the `context` string disambiguates).
    /// `context` carries the full factor-site tag + the Lu error
    /// details so callers can log diagnostically.
    DoublyFailedFactor {
        /// Position at which the factor failed: `x_curr` at the start
        /// of the failed Newton iter for the in-Newton-loop case, or
        /// `x_final` for the IFT-adjoint post-Newton case.
        x_partial: Vec<f64>,
        /// Newton iter number for the in-loop case, or `0` for the
        /// post-Newton IFT-adjoint case (the `context` string
        /// disambiguates).
        last_iter: usize,
        /// Full factor-site tag + Lu error details (preserves the
        /// pre-F3 panic message text for diagnostic logging).
        context: String,
    },
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

    /// Graceful-failure counterpart to [`Self::step`] (F3.3 per
    /// `docs/F3_LM_REGULARIZATION_SPEC.md` §2.5). Returns the same
    /// converged [`NewtonStep`] on the happy path; on the three
    /// failure surfaces (`ArmijoStall`, `NewtonIterCap`,
    /// `DoublyFailedFactor`) returns `Err(SolverFailure)` instead of
    /// panicking. The caller decides whether to abort the parent
    /// computation or accept the partial solve.
    ///
    /// `ArmijoStall` is governed by
    /// [`LmConfig::on_saturation`](crate::LmConfig::on_saturation):
    /// `PanicOnStall` forwards to [`Self::step`]'s panic (preserving
    /// pre-F3 semantics for callers that opted in to LM but not to
    /// graceful failure); `ReturnFailed` returns `Err`. When LM is
    /// disabled (`SolverConfig::lm_regularization == None`), the
    /// effective policy is `PanicOnStall` — calling `try_step` on
    /// an LM-disabled solver yields the same panic-on-stall surface
    /// as `step` itself. Graceful Armijo-stall handling REQUIRES
    /// opting into LM with `ReturnFailed` (or constructing an
    /// `LmConfig` explicitly setting that policy). The other two
    /// variants always return `Err` regardless of policy — they
    /// were already model-level non-recoverable conditions pre-F3.
    ///
    /// REQUIRED with no default impl per spec §2.5 — a default
    /// `Ok(self.step(...))` would mislead callers (the signature
    /// says `Result`, implying fallible, but the body panics).
    /// Forcing each impl to choose explicitly prevents future
    /// `Solver` impls from silently inheriting the wrong contract.
    ///
    /// # Errors
    /// Returns [`SolverFailure`] on Armijo stall (when the effective
    /// policy is `ReturnFailed`), Newton iter cap (always), or
    /// doubly-failed factor (always). When the effective policy is
    /// `PanicOnStall` (the LM-disabled default), Armijo stall panics
    /// instead of returning. See variant docs for `x_partial`
    /// semantics.
    fn try_step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure>;

    /// Graceful-failure counterpart to [`Self::replay_step`]. Same
    /// semantics as [`Self::try_step`] but on the pure-function
    /// (tape-free) path. REQUIRED with no default impl, same
    /// rationale as `try_step`.
    ///
    /// # Errors
    /// Same as [`Self::try_step`].
    fn try_replay_step(
        &self,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta: &Tensor<f64>,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure>;

    /// Current integration time-step (seconds).
    fn current_dt(&self) -> f64;

    /// Newton convergence tolerance (residual norm).
    fn convergence_tol(&self) -> f64;
}
