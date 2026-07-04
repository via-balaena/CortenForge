//! Coupled-rollout failure surfaced as a value — [`RolloutError`] and its
//! `Display`/`Error` impls (see [`crate::StaggeredCoupling`]'s `try_`-prefixed rollouts).

use sim_soft::SolverFailure;
use std::fmt;

/// A coupled-rollout failure surfaced as a value instead of a panic.
///
/// The closed-loop grip rollout re-solves the soft buffer at every step. On a
/// stiff-contact design/policy the co-design optimizer explores (an aggressive
/// holding gain pressing a finite end-effector deep into the coarse buffer), that
/// soft solve can fail to converge — a *genuinely infeasible* equilibrium, not a
/// solver bug (raising the Newton cap converts it to an Armijo stall at a plateau
/// residual; the deformation gradient never inverts). The panic-path rollouts
/// ([`StaggeredCoupling::coupled_trajectory_design_policy_friction_gradient`](crate::StaggeredCoupling::coupled_trajectory_design_policy_friction_gradient) and
/// friends) surface that as a panic, matching the soft solver's fail-closed
/// contract. The `try_`-prefixed siblings return this error instead, so the
/// optimizer can skip the infeasible point and keep its best feasible iterate
/// (the robustness `cf_codesign::OptConfig::reject_infeasible` relies on) without a
/// `catch_unwind`.
///
/// **Which fail-closes convert to this error.** ALL of them. The `try_`-prefixed
/// rollout routes through the soft solver's `try_replay_step`, whose contract is
/// unconditional: every [`SolverFailure`] variant — [`SolverFailure::NewtonIterCap`],
/// [`SolverFailure::DoublyFailedFactor`], [`SolverFailure::ValidityViolation`] (a tet
/// over-stretching / inverting past the material's validity domain), and
/// [`SolverFailure::ArmijoStall`] (a non-SPD tangent / near-singular condensed system) —
/// surfaces as `Err`, never a panic, regardless of the solver's LM config. So all four
/// become a `RolloutError` here. (`ArmijoStall` was historically the one residual still
/// on the panic path; making the graceful API unconditional closed it — the grip itself
/// hits the Newton iter-cap and validity tear, but an aggressive design elsewhere can
/// reach the stall, and now it skips like any other infeasible point.)
#[derive(Debug)]
#[non_exhaustive]
pub struct RolloutError {
    /// The 0-based rollout step at which the soft solve failed.
    pub step: usize,
    /// The underlying soft-solver failure. Named `failure` (not `source`) so it does
    /// not read as the [`std::error::Error::source`] cause chain — `SolverFailure` is
    /// not an `Error`, so it is exposed as structured data, not as a `&dyn Error`.
    pub failure: SolverFailure,
}

impl fmt::Display for RolloutError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Render the soft failure concisely: name the variant + its scalar diagnostics, but OMIT the
        // per-variant `x_partial` (a full DOF vector — noise in a panic/log message; the structured
        // `failure` field still carries it for programmatic inspection). `SolverFailure` is
        // `Debug`-only, so this hand-formats rather than delegating to its `Display`.
        write!(f, "coupled grip rollout failed at step {}: ", self.step)?;
        match &self.failure {
            SolverFailure::NewtonIterCap {
                max_iter,
                last_r_norm,
                ..
            } => write!(
                f,
                "soft Newton iter-cap ({max_iter} iters) reached without convergence (residual \
                 {last_r_norm:e}) — infeasible design (the buffer cannot re-equilibrate)",
            ),
            SolverFailure::ArmijoStall {
                last_iter,
                last_r_norm,
                ..
            } => write!(
                f,
                "soft Armijo line-search stalled at Newton iter {last_iter} (residual \
                 {last_r_norm:e}) — non-SPD tangent / infeasible design",
            ),
            SolverFailure::DoublyFailedFactor { context, .. } => {
                write!(f, "soft tangent factorization doubly failed: {context}")
            }
            SolverFailure::ValidityViolation { tet_id, .. } => write!(
                f,
                "soft buffer left its validity domain at tet {tet_id} (a tet over-stretched / \
                 inverted) — infeasible design (the grip indents the coarse buffer too hard)",
            ),
        }
    }
}

// `source()` returns `None`: `SolverFailure` does not implement `std::error::Error`
// (it is a `Debug`-only failure value), so it cannot be returned as a `&dyn Error`
// cause. The structured `source` field above exposes it for programmatic inspection.
impl std::error::Error for RolloutError {}
