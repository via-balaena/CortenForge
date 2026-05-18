//! Levenberg-Marquardt regularization config for non-PD tangent rescue.
//!
//! Per `docs/F3_LM_REGULARIZATION_SPEC.md`. F3.1 lands the **types only**
//! (this module + the `SolverConfig::lm_regularization` field opt-in
//! gate). No execution path consumes these types yet — F3.2 wires the
//! `+λI` retry loop into [`super::backward_euler::CpuNewtonSolver::
//! factor_free_tangent`] alongside the in-solve `LmState` plumbing.
//!
//! Bit-equality invariant for F3.1: with `SolverConfig::lm_regularization
//! = None` (the [`super::backward_euler::SolverConfig::skeleton`]
//! default), the new types are inert — no behavior change vs pre-F3.

/// Per-stall policy applying ONLY to `SolverFailure::ArmijoStall`.
///
/// The other failure variants (`NewtonIterCap`, `DoublyFailedFactor`,
/// added at F3.3) always return `Err` on `try_step` and always panic on
/// `step`, regardless of this setting — there is no per-variant policy
/// for them.
///
/// The "saturation" in the name refers specifically to LM saturation
/// (retries exhausted at `λ_max`) followed by Armijo stall on the
/// un-regularized LU fallback step (per spec §2.5 step 5b). With LM
/// disabled, the policy applies to first-Armijo-stall (today's only
/// stall surface).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaturationPolicy {
    /// Today's behavior: panic with the existing Armijo-stall message
    /// (augmented to note LM-saturated when applicable). `Solver::step`
    /// always uses this regardless of LM enabled-ness; `try_step`
    /// dispatches to this when `PanicOnStall` is set.
    PanicOnStall,
    /// Used only by `Solver::try_step`. Returns
    /// `Err(SolverFailure::ArmijoStall { x_partial, last_iter,
    /// last_r_norm })` — the caller decides whether to accept the
    /// partial solve. `Solver::step` still panics on Armijo stall
    /// regardless of this setting; callers wanting graceful failure
    /// must use `try_step`.
    ReturnFailed,
}

/// Tunables for the Marquardt-style `+λI` retry loop.
///
/// Carried on `SolverConfig::lm_regularization` as `Option<LmConfig>`;
/// `None` is the bit-equal-to-pre-F3 opt-out. Fork-B (cf-device-design)
/// consumers opt in via [`Self::fork_b`]. F3.2 wires the loop into
/// `factor_free_tangent`; until then this struct is config-only.
#[derive(Clone, Copy, Debug)]
pub struct LmConfig {
    /// Seed λ at first non-PD detection. Computed as
    /// `seed_relative * max_diag_of_assembled_tangent`.
    pub seed_relative: f64,
    /// Multiplicative bump per non-PD retry beyond the seed retry.
    pub up_factor: f64,
    /// Multiplicative decay per `Llt` success with `λ > 0`. Floored at
    /// `λ_min = 0.0` (no sticky-warm policy — λ may decay all the way
    /// to zero between Newton iters; the next non-PD detection re-seeds
    /// from `seed_relative * max_diag`).
    pub down_factor: f64,
    /// Ceiling as multiple of `max_diag`. Above this, the regularized
    /// system degenerates to gradient descent with step size
    /// `1 / λ_max`, which is worse than the existing LU fallback.
    pub max_relative: f64,
    /// Backstop against infinite retry loops. With the rule
    /// `λ = max(λ * up_factor, λ_seed)`, retry 1 seeds (no
    /// multiplication), retries 2..N each multiply by `up_factor` — so
    /// the budget spans `λ_seed` → `λ_seed × up_factor^(N-1)`.
    pub max_retries_per_iter: usize,
    /// What happens when λ saturates at `λ_max` AND the saturated LU
    /// fallback step still Armijo-stalls. See [`SaturationPolicy`].
    pub on_saturation: SaturationPolicy,
}

impl LmConfig {
    /// Fork-B preset (cf-device-design insertion-ramp consumers). Spec
    /// §2.3 defaults, `ReturnFailed` saturation — callers route through
    /// `Solver::try_step` and propagate the `Err(SolverFailure)` as an
    /// anyhow error rather than panicking.
    #[must_use]
    pub const fn fork_b() -> Self {
        Self {
            seed_relative: 1e-6,
            up_factor: 10.0,
            down_factor: 0.5,
            max_relative: 1e3,
            max_retries_per_iter: 8,
            on_saturation: SaturationPolicy::ReturnFailed,
        }
    }
}

#[cfg(test)]
#[allow(clippy::float_cmp)] // exact-constant assertions on compile-time
// preset values — crate convention (see
// `material/silicone_table.rs` test allows).
mod tests {
    use super::*;

    #[test]
    fn fork_b_preset_matches_spec_defaults() {
        // Spec §2.3 LmConfig defaults table.
        let cfg = LmConfig::fork_b();
        assert_eq!(cfg.seed_relative, 1e-6);
        assert_eq!(cfg.up_factor, 10.0);
        assert_eq!(cfg.down_factor, 0.5);
        assert_eq!(cfg.max_relative, 1e3);
        assert_eq!(cfg.max_retries_per_iter, 8);
        assert_eq!(cfg.on_saturation, SaturationPolicy::ReturnFailed);
    }

    #[test]
    fn fork_b_is_const_constructible() {
        // Compile-time witness that fork_b() stays const-fn — required
        // so consumers can build SolverConfig in const contexts.
        const _CFG: LmConfig = LmConfig::fork_b();
    }

    #[test]
    fn skeleton_solver_config_does_not_opt_into_lm() {
        // Bit-equal invariant pinned per spec §2.3: `SolverConfig::
        // skeleton()` MUST default `lm_regularization` to `None` so the
        // 25+ existing test sites that route through `skeleton()` stay
        // bit-equal vs pre-F3. A future "everyone should benefit, flip
        // the default" change would break this and the whole regression
        // net — pin it here with a focused error message.
        assert!(
            super::super::SolverConfig::skeleton()
                .lm_regularization
                .is_none()
        );
    }
}
