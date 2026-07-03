//! Levenberg-Marquardt regularization config + per-solve state for
//! non-PD tangent rescue.
//!
//! Per `docs/F3_LM_REGULARIZATION_SPEC.md`. Module surface:
//! - [`LmConfig`] (pub) — tunables; opt in via
//!   `SolverConfig::lm_regularization` (F3.1).
//! - `LmState` (`pub(super)`) — mutable in-solve state, threaded
//!   through `factor_free_tangent` / `factor_and_solve_free` /
//!   `factor_at_position` from `solve_impl`. The `disabled()`
//!   constructor is the bit-equal short-circuit when
//!   `lm_regularization == None` (F3.2).
//!
//! Bit-equality invariant: with `SolverConfig::lm_regularization =
//! None` (the [`super::backward_euler::SolverConfig::skeleton`]
//! default), `LmState::disabled` makes `can_bump` permanently false
//! and `factor_free_tangent`'s retry loop reduces to a single Llt
//! attempt followed by direct LU fallback — observably bit-equal vs
//! pre-F3 including the existing `"sim-soft: faer LU fallback fired"`
//! stderr line.

/// Tunables for the Marquardt-style `+λI` retry loop.
///
/// Carried on `SolverConfig::lm_regularization` as `Option<LmConfig>`;
/// `None` is the bit-equal-to-pre-F3 opt-out. Fork-B (cf-sim-research)
/// consumers opt in via [`Self::fork_b`]; F3.2 consumes this config
/// inside the retry loop in `factor_free_tangent`.
#[derive(Clone, Copy, Debug)]
pub struct LmConfig {
    /// Seed λ at first non-PD detection. Computed as
    /// `seed_relative * max_diag_of_assembled_tangent`. Default
    /// `1e-6` (per [`Self::fork_b`]).
    pub seed_relative: f64,
    /// Multiplicative bump per non-PD retry beyond the seed retry.
    /// Default `10.0` (per [`Self::fork_b`]).
    pub up_factor: f64,
    /// Multiplicative decay per `Llt` success with `λ > 0`. Floored at
    /// `λ_min = 0.0` (no sticky-warm policy — λ may decay all the way
    /// to zero between Newton iters; the next non-PD detection re-seeds
    /// from `seed_relative * max_diag`). Default `0.5` (per
    /// [`Self::fork_b`]).
    pub down_factor: f64,
    /// Ceiling as multiple of `max_diag`. Above this, the regularized
    /// system degenerates to gradient descent with step size
    /// `1 / λ_max`, which is worse than the existing LU fallback.
    /// Default `1e3` (per [`Self::fork_b`]).
    pub max_relative: f64,
    /// Backstop against infinite retry loops. With the rule
    /// `λ = max(λ * up_factor, λ_seed)`, retry 1 seeds (no
    /// multiplication), retries 2..N each multiply by `up_factor` — so
    /// the budget spans `λ_seed` → `λ_seed × up_factor^(N-1)`. Default
    /// `8` (per [`Self::fork_b`]).
    pub max_retries_per_iter: usize,
}

impl LmConfig {
    /// Fork-B preset (cf-sim-research insertion-ramp consumers). Spec
    /// §2.3 defaults — callers route through `Solver::try_step`, which
    /// returns `Err(SolverFailure)` on any fail-close (including a
    /// saturated Armijo stall) rather than panicking.
    #[must_use]
    pub const fn fork_b() -> Self {
        Self {
            seed_relative: 1e-6,
            up_factor: 10.0,
            down_factor: 0.5,
            max_relative: 1e3,
            max_retries_per_iter: 8,
        }
    }
}

/// Per-`solve_impl` mutable LM state.
///
/// Lives for the duration of one Newton solve (one `solve_impl` call):
/// created at iter 0, mutated by each `factor_free_tangent` retry within
/// an iter, and PERSISTS across Newton iters within the same solve
/// (per spec §2.2 — λ does not reset between Newton iters; the
/// `down_factor` decay rule handles well-conditioned mid-solve regions).
///
/// The `disabled()` constructor is the bit-equal short-circuit for
/// `SolverConfig::lm_regularization == None`: `lambda` permanently `0.0`
/// and `can_bump()` permanently `false`, so the `factor_free_tangent`
/// retry loop reduces to a single `Llt` attempt followed by direct LU
/// fallback (today's pre-F3 path), with zero allocation in the happy
/// path. The other two constructors carry an active `LmConfig` and seed
/// the loop appropriately:
/// - [`Self::from_config`]: initial state for the start of `solve_impl`
///   (λ = 0 — first non-PD detection runs the seed branch).
/// - [`Self::from_config_with_seed`]: passes the Newton-final λ into the
///   IFT-adjoint factor at `factor_at_position` (per spec §2.1) so the
///   adjoint factor starts warm if late-iter LM activity didn't fully
///   decay.
#[derive(Clone, Copy, Debug)]
pub(super) struct LmState {
    /// `None` when LM is disabled (the bit-equal short-circuit).
    /// `Some(cfg)` carries the tunables used by `on_non_pd` /
    /// `on_llt_success` / `can_bump`.
    config: Option<LmConfig>,
    /// Current regularization magnitude. `0.0` means "no `+λI` applied
    /// at this attempt" (the Llt input is the original un-regularized
    /// triplets — `Cow::Borrowed` in `factor_free_tangent`).
    lambda: f64,
    /// Retry counter for the in-flight `factor_free_tangent` call.
    /// Reset to 0 by [`Self::begin_factor_call`] at the top of each
    /// call; bumped by [`Self::on_non_pd`] on each non-PD failure.
    /// Used to gate retry-budget exhaustion AND to log per-call summary
    /// lines.
    retry_count: usize,
}

impl LmState {
    /// Sentinel state for `SolverConfig::lm_regularization == None`.
    /// `lambda` permanently `0.0`; `can_bump` permanently `false` —
    /// the `factor_free_tangent` retry loop short-circuits to today's
    /// pre-F3 path on the first non-PD detection.
    #[must_use]
    pub(super) const fn disabled() -> Self {
        Self {
            config: None,
            lambda: 0.0,
            retry_count: 0,
        }
    }

    /// Active-LM state seeded at λ = 0. Initial state for the start of
    /// `solve_impl` when `SolverConfig::lm_regularization == Some(cfg)`.
    /// The first non-PD detection runs the "seed" branch of
    /// [`Self::on_non_pd`].
    #[must_use]
    pub(super) const fn from_config(config: LmConfig) -> Self {
        Self {
            config: Some(config),
            lambda: 0.0,
            retry_count: 0,
        }
    }

    /// Active-LM state seeded at a non-zero λ. Used by `Solver::step`
    /// to thread the Newton-final λ into the IFT-adjoint factor (per
    /// spec §2.1). With λ > 0 at call start, the first Llt attempt uses
    /// the regularized tangent; if it succeeds (the common case for the
    /// adjoint factor on the SPD happy path), [`Self::on_llt_success`]
    /// decays λ for any subsequent factor calls on the same state — but
    /// since the adjoint factor is a one-shot, the decay is functionally
    /// dead.
    #[must_use]
    pub(super) const fn from_config_with_seed(config: LmConfig, seed_lambda: f64) -> Self {
        Self {
            config: Some(config),
            lambda: seed_lambda,
            retry_count: 0,
        }
    }

    /// Reset the per-call retry counter while preserving cross-iter λ.
    /// Called at the top of `factor_free_tangent`; the retry budget
    /// (`config.max_retries_per_iter`) starts fresh per call.
    pub(super) const fn begin_factor_call(&mut self) {
        self.retry_count = 0;
    }

    /// True when LM is active AND we have retries remaining AND λ has
    /// not yet hit the ceiling. The disabled state returns `false`
    /// unconditionally regardless of `max_diag` (the short-circuit).
    pub(super) fn can_bump(&self, max_diag: f64) -> bool {
        let Some(cfg) = self.config else {
            return false;
        };
        self.retry_count < cfg.max_retries_per_iter && self.lambda < cfg.max_relative * max_diag
    }

    /// Apply the seed-or-bump rule on non-PD detection per spec §2.2:
    /// `λ = max(λ * up_factor, λ_seed)`, then clamp to `λ_max`.
    /// First non-PD per call seeds (λ was 0 → `0 * up_factor = 0`, so
    /// `max(0, seed) = seed`). Subsequent retries multiply by
    /// `up_factor`. Bumps `retry_count` regardless. Caller is expected
    /// to gate via [`Self::can_bump`] first.
    //
    // expect_used: the `can_bump` gate (the caller's contract per the
    // doc above) returns false unconditionally on the disabled state,
    // so a disabled state reaching this method is a programmer-bug at
    // a call site that skipped the gate.
    #[allow(clippy::expect_used)]
    pub(super) fn on_non_pd(&mut self, max_diag: f64) {
        let cfg = self
            .config
            .expect("LmState::on_non_pd called on disabled state — gate via can_bump first");
        let seed = cfg.seed_relative * max_diag;
        let bumped = (self.lambda * cfg.up_factor).max(seed);
        // Clamp at the ceiling so saturation gating via `can_bump`
        // observes `lambda >= max_relative * max_diag` reliably.
        self.lambda = bumped.min(cfg.max_relative * max_diag);
        self.retry_count += 1;
    }

    /// Apply the decay rule on Llt success per spec §2.2: `λ *=
    /// down_factor` when `λ > 0`; floor at `λ_min = 0.0` (no
    /// sticky-warm — the next non-PD re-seeds via [`Self::on_non_pd`]).
    /// No-op on the disabled state.
    pub(super) fn on_llt_success(&mut self) {
        let Some(cfg) = self.config else {
            return;
        };
        if self.lambda > 0.0 {
            self.lambda *= cfg.down_factor;
        }
    }

    /// Current λ value. Used by `solve_impl` to snapshot the
    /// Newton-final λ for the IFT-adjoint factor seed, and by the
    /// per-call logging in `factor_free_tangent`.
    pub(super) const fn lambda(&self) -> f64 {
        self.lambda
    }

    /// Per-call retry counter. Used by the post-loop summary logs
    /// (success-summary, saturation-summary) to print the number of
    /// retries the loop took.
    pub(super) const fn retry_count(&self) -> usize {
        self.retry_count
    }

    /// True when this state has an active `LmConfig` (the non-disabled
    /// path). Used to gate LM-specific log emission so the disabled
    /// path stays observably-quiet (bit-equal vs pre-F3 stderr).
    pub(super) const fn is_active(&self) -> bool {
        self.config.is_some()
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
    }

    #[test]
    fn fork_b_is_const_constructible() {
        // Compile-time witness that fork_b() stays const-fn — required
        // so consumers can build SolverConfig in const contexts.
        const _CFG: LmConfig = LmConfig::fork_b();
    }

    #[test]
    fn disabled_state_can_never_bump() {
        // Bit-equal short-circuit invariant: `disabled()` must return
        // `can_bump == false` for ANY `max_diag` (including the
        // pathological zero / negative / NaN cases) so the
        // `factor_free_tangent` retry loop reduces to a single Llt
        // attempt + direct LU fallback at LM-disabled call sites.
        let s = LmState::disabled();
        assert!(!s.can_bump(0.0));
        assert!(!s.can_bump(1.0));
        assert!(!s.can_bump(1e9));
        assert!(!s.can_bump(-1.0));
        assert_eq!(s.lambda(), 0.0);
        assert_eq!(s.retry_count(), 0);
        assert!(!s.is_active());
    }

    #[test]
    fn from_config_seeds_then_bumps_per_marquardt_rule() {
        // Spec §2.2 retry-count math: with seed_relative=1e-6,
        // up_factor=10, max_diag=1.0, max_retries_per_iter=8 — the
        // sequence spans 1e-6 → 1e-6 × 10^7 = 10.0 over 8 retries
        // (retry 1 seeds; retries 2..8 each multiply by 10).
        let mut s = LmState::from_config(LmConfig::fork_b());
        assert_eq!(s.lambda(), 0.0);
        assert_eq!(s.retry_count(), 0);
        assert!(s.is_active());
        assert!(s.can_bump(1.0));

        // Retry 1 — seed branch (lambda * up_factor = 0; max with seed).
        s.on_non_pd(1.0);
        assert_eq!(s.lambda(), 1e-6);
        assert_eq!(s.retry_count(), 1);

        // Retries 2..8 — multiply by up_factor=10.
        for i in 2..=8 {
            s.on_non_pd(1.0);
            assert_eq!(s.retry_count(), i);
        }
        // Final λ = 1e-6 × 10^7 = 10.0.
        assert!(
            (s.lambda() - 10.0).abs() < 1e-12,
            "λ after 8 retries should be 10.0, got {}",
            s.lambda()
        );

        // After 8 retries, can_bump returns false (retry_count >=
        // max_retries_per_iter), regardless of ceiling headroom.
        assert!(!s.can_bump(1.0));
    }

    #[test]
    fn lambda_clamps_at_max_relative_ceiling() {
        // Spec §2.2 ceiling: λ_max = max_relative × max_diag. Construct
        // a tight-ceiling config (max_relative=1.0, max_diag=1.0 →
        // ceiling=1.0) where the second non-PD bump would jump above
        // the ceiling and verify clamp semantics. `can_bump` then
        // reads `lambda >= max_relative * max_diag` as the
        // saturation signal.
        let tight = LmConfig {
            seed_relative: 0.5, // seed = 0.5 → bump 2 would jump to 5.0
            up_factor: 10.0,
            down_factor: 0.5,
            max_relative: 1.0,
            max_retries_per_iter: 32, // un-cap retries; isolate clamp
        };
        let mut s = LmState::from_config(tight);
        // Seed: λ = 0.5, below ceiling=1.0, can_bump still true.
        s.on_non_pd(1.0);
        assert_eq!(s.lambda(), 0.5);
        assert!(s.can_bump(1.0));
        // Bump 2: 0.5 × 10 = 5.0, would exceed ceiling=1.0, clamp to 1.0.
        s.on_non_pd(1.0);
        assert_eq!(s.lambda(), 1.0);
        // At ceiling: can_bump returns false (lambda >= ceiling).
        assert!(!s.can_bump(1.0));
    }

    #[test]
    fn on_llt_success_decays_lambda_no_sticky_warm() {
        // Spec §2.2 decay rule: λ_min = 0.0 — λ allowed to decay all the
        // way to zero (no sticky-warm); next non-PD re-seeds. Sequence:
        // seed → decay × 0.5 repeatedly → still positive but tiny.
        let mut s = LmState::from_config(LmConfig::fork_b());
        s.on_non_pd(1.0); // λ = 1e-6
        let lambda_after_seed = s.lambda();
        s.on_llt_success(); // λ → 5e-7
        assert!(
            0.5_f64.mul_add(-lambda_after_seed, s.lambda()).abs() < 1e-18,
            "decay should halve λ exactly, got {} vs {}",
            s.lambda(),
            0.5 * lambda_after_seed
        );
        // Decay many times — λ stays positive but shrinks toward 0.
        for _ in 0..20 {
            s.on_llt_success();
        }
        assert!(s.lambda() > 0.0);
        assert!(s.lambda() < lambda_after_seed);
    }

    #[test]
    fn on_llt_success_is_noop_when_lambda_zero() {
        // Decay on λ = 0 must be a no-op (no NaN, no negative drift).
        let mut s = LmState::from_config(LmConfig::fork_b());
        assert_eq!(s.lambda(), 0.0);
        s.on_llt_success();
        assert_eq!(s.lambda(), 0.0);
    }

    #[test]
    fn on_llt_success_is_noop_on_disabled() {
        // Disabled state has no config; decay must short-circuit
        // without panic.
        let mut s = LmState::disabled();
        s.on_llt_success();
        assert_eq!(s.lambda(), 0.0);
    }

    #[test]
    fn from_config_with_seed_carries_lambda() {
        // IFT-adjoint factor seed path: Newton-final λ propagates into
        // the fresh adjoint state.
        let s = LmState::from_config_with_seed(LmConfig::fork_b(), 1e-3);
        assert_eq!(s.lambda(), 1e-3);
        assert_eq!(s.retry_count(), 0);
        assert!(s.is_active());
    }

    #[test]
    fn begin_factor_call_resets_retry_count_but_preserves_lambda() {
        // Cross-iter persistence invariant per spec §2.2 "per-iter vs
        // per-step state": λ is preserved across factor_free_tangent
        // calls (cross-Newton-iter), but retry budget is per-call.
        let mut s = LmState::from_config(LmConfig::fork_b());
        s.on_non_pd(1.0);
        s.on_non_pd(1.0);
        assert_eq!(s.retry_count(), 2);
        let lambda_before = s.lambda();
        s.begin_factor_call();
        assert_eq!(s.retry_count(), 0);
        assert_eq!(s.lambda(), lambda_before);
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
            crate::solver::SolverConfig::skeleton()
                .lm_regularization
                .is_none()
        );
    }
}
