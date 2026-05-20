# F3 — sim-soft Levenberg-Marquardt regularization — design spec

> **STATUS — FALSIFIED 2026-05-18 EVENING** (`sim-arc/sl-4-intruder-render`)
>
> F3.1 → F3.4 shipped; visual gate on iter-1 `sock_over_capsule.cleaned.stl`
> produced a non-monotone outcome that maps to none of §3's A/B/C/D
> categories. F3.4 LM opt-in **reverted on dev** (1-line revert in
> `tools/cf-device-design/src/insertion_sim.rs::insertion_solver_config`);
> F3.1 → F3.4 sim-soft + surface plumbing **KEPT** (architectural wins
> independent of the LM mental model).
>
> See **`docs/F3_FALSIFICATION_BOOKMARK.md`** for: three-gate data,
> three-failure-class mental-model update, future-arc candidate ladder
> (A gated-LM / B material-validity safe-step / C smoothed contact / D
> mesh refinement), recon recommendation.
>
> This spec is preserved AS WRITTEN for audit trail — it remains a
> correctly-designed implementation of an incorrectly-modeled failure
> class. The recon next session may invalidate the LM mechanism but
> the SolverFailure / try_step API surface defined here is sound and
> consumed by every candidate.

**Status (historical)**: DESIGN SPEC. No implementation yet. Per [[feedback-bookmark-when-surface-levers-exhaust]] three-session pattern (design-spec → implementation → visual gate): this is session 1. F3.1 + F3.2 land in session 2; cf-device-design opt-in + visual gate is session 3.

**Predecessor docs**:
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — original bookmark (Q1 bisection identified cavity-inset as the driver).
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — F4 + F2 falsifications. §10 carries the F3 design-question scaffold this doc fills out.
- `docs/F3_FALSIFICATION_BOOKMARK.md` — **this spec's empirical falsification, written same-session as F3.4** (2026-05-18 EVENING).
- Memory: [[project-cavity-inset-stall-bookmark]], [[project-f4-falsification-postmortem]], [[project-sl-4-arc-shipped]].

---

## 1. What F3 attacks (failure-class statement)

The cavity-inset stall (cavity > 3 mm in cf-device-design's sliding insertion ramp) manifests as Armijo line-search stalling at `α = 4.77e-7 = 0.5^21` (the `max_line_search_backtracks = 20` cap), with `r_norm` floored at ~0.55 N and ~14–30 `faer LU fallback fired` lines per failed step.

**F2's empirical falsification** (postmortem §10) established that κ scales the *magnitude* of every contact contribution by the same factor, leaving the *eigenstructure* of the tangent unchanged. Newton converges in half as many iters with κ halved, but the r_norm floor at which Armijo gives up moves only ~4% (0.572 → 0.551 — noise). The non-PD eigenvalues stay non-PD; the Newton step from `H_indef^{-1} · (-r)` keeps pointing into non-descent directions; Armijo keeps failing to find a step.

**The failure class F3 targets**: the assembled free-DOF tangent `A = M/dt² + K(x) + H_contact(x)` is indefinite at the current Newton iterate. The LU fallback produces *a* solution `δ = A^{-1} · (-r)`, but `δ` is not guaranteed to be a descent direction (the negative eigenvalues of `A` invert the projection of `-r` onto those modes). Armijo backtracking cannot rescue a non-descent direction — it can only shrink along the supplied direction.

**F3's lever**: add `λI` to the diagonal before factorization. For `λ > max|negative eigenvalue of A|`, the regularized tangent `A + λI` is SPD, the Llt factor succeeds, `δ_LM = (A + λI)^{-1} · (-r)` is *guaranteed* a descent direction (`δ_LM^T · (-r) > 0` for SPD systems), and Armijo finds a step. λ-adaptation lets us pay only as much regularization as needed — small λ in well-conditioned regions (near-Newton convergence), large λ near pathologies (steepest-descent fallback).

**What F3 does NOT attack**:
- **Active-set chattering**: if the set of active contact pairs changes between Newton iters, `H_contact(x)` is effectively discontinuous and *no* second-order method (regularized or not) converges quadratically through chattering boundaries. F5 (mesh refinement / smoothed contact functions) is the right tool for chattering. F3's response to chattering will be visible as λ saturating without resolving the stall (see §6 falsifier).
- **Sliding-mode load-class miscalibration**: F4.3 introduced a load class (uniform engineered interference at active vertices) the pre-F4 solver was never calibrated against and the F4 mental model assumed already existed. F3 inherits the corrected mental model from postmortem §4 and operates on the pre-F4 shrunk-scan model with pose-mismatch interference only.
- **Fork-A / production-solver semantics**: see §5.4. Fork-A is future work.

---

## 2. The five §10 design questions — answered

### 2.1 Where the `+λI` lands

**Site**: `sim/L0/soft/src/solver/backward_euler.rs`, function `factor_free_tangent` (line 710). The triplet input already contains diagonal `(k, k)` entries from the mass-diagonal scatter at line 1230 of `assemble_free_hessian_triplets` — so `+λI` is implemented by **cloning the triplets and mutating existing diagonal entries in place**, not by appending. Appending would fail: `SparseColMat::try_new_from_triplets` errors on duplicate triplets (per the comment at line 696-700; the BTreeMap accumulator at assembly time exists exactly to dedupe).

The new shape of `factor_free_tangent`:

```rust
fn factor_free_tangent(
    &self,
    triplets: &[Triplet<usize, usize, f64>],
    lm_state: &mut LmState,    // mutable per-Newton-iter LM state
    context: &str,
) -> FactoredFreeTangent {
    // Snapshot max_diag once per call (before retry loop). λ scaling is
    // relative to this. Walk triplets for entries with row == col;
    // O(n_triplets), negligible vs factor cost.
    let max_diag = triplets_max_diag(triplets);

    loop {
        // λ = 0 → borrow the original triplets (no clone, bit-equal to
        // pre-F3). λ > 0 → clone + mutate (k,k) entries in place.
        let regularized: Cow<[Triplet<_, _, _>]> = if lm_state.lambda > 0.0 {
            Cow::Owned(triplets_with_diagonal_offset(triplets, lm_state.lambda))
        } else {
            Cow::Borrowed(triplets)
        };
        match Llt::try_new_with_symbolic(self.symbolic.clone(), build_mat(&regularized), Side::Lower) {
            Ok(llt) => {
                lm_state.on_llt_success();   // decay λ (down_factor; floor at 0)
                return FactoredFreeTangent::Llt(llt);
            }
            Err(LltError::Numeric(_)) if lm_state.can_bump(max_diag) => {
                lm_state.on_non_pd(max_diag); // seed-or-bump λ, retry
                continue;
            }
            Err(LltError::Numeric(numeric_err)) => {
                // λ-saturated. Fall through to today's LU fallback on
                // the ORIGINAL (un-regularized) triplets — this preserves
                // backward-compat with the lm_regularization: None path
                // and makes saturation a true graceful-give-up (best step
                // the un-regularized LU can produce). DO NOT pass the
                // regularized triplets here; that would conflate "LM
                // saturation" with "LU on λ_max-regularized system" and
                // make the saturation surface a moving target.
                return self.lu_fallback(triplets, numeric_err, context);
            }
            Err(LltError::Generic(faer_err)) => panic!(...),
        }
    }
}

// Helper. Clones triplets, walks the clone, adds λ to each (k, k) entry
// in place. Preserves triplet ordering (the symbolic factor depends on
// it). O(n_triplets) clone + O(n_triplets) walk = O(n_triplets).
fn triplets_with_diagonal_offset(
    triplets: &[Triplet<usize, usize, f64>],
    lambda: f64,
) -> Vec<Triplet<usize, usize, f64>> {
    let mut out = triplets.to_vec();
    for t in &mut out {
        if t.row == t.col {
            t.val += lambda;
        }
    }
    out
}
```

The LU fallback path stays intact behind the saturation guard. When LM is disabled (`SolverConfig::lm_regularization: None`), `lm_state.lambda` is permanently 0 AND `lm_state.can_bump(max_diag)` is permanently `false`, so the function reduces bit-equally to today's code (try Llt on the borrowed-as-is triplets; on non-PD fall back to LU; on other LltError panic). This is the backward-compat invariant.

**`factor_at_position` (IFT adjoint) behavior**: by symmetry, `factor_at_position` also engages the LM retry loop — there is exactly one factor site (`factor_free_tangent`) and one behavior. The IFT adjoint runs at converged `x_final` on what is typically the SPD happy path (we just converged), so in practice `λ = 0` is the expected value and LM does not fire. The Newton-iter final `LmState` is snapshotted in `solve_impl` and passed to `factor_at_position` as a fresh `LmState` initialized from that final λ (small-but-non-zero λ values from late-iter LM activity carry into the adjoint factor for consistency). Internal API change: `factor_at_position(x_curr, dt)` becomes `factor_at_position(x_curr, dt, lm_seed_lambda)`; `solve_impl` passes the Newton-final λ. For consumers who don't care about the IFT adjoint (Fork-B / cf-device-design — see §2.4), this is harmless to correctness but adds minor cycle cost when LM happens to fire at the adjoint factor (rare; expected `λ = 0` short-circuits the retry loop).

**Why this site, not the assembly site**: `assemble_free_hessian_triplets` is the natural place to *inflate* the diagonal, but pushing `+λI` into the assembly means re-assembling the entire Hessian on every LM bump-and-retry (active-pair walk, per-tet tangent recompute, scatter — order-of-magnitude more expensive than the factor itself). Doing it inside `factor_free_tangent` keeps assembly out of the retry loop — the triplets are immutable input, only the diagonal modification is per-retry. With λ-bumps capped at 5-10 per Newton iter (see §2.2), the retry overhead is bounded by `max_retries_per_iter × (clone_triplets + Llt)`.

### 2.2 How λ adapts (Marquardt-style in-iter retry)

**Pattern**: classic Marquardt with multiplicative up/down adjustment.

| Event | Effect on λ |
|---|---|
| Llt succeeds, `λ > 0` | `λ *= down_factor` (decay toward 0) |
| Llt succeeds, `λ == 0` | unchanged (stay at 0; happy path) |
| Llt non-PD, `λ < λ_max` | `λ = max(λ * up_factor, λ_seed)`; refactor same iter |
| Llt non-PD, `λ ≥ λ_max` | fall through to LU + Armijo (saturation graceful-give-up) |

**Defaults** (provisional; tunable via `LmConfig`):
- `λ_seed = 1e-6 * max_diag_of_assembled_tangent` — first non-zero λ value (recomputed per `factor_free_tangent` call from current triplets' diagonal). Seed scales with the mass + element-stiffness order so it's geometry-aware. `max_diag` must be `> 0`; a `debug_assert!(max_diag > 0.0)` in `factor_free_tangent` fails fast on the impossible-case (empty triplets / zero-mass DOFs — both already excluded by the new() validity gates).
- `up_factor = 10.0` — aggressive bump applied to retries AFTER the seed retry; see retry-count math below.
- `down_factor = 0.5` — gentle decay; preserves quadratic convergence near the solution by not stomping λ to zero between iters.
- `λ_max = 1e3 * max_diag` — ceiling; above this, the regularized system is essentially `λ_max · I · δ = -r` (gradient descent with step size `1/λ_max`), which is worse than the existing LU fallback.
- `λ_min = 0.0` — decay floor. λ is allowed to decay all the way to zero (no sticky-warm policy); the next non-PD detection re-seeds from `λ_seed`. Simpler than maintaining a permanent floor and avoids the question of "is the warm λ helping or hurting after the active-set shifts."
- `max_retries_per_iter = 8` — backstop against infinite retry loops. With the rule `λ = max(λ * up_factor, λ_seed)`, retry 1 seeds (no multiplication), retries 2-8 each multiply by 10. So 8 retries spans `λ_seed → λ_seed × 10^7`.

**Numerical sanity of the defaults — back-of-envelope at cf-device-design's failure point**:
- Mass diagonal `mass / dt²` dominates `max_diag` at fine BCC meshes (~1e3 N/m per DOF at default cell-size + `dt = STATIC_DT`).
- `λ_seed ≈ 1e-3 N/m` at the cavity-inset failure case.
- Negative-eigenvalue magnitude from contact Hessian: `κ · ‖grad sd‖² ~ 1e3` per active pair at `INSERTION_CONTACT_KAPPA = 1e3`.
- To dominate that requires λ ≈ 1e3, which is 6 orders-of-magnitude above seed (1e-3 → 1e3). With the `seed + multiply` rule that's **7 retries** (1 seed + 6 multiplications by 10).
- With `max_retries_per_iter = 8`, this leaves **1 retry of headroom** — VERY TIGHT. If F3.2's empirical observation shows 8 retries hit saturation, the immediate response is to bump `up_factor` to `100.0` (4 mults instead of 6) or widen `seed_relative` to `1e-3` (1 order above the target, 4 mults headroom).

**Bidirectional tuning guidance** (F3.2 implementation will need to empirically tune the seed-scaling):
- If LM fires at the cavity = 3 mm baseline (where pre-F3 converges 16/16 with occasional LU-fallback firings) AND adds extra Newton iterations → `seed_relative` is too aggressive; lower it (e.g., 1e-8) so LM only activates for the harder cavity > 3 mm cases.
- If LM saturates at `λ_max` without rescuing cavity = 5 mm → `seed_relative` is too conservative (we run out of retry budget before reaching the needed regularization) OR `up_factor` is too small; either widen seed (1e-3 or 1e-2) or bump up_factor to 100.0. The visual-gate trace at F3.4 makes which one obvious from the saturation log.

The default `1e-6 · max_diag` is the principled starting point (six-orders-of-magnitude headroom below the dominant diagonal); if empirical results push it more than ±2 orders, that's a finding worth documenting at F3.5.

**Per-iter vs per-step state**: `LmState` lives for the duration of one `solve_impl` call (one time-step / one Newton solve). It is created at Newton-iter 0 with `λ = 0`, mutated by `factor_free_tangent` retries within an iter, and persists across Newton iters within the same `solve_impl`. λ does NOT reset between Newton iters — the decay rule (`down_factor` per Llt success, floor at `λ_min = 0`) handles the case where the problem becomes well-conditioned mid-solve.

**Why classic Marquardt over alternatives**:
- *Constant `λ`*: simpler but worse — small constant λ fails to rescue deep pathologies; large constant λ kills quadratic convergence on the easy iters.
- *Per-pivot-count adaptation*: counts the `NonPositivePivot` index from `LltError::Numeric` and bumps proportionally. Doable but the index isn't a structurally-meaningful quantity (it's the row at which the Cholesky factor first hits a negative pivot; the *number* of indefinite modes can be many more). The simpler up/down rule converges to the right λ in 1-2 retries empirically and is the textbook choice.
- *Trust-region with reduction ratio*: full TR adapts λ from the ratio of actual-to-predicted residual reduction. Most general; most code. Defer to future work if classic Marquardt isn't enough — `LmConfig::policy: AdaptationPolicy` makes this swap-in.

### 2.3 Backward-compat: λ = 0 default, opt-in

**The constraint**: every existing sim-soft test must pass *bit-equal* under F3's new code path. The hardest evidence is `tests/multi_element_isolation.rs` (lines 229-233), which asserts on exact iter counts; if F3 adds even one extra Newton iter on these tests, they break.

**The design**: `SolverConfig` gains one field:

```rust
pub struct SolverConfig {
    pub dt: f64,
    pub tol: f64,
    pub density: f64,
    pub max_newton_iter: usize,
    pub max_line_search_backtracks: usize,
    pub gravity_z: f64,
    /// Levenberg-Marquardt regularization for non-PD tangent rescue
    /// per F3 spec. `None` (default) preserves pre-F3 behavior bit-equal:
    /// `factor_free_tangent` tries `Llt` and on non-PD falls through to
    /// `Lu` immediately, no `+λI`. `Some(LmConfig)` opts into the
    /// in-iter Marquardt adapter; the `Lu` fallback then becomes the
    /// λ-saturation surface.
    pub lm_regularization: Option<LmConfig>,
}
```

**`SolverConfig::skeleton()` keeps `lm_regularization: None`** — preserves the existing 25+ test sites bit-equal without touching them. Only consumers that explicitly want LM (cf-device-design's `insertion_solver_config()`) set the field.

**The const-fn constraint**: `SolverConfig::skeleton()` is `const fn` (line 104). `Option<LmConfig>::None` is const-constructible, so this stays const-fn.

**`LmConfig`**:

```rust
#[derive(Clone, Copy, Debug)]
pub struct LmConfig {
    /// Seed λ at first non-PD detection. Computed as
    /// `seed_relative * max_diag_of_assembled_tangent`. Default 1e-6.
    pub seed_relative: f64,
    /// Multiplicative bump per non-PD retry. Default 10.0.
    pub up_factor: f64,
    /// Multiplicative decay per Llt success with λ > 0. Default 0.5.
    pub down_factor: f64,
    /// Ceiling as multiple of `max_diag`. Default 1e3.
    pub max_relative: f64,
    /// Max retries per Newton iter before saturation handoff. Default 8.
    pub max_retries_per_iter: usize,
    /// What happens when λ saturates AND the saturated LU step still
    /// Armijo-stalls. See §2.5.
    pub on_saturation: SaturationPolicy,
}

/// Per-stall policy applying ONLY to `SolverFailure::ArmijoStall`. The
/// other two `SolverFailure` variants (`NewtonIterCap`,
/// `DoublyFailedFactor`) always return `Err` on `try_step` and always
/// panic on `step`, regardless of this setting — there is no
/// "PanicOnIterCap" policy because the iter-cap failure is rare and
/// the existing test regression-net already pins on its panic.
///
/// The "saturation" in the name refers specifically to LM saturation
/// (retries exhausted at λ_max) followed by Armijo stall on the LU
/// fallback step (per §2.5 step 5b). With LM disabled, the policy
/// applies to first-Armijo-stall (today's only stall surface).
#[derive(Clone, Copy, Debug)]
pub enum SaturationPolicy {
    /// Today's behavior: panic with the existing Armijo-stall message
    /// (augmented to note LM-saturated when applicable). `Solver::step`
    /// always uses this regardless of LM enabled-ness; `try_step`
    /// dispatches to this when `PanicOnStall` is set.
    PanicOnStall,
    /// Used only by `Solver::try_step`. Returns
    /// `Err(SolverFailure::ArmijoStall { x_partial, last_iter, last_r_norm })`
    /// — the caller decides whether to accept the partial solve.
    /// `Solver::step` still panics on Armijo stall regardless of this
    /// setting; callers wanting graceful failure must use `try_step`.
    ReturnFailed,
}

impl LmConfig {
    pub const fn fork_b() -> Self {
        Self {
            seed_relative: 1e-6, up_factor: 10.0, down_factor: 0.5,
            max_relative: 1e3, max_retries_per_iter: 8,
            on_saturation: SaturationPolicy::ReturnFailed,
        }
    }
}
```

**Bit-equal verification (F3.1 acceptance gate)**: the concrete pass criterion is `cargo test -p sim-soft --release` returning green with no test modifications. If the F3.1 plumbing is truly inert at `lm_regularization: None` (the dispatch in `factor_free_tangent` falls through to today's exact path before touching any LM state), this passes trivially. Specifically inspect for regressions: `multi_element_isolation.rs` (exact iter counts at lines 229-233), `solver_convergence.rs` (iter count < 10 at line 58), `concentric_lame_shells.rs` (50-iter cap at line 568+). If ANY of these break, the F3.1 implementation is wrong — fix before F3.2 begins.

### 2.4 Fork-A vs Fork-B semantics

**Fork B (cf-device-design)**: opts in via `insertion_solver_config()` setting `lm_regularization = Some(LmConfig::fork_b())` AND switching from `Solver::step` to `Solver::try_step` at the insertion-ramp call sites. Saturation policy is `ReturnFailed`. The relative-comparison tool already tolerates loose `tol = 1e-1` and explicitly accepts "loose-but-physically-exact convergences" (`insertion_sim.rs:888`). On λ saturation, the insertion-step caller (`run_single_insertion_step`, `run_sliding_insertion_ramp`) receives an `Err(SolverFailure::ArmijoStall)` and decides whether to abort the ramp or accept the partial. Default behavior at insertion-ramp consumers: abort with the same anyhow-error propagation pattern as today's panic-catch path, but with no actual panic — the failure is structured data.

**Fork-B's `factor_at_position` is functionally dead**: cf-device-design does NOT consume Newton-step gradients (no `theta_var` flows downstream to a backward pass). The IFT adjoint factor at `x_final` runs as a no-op by-product of `Solver::step`'s tape-pushing contract; with LM enabled at Fork-B, the factor still happens, but its LM behavior is invisible to anything Fork-B cares about. F3.4's visual gate therefore doesn't need to validate IFT-adjoint LM semantics — they're inert in the cf-device-design test surface. (Fork-A consumers will care eventually; see below.)

**Fork A (sim-hard / production solver — every other sim-soft consumer)**: NOT in this spec. The `lm_regularization` field exists in `SolverConfig`; Fork-A consumers leave it `None` for now. When a production consumer eventually needs LM (e.g., the rows' growing ramp at extreme contact concentrations), the spec for Fork-A is a future doc that:
1. Decides on stricter saturation policy (`SaturationPolicy::PanicOnStall` is the natural default — production should not silently degrade).
2. Considers tighter `max_relative` (smaller ceiling — stop early on degenerate input rather than slide into pure gradient descent).
3. Validates the **numerical consistency** of gradient values computed through a non-zero-λ IFT adjoint factor. F3 already threads `lm_seed_lambda` through `factor_at_position` per §2.1 — the adjoint factor uses the Newton-final λ as its SEED, then runs its own retry loop if the adjoint tangent is also non-PD (so the actual λ at the adjoint factor may be ≥ the seed). The question Fork-A needs to answer is whether those gradients are USEFUL for backprop — i.e., does `(A + λI)^{-1}` produce an adjoint that's close enough to `A^{-1}` for downstream learning, or does the λI regularization bias the gradients in ways that break training. Numerical-consistency study, not API-shape. Fork-B doesn't consume gradients so doesn't surface this.

The reason Fork-A is deferred: the gradient-consuming consumers haven't surfaced an LM need yet, and the right design for them depends on what failure mode surfaces first. Speculating now risks designing for the wrong production case.

### 2.5 Convergence-failure surface

Three failure surfaces under F3 (with `LmConfig::Some(...)`):

1. **Llt-PD with λ = 0 + Newton converges**: identical to today. No code change visible.
2. **Llt-non-PD + λ bump succeeds + Newton converges**: new path. LM does its job. Visible only via the log policy below.
3. **Llt-non-PD + λ saturates AT `λ_max` + LU fallback fires**: the saturation surface, but NOT necessarily a terminal failure. Sequence:
   1. LM retries up to `max_retries_per_iter`.
   2. Saturation → fall through to LU on the original (un-regularized) triplets.
   3. LU produces a step δ (non-descent direction possible due to non-PD tangent).
   4. Armijo backtracks on δ.
   5a. **Armijo accepts a step** → Newton continues to next iter. LM saturated this iter; the solve isn't dead. This is the "best-effort recovery" path and is treated as success at the Newton-loop level.
   5b. **Armijo stalls** → terminal failure for this Newton iter. Behavior is governed by which `Solver` method the caller invoked AND by `SaturationPolicy`:
       - `Solver::step` (existing, infallible signature): always panics on Armijo stall regardless of `on_saturation`. Preserves today's panic-on-stall semantics; backward-compat for every existing caller.
       - `Solver::try_step` (new method, see "Public API change" below): dispatches on `on_saturation`. With `PanicOnStall`, behaves like `step`. With `ReturnFailed`, returns `Err(SolverFailure::ArmijoStall { x_partial, last_iter, last_r_norm })` for the caller to inspect.

The implementer should NOT confuse "saturation" (step 2-3, recoverable if Armijo accepts) with "Armijo stall after saturation" (step 5b, the actual terminal failure surface).

**Why graceful-give-up requires both opt-in AND a new method**: see §7 decision-trace entry on `try_step`. Short version: backward-compat of 30+ existing `Solver::step` callers requires keeping `step`'s signature stable; the new `try_step` keeps `step` bit-equal-compatible and forces graceful-failure consumers to make an explicit choice.

**Public API change** (the only public API change in this spec):

```rust
pub trait Solver: Send + Sync {
    type Tape;

    // Existing methods — unchanged signatures, panic-on-failure semantics preserved.
    fn step(...) -> NewtonStep<Self::Tape>;
    fn replay_step(...) -> NewtonStep<Self::Tape>;
    fn current_dt(&self) -> f64;
    fn convergence_tol(&self) -> f64;

    // NEW. REQUIRED method (no default impl). Each Solver impl must
    // explicitly choose its failure semantics: either forward to
    // `step` and panic (e.g., `Ok(self.step(...))` — explicit
    // panic-forwarder for impls that don't support LM), or implement
    // real graceful-failure dispatch on `SaturationPolicy`.
    //
    // Why no default: a default `Ok(self.step(...))` would mislead
    // callers — the signature says `Result` (implies fallible) but
    // the body panics. Forcing each impl to choose explicitly
    // prevents future Solver impls from silently inheriting the
    // wrong contract. Today only `CpuNewtonSolver` impls Solver, so
    // this requirement is trivial; `GpuNewtonSolver` (Phase E) will
    // explicitly choose its semantics at impl time.
    fn try_step(
        &mut self,
        tape: &mut Self::Tape,
        x_prev: &Tensor<f64>,
        v_prev: &Tensor<f64>,
        theta_var: Var,
        dt: f64,
    ) -> Result<NewtonStep<Self::Tape>, SolverFailure>;

    // Same shape, for replay path. Also required.
    fn try_replay_step(...) -> Result<NewtonStep<Self::Tape>, SolverFailure>;
}

#[derive(Debug)]
pub enum SolverFailure {
    /// Armijo line-search stalled after exhausting LM retries (or with
    /// LM disabled, on the first Armijo stall).
    ///
    /// `x_partial` is the `x_curr` at the START of the failed Newton
    /// iter — i.e., the best-known position from which Armijo found
    /// no descent step. NOT the most-recent `trial_x` from
    /// backtracking (that's a worse-residual rejected candidate);
    /// NOT a partial-Armijo-accepted x (no such thing — Armijo
    /// either accepts a step or stalls).
    ArmijoStall { x_partial: Vec<f64>, last_iter: usize, last_r_norm: f64 },
    /// Newton iter cap reached without converging. `x_partial` is the
    /// last `x_curr` at the moment the iter loop exited (i.e., after
    /// the last accepted Armijo step). The new Result variant of
    /// today's `panic!` at solve_impl line 902-909.
    NewtonIterCap { x_partial: Vec<f64>, max_iter: usize, last_r_norm: f64 },
    /// Doubly-failed factor (Llt non-PD AND LU also failed). This was
    /// already a model-level non-recoverable condition; surfacing it as
    /// `Err` rather than `panic!` is the consistent graceful-failure
    /// posture. `x_partial` is the `x_curr` at the start of the failed
    /// Newton iter (same semantics as ArmijoStall) — the factor failure
    /// happens during assembly + factor of the tangent at this position;
    /// the position itself is well-defined.
    DoublyFailedFactor { x_partial: Vec<f64>, last_iter: usize, context: String },
}
```

**`NewtonStep<T>` stays a struct** (today's shape at `solver/mod.rs:40-56`). `SolverFailure` is the new failure-carrier type; `NewtonStep` only ever represents a converged step (its existing invariant).

**`armijo_backtrack` changes**: returns `Result<Vec<f64>, ArmijoStall>` instead of panicking directly. `ArmijoStall` here is a **local struct** in `backward_euler.rs` carrying `(x_curr, iter, r_norm)` — distinct from the public `SolverFailure::ArmijoStall` variant. `solve_impl` translates the local struct back to `panic!` (preserving today's panic message; observable behavior unchanged); `try_solve_impl` (new mirror of `solve_impl` with `Result` return) translates to `Err(SolverFailure::ArmijoStall { x_partial: stall.x_curr, last_iter: stall.iter, last_r_norm: stall.r_norm })`. The internal API change is local; external callers of `step` see no change.

**Logging policy** (the per-retry warn-log surface):
- **First non-PD detection per Newton iter, λ transitioning from 0** logs `"sim-soft: LM seeded λ = {…} at Newton iter {…}"`.
- **First non-PD detection per Newton iter, λ already > 0** (from a previous iter's activity that hasn't fully decayed) logs `"sim-soft: LM bumped λ = {…} (was {…}) at Newton iter {…}"`.
- **Subsequent retries within the same Newton iter** are silent during the bump loop; the post-loop outcome line summarizes: `"sim-soft: LM converged in {N} retries to λ = {…} at Newton iter {…}"` (on Llt success) or `"sim-soft: LM saturated at λ = λ_max = {…} after {N} retries at Newton iter {…}; falling back to LU on un-regularized triplets"` (on saturation).
- **Decay events** (Llt success with λ > 0 after non-decay activity) are silent — the post-success log already captures the salient information.

This bounds total log volume at ~2 lines per Newton iter when LM is active, regardless of how many retries fire. At the cavity-inset failure case (~150 Newton iters per stalled step), that's a few hundred lines per failed step — comparable to today's LU fallback log noise.

---

## 3. Falsifier (when F3 doesn't work, what we learn)

The cleanest empirical test is the cavity = 5 mm visual gate in cf-device-design (the original failure that surfaced the cavity-inset stall bookmark). Four outcomes:

| Outcome | What it means | Next action |
|---|---|---|
| **A. F3 converges 16/16 at cavity = 5 mm AND cavity = 8 mm** | F3 nailed the failure class. Ship + lift the UI cap. | F3.5 polish + cap UI at validated ceiling. |
| **B. F3 converges 16/16 at cavity = 5 mm but stalls at cavity = 8 mm** | F3 helps but design-space ceiling lies between 5 and 8 mm. Ship with cap at validated boundary. | F3.5 + bookmark cavity = 8 mm separately as future arc (probably F5 mesh refinement, F1 adaptive-tol, or accept-the-limit). |
| **C. F3 reaches better r_norm but still Armijo-stalls at cavity = 5 mm** | Direction was the problem AND something else is. The "something else" is probably *active-set chattering* — r_norm oscillates between Newton iters rather than monotone-descending. Inspect the per-iter r_norm trace; if oscillation present, F5 (mesh refinement or smoothed contact) is the next-arc. | Bookmark + recon. |
| **D. F3 r_norm floor barely moves (same ~0.55 N)** | +λI didn't help the eigenstructure either — implies the failure is upstream of the tangent (e.g., the residual itself is poorly-scaled, or the active-set churn rate is so high that no second-order method can converge). | Bookmark + recon; candidate fixes are radical: smoothed contact functions, larger element refinement, or accept design-space cap at 3 mm and document. |

**Structural sufficiency analysis (banked for the cold-reader)**: rank-deficient negative eigenvalues are addressable by `+λI` (λ > max|neg-eigenvalue| dominates them); zero-row blocks are impossible by FEM construction (every free DOF gets non-zero mass + element contributions). The remaining structural concern is **active-set discontinuity** — the contact Hessian `H_contact(x)` jumps when an active pair switches on/off, and `+λI` cannot smooth a discontinuity. This is outcome C/D above. F3 is structurally sufficient *for the static-active-set non-PD case*; F3 is structurally insufficient *for the chattering case*. The visual gate distinguishes them.

**Outcome C/D triggers a recon arc, NOT in-session F3 iteration.** If the structural-sufficiency analysis above is wrong, the implementation session discovers it; the clean response is bookmark + recon (three-session pattern continues), not LM-tuning whack-a-mole.

---

## 4. Sub-leaf ladder

Sized for two follow-up sessions (implementation, visual gate) with explicit acceptance gates between each.

### F3.0 — design spec — THIS SESSION
- This doc.
- Commit + push to origin on branch `sim-arc/sl-4-intruder-render`.
- No code changes.

### F3.1 — `LmConfig` + `SaturationPolicy` + `SolverConfig::lm_regularization` field — session 2 (small)
- Add `LmConfig` + `SaturationPolicy` types in a new `sim/L0/soft/src/solver/lm.rs` (or inline in `backward_euler.rs` if it stays small; new module is cleaner).
- Add `SolverConfig::lm_regularization: Option<LmConfig>` field; default `None`. Const-fn preserved.
- **NO plumbing yet** — `factor_free_tangent` / `factor_and_solve_free` / `factor_at_position` / `solve_impl` signatures unchanged. F3.2 introduces the `&mut LmState` plumbing alongside the math. (Avoids the awkward intermediate state where signatures change but bodies are inert — that's the kind of state that invites premature merging.)
- Acceptance: `cargo test -p sim-soft --release` green; types compile + match the spec; trivially bit-equal since no execution path touches the new types.
- Sizing: ~80 LOC of new types + SolverConfig field. Very quick.

### F3.2 — Plumbing + `factor_free_tangent` LM retry loop + unit test — session 2 (medium)
- Add `LmState` internal struct (mutable in-solve state: current λ, retry count). `LmState::disabled()` returns a state where `lambda == 0.0` permanently and `can_bump(max_diag)` is permanently `false`.
- Plumb `&mut LmState` through `factor_free_tangent`, `factor_and_solve_free`, and `factor_at_position`. Initialize in `solve_impl` from `self.config.lm_regularization` (disabled state when `None`).
- Implement the §2.1 retry loop in `factor_free_tangent` (pseudocode is the source of truth; max_diag snapshot + `debug_assert!(max_diag > 0.0)` + `triplets_with_diagonal_offset` helper + Llt-retry-bump-saturate logic + LU fallback on original triplets when saturated).
- Implement the §2.5 logging policy — four distinct log forms total (seed-line, bump-line, success-summary-line, saturation-summary-line). This is non-trivial; budget ~30 LOC for the logging alone.
- Add a new unit test in `backward_euler.rs`'s `mod tests` mirroring `factor_free_tangent_falls_through_to_lu_on_non_pd` (the existing non-PD fixture, line 1570) but with LM enabled — assert that Llt returns `Ok` after the LM bump rather than falling through to LU. The existing non-PD test stays as the LM-disabled baseline.
- **No cavity visual gates here** — defer to F3.4 where Fork-B opt-in is wired up. Running F3.2's LM enabled at cavity = 3 mm would require either temporarily patching `insertion_solver_config()` (and reverting) or wiring a temp test fixture; both are clunky. The unit test on the existing non-PD fixture is sufficient F3.2 verification — visual gates belong at F3.4.
- Acceptance: existing LU fallback unit test passes bit-equal (internal-API plumbing changes are inert at `lm_regularization: None` — the `disabled()` state short-circuits all LM logic); new LM unit test passes (LM enabled, Llt-Ok after bump); all other sim-soft tests pass bit-equal (only `Solver::step`'s public signature is unchanged — test sites that call `step` see no signature change AND no behavior change).
- Sizing: ~280 LOC (plumbing ~50, retry loop + helpers ~150, logging ~30, unit test ~50). Most of the math complexity lives here.

**Note on `tests/contact_stability.rs`**: this is the natural F3 regression-test harbor — it already catches solver panics (`NonPositivePivot`, Armijo stall, Newton non-convergence) across a κ × cell_size scan grid. Once F3.3 lands and `try_step` is available, augment the scan harness with an LM-enabled mode and assert that all previously panicking grid points either (a) converge under LM or (b) return `Err(SolverFailure)` cleanly. Defer to F3.5 if F3.4 visual gate ships clean — at that point the scan harness becomes a permanent regression net.

### F3.3 — Decay rule + saturation policy + `try_step` + `SolverFailure` — session 2 (medium)
- Implement decay (`down_factor` on Llt success when λ > 0; floor at `λ_min = 0` per §2.2).
- Implement saturation: when retries exceed `max_retries_per_iter` OR `λ ≥ λ_max`, fall through to LU on the **original (un-regularized) triplets** (per §2.1) and let Armijo run. NOT terminal — if Armijo accepts the LU step, Newton continues normally (per §2.5 step 5a).
- Add `SolverFailure` enum (all three variants — `ArmijoStall`, `NewtonIterCap`, `DoublyFailedFactor`) + `Solver::try_step` + `Solver::try_replay_step` per §2.5 public-API change. Both `try_*` methods are REQUIRED (no default impl); `CpuNewtonSolver` implements them to dispatch.

  **Dispatch semantics** (per §2.5 `SaturationPolicy` docstring):
  - For `ArmijoStall`: `PanicOnStall` panics (forward to existing `step`/`replay_step`); `ReturnFailed` returns `Err(SolverFailure::ArmijoStall)`.
  - For `NewtonIterCap` + `DoublyFailedFactor`: `try_step` ALWAYS returns `Err` regardless of `SaturationPolicy`; `step` ALWAYS panics regardless of policy. There is no per-variant policy for these two.

  **Three mirror methods needed** (Result-returning variants of the existing panic-returning chain):
  - `try_factor_free_tangent(triplets, lm_state, context) -> Result<FactoredFreeTangent, SolverFailure::DoublyFailedFactor>` — mirror of `factor_free_tangent` (line 710) with the doubly-failed-factor panic at lines 749-757 translated to `Err`.
  - `try_factor_and_solve_free(triplets, r_full, newton_iter, r_norm, lm_state) -> Result<Vec<f64>, SolverFailure>` — mirror of `factor_and_solve_free` (line 787) routing through `try_factor_free_tangent`.
  - `try_solve_impl(x_prev, v_prev, theta, dt) -> Result<NewtonStep, SolverFailure>` — mirror of `solve_impl` (line 830) routing through `try_factor_and_solve_free` AND `armijo_backtrack`'s Result variant; translates the iter-cap panic (lines 902-909) to `Err(SolverFailure::NewtonIterCap)`.

- `armijo_backtrack` is REWRITTEN to return `Result<Vec<f64>, ArmijoStall>` (a local `ArmijoStall` struct in `backward_euler.rs`, distinct from `SolverFailure::ArmijoStall` the public variant). Both callers translate:
  - `solve_impl` (existing): `armijo_backtrack(...).unwrap_or_else(|stall| panic!(...))` — preserves today's panic message. Observable behavior unchanged from today even though the source-level body differs.
  - `try_solve_impl` (new): `armijo_backtrack(...).map_err(|stall| SolverFailure::ArmijoStall { x_partial: stall.x_curr, last_iter: stall.iter, last_r_norm: stall.r_norm })`.
- Acceptance: all existing tests still pass under `cargo test -p sim-soft --release` (bit-equal); the new LM-enabled-on-non-PD unit test from F3.2 still passes; new unit tests cover the three `SolverFailure` variants on `try_step`:
  - `ArmijoStall` × `PanicOnStall` (asserts panic) and × `ReturnFailed` (asserts `Err(SolverFailure::ArmijoStall)`).
  - `NewtonIterCap` on `try_step` (asserts `Err(SolverFailure::NewtonIterCap)` regardless of policy).
  - `DoublyFailedFactor` on `try_step` (asserts `Err(SolverFailure::DoublyFailedFactor)` regardless of policy).
- **Fixture construction guidance**: forcing saturation requires a deeply-non-PD tangent that exhausts the 8-retry budget. The existing `factor_free_tangent_falls_through_to_lu_on_non_pd` fixture (line 1570) rescues at retry 1 — too shallow. Construct a fixture matrix with many negative eigenvalues at magnitudes near `max_diag` (so `λ_seed` and `λ_seed × 10` are both insufficient). For NewtonIterCap: a `max_newton_iter = 2` SolverConfig on a problem that needs 5 iters to converge. For DoublyFailedFactor: a rank-deficient matrix where neither Llt nor Lu produces a valid factor.
- Sizing: ~200 LOC for the math + ~60 LOC for the trait additions + ~80 LOC for the unit tests (4 cases above + fixture construction). Bundle-able with F3.2 (and F3.1) in one implementation session if energy allows; split otherwise.

### F3.4 — cf-device-design opt-in + visual gate — session 3
- `insertion_sim.rs::insertion_solver_config()` adds `config.lm_regularization = Some(LmConfig::fork_b())`.
- Update **BOTH** insertion-ramp consumer functions — `run_single_insertion_step` AND `run_sliding_insertion_ramp` — to use `Solver::try_step` instead of `Solver::step` at each internal call site. Per §2.4, both are Fork-B consumers. Implementer should grep `solver.step` within both functions; today there are typically 1-2 call sites per function, but verify by inspection.
- Handle the full `SolverFailure` enum in the call-site error translation (not just `ArmijoStall`). Today's `Ok` arm — propagate the converged step up the call chain — is unchanged from how `Ok(self.step(...))` would be handled. Pattern:
  ```rust
  match solver.try_step(tape, &x_prev, &v_prev, theta_var, dt) {
      Ok(step) => step,  // unchanged from today: propagate up
      Err(SolverFailure::ArmijoStall { last_iter, last_r_norm, .. }) =>
          return Err(anyhow!("Armijo stall at iter {last_iter}, r_norm {last_r_norm:e}")),
      Err(SolverFailure::NewtonIterCap { max_iter, last_r_norm, .. }) =>
          return Err(anyhow!("Newton iter cap {max_iter} reached, r_norm {last_r_norm:e}")),
      Err(SolverFailure::DoublyFailedFactor { last_iter, context, .. }) =>
          return Err(anyhow!("Solver factor doubly failed at iter {last_iter}: {context}")),
  }
  ```
- Run cavity = 3 mm visual gate (baseline regression check).
- Run cavity = 5 mm visual gate (the load-bearing test).
- Run cavity = 8 mm visual gate (the F4.1 UI cap — does F3 lift it?).
- Decide outcome A/B/C/D per §3.
- **Acceptance**: cavity = 3 mm converges 16/16 within ≤ 1.5× baseline iter count (`max_newton_iter` budget for SL.4.3 baseline was ~30-40 iters per converged step; F3-enabled budget headroom is the 150 cap minus 60 = 90 iters of slack — generous). LU-fallback firings replaced by LM bumps in the trace (per §2.5 logging). Visual outcome at cavity = 5 mm is A/B (ship) or C/D (bookmark + recon).
- Sizing: ~40 LOC code (2-4 call sites × match block + opt-in field) + 3 manual visual-gate runs + outcome decision + (if A/B) F3.5 polish. Outcome-dependent on what comes next.

### F3.5 — post-ship polish — session 3 (only if outcomes A/B)
- Update `docs/CAVITY_INSET_STALL_BOOKMARK.md` §11 with F3 resolution.
- Update memory entries [[project-cavity-inset-stall-bookmark]] + [[project-f4-falsification-postmortem]] + this spec doc with shipped status.
- Adjust UI cap in cf-device-design slider (`sliding_ramp_panel.rs` or wherever F4.1 landed) to the validated ceiling.
- Cold-read pass per [[feedback-cold-read-review-post-ship]] — this is a post-IMPLEMENTATION cold-read of the F3.1-F3.4 diff, distinct from this spec's own pre-implementation cold-read passes (3b4ec375 + 6e65aaf6 + ab40683b + the pass-3 polish commit that lands this section). Different scope: spec passes refine the design artifact; F3.5 cold-read finds implementation bugs + doc drift + test gaps in the shipped code.

---

## 5. What this spec does NOT specify

- **Fork-A semantics** (§2.4). Deferred until a production consumer surfaces.
- **Per-element / per-pair LM regularization**. Only global `+λI` is in scope. Per-block LM is a research direction (e.g., LM only on contact-DOF blocks) that adds complexity; F3 punts.
- **Trust-region adaptation** (§2.2 alternatives). Classic Marquardt is the simplest pattern that addresses the failure class; trust-region is the natural next step IF Marquardt is insufficient at outcome B/C.
- **Multi-step LM state**: λ resets to 0 at each new `solve_impl` (each time-step / each sliding-ramp step). Per-step persistence (warm-starting LM from the previous step's converged λ) is a likely optimization but not in this spec.
- **The IFT adjoint's λ-consistency for backprop** (§2.4 Fork-A item 3). F3 wires `lm_seed_lambda` through `factor_at_position` (the API plumbing is done in this spec). What F3 does NOT validate is whether gradients computed through a non-zero-λ adjoint factor are numerically useful for downstream backprop training. That's a numerical-consistency study deferred to whenever a Fork-A consumer needs it.

---

## 6. Anchors

**Predecessor docs**:
- `docs/F4_FALSIFICATION_POSTMORTEM.md` §10 — the question scaffold this doc answers.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — Q1 bisection + design-space framing.
- `docs/SL_4_IN_PROGRESS_BOOKMARK.md` + [[project-sl-4-arc-shipped]] — sliding-intruder render arc + the SL.4.3 visual gate that established the cavity = 3 mm 16/16-converging baseline.

**Code sites**:
- `sim/L0/soft/src/solver/backward_euler.rs:710-766` — `factor_free_tangent` (the F3.2 modification site).
- `sim/L0/soft/src/solver/backward_euler.rs:787-809` — `factor_and_solve_free` (carries `LmState` through).
- `sim/L0/soft/src/solver/backward_euler.rs:830-910` — `solve_impl` (initializes `LmState` from `SolverConfig::lm_regularization`).
- `sim/L0/soft/src/solver/backward_euler.rs:924-975` — `armijo_backtrack` (returns `Result<Vec<f64>, ArmijoStall>`; existing `solve_impl` keeps `panic!` for back-compat, new `try_solve_impl` translates to `Err(SolverFailure::ArmijoStall)`).
- `sim/L0/soft/src/solver/mod.rs:40-56` — `NewtonStep<T>` struct (unchanged shape; F3 does NOT add a `Failed` variant — see §2.5 for the `SolverFailure` enum + `try_step` rationale).
- `sim/L0/soft/src/solver/backward_euler.rs:1148-1234` — `assemble_free_hessian_triplets` (the mass-diagonal scatter pattern `+λI` clones).
- `sim/L0/soft/src/solver/backward_euler.rs:1570+` — `factor_free_tangent_falls_through_to_lu_on_non_pd` test (the F3.2 unit-test scaffold).
- `tools/cf-device-design/src/insertion_sim.rs:899-905` — `insertion_solver_config()` (the F3.4 opt-in site).
- `tools/cf-device-design/src/insertion_sim.rs:907-915` — `INSERTION_CONTACT_KAPPA = 1.0e3` docstring (the rationale F3 inherits — Fork-B looser-but-physically-exact framing).

**Tests likely to surface bit-equality regressions** (F3.1 acceptance):
- `sim/L0/soft/tests/multi_element_isolation.rs:229-233` — exact iter count assertions.
- `sim/L0/soft/tests/solver_convergence.rs:58` — `step.iter_count < cfg.max_newton_iter` assertion.
- `sim/L0/soft/tests/concentric_lame_shells.rs` — 50-iter cap regression net.

**Natural F3 regression-test harbor** (F3.2 unit + F3.3 saturation + F3.5 long-term):
- `sim/L0/soft/tests/contact_stability.rs` — already a κ × cell_size scan harness catching solver panics (`NonPositivePivot`, Armijo stall, Newton non-convergence). Augment with an LM-enabled mode once F3.3 lands `try_step` + `SolverFailure`; assert all previously panicking grid points either converge under LM or return clean `Err(SolverFailure)`. Defer the augment to F3.5 if F3.4 visual gate ships clean; the scan harness becomes the permanent regression net.

**Memory references**:
- [[feedback-bookmark-when-surface-levers-exhaust]] — the three-session pattern.
- [[feedback-implement-measure-revert-pattern]] — F4 + F2 earned their keep; F3 doesn't because the blast radius is bigger.
- [[feedback-autonomous-architecture]] — spec authored autonomously per granted authority.
- [[project-cavity-inset-stall-bookmark]] — the original bookmark this arc closes (pending F3.4 outcome).

---

## 7. Decision trace (why this spec, not alternatives)

- **Why not `K + βM` mass-damping instead of `+λI`**: mass-damping is the standard rescue for transient time-stepping; sim-soft is quasi-static at `dt = 1e-2` with all damping already baked into the mass-diagonal scatter. Adding a separate `β` knob fights with the existing mass diagonal. `+λI` is the cleaner geometric-regularization knob.
- **Why not trust-region / Dogleg / Levenberg-Marquardt-with-reduction-ratio**: more general; more code; defer until classic Marquardt is empirically insufficient.
- **Why not "just lower κ further at cf-device-design"**: F2 falsified this — see postmortem §10. The existing `INSERTION_CONTACT_KAPPA = 1e3` stays load-bearing alongside F3: κ scales per-pair penalty magnitude (the residual force at any given interpenetration), F3 scales the regularization of the assembled tangent's eigenstructure. The two knobs do different jobs; F3 doesn't subsume κ tuning.
- **Why not "smoothed barrier contact (Hauser-Pal or IPC-style)"**: the right tool for active-set chattering, NOT non-PD tangent direction. F3 attacks the direction problem. If the visual gate falsifies F3 with outcome C/D, smoothed barriers become a candidate next-arc.
- **Why opt-in by default, not default-on**: bit-equality of the 25+ existing test regression net is non-negotiable. Opt-in is the only safe path. Once F3 ships and bakes in the field, the discussion of flipping default-on is a future polish arc.
- **Why a separate `LmConfig` type, not flat fields on `SolverConfig`**: encapsulation. `LmConfig` is its own design surface with 5-6 tunables; flattening them onto SolverConfig pollutes the latter's surface for a feature most consumers don't use. `Option<LmConfig>` keeps the gate clean.
- **Why classic Marquardt up/down rule, not constant-λ**: see §2.2 alternatives. Up/down adapts to the local conditioning; constant is either too small (no rescue at bad regions) or too large (kills quadratic convergence at good regions).
- **Why a new `try_step` trait method instead of changing `Solver::step`'s return to `Result`**: changing `step`'s return cascades through ~30+ existing test sites + insertion-ramp callers. Adding `try_step` keeps `step` bit-equal-compatible and forces graceful-failure consumers to make an explicit choice — both by setting `SaturationPolicy::ReturnFailed` AND by calling the new method. Two-key authentication for "I really want this solver to gracefully degrade." `try_step` is REQUIRED (no default impl) so future Solver impls can't silently inherit the wrong contract — a default `Ok(self.step(...))` would mislead callers since the signature says `Result` (implies fallible) but the body panics. See §2.5 for the dispatch details.

---

## 8. Next-session prep

When you (cold-reader) pick up F3.1:
1. Read this spec start-to-finish.
2. Read `docs/F4_FALSIFICATION_POSTMORTEM.md` §10 for the falsification context that motivates F3.
3. Skim `sim/L0/soft/src/solver/backward_euler.rs:680-810` (the `factor_free_tangent` + `factor_and_solve_free` neighborhood).
4. Start with F3.1 (plumbing only) — the bit-equal acceptance gate is the riskiest part. Confirm it before touching the math.
5. F3.2 + F3.3 can bundle into the same session as F3.1 if energy holds; the visual-gate session (F3.4) is its own beast — do not bundle.

Re-launch command for the visual gate (F3.4, session 3):
```
cargo run --release -p cf-device-design -- ~/scans/sock_over_capsule.cleaned.stl
```
(positional arg, not `--cleaned-stl`)

If during F3.1/F3.2 implementation a sub-decision contradicts this spec, **stop and bookmark** rather than improvise — the spec is the alignment artifact for this arc, and silent divergence loses the alignment.
