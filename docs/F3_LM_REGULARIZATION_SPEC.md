# F3 — sim-soft Levenberg-Marquardt regularization — design spec

**Status**: DESIGN SPEC. No implementation yet. Per [[feedback-bookmark-when-surface-levers-exhaust]] three-session pattern (design-spec → implementation → visual gate): this is session 1. F3.1 + F3.2 land in session 2; cf-device-design opt-in + visual gate is session 3.

**Predecessor docs**:
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — original bookmark (Q1 bisection identified cavity-inset as the driver).
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — F4 + F2 falsifications. §10 carries the F3 design-question scaffold this doc fills out.
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

**Site**: `sim/L0/soft/src/solver/backward_euler.rs`, function `factor_free_tangent` (line 710), as a 2-line addition cloned from the existing mass-diagonal scatter at line 1230 of `assemble_free_hessian_triplets`. The triplet container is already a `BTreeMap<(col, row), f64>` keyed on lower-triangle indices, so adding `λ` to each `(k, k)` entry is a direct insertion before the `Llt::try_new_with_symbolic` call.

The new shape of `factor_free_tangent`:

```rust
fn factor_free_tangent(
    &self,
    triplets: &[Triplet<usize, usize, f64>],
    lm_state: &mut LmState,    // <-- new: mutable per-iter LM state
    context: &str,
) -> FactoredFreeTangent {
    loop {
        let regularized = if lm_state.lambda > 0.0 {
            add_diagonal(triplets, lm_state.lambda, self.n_free)
        } else {
            Cow::Borrowed(triplets)
        };
        match Llt::try_new_with_symbolic(...regularized...) {
            Ok(llt) => {
                lm_state.on_llt_success();   // decay λ
                return FactoredFreeTangent::Llt(llt);
            }
            Err(LltError::Numeric(_)) if lm_state.can_bump() => {
                lm_state.on_non_pd();         // bump λ, retry
                continue;
            }
            Err(LltError::Numeric(numeric_err)) => {
                // λ-saturated: existing LU fallback + warn-log path.
                return self.lu_fallback(triplets, numeric_err, context);
            }
            Err(LltError::Generic(...)) => panic!(...),
        }
    }
}
```

The LU fallback path stays intact behind the saturation guard. When LM is disabled (`SolverConfig::lm_regularization: None`), `lm_state.lambda` is permanently 0 AND `lm_state.can_bump()` is permanently `false`, so the function reduces bit-equally to today's code (try Llt; on non-PD fall back to LU; on other LltError panic). This is the backward-compat invariant.

`factor_at_position` (the IFT-adjoint post-convergence factor at `x_final`) takes its own fresh `LmState` initialized from the converged Newton iter's final λ; in practice the IFT adjoint is on the SPD happy path (we just converged) and `λ = 0` is the expected value. The interface signature change is internal — `factor_at_position` is a private method.

**Why this site, not the assembly site**: `assemble_free_hessian_triplets` is the natural place to *inflate* the diagonal, but pushing `+λI` into the assembly means re-assembling the entire Hessian on every LM bump-and-retry. Doing it inside `factor_free_tangent` keeps the assembly cost out of the retry loop — the triplets are immutable input, only the diagonal modification is per-retry. With λ-bumps capped at 5-10 per Newton iter (see §2.2), the retry overhead is bounded.

### 2.2 How λ adapts (Marquardt-style in-iter retry)

**Pattern**: classic Marquardt with multiplicative up/down adjustment.

| Event | Effect on λ |
|---|---|
| Llt succeeds, `λ > 0` | `λ *= down_factor` (decay toward 0) |
| Llt succeeds, `λ == 0` | unchanged (stay at 0; happy path) |
| Llt non-PD, `λ < λ_max` | `λ = max(λ * up_factor, λ_seed)`; refactor same iter |
| Llt non-PD, `λ ≥ λ_max` | fall through to LU + Armijo (saturation graceful-give-up) |

**Defaults** (provisional; tunable via `LmConfig`):
- `λ_seed = 1e-6 * max_diag_of_assembled_tangent` — first non-zero λ value (computed once, cached on the solver). Seed scales with the mass + element-stiffness order so it's geometry-aware.
- `up_factor = 10.0` — aggressive bump; 5-10 retries reach `λ = 10^5 · λ_seed` which is enough to dominate any contact-Hessian negative eigenvalue we've seen in practice (κ = 1e3 contact Hessian contributions are O(1e3); mass diagonal is O(1e6) at typical mesh densities).
- `down_factor = 0.5` — gentle decay; preserves quadratic convergence near the solution by not stomping λ to zero between iters.
- `λ_max = 1e3 * max_diag` — ceiling; above this, the regularized system is essentially `λ_max · I · δ = -r` (gradient descent with step size `1/λ_max`), which is worse than the existing LU fallback.
- `max_retries_per_iter = 8` — backstop against infinite retry loops; with `up_factor = 10`, 8 retries spans `λ_seed → 1e8 · λ_seed`, more than enough to hit any sensible `λ_max`.

**Per-iter vs per-step state**: `LmState` lives for the duration of one `solve_impl` call (one time-step / one Newton solve). It is created at Newton-iter 0 with `λ = 0`, mutated by `factor_free_tangent` retries within an iter, and persists across Newton iters within the same `solve_impl`. λ does NOT reset between iters — the decay rule (`down_factor` per Llt success) handles the case where the problem becomes well-conditioned mid-solve.

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

#[derive(Clone, Copy, Debug)]
pub enum SaturationPolicy {
    /// Today's behavior: panic with the existing Armijo-stall message.
    Panic,
    /// Return a best-effort `NewtonStep::new_failed(x_curr, last_iter, last_r_norm)`
    /// — the caller decides whether to accept the partial solve.
    GiveUp,
}

impl LmConfig {
    pub const fn fork_b() -> Self {
        Self {
            seed_relative: 1e-6, up_factor: 10.0, down_factor: 0.5,
            max_relative: 1e3, max_retries_per_iter: 8,
            on_saturation: SaturationPolicy::GiveUp,
        }
    }
}
```

**Bit-equal verification (F3.1 acceptance gate)**: run `cargo test -p sim-soft --release` post-F3.1; every existing test passes without modification. Specifically inspect `multi_element_isolation.rs` (exact iter counts), `solver_convergence.rs` (iter count < 10), and `concentric_lame_shells.rs` (50-iter cap). If any test breaks, the dispatch in `factor_free_tangent` is wrong (it's not falling through to the `None` path bit-equally) — fix before F3.2.

### 2.4 Fork-A vs Fork-B semantics

**Fork B (cf-device-design)**: opts in via `insertion_solver_config()` setting `lm_regularization = Some(LmConfig::fork_b())`. Saturation policy is `GiveUp`. The relative-comparison tool already tolerates loose `tol = 1e-1` and explicitly accepts "loose-but-physically-exact convergences" (`insertion_sim.rs:888`). On λ saturation, the insertion-step caller (`run_single_insertion_step`, `run_sliding_insertion_ramp`) sees a non-converged step and decides whether to abort the ramp or accept the partial. Default behavior at insertion-ramp consumers: abort with the same anyhow-error pattern as today's `replay_step panicked at...` propagation, but no panic.

**Fork A (sim-hard / production solver — every other sim-soft consumer)**: NOT in this spec. The `lm_regularization` field exists in `SolverConfig`; Fork-A consumers leave it `None` for now. When a production consumer eventually needs LM (e.g., the rows' growing ramp at extreme contact concentrations), the spec for Fork-A is a future doc that:
1. Decides on stricter saturation policy (`SaturationPolicy::Panic` is the natural default — production should not silently degrade).
2. Considers tighter `max_relative` (smaller ceiling — stop early on degenerate input rather than slide into pure gradient descent).
3. Re-examines the IFT adjoint at `factor_at_position`: today the adjoint runs at `x_final` after Newton converged, so the assembled tangent is the converged one — the same tangent that Newton last factored without issue. If Newton converged via LM with `λ > 0` AT THE LAST ITER, the IFT adjoint's `factor_at_position` factor should use the same λ for consistency. This is a Fork-A concern because Fork-B does not consume gradients.

The reason Fork-A is deferred: the gradient-consuming consumers haven't surfaced an LM need yet, and the right design for them depends on what failure mode surfaces first. Speculating now risks designing for the wrong production case.

### 2.5 Convergence-failure surface

Three failure surfaces under F3 (with `LmConfig::Some(...)`):

1. **Llt-PD with λ = 0 + Newton converges**: identical to today. No code change visible.
2. **Llt-non-PD + λ bump succeeds + Newton converges**: new path. LM does its job. Visible only as warn-log lines `"sim-soft: LM bumped λ to {…} at Newton iter {…}"` (analogous to today's LU fallback log).
3. **Llt-non-PD + λ saturates AT `λ_max` + LU fallback fires + Armijo stalls**: the saturation surface. Behavior is governed by `SaturationPolicy`:
   - `Panic` (Fork-A default, also today's behavior with LM disabled): panic with the existing `"Armijo line-search stalled at Newton iter {…}"` message, augmented to note LM-saturated when applicable.
   - `GiveUp` (Fork-B default): `armijo_backtrack` returns a `Result<Vec<f64>, ArmijoStall>` instead of panicking; `solve_impl` returns a `NewtonStep::new_failed(x_curr, last_iter, last_r_norm)` for the caller to inspect.

**Why graceful-give-up is opt-in, not default**: today's panic policy on Armijo stall is documented as a "book-level finding" (scope §11 S-2) — the existing tests' regression-net depends on these panics being detectable failures. Flipping the default to "silently return" would risk masking real solver failures in production code. The opt-in flag forces explicit acknowledgment: "I want this solver to gracefully fail rather than panic" is a deliberate choice cf-device-design makes; sim-soft tests + future production consumers stay panic-on-stall by default.

**`NewtonStep` shape change**:

```rust
pub enum NewtonStep<T: Tape> {
    Converged { x_final: Vec<f64>, iters: usize, final_r_norm: f64, tape_entry: T::Entry },
    /// New variant added by F3 — only constructed when `SolverConfig::lm_regularization`
    /// is `Some` AND `on_saturation == GiveUp` AND the solve actually fails.
    Failed { x_partial: Vec<f64>, iters: usize, last_r_norm: f64, reason: FailureReason },
}
```

Today's `NewtonStep` has a single tape-entry payload; `Solver::step`'s return type changes from infallible to `Result<…, SolverFailure>` OR remains infallible but with a richer variant. The exact shape is an F3.3 implementation detail — the spec commitment is: the failure surface is structured (caller can inspect) and never silently produces a degraded `Converged` value.

**`armijo_backtrack` changes**: signature returns `Result<Vec<f64>, ArmijoStall>` instead of panicking directly. `solve_impl` translates to either `panic!` or `NewtonStep::Failed` based on `SolverConfig`. The internal API change is local; external callers (today `solve_impl` is the only caller) update with it.

---

## 3. Falsifier (when F3 doesn't work, what we learn)

The cleanest empirical test is the cavity = 5 mm visual gate in cf-device-design (the original failure that surfaced the cavity-inset stall bookmark). Four outcomes:

| Outcome | What it means | Next action |
|---|---|---|
| **A. F3 converges 16/16 at cavity = 5 mm AND cavity = 8 mm** | F3 nailed the failure class. Ship + lift the UI cap. | F3.5 polish + cap UI at validated ceiling. |
| **B. F3 converges 16/16 at cavity = 5 mm but stalls at cavity = 8 mm** | F3 helps but design-space ceiling lies between 5 and 8 mm. Ship with cap at validated boundary. | F3.5 + bookmark cavity = 8 mm separately as future arc (probably F5 mesh refinement, F1 adaptive-tol, or accept-the-limit). |
| **C. F3 reaches better r_norm but still Armijo-stalls at cavity = 5 mm** | Direction was the problem AND something else is. The "something else" is probably *active-set chattering* — r_norm oscillates between Newton iters rather than monotone-descending. Inspect the per-iter r_norm trace; if oscillation present, F5 (mesh refinement or smoothed contact) is the next-arc. | Bookmark + recon. |
| **D. F3 r_norm floor barely moves (same ~0.55 N)** | +λI didn't help the eigenstructure either — implies the failure is upstream of the tangent (e.g., the residual itself is poorly-scaled, or the active-set churn rate is so high that no second-order method can converge). | Bookmark + recon; candidate fixes are radical: smoothed contact functions, larger element refinement, or accept design-space cap at 3 mm and document. |

**The "structural sufficiency" question the user asked me to bake in**: I argued in my pre-spec note that rank-deficient negative eigenvalues are addressable (λ > max|neg-eigenvalue| dominates them) and zero-row blocks are impossible by FEM construction (every free DOF gets non-zero mass + element contributions). The remaining structural concern is **active-set discontinuity** — the contact Hessian `H_contact(x)` jumps when an active pair switches on/off, and `+λI` cannot smooth a discontinuity. This is outcome C/D above. F3 is structurally sufficient *for the static-active-set non-PD case*; F3 is structurally insufficient *for the chattering case*. The visual gate distinguishes them.

**Don't punt outcome C/D to implementation**: if the spec-time analysis above is wrong about active-set chattering being the limit, the implementation session will discover it. The clean response is to bookmark + recon, not to keep iterating on F3 LM tuning. Three-session pattern continues.

---

## 4. Sub-leaf ladder

Sized for two follow-up sessions (implementation, visual gate) with explicit acceptance gates between each.

### F3.0 — design spec — THIS SESSION
- This doc.
- Commit + push to origin on branch `sim-arc/sl-4-intruder-render`.
- No code changes.

### F3.1 — `SolverConfig` plumbing + `LmConfig` + `LmState` types — session 2 (small)
- Add `SolverConfig::lm_regularization: Option<LmConfig>` field; default `None`. Const-fn preserved.
- Add `LmConfig` + `SaturationPolicy` types in a new `sim/L0/soft/src/solver/lm.rs` (or inline in `backward_euler.rs` if it stays small; new module is cleaner).
- Add `LmState` internal struct (mutable in-solve state: current λ, retry count, cached `max_diag` snapshot for λ scaling). `LmState::disabled()` returns a state where `lambda == 0.0` permanently and `can_bump()` is `false`.
- Plumb `&mut LmState` through `factor_and_solve_free` and `factor_at_position` callers. Initialize in `solve_impl` from `self.config.lm_regularization`.
- NO behavior change yet. `factor_free_tangent` ignores the new param. All existing tests must pass bit-equal.
- Acceptance: `cargo test -p sim-soft --release` green; specifically `multi_element_isolation.rs` exact iter counts unchanged.
- Sizing: ~150 LOC across SolverConfig + new module + plumbing. Quick.

### F3.2 — `factor_free_tangent` LM retry loop + unit test — session 2 (medium)
- Implement the §2.1 retry loop in `factor_free_tangent`. `add_diagonal` helper (`Cow<[Triplet]>` returning either the borrowed input when λ == 0 or an owned vec with `+λI` injected).
- Pre-compute `max_diag` once per Newton iter (the seed scaling). Storage: `LmState::seed_lambda` cached on first computation, refreshed at each Newton iter (the diagonal magnitudes shift with x).
- Add a new unit test in `backward_euler.rs`'s `mod tests` mirroring `factor_free_tangent_falls_through_to_lu_on_non_pd` (the existing non-PD fixture, line 1570) but with LM enabled — assert that Llt returns `Ok` after the LM bump rather than falling through to LU. The existing non-PD test stays as the LM-disabled baseline.
- Run cavity = 3 mm visual gate manually (the baseline that converged 16/16 pre-F4): the trace should be bit-equal-or-better; if LM fires *at all* at the baseline, the seed/up_factor are mistuned.
- Acceptance: existing LU fallback test passes (LM disabled); new LM test passes; cavity = 3 mm still converges 16/16 with LM enabled (Fork-B preset).
- Sizing: ~200 LOC. Most of the math complexity lives here.

### F3.3 — Decay rule + saturation policy + `NewtonStep::Failed` — session 2 (medium)
- Implement decay (`down_factor` on Llt success when λ > 0).
- Implement saturation: when retries exceed `max_retries_per_iter` OR `λ ≥ λ_max`, dispatch on `SaturationPolicy::Panic | GiveUp`. `GiveUp` requires `armijo_backtrack` returning a `Result` and `solve_impl` translating into `NewtonStep::Failed`.
- Update `Solver::step` / `Solver::replay_step` return type — Result-or-richer-NewtonStep. Choice depends on whether the trait's existing Solver consumers can tolerate a Result. Today only `CpuNewtonSolver` impls Solver and only `insertion_sim.rs` consumers + the tests call it. If changing the trait to `Result` cascades, add a `try_step` method instead and leave `step` as the panic-fallback for backward-compat.
- Acceptance: all existing tests still pass; saturation-policy unit test (force a degenerate matrix, assert `Panic` panics and `GiveUp` returns `NewtonStep::Failed`).
- Sizing: ~150 LOC plus consumer updates. Bundle-able with F3.2 in one implementation session if energy allows; can be split into a second implementation session if needed.

### F3.4 — cf-device-design opt-in + visual gate — session 3
- `insertion_sim.rs::insertion_solver_config()` adds `config.lm_regularization = Some(LmConfig::fork_b())`.
- Update `run_sliding_insertion_ramp` to handle `NewtonStep::Failed` gracefully (anyhow-error propagation, no panic).
- Run cavity = 3 mm visual gate (baseline regression check — must still converge 16/16).
- Run cavity = 5 mm visual gate (the load-bearing test).
- Run cavity = 8 mm visual gate (the F4.1 UI cap — does F3 lift it?).
- Decide outcome A/B/C/D per §3.
- Sizing: ~20 LOC code + 3 manual visual-gate runs. Outcome-dependent on what comes next.

### F3.5 — post-ship polish — session 3 (only if outcomes A/B)
- Update `docs/CAVITY_INSET_STALL_BOOKMARK.md` §11 with F3 resolution.
- Update memory entries [[project-cavity-inset-stall-bookmark]] + [[project-f4-falsification-postmortem]] + this spec doc with shipped status.
- Adjust UI cap in cf-device-design slider (`sliding_ramp_panel.rs` or wherever F4.1 landed) to the validated ceiling.
- Cold-read pass per [[feedback-cold-read-review-post-ship]].

---

## 5. What this spec does NOT specify

- **Fork-A semantics** (§2.4). Deferred until a production consumer surfaces.
- **Per-element / per-pair LM regularization**. Only global `+λI` is in scope. Per-block LM is a research direction (e.g., LM only on contact-DOF blocks) that adds complexity; F3 punts.
- **Trust-region adaptation** (§2.2 alternatives). Classic Marquardt is the simplest pattern that addresses the failure class; trust-region is the natural next step IF Marquardt is insufficient at outcome B/C.
- **Multi-step LM state**: λ resets to 0 at each new `solve_impl` (each time-step / each sliding-ramp step). Per-step persistence (warm-starting LM from the previous step's converged λ) is a likely optimization but not in this spec.
- **The IFT adjoint's λ-consistency** (§2.4 Fork-A discussion). Today `factor_at_position` runs at converged `x_final` on the SPD happy path with `λ = 0`; if a Fork-A consumer eventually surfaces a case where the converged tangent at `x_final` is itself non-PD (requiring LM at the adjoint), spec that then.

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
- `sim/L0/soft/src/solver/backward_euler.rs:924-975` — `armijo_backtrack` (returns `Result` under `GiveUp` policy).
- `sim/L0/soft/src/solver/backward_euler.rs:1148-1234` — `assemble_free_hessian_triplets` (the mass-diagonal scatter pattern `+λI` clones).
- `sim/L0/soft/src/solver/backward_euler.rs:1570+` — `factor_free_tangent_falls_through_to_lu_on_non_pd` test (the F3.2 unit-test scaffold).
- `tools/cf-device-design/src/insertion_sim.rs:899-905` — `insertion_solver_config()` (the F3.4 opt-in site).
- `tools/cf-device-design/src/insertion_sim.rs:907-915` — `INSERTION_CONTACT_KAPPA = 1.0e3` docstring (the rationale F3 inherits — Fork-B looser-but-physically-exact framing).

**Tests likely to surface bit-equality regressions** (F3.1 acceptance):
- `sim/L0/soft/tests/multi_element_isolation.rs:229-233` — exact iter count assertions.
- `sim/L0/soft/tests/solver_convergence.rs:58` — `step.iter_count < cfg.max_newton_iter` assertion.
- `sim/L0/soft/tests/concentric_lame_shells.rs` — 50-iter cap regression net.

**Memory references**:
- [[feedback-bookmark-when-surface-levers-exhaust]] — the three-session pattern.
- [[feedback-implement-measure-revert-pattern]] — F4 + F2 earned their keep; F3 doesn't because the blast radius is bigger.
- [[feedback-autonomous-architecture]] — spec authored autonomously per granted authority.
- [[project-cavity-inset-stall-bookmark]] — the original bookmark this arc closes (pending F3.4 outcome).

---

## 7. Decision trace (why this spec, not alternatives)

- **Why not `K + βM` mass-damping instead of `+λI`**: mass-damping is the standard rescue for transient time-stepping; sim-soft is quasi-static at `dt = 1e-2` with all damping already baked into the mass-diagonal scatter. Adding a separate `β` knob fights with the existing mass diagonal. `+λI` is the cleaner geometric-regularization knob.
- **Why not trust-region / Dogleg / Levenberg-Marquardt-with-reduction-ratio**: more general; more code; defer until classic Marquardt is empirically insufficient.
- **Why not "just lower κ further at cf-device-design"**: F2 falsified this — see postmortem §10.
- **Why not "smoothed barrier contact (Hauser-Pal or IPC-style)"**: the right tool for active-set chattering, NOT non-PD tangent direction. F3 attacks the direction problem. If the visual gate falsifies F3 with outcome C/D, smoothed barriers become a candidate next-arc.
- **Why opt-in by default, not default-on**: bit-equality of the 25+ existing test regression net is non-negotiable. Opt-in is the only safe path. Once F3 ships and bakes in the field, the discussion of flipping default-on is a future polish arc.
- **Why a separate `LmConfig` type, not flat fields on `SolverConfig`**: encapsulation. `LmConfig` is its own design surface with 5-6 tunables; flattening them onto SolverConfig pollutes the latter's surface for a feature most consumers don't use. `Option<LmConfig>` keeps the gate clean.
- **Why classic Marquardt up/down rule, not constant-λ**: see §2.2 alternatives. Up/down adapts to the local conditioning; constant is either too small (no rescue at bad regions) or too large (kills quadratic convergence at good regions).

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
