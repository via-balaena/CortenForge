# F3 recon — candidate A — gated LM — design spec

> **STATUS — DRAFT, no implementation yet.** Recon-iter-1 of the F3
> falsification bookmark's §4 four-candidate ladder. Picks candidate A
> (gated LM) per `docs/F3_FALSIFICATION_BOOKMARK.md` §5 recommendation
> ("the natural first stop"). Implementation lands same session per
> [[feedback-implement-measure-revert-pattern]] (≲ 100 LOC, scope is
> tight enough to implement-measure-revert in-session).

**Predecessor docs**:
- `docs/F3_FALSIFICATION_BOOKMARK.md` — the falsification bookmark this
  spec acts on (§1 TL;DR, §2 three-failure-class mental model,
  §4 four-candidate ladder, §6 per-gate verbatim stderr observations).
- `docs/F3_LM_REGULARIZATION_SPEC.md` — the FALSIFIED F3 spec. §F3.1
  bit-equal-when-dormant contract + §2.5 `SolverFailure` /
  `Solver::try_step` API surface are KEPT — this spec inherits both.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — the original arc all
  candidates target.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — F4 + F2 falsifications. §10
  scored F3 as "the only remaining candidate" pre-empirical-test;
  candidate A is F3's mechanism narrowed via the falsification.

**Predecessor memory**: [[project-f3-falsification-bookmark]],
[[project-f3-lm-regularization-spec]],
[[project-cavity-inset-stall-bookmark]],
[[project-f4-falsification-postmortem]], [[project-sl-4-arc-shipped]].

---

## TL;DR

F3 (always-on LM) **broke the cavity = 3 mm baseline** (16/16 → 0/16,
Armijo stall iter 10 r_norm = 1.048e-1 just 5% above tol = 0.1) by
replacing pre-F3's adequate LU-fallback step with a large-λ LM step that
is effectively gradient descent (at λ = 3.17e3, the LM step's
linearized residual decrease falls below Armijo's `c1 = 1e-4`
tolerance — see §2.1 for the math).

**Gated A's mechanism**: at each Newton iter, first try the pre-F3 LU
fallback step + Armijo. If Armijo accepts, continue (the cavity = 3 mm
baseline path — bit-equal to pre-F3 through to convergence). If Armijo
stalls on the LU step, escalate: re-factor with LM bumped λ, retry the
solve + Armijo. λ persists across escalation iters per the F3 spec
§2.2 "no sticky-warm" decay rule.

The empirical bet: at cavity = 3 mm the LU + Armijo path completes
unaided (no escalation, bit-equal to pre-F3 16/16). At cavity = 5 mm
the LU + Armijo path stalls at the pre-F3 Run 2 floor (r_norm = 1.78
after a short ~5-iter trajectory per `docs/CAVITY_INSET_STALL_BOOKMARK.md`
§8 Run 2 — `~57 → 30 → 10 → 2 → plateau at 1.78`) and gated LM
escalates at that stall point. If LM can produce an Armijo-accepting
step there, the run extends past pre-F3's stall and may reach tol. If
LM can't escalate (class 2 chattering dominates the late-iter
geometry), gated A degrades gracefully to pre-F3 stall behavior at
cavity = 5 mm.

Cavity = 8 mm is **OUT OF SCOPE** for this spec — class 3 (Yeoh material
validity) is orthogonal to A. If A ships, the UI cap is lowered to 5 mm
per §6 A.4. Candidate B addresses class 3 separately.

---

## 1. The three failure classes (inherited mental model)

Direct lift from the falsification bookmark §2. Repeated here so this
spec is self-contained for the cold reader.

1. **Indefinite tangent / non-descent direction** — assembled
   `A = M/dt² + K(x) + H_contact(x)` is indefinite at the Newton
   iterate; the LU fallback step `δ = A⁻¹ · (-r)` is not guaranteed a
   descent direction (negative eigenvalues invert the projection of `-r`
   onto those modes). F3's `+λI` makes `A + λI` SPD; the LM step is a
   descent direction. **Candidate A attacks this class.**

2. **LM-vs-LU trajectory drift at well-conditioned regions** — NEW
   class created by F3's eager activation. At cavity = 3 mm the pre-F3
   LU step was adequate for loose-tol convergence in ~30 Newton iters.
   F3.4 replaced every Llt-non-PD LU step with an LM step, putting
   Newton on a slower-descent trajectory that didn't bridge the loose
   tol within the iter cap. **Candidate A REMOVES this class** by
   keeping LM dormant on the well-conditioned baseline.

3. **Yeoh material validity at deep insets** — pre-existing, exposed at
   cavity = 8 mm by F3 enabling deeper Newton walks. Phase 4 Decision Q
   fail-closed `validity violation at tet 1324: max_stretch_deviation =
   1.209 exceeds bound 1.000`. Geometry-inherent (cavity wall must
   stretch 2.2× somewhere at 8 mm inset); orthogonal to LM mechanism.
   **Candidate A does NOT address this class** — candidate B
   (material-validity safe-step) is the orthogonal arc.

---

## 2. Mechanism — gated-on-Armijo-failure LM

### 2.1 Why F3's eager activation broke baseline (math context)

F3's LM step is `δ_LM = (A + λI)⁻¹ · (-r)`. The Armijo decrease
condition is:

```
‖r(x + α · δ_LM)‖ ≤ (1 − α · c1) · ‖r(x)‖
```

Linearizing `r(x + α · δ_LM) ≈ r(x) + α · A · δ_LM`:

```
A · δ_LM = A · (A + λI)⁻¹ · (-r) = -(I − λ · (A + λI)⁻¹) · r
       = -r + λ · (A + λI)⁻¹ · r
```

So `‖r(x + α · δ_LM)‖ ≈ ‖(1 − α) · r + α · λ · (A + λI)⁻¹ · r‖`.

When λ is large enough that `(A + λI) ≈ λ · I` (the regime F3 enters at
`λ = 3.17e3` per Gate A trace), `(A + λI)⁻¹ ≈ λ⁻¹ · I`, so
`λ · (A + λI)⁻¹ ≈ I`. The Armijo trial residual becomes:

```
‖(1 − α) · r + α · I · r‖ = ‖r(x)‖    (independent of α)
```

Armijo's RHS `(1 − α · c1) · ‖r‖` is strictly less than `‖r‖` for
α > 0 and c1 > 0 (`c1 = 1e-4` in sim-soft), so **Armijo can never
accept a large-λ LM step**: the linearized residual decrease is zero,
Armijo's required decrease is positive, the condition fails for every α.

This is the structural mechanism behind the Gate A iter-10 stall: F3.4
seeded `λ = 3.17e-2` and bumped to `λ = 3.17e3` (6 retries) at iter 1;
subsequent iters 2-9 were Llt-PD (no LM activity, but `λ` decayed from
3.17e3 by `down_factor = 0.5` per success → 3.17e3 × 0.5⁹ ≈ 6.2 N/m
by iter 10's factor call). At iter 10, λ ≈ 6.2 acts as a per-DOF
diagonal additive. For eigenmodes of A with eigenvalue ≫ 6.2 (the
stiff Yeoh + dense-mesh modes), the LM step approximates the Newton
step and Armijo can accept. For eigenmodes of A with eigenvalue ≲ 6.2
(the soft contact-DOF modes where conditioning was already poor at
this near-converged iterate, r_norm = 1.05e-1), the LM step is in the
gradient-descent regime per the above math and Armijo cannot accept.
The Armijo backtrack tests the WHOLE step at each α; if any
significant fraction of r is in the soft modes, the trial residual
stays close to ‖r‖ — Armijo stalls.

Pre-F3 at the same Gate A iter 10: LU fallback step on the (assumed
PD by iter 10 — gate A's silence at iters 2-9 confirms this) tangent
is a real Newton update. Armijo accepts with α = 1 or near-1. Newton
proceeds to convergence in ~30-40 iters.

### 2.2 Gated activation rule

**Per-Newton-iter algorithm** (replaces the body of `try_solve_impl`'s
inner iter, lines 1411-1457 of `backward_euler.rs`):

```
1. Assemble triplets at x_curr.
2. Factor + solve with LM SUPPRESSED → δ_LU.
   (LM-suppressed = `LmState::disabled()` semantics: Llt then direct
   LU fallback on non-PD, no `+λI` retry. Per spec §F3.1 this is
   bit-equal to pre-F3.)
3. Try Armijo on δ_LU.
4a. Armijo accepts → x_curr ← accepted iterate, advance to next iter.
    LM state is untouched (lambda stays at its persistent value, which
    is 0 at the start of solve and decays toward 0 between escalations).
4b. Armijo stalls → ESCALATE:
    4b.i.   Re-factor + re-solve with LM ENABLED (using the
            persistent lm_state, which seeds-or-bumps λ on this
            iter's first non-PD detection).
            δ_LM falls out.
    4b.ii.  Try Armijo on δ_LM.
            If accept → x_curr ← accepted iterate. Advance to next iter.
            If stall → SolverFailure::ArmijoStall (no further escalation
            this iter — already in the LM-rescue regime).
```

**Persistence of λ across escalation iters**: per F3 spec §2.2,
`LmState::lambda` persists across `factor_free_tangent` calls within
one `solve_impl`. The decay rule (`down_factor` on Llt success when λ >
0) handles well-conditioned mid-solve regions. The gated variant
preserves this: between escalations, λ decays toward 0 via the same
mechanism (each Llt-PD iter inside the LM-enabled retry loop decays λ).

**Cross-iter behavior** at cavity = 3 mm Gate A baseline-restoration:
- Iter 1: step 2 (LU + Armijo). Iter 1's Llt is non-PD (per Gate A log),
  so the LU fallback fires (one `"sim-soft: faer LU fallback fired"`
  stderr line — same as pre-F3). Armijo accepts the LU step (the
  bookmark §6 Gate A note "iters 2-9 silent — Llt-PD throughout"
  implies pre-F3's iter-1 LU step was accepted; gated A inherits this).
  No escalation. λ stays 0.
- Iters 2-9: Llt-PD, no LU activity, Armijo accepts. λ stays 0.
- Iter 10+: same as pre-F3 — Newton converges in ~30-40 iters.

**Cross-iter behavior** at cavity = 5 mm Gate B (the question A
empirically tests). The **apples-to-apples pre-F3 baseline** is
`docs/CAVITY_INSET_STALL_BOOKMARK.md` §8 Run 2 (cavity = 5 mm +
layers 10+3 mm — default layers, isolated cavity-only change), NOT
the §1b "original failure" trace (cavity = 5 mm + layers 5+5 mm,
which stalls at iter 63 r_norm 0.572 by way of a different mechanism
— more silicone in active contact at thinner Layer 0 creates a
different conditioning surface). The F3 falsification bookmark §6
Gate B's "0.55 N pre-F3" reference inadvertently quoted the §1b
layers-changed case; the correct cavity-only baseline is Run 2's
**r_norm = 1.78** after a `~57 → 30 → 10 → 2 → 1.78` ~5-iter
trajectory.

- Iters 0..N (N ≈ 4-5 per Run 2 trajectory): pre-F3 LU + Armijo
  (escalation never triggers — pre-F3 trace shows Armijo accepting
  through these iters even though Llt is non-PD every iter). Bit-equal
  to pre-F3.
- Iter N+1: pre-F3 Armijo stall at r_norm ≈ 1.78. Gated A escalates:
  factor with LM (lm_state's persistent λ seeds), get δ_LM, try Armijo.
- If LM step at iter N+1 is acceptable to Armijo → x_curr advances.
  λ persists; iter N+2 tries LU + Armijo first.
  - If iter N+2 LU + Armijo passes → λ decays; gated returns to dormant.
  - If iter N+2 LU + Armijo stalls → escalate again, λ bumps further.
- If LM step at iter N+1 is NOT acceptable → SolverFailure::ArmijoStall.
  Gated A failed to rescue; we know A's mechanism doesn't address
  class 2's late-iter chattering geometry.

The F3.4 always-on Gate B trace (r_norm floor 0.27 across 34 iters)
is **NOT directly predictive** of gated A's outcome — F3.4 took a
different Newton trajectory from iter 1 onward (always-on LM steps);
gated A's trajectory matches pre-F3 through iter N then escalates at
iter N+1. The empirical question is whether late escalation from
r_norm 1.78 dips below tol = 0.1.

### 2.3 Where the code change lands

**`sim/L0/soft/src/solver/backward_euler.rs::try_solve_impl`** (lines
1367-1464). The inner Newton-iter body is restructured per §2.2's
algorithm. Today's call sequence:

```rust
let delta_free = self.try_factor_and_solve_free(
    &triplets, &r_full, newton_iter, r_norm, &mut lm_state
).map_err(|info| SolverFailure::DoublyFailedFactor { ... })?;
x_curr = self.armijo_backtrack(
    &x_curr, ..., &delta_free, r_norm, newton_iter
).map_err(|stall| SolverFailure::ArmijoStall { ... })?;
```

New shape (only `try_solve_impl` body changes; the called methods stay
intact):

```rust
// Disabled LM state for the first attempt — bit-equal to pre-F3.
let mut lm_disabled = LmState::disabled();
let delta_lu = self.try_factor_and_solve_free(
    &triplets, &r_full, newton_iter, r_norm, &mut lm_disabled
);

// Try LU + Armijo first (gated A's "first pass"). Failure ALWAYS
// escalates regardless of which surface tripped (Armijo-stall on a
// valid LU step OR DoublyFailedFactor from try_factor_and_solve_free
// — both might be rescued by LM regularization). The first-pass
// failure data is DISCARDED on escalation; only the SECOND-pass
// failure data populates the surfaced SolverFailure variant (per
// §2.5: "the LU step already had its chance; escalation is the
// recovery attempt; if escalation also fails, THAT'S the
// committed-iterate failure").
let lu_outcome = delta_lu.and_then(|δ| {
    self.armijo_backtrack(&x_curr, ..., &δ, r_norm, newton_iter)
        .map_err(|stall| /* convert ArmijoStallInfo to DoublyFailedFactorInfo-shaped sibling — illustrative */)
});
match lu_outcome {
    Ok(x_accepted) => {
        x_curr = x_accepted;
    }
    Err(_first_pass_failure_discarded) => {
        // ESCALATE to LM-enabled retry. `lm_state` (the OUTER
        // persistent state, with cross-iter lambda) takes over.
        let delta_lm = self.try_factor_and_solve_free(
            &triplets, &r_full, newton_iter, r_norm, &mut lm_state
        ).map_err(|info| SolverFailure::DoublyFailedFactor { ... })?;
        x_curr = self.armijo_backtrack(
            &x_curr, ..., &delta_lm, r_norm, newton_iter
        ).map_err(|stall| SolverFailure::ArmijoStall { ... })?;
    }
}
```

The cleanest factoring is a private helper
`try_factor_solve_armijo(x_curr, ..., lm_state) -> Result<Vec<f64>,
SolverFailure>` that owns the inner factor + solve + Armijo. Inline in
`try_solve_impl` if the helper grows; refactor if it adds value.

**`try_solve_impl`'s lm_state argument**: stays as today — the
persistent (cross-iter) λ-carrier. Initialized from
`SolverConfig::lm_regularization`. When LM is disabled
(`lm_regularization == None`), `lm_state` IS the disabled sentinel,
and the "ESCALATE" branch's `try_factor_and_solve_free(&mut lm_state)`
call short-circuits to pre-F3 LU fallback → same δ_LU as the first
pass → same Armijo stall on the second attempt. Net effect at
LM-disabled: gated A is a no-op overlay — second factor + solve + Armijo
happens but produces the same result, then SolverFailure::ArmijoStall
fires. This is a 2× cost overhead per Armijo-stalling iter when
LM-disabled.

**Optimization for LM-disabled**: short-circuit the escalation when
`lm_state.is_active() == false` — convert the first-pass failure
DIRECTLY into a `SolverFailure` and return, skipping the redundant
second factor + solve (which would short-circuit to the same result
because the OUTER lm_state is itself disabled). This avoids the 2×
factor cost at LM-disabled stalls. Handles BOTH failure modes
(ArmijoStall + DoublyFailedFactor) — pre-F3 ArmijoStall + pre-F3
DoublyFailedFactor surfaces are both preserved bit-equally. ~5 LOC of
guard:

```rust
match lu_outcome {
    Ok(x_accepted) => x_curr = x_accepted,
    Err(failure_variant) if !lm_state.is_active() => {
        // LM-disabled short-circuit: the first-pass result IS the
        // committed-iterate failure; no rescue mechanism to try.
        return Err(failure_variant);
    }
    Err(_) => { /* escalate per above */ }
}
```

This optimization is REQUIRED for the F3 spec's §F3.1 bit-equal-when-
dormant invariant: with `lm_regularization == None`, the existing test
sites must not see a 2× wall-clock-cost regression on stalling fixtures
(see `factor_free_tangent_falls_through_to_lu_on_non_pd` at
backward_euler.rs:1570+, referenced in §8 anchors).

### 2.4 LmState extension — none required

The gated algorithm reuses today's `LmState` surface unchanged. The new
control flow lives in `try_solve_impl`; `LmState` is just a persistent
λ-carrier consumed by `try_factor_and_solve_free`. No new fields on
`LmState`.

This is intentional: a `LmConfig::activation_gate` enum (with `Eager` /
`Gated` variants) would let a future Fork-A consumer pick Eager LM
without code duplication. **This spec does NOT add the enum** — gated
A is the only LM behavior; Eager is removed. Rationale:

- F3.4 already empirically falsified Eager LM. Keeping it as an option
  is dead-code preservation, not future-flexibility.
- If a future Fork-A consumer surfaces a need for Eager LM (e.g., a
  consumer where every step is non-PD and the Armijo-first overhead
  dominates), they can add the flag then. YAGNI.
- The cf-device-design `fork_b()` preset modifies in place (no
  `fork_b_gated()` sibling). Single behavior, single preset.

### 2.5 No new SolverFailure variant

The gated escalation's failure surface is `SolverFailure::ArmijoStall`
(the same one F3 spec §2.5 documented). The variant's fields
(`x_partial`, `last_iter`, `last_r_norm`) are populated from the
ESCALATED LM-step's Armijo stall, not the first-pass LU-step stall —
because the LM step's stall is the "real" failure (the LU step
already had its chance and escalation was the recovery attempt that
also failed).

If a future cold-reader wants to distinguish "first-pass LU stall" from
"escalated LM stall" in the error message, that's a `panic!` /
`anyhow!` message-format change in the cf-device-design consumer, not
a new SolverFailure variant. Per spec §F3.1's "describe the failure
surface, not the rescue mechanism" principle — the FAILURE is "Armijo
couldn't find a descent step at iter N"; gated vs Eager is a recovery
mechanism, irrelevant to the failure shape.

**Implications for `solver_failure_message`** (cf-device-design):
unchanged. The MAINTENANCE NOTE comments at `insertion_sim.rs:1093` +
the inline match arms in `run_single_insertion_step` continue to pin
the variant arms. Gated A adds no variant.

---

## 3. Backward-compat contract (bit-equal-when-dormant — F3 spec §F3.1 carry-over)

**The contract**: every existing sim-soft test must pass *bit-equal*
under candidate A's new code path. The hardest evidence is
`tests/multi_element_isolation.rs:229-233` (exact iter count
assertions), `tests/solver_convergence.rs:58` (iter count < 10 at the
existing tight tol), `tests/concentric_lame_shells.rs` (50-iter cap),
plus `factor_free_tangent_falls_through_to_lu_on_non_pd` (the
existing non-PD fixture asserting LU fallback fires).

**The design** (preserves the contract):
- `SolverConfig::lm_regularization == None` is the default
  (`SolverConfig::skeleton()` unchanged per F3.1's existing pin in
  `lm.rs` test `skeleton_solver_config_does_not_opt_into_lm`).
- The §2.3 LM-disabled short-circuit returns `SolverFailure::ArmijoStall`
  immediately on first-pass LU-step Armijo stall (no second factor,
  no wall-clock 2× cost).
- The first-pass LU + Armijo at LM-disabled is bit-equal to today's
  `try_factor_and_solve_free` + `armijo_backtrack` (same triplets,
  same factor path, same Armijo loop).
- `Solver::step` (the panic-on-stall API) routes through
  `try_solve_impl` and panics on its `SolverFailure::ArmijoStall` —
  same panic message as today (preserved via
  `armijo_stall_panic_message`).

**The bit-equal verification gate**: `cargo test -p sim-soft --release`
green with no test modifications. If any of the regression-net tests
break, the gated implementation has a bug — fix before proceeding.

**Hot-path performance**: the first-pass LU + Armijo is the only added
work at LM-disabled call sites IF the iter converges. At LM-disabled +
stalling iter (the failure surface), the short-circuit fires before
the second factor — 0% overhead. At LM-enabled + stalling iter, the
2× factor cost is real but only on the escalation iter (most iters at
cavity = 3 mm baseline don't escalate; bookkeeping cost is bounded).

---

## 4. Fork-A vs Fork-B (carry-over from F3 spec §2.4)

- **Fork B (cf-device-design)**: opts in via `insertion_solver_config()`
  setting `config.lm_regularization = Some(LmConfig::fork_b())`. Same
  ONE-LINE re-opt-in pattern that F3.4 used and that bookmark §4 A's
  "Implementation cost" line describes. The cf-device-design call sites
  (`run_single_insertion_step` lines 1100, `run_sliding_insertion_ramp`
  line 2515) ALREADY route through `try_replay_step` + `solver_failure_
  message` + `catch_unwind` belt-and-suspenders. The surface plumbing is
  100% reusable — only the 1-line opt-in changes.

- **Fork A (production)**: NOT in this spec. `lm_regularization` stays
  `None` for all sim-soft consumers other than cf-device-design. If a
  future Fork-A consumer needs LM, they revisit the gated-vs-eager
  question at that consumer's empirical surface; YAGNI on the
  abstraction today.

---

## 5. Falsifier (when A doesn't work, what we learn)

Four outcomes ranked by what they tell us about the failure-class
mental model:

All comparisons against the **apples-to-apples pre-F3 baseline**:
cavity = 5 mm + default layers 10+3 mm = Run 2 r_norm floor 1.78
(`docs/CAVITY_INSET_STALL_BOOKMARK.md` §8). NOT the layers-also-changed
§1b case (r_norm 0.572) — see §2.2 caveat.

| Outcome | Cavity 3 mm | Cavity 5 mm | Cavity 8 mm | What it means | Next action |
|---|---|---|---|---|---|
| **A. SHIP-IT** | 16/16 converged (baseline restored) | converges OR r_norm floor < 1.78 (any improvement vs pre-F3 Run 2) | (out of scope — Yeoh validity expected) | Gated A mechanism works. Class 2 was the spec's diagnosis error; the mechanism is fine. | Ship + cap cavity slider at 5 mm (§6 A.4). Bookmark cavity = 8 mm separately for candidate B. |
| **B. PARTIAL SHIP-IT** | 16/16 converged | stalls at r_norm ≈ 1.78 (pre-F3 Run 2 baseline — escalation didn't help) | (out of scope) | Gated A restores baseline but doesn't address class 1 at cavity = 5 mm — late-iter geometry is dominated by class 2 (chattering) not class 1 (indefinite tangent). | Ship + cap cavity at 4 mm (class 2 is the next-arc target). Recon for C (smoothed contact). |
| **C. REGRESSION** | < 16/16 converged | (any) | (any) | Gated A broke the baseline despite the bit-equal contract. Implementation bug in §2.3's first-pass LU + Armijo path. | Revert. Debug. Re-implement. |
| **D. WORSE-THAN-F3** | 16/16 converged | r_norm floor > 1.78 (worse than pre-F3 Run 2) | (any) | Escalation produces a WORSE Newton trajectory than pre-F3 (the LM step at the escalation iter takes Newton into a deeper non-descent valley). The LM mechanism is structurally wrong for class 2. | Revert. Bookmark + recon for C (smoothed contact — the F3 spec §3 outcome-C/D path the spec warned about). |

**Outcome A vs B**: the binary question A empirically tests. A's
mechanism is sound iff the cavity = 5 mm late-iter stall (pre-F3 Run
2 r_norm = 1.78 after ~5 iters) yields to LM rescue. The bookmark §6
Gate B trace (F3.4 with always-on LM) reached r_norm = 0.27 across 34
iters — that's evidence the mechanism CAN drive r_norm well below 1.78,
but doesn't prove gated escalation gets there (F3.4's improvement
came from eager-LM taking a *different* Newton trajectory from iter 1
onward; gated A's trajectory matches pre-F3 through iter ~5 then
escalates only at the stall point).

**Outcome D**: the pessimistic case. If gated escalation at the stall
iter puts Newton into a worse trajectory than pre-F3's stall (because
the LM step lands in a worse-conditioned region than the LU step's
stall point), gated A degrades behavior. Plausible but the F3.4 Gate
B data doesn't suggest it (F3.4's r_norm trajectory was
monotone-decreasing modulo active-set-discontinuity jumps; F3.4
reached r_norm = 0.27 — well below pre-F3 Run 2's 1.78).

**Recommendation**: outcome A or B is the realistic prior. C is an
implementation-bug check. D is the surprise case that bookmarks +
escalates to C (smoothed contact).

---

## 6. Sub-leaf ladder

Sized for ≲ 100 LOC sim-soft + ~50 LOC tests + 1-line cf-device-design,
per [[feedback-implement-measure-revert-pattern]]. Implementation lands
**same session** (this spec is the session's first artifact; A.1-A.3
follow inline, A.4 is outcome-dependent).

### A.0 — recon spec — THIS SESSION's first artifact

- This doc.
- No code changes.

### A.1 — gated `try_solve_impl` restructure — same session (small)

- Restructure `try_solve_impl`'s inner Newton-iter body per §2.2's
  algorithm + §2.3's code shape.
- Add the LM-disabled short-circuit guard (§2.3) for bit-equality.
- NO changes to `LmState` / `LmConfig` / `factor_free_tangent` /
  `try_factor_free_tangent` (the inner factor + retry loop stays
  intact — gated A's mechanism lives entirely at the Newton-iter level).
- Acceptance: `cargo test -p sim-soft --release` green; existing
  factor-fallback fixture (the LM-disabled non-PD fixture) still
  asserts LU fallback fires bit-equally.
- Sizing: ~80 LOC body restructure + helper extraction.

### A.2 — gated-LM unit test + LM-enabled stall test — same session (small)

- New unit test in `backward_euler.rs::mod tests` exercising the
  gated escalation. **Pragmatic fixture construction**: rather than a
  hand-crafted multi-iter fixture (which is hard to construct so iter
  0's LU step Armijo-passes AND iter 1's LU step Armijo-stalls), reuse
  the existing `factor_free_tangent_falls_through_to_lu_on_non_pd`
  fixture pattern at backward_euler.rs:1570+ — it constructs a small
  non-PD system. Run try_step with LM-enabled (Fork-B config); ASSERT
  the LM activity log lines fire only IF the first-pass LU + Armijo
  fails (which depends on the fixture's residual landscape — verify
  empirically and pin the observed behavior with explicit assertions).
  If the fixture's LU step Armijo-passes (no escalation), construct a
  second fixture that forces escalation (e.g., a deliberately
  Armijo-hostile residual + non-PD tangent).
- Augment the existing F3 saturation test (`try_step_returns_err_on_*`
  at backward_euler.rs:2355+) to verify the gated short-circuit at
  LM-disabled: assert one factor + one Armijo + ArmijoStall (or
  DoublyFailedFactor) — observable via the absence of a SECOND
  `"sim-soft: faer LU fallback fired"` stderr line on a stalling
  fixture.
- Acceptance: both tests pass; F3.1-F3.4 existing tests pass bit-equal.
- Sizing: ~60 LOC of test code; fixture-construction effort is the
  uncertain part — budget +20 LOC if the second fixture is required.

### A.3 — cf-device-design opt-in — same session (one-liner)

- `insertion_sim.rs::insertion_solver_config()` re-adds
  `config.lm_regularization = Some(LmConfig::fork_b())`. Per §4 Fork-B
  carry-over, the surface plumbing (`try_replay_step` +
  `solver_failure_message` + `catch_unwind` belt-and-suspenders) is
  already in place from F3.4 and stays.
- Update the inline comment at lines 904-911 to reflect the gated
  mechanism (was: "F3.4 LM opt-in REVERTED" → now: "F3 recon candidate
  A: gated LM — engage only on first-pass Armijo failure. See
  docs/F3_RECON_A_GATED_LM_SPEC.md.")
- Acceptance: `cargo test -p cf-device-design --release` green; clippy
  clean.
- Sizing: ~5 LOC (1-line opt-in + comment refresh).

### A.4 — outcome-dependent ship / revert — user-driven gates

Per §5 falsifier matrix:
- **Outcome A**: keep A.1-A.3 commits; cap cavity slider at 5 mm (~10
  LOC in cf-device-design UI); update the F3 falsification bookmark
  with "RESOLVED by candidate A" footer; bookmark cavity = 8 mm
  separately for candidate B; update memory entries.
- **Outcome B**: keep A.1-A.3 commits; cap cavity slider at 4 mm; same
  bookkeeping as A; bookmark candidate C (smoothed contact) for the
  cavity = 5 mm class-2 chattering.
- **Outcome C**: revert A.1-A.3; debug; cold-read pass on the
  bit-equal contract violation; retry.
- **Outcome D**: revert A.1-A.3; bookmark D's failure data; escalate
  to candidate C recon.

---

## 7. What this spec does NOT specify

- **Candidate B (material-validity safe-step)**. Orthogonal class-3 arc.
  If A ships, B is the next arc; its own recon spec lands then.
- **Candidate C (smoothed contact)**. Orthogonal class-2 arc, larger
  blast radius (~300+ LOC, calibration work). Triggers from outcome
  B/D — if A doesn't fully resolve cavity = 5 mm, C is the natural
  next mechanism.
- **Candidate D (mesh refinement)**. High-cost class-3 arc; defer
  unless B + C combined are insufficient.
- **Trust-region LM** (the F3 spec §2.2 alternative). The "more
  general" λ-adaptation rule. Gated A is strictly simpler; if A's
  outcome A succeeds, TR-LM is dead. If A's outcome B/D fires, TR-LM
  is a candidate alongside C in a future recon.
- **`LmConfig::activation_gate` enum** (per §2.4). YAGNI — gated is the
  only LM behavior; Eager is removed. Re-introduce only if Fork-A
  surfaces a concrete need.
- **`SolverFailure` variant additions** (per §2.5). Gated A's failure
  surface IS today's `SolverFailure::ArmijoStall`. No variant change.
- **IFT-adjoint factor's λ behavior**. F3 spec §2.4 carry-over: Fork-B
  doesn't consume IFT gradients. Gated A inherits the spec's
  `lm_seed_lambda` threading through `factor_at_position` /
  `try_factor_at_position` unchanged.
- **Lowering the cavity-inset UI cap to 4 mm now** (without empirical
  result). The cap reduction is outcome-dependent per §6 A.4 — if A
  works, 5 mm; if A partial, 4 mm. Don't pre-emptively cap.

---

## 8. Anchors

**Predecessor docs**:
- `docs/F3_FALSIFICATION_BOOKMARK.md` — the falsification this acts on.
- `docs/F3_LM_REGULARIZATION_SPEC.md` — `SolverFailure` + `try_step` API
  surface (§2.5) + bit-equal contract (§F3.1) — both KEPT and inherited.
- `docs/CAVITY_INSET_STALL_BOOKMARK.md` — the original arc.
- `docs/F4_FALSIFICATION_POSTMORTEM.md` — F4 + F2 falsification context.

**Code sites — Section A.1 (gated restructure)**:
- `sim/L0/soft/src/solver/backward_euler.rs:1367-1464` —
  `try_solve_impl`. THE landing site.
- `sim/L0/soft/src/solver/backward_euler.rs:1170-1192` —
  `try_factor_and_solve_free` (consumed by both first-pass + escalation
  paths; unchanged).
- `sim/L0/soft/src/solver/backward_euler.rs:1483-1532` —
  `armijo_backtrack` (consumed by both paths; unchanged).
- `sim/L0/soft/src/solver/backward_euler.rs:1329-1352` — `solve_impl`
  (the panic-on-failure wrapper around `try_solve_impl`; unchanged —
  routes panic on the SolverFailure variants per its existing
  `unwrap_or_else`).
- `sim/L0/soft/src/solver/lm.rs:140-262` — `LmState` (UNCHANGED — gated
  algorithm reuses today's surface).
- `sim/L0/soft/src/solver/lm.rs:55-103` — `LmConfig` + `fork_b` preset
  (UNCHANGED — gated is the only behavior, no enum variant added).

**Code sites — Section A.2 (tests)**:
- `sim/L0/soft/src/solver/backward_euler.rs:1570+` —
  `factor_free_tangent_falls_through_to_lu_on_non_pd` fixture (the
  existing LM-disabled regression baseline; gated A must keep it
  bit-equal).
- `sim/L0/soft/src/solver/backward_euler.rs:2355+` —
  `try_step_returns_err_on_newton_iter_cap_regardless_of_policy` (the
  existing F3.3 saturation test; augment with the gated short-circuit
  assertion).
- `sim/L0/soft/src/solver/lm.rs:264-449` — `mod tests` (the LmState
  unit tests, ALL UNCHANGED since LmState is unchanged).

**Code sites — Section A.3 (cf-device-design opt-in)**:
- `tools/cf-device-design/src/insertion_sim.rs:899-913` —
  `insertion_solver_config`. THE one-liner site.
- `tools/cf-device-design/src/insertion_sim.rs:1093-1129` —
  `run_single_insertion_step` SolverFailure match arms (UNCHANGED —
  gated A produces the same variants).
- `tools/cf-device-design/src/insertion_sim.rs:1861-1900` —
  `solver_failure_message` formatter (UNCHANGED — no new variants).
- `tools/cf-device-design/src/insertion_sim.rs:2429-2569` —
  `run_sliding_insertion_ramp` `try_replay_step` + `catch_unwind`
  belt-and-suspenders (UNCHANGED).

**Falsifier evidence — verbatim from F3.4 stderr logs**
(`/tmp/f34_gui.log` + `/tmp/f34_gui2.log`):

Gate A (cavity = 3 mm, F3.4 always-on LM, BROKE 16/16 → 0/16):
- Iter 1: LM seeded λ = 3.17e-2, converged in 6 retries to λ = 3.17e3
  at r_norm = 10.94. Subsequent iters 2-9 silent (Llt-PD).
- Iter 10: Armijo stall at r_norm = 1.048e-1 (just 5% above tol = 0.1).
- Diagnosis: iter 1's λ = 3.17e3 decayed at 0.5× per iter (down_factor),
  hitting λ ≈ 6 N/m at iter 10. That's gradient-descent regime →
  Armijo couldn't accept any α per §2.1 math.

Gate B (cavity = 5 mm, F3.4 always-on LM, 0/16 but r_norm floor halved):
- 7 LM activations (iters 1, 2, 3, 4, 6, 8, 15) all rescued the factor
  via LM bump to λ ∈ [3.6e1, 1.13e3]. ALL Llt-PD post-bump.
- r_norm sequence: 10.94 → 56.65 → 33.47 → 32.78 → 12.18 → 4.55 → 0.98
  → ... → 0.27 (iter 34 Armijo stall).
- **Note the iter-1→iter-2 jump from 10.94 to 56.65** — Armijo accepted
  some step at iter 1 (satisfying `trial_norm ≤ (1−α·c1)·10.94`) BUT
  the assembled residual at iter 2 (post-step) is 56.65. This is
  active-set chattering: the LM-step trajectory crossed an active-pair
  on/off boundary; the new active set's residual at the new x is much
  higher than the old active set's residual was at the same x.
  Candidate A doesn't address this; candidate C (smoothed contact)
  does.

Gate C (cavity = 8 mm, F3.4 always-on LM, 1/16 then Yeoh validity panic):
- Step 1 converged via LM activity at iters 4, 15, 17, 22 (λ trajectory
  up to ~1.5e4 then decayed). Newton walked deep enough to seat
  intruder 5.21 of 83.35 mm.
- Step 2 panicked at sim-soft:678: `validity violation at tet 1324:
  max_stretch_deviation = 1.209 ... F = [2.209, 0.842, 0.624]`.
- **Class 3** (Yeoh material validity, orthogonal to LM mechanism).
  Candidate B addresses; A does not.

**Per-layer state at Gate C abort** (from Validations panel readout,
preserved here for B-recon context):
- Layer 0 (innermost, Ecoflex 00-30 + 50% Slacker, 10 mm thick): 46634
  tets, principal stretch range **0.26..2.30** — the 2.30 outlier is
  what tripped Yeoh validity.
- Layer 1 (outermost, Dragon Skin 20A, 3 mm Δ): 19653 tets, stretch
  range 0.54..1.34 (no validity violation).

---

## 9. Implementation expectations + decision posture

- Per [[feedback-autonomous-architecture]] grant, drive the spec +
  implementation autonomously. Flag only big departures from this
  doc.
- Per [[feedback-implement-measure-revert-pattern]] the implementation
  + measure + decide flow is in-session: sub-leaves A.1 + A.2 + A.3
  here, A.4 user-driven gates after.
- Per [[feedback-bookmark-when-surface-levers-exhaust]] if implementation
  reveals a surprise (e.g., a new SolverFailure variant is actually
  required despite §2.5 saying otherwise; LmState's lifecycle doesn't
  compose with gated escalation; or A grows past ~150 LOC), STOP and
  bookmark — three-session pattern continues.
- Per [[feedback-cold-read-review-post-ship]] cold-read pass after
  ship. Size-gate per [[feedback-cold-read-two-passes-for-non-trivial-
  diffs]]: pass-1 sentence-level always; pass-2 structural if diff
  ≳ 300 LOC or has parallel surfaces (gated A has none in this spec —
  factor_free_tangent vs try_factor_free_tangent are already paralleled
  by F3.3's mirror-on-change MAINTENANCE NOTES; gated A doesn't touch
  them).
- Per [[feedback-spec-falsified-revert-opt-in-keep-surface]] (banked
  from F3 falsification) — gated A's design preserves the bit-equal-
  when-dormant contract so a future falsification can still do a
  1-line revert. The §6 A.4 outcome-C / outcome-D revert paths are
  the explicit revert surface.
- Per [[feedback-workaround-removal-verification]] (banked from F3.4
  hotfix) — gated A does NOT remove the F3.4 `catch_unwind`
  belt-and-suspenders in `run_sliding_insertion_ramp`. The undocumented
  panic class (Yeoh material validity at backward_euler.rs:678) is
  still possible at outcome A's cavity = 5 mm if gated escalation lets
  Newton walk into a high-stretch region. Catch_unwind stays.
