# Thermo-Computing Doc Review — 2026-04-09

> **Branch**: `feature/thermo-doc-review`
> **Reviewer**: Claude (Opus 4.6)
> **Scope**: `MASTER_PLAN.md` + `THERMO_CHASSIS_DESIGN.md`
> **Status**: Open checklist. Items applied one-by-one, each as its own commit.

This document is a working checklist for revising the master plan and chassis
design after the 2026-04-09 review pass. Items are grouped by severity. Each
item has an explicit acceptance criterion so "done" is unambiguous. Apply
items in the order listed (must-fix → should-fix → nits), one at a time, with
the sharpen-the-axe discipline carried over from the original work.

The full review reasoning is preserved verbatim below the checklist so future
sessions can reconstruct the *why* without re-running the analysis.

---

## Checklist

### Must-fix (5)

- [x] **M1** — Fix the off-by-2 wording bug in the Phase 1 sampling-error
  arithmetic. Both docs use `0.22 · (½kT)` and "±22% sampling noise"
  inconsistently. Pick one convention (recommend: error expressed as fraction
  of expected mean `½kT`) and propagate. **Acceptance**: master plan §The Gap
  → Phase 1 Validation correction box and chassis Decision 5 side finding
  both state std-error consistently as a fraction of `½kT` (≈45% for one run,
  ≈4.5% for option β). ✓ *Applied 2026-04-09: master plan correction box and
  chassis side finding now use `(½kT)·√2/√10 ≈ 0.45·(½kT)` and explicitly
  state the "fraction of expected mean ½kT" tolerance convention.*
- [x] **M2** — Add a chassis-level decision (Decision 7) for stochastic
  gating in FD/autograd contexts. Currently filed only as a Phase 5+ caveat.
  Three candidate patterns to compare honestly: (1) orthogonal `Stochastic`
  trait with `set_stochastic(active: bool)`, (2) RNG snapshot/restore on
  `PassiveStack`, (3) sim-core `disable_passive_callback` flag (forbidden by
  item 8). **Acceptance**: chassis doc has a Decision 7 section, schemes
  compared, recommendation made, decision logged. ✓ *Applied 2026-04-09 —
  Decision 7 added with full Scheme A (Stochastic trait + RAII gating) vs
  Scheme B (RNG snapshot/restore) treatment. Scheme A recommended on five
  grounds: correctness for state-independent noise (matches every roadmap
  component), RAII forgetting-impossible, ~40 LOC vs ~150 LOC, additive
  on-top-of-A path for B if state-dependent noise ever lands, matches
  Decision 4 orthogonal-trait pattern. Decision 1 trait gains `as_stochastic`
  default-None hook; Decision 2 stack gains `set_all_stochastic` and
  `disable_stochastic` RAII guard. File inventory bumped: component.rs
  30→70, stack.rs 120→160, total 810→890. Chassis design round complete
  table updated to 7 rows. Master plan §Phase 1 caveat 1 (FD perturbation)
  marked RESOLVED with reference to Decision 7.*
- [x] **M3** — Move Q5 (cf-design end-to-end differentiability) from
  "unresolved, gates Phase 5" to active foreground recon, scheduled in
  parallel with the Phase 1 spec drafting. **Acceptance**: master plan §5
  Q5 entry updated to "in-progress recon, target resolution before Phase 1
  ships," with a recon log entry opened when the recon starts. ✓ *Applied
  2026-04-09 — Q5 entry expanded with the asymmetric-risk argument, four
  plausible "no" reactions, and concrete recon scope (~half-day reading
  cf-design autograd integration, SDF boolean differentiability, mesh
  extraction, the cf-design Phase 5 differentiable-optimization spec).
  Phase 5 entry in §The Gap gains a "gates on Q5" forward-pointer. The
  recon log entry itself is deferred until the recon starts (history is
  written when it happens, not preemptively).*
- [x] **M4** — Add `Welford::reset()` and `Welford::merge(&other)` to
  Decision 5's `WelfordOnline` API. `reset()` is needed because the burn-in
  story breaks for streaming Welford. `merge()` is needed for option β (100
  independent trajectories combined into one statistic) and for parallel-env
  Phase 4+. **Acceptance**: chassis Decision 5 final API surface lists both
  methods with one-sentence rationale each. ✓ *Applied 2026-04-09: both
  methods added to the final API surface, full rationale + Chan/Pébay
  parallel-merge formula in the sub-decisions list, struct gains `#[derive(Clone)]`,
  test_utils.rs LOC bumped 150→170, total footprint 790→810.*
- [x] **M5** — Tighten the `PassiveComponent` trait contract on what it's
  allowed to write to. Currently the trait gives `&mut Data` and the contract
  is implied by prose, not enforced. Two options: (1) hard doc-contract +
  debug-mode snapshot assertion in `PassiveStack::install`, (2) signature
  change to `fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>)`
  with the stack providing a scratch buffer. Pick one. **Acceptance**:
  chassis Decision 1 trait doc-comment is a hard contract; if option (2) is
  chosen, the signature is updated and Decision 2's stack implementation
  reflects the scratch-buffer pattern. ✓ *Applied 2026-04-09 — chose option
  (2). Trait now `apply(&self, &Model, &Data, &mut DVector<f64>)`. Decision
  1 has a "Trait contract revision" sub-section explaining the change and
  consistency with item 2 / item 4 / item 6 "loud over silent" line.
  Decision 2's Scheme B sketch and Final API surface install body now
  allocate a scratch buffer per step and fold once into qfrc_passive.
  Master plan §Phase 1 pseudocode updated `qfrc_passive[i] +=` →
  `qfrc_out[i] +=` with prose nudge. Recon log entries left alone (history).*

### Should-fix (6)

- [ ] **S1** — Resolve Q3 (`thrml-rs` existence) with a 5-minute web search.
  No defensible reason to defer. **Acceptance**: Q3 entry in master plan §5
  marked RESOLVED with the answer and a one-line recon log entry.
- [ ] **S2** — Rewrite master plan §3 Current State to reflect the
  chassis-design-round outcome (so future-Jon doesn't have to read 10 recon
  log entries to reconstruct the snapshot). Bigger fix, optional: split into
  `MASTER_PLAN.md` (forward-looking, kept current) + `MASTER_PLAN_RECON_LOG.md`
  (append-only, never edited). **Acceptance**: §3 paragraph reflects the
  current chassis state at minimum; split is a follow-on if desired.
- [ ] **S3** — Reconsider `install_per_env`'s return type. Currently
  `Vec<Model>` discards the per-env stacks; introspection (e.g., per-env
  `diagnostic_summary`) requires recreating them. Recommend `EnvBatch {
  models: Vec<Model>, stacks: Vec<Arc<PassiveStack>> }` — common case is a
  one-token unwrap, introspection case works for free. Borderline call.
  **Acceptance**: chassis Decision 3 either keeps `Vec<Model>` with explicit
  acknowledgment of the trade-off, or updates the API to `EnvBatch`.
- [ ] **S4** — Add a "passive forces only" framing paragraph to chassis §0.
  D1 (flashing ratchet) hits this at the earliest payoff: it needs both a
  `cb_passive` consumer (the potential) and a `cb_control` consumer (the
  flash schedule). The chassis is silent on this and should sketch the
  multi-callback shared-state pattern in 5 sentences. **Acceptance**: chassis
  §0 has a paragraph explicitly naming "passive forces only" and showing the
  shared-Arc pattern for D1-style components.
- [ ] **S5** — Call out D4's external dependencies. Currently D4 says "no fab
  dependency" but understates the cost: 3D printer with reproducible
  mechanical properties, material consistency, high-speed video + state
  classification, calibration loop for measuring effective `T`/`γ`/barrier.
  **Acceptance**: master plan §2 D4 entry has an "external dependencies"
  subsection naming the printer + measurement infrastructure as real
  engineering effort.
- [ ] **S6** — Add a one-sentence "future direction" note for thermostat
  persistence/checkpointing. ChaCha8 state + gamma + kT + seed are trivially
  serializable; ml-bridge already has policy persistence as a precedent.
  **Acceptance**: chassis Decision 6 (or §2 Future Extensions) mentions
  serialization as a known additive direction.

### Nits (4)

- [ ] **N1** — Compress the "headline research claim" paragraph in Vision §1
  to a tagline. The Synthesis sentence at the end of §2 already says it
  better; consider deleting the §1 longer version or swapping their roles.
  **Acceptance**: §1 has a single-sentence headline claim; the §2 Synthesis
  sentence is the canonical version.
- [ ] **N2** — Specify the chassis convention for `n_sigma` in
  `assert_within_n_sigma`. Currently undefined. Recommend 3σ as the default,
  documented in chassis Decision 5. **Acceptance**: Decision 5 names 3σ as
  the chassis convention with a one-line rationale.
- [ ] **N3** — Add a 5-line legend defining the Research Directions metadata
  categories (`reach`: low/early/mid/long-term; `novelty`: low/med/high/very
  high/extreme). Currently used in §2 and the priority ladder but undefined.
  **Acceptance**: §2 has a legend block at the top defining the categories.
- [ ] **N4** — Add a `debug_assert!` after the defensive `clear_passive_callback()`
  call inside `install_per_env`. Belt-and-suspenders: if the chassis says
  the call is "non-negotiable," the assertion validates the assumption.
  **Acceptance**: chassis Decision 3 final API surface includes the
  debug-assert line.

---

## What I would NOT change (for the record)

These choices are right and shouldn't be relitigated during this revision pass:

1. **Sibling crate `sim-thermostat`** — keeps sim-core rand-free; the
   dep-graph property is observable from the graph alone.
2. **Builder pattern** — matches ml-bridge, no `Arc::new({ let mut s = ... })`
   dance, one shape for N=1 and N≥2.
3. **`ChaCha8Rng` over `StdRng`** — silent-failure-on-rand-version-bump
   argument is real; consistent with the "loud over silent" line elsewhere.
4. **`cb_passive` over `qfrc_applied`** — item 3's supersession of item 2 is
   the textbook win for the recon discipline. Field-disjointness composition
   with ml-bridge is a free win.
5. **Broad `PassiveComponent` trait** — gives D1 (ratchet) and D2 (driver)
   a home. Orthogonal `Diagnose` recovers everything narrow-Thermostat
   offered.
6. **Minimal `Diagnose` with one method** — YAGNI-vs-extensibility balance
   is right; non-breaking extension path.
7. **Welford over `(sum, sum_sq)`** — catastrophic cancellation case is
   real for the equipartition test; ship-once amortizes across phases.
8. **Sharpen-the-axe + recon log structure** — cleanest "show your work"
   record in the project. Keep doing this.
9. **Phase 1 = 1-DOF DHO equipartition** — smallest possible foothold;
   right physical invariant.
10. **D1 (Brownian motor) at priority 2** — earliest payoff, stunning
    visual, well-grounded literature.

---

## Verification step taken before writing this review

The entire `cb_passive` design rests on `qfrc_passive` being auto-cleared at
the start of `mj_fwd_passive`. Verified directly in the code:

- `sim/L0/core/src/forward/passive.rs:368` — `data.qfrc_passive.fill(0.0)` at
  the top of `mj_fwd_passive` (or the per-DOF awake-only variant at line
  361 when sleep is active). ✓
- `sim/L0/core/src/forward/passive.rs:681` — aggregation uses `=`
  (`qfrc_passive[dof] = qfrc_spring[dof] + qfrc_damper[dof]`). ✓
- `sim/L0/core/src/forward/passive.rs:683-705` — gravcomp/fluid use `+=`. ✓
- `sim/L0/core/src/forward/passive.rs:719` — `cb_passive` fires AFTER
  aggregation (so a thermostat using `+=` correctly composes with whatever
  the engine deposited). ✓
- `sim/L0/core/src/types/data.rs:158` — `pub qfrc_passive: DVector<f64>`. ✓

Bonus finding (not in either doc): the comment at `passive.rs:707-712` shows
that future `mj_contactPassive()` will be inserted *before* `cb_passive` and
*must* use `+=` on `qfrc_passive`. So when contact-passive lands, the
thermostat composes correctly with it for free. The cb_passive design is
even more future-proof than the recon log claims.

The foundation is rock solid. The chassis decisions all build on a verified
substrate.

---

## Suggested order of attack

- [x] **M1** — One-paragraph wording fix in two places. ~10 min. ✓ done
- [x] **M4** — `Welford::reset()` and `Welford::merge()` added to Decision 5
  API surface. ~10 min in the doc. ✓ done
- [x] **M5** — Tighten the `PassiveComponent` doc-contract (option 1:
  doc-only) or change the signature (option 2). ~10-30 min. ✓ done — chose
  option 2 (signature change with scratch buffer).
- [x] **M2** — Add Decision 7 to the chassis doc — stochastic gating for
  FD/autograd contexts. ✓ done — Scheme A (`Stochastic` trait + RAII guard)
  recommended; full two-scheme treatment in chassis Decision 7.
- [ ] **S1** — Resolve Q3 (`thrml-rs`) with a 5-min web search.
- [x] **M3** — Open Q5 (cf-design differentiability) as foreground recon in
  parallel with Phase 1 spec drafting. ✓ done — Q5 escalated, recon scope
  named, recon log entry deferred until the recon actually starts.
- [ ] **S2** — Rewrite §3 Current State (or split into two files).
- [ ] **S3, S4, S5, S6** — Smaller doc additions, ~15 min total.
- [ ] **N1, N2, N3, N4** — Polish pass.
- [ ] Draft `PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md` against the revised chassis.

---

## Full review reasoning (preserved verbatim)

> The text below is the original 2026-04-09 review response, kept unedited
> so the reasoning behind each item can be reconstructed without re-running
> the analysis pass.

### Overall verdict

These are **good documents.** Better than good — the recon log discipline is
the best example of "show your work" I've seen in this project, the cb_passive
discovery genuinely improved the design (item 2 → item 3 supersession is a
textbook win for sharpen-the-axe), and the chassis-decision-by-decision
structure is clean. If we shipped this exact chassis I would not lose sleep
over it.

That said: the user said Gordon Ramsay, so here is the actually picky list.
Split into **must-fix** (real issues that should land before the Phase 1 spec
gets drafted), **should-fix** (medium-stakes things that will save pain
later), and **nits**. I'll close with what I would *not* change.

---

### Must-fix (5 items)

#### M1. The Phase 1 sampling-error arithmetic has an off-by-2 wording bug

This is the most concrete error in either document. In the master plan §The
Gap → Phase 1 Validation correction box (lines ~451-459) and again in
`THERMO_CHASSIS_DESIGN.md` Decision 5 side finding (lines ~1257-1261):

> the standard error of the sample mean is then `√2/√10 · (½kT) ≈ 0.22 · (½kT)`
> — about **±22% sampling noise**

Walk it: `√2/√10 ≈ 0.447`. `0.447 · (½kT) ≈ 0.224 kT`. That's 22% **of kT**,
but the *expected mean* is `½kT = 0.5 kT`, so the relative error against the
actual mean is **~45%**, not 22%. The text writes `0.22 · (½kT)` in one
breath and "±22% sampling noise" in the next, and those are two different
numbers (`0.11` vs `0.224`).

Why this matters: option (β) — 100 independent trajectories — gives
`N_eff ≈ 1000`, so the standard error is `0.707/√1000 ≈ 0.022`, which is
**4.5% relative to ½kT**, not the 2% the master plan implies. That's still
good enough for a meaningful Phase 1 gate, but **the spec needs to pick a
clean convention** ("error expressed as fraction of the expected mean ½kT"
is the natural one) and propagate it consistently. Otherwise the Phase 1
spec will inherit the ambiguity and the test author will write `n_sigma`
against a tolerance that's quietly wrong by 2×.

Concretely: in both docs, replace the phrase with something like *"std error
of mean ≈ 0.22·kT, which is **~45% of the target ½kT mean** for a single
10⁵-step run; option (β) with 100 trajectories brings this to ~4.5% of the
target."*

This is small but non-trivial because tolerance numbers in validation gates
compound — sloppy here means sloppy in every later phase that copies the
pattern.

#### M2. The `PassiveComponent` trait has no story for "noise off / deterministic mode"

This is the biggest chassis-level miss, and it's the one I most want pushback
on (or acceptance of) before committing the chassis.

**The problem.** Phase 5 (differentiable cf-design) and Phase 5+ caveats note
that `cb_passive` fires inside `forward_skip()`, which is called from
`forward_acc()`, which is called from finite-difference perturbation paths.
This means: when the autograd engine perturbs `qpos[i]` and re-runs `forward()`
to read `qacc`, the thermostat **also fires and consumes RNG** during the
perturbed evaluation. The perturbed and unperturbed runs see *different noise
samples*, the FD derivative is dominated by `Var(noise)/h_perturbation`, and
the entire derivative signal drowns in thermal jitter.

This is currently filed as "Phase 5+ caveat — gating or RNG snapshot/restore."
The chassis doc never mentions it. This is exactly the class of question the
chassis is supposed to answer up front, because *retrofitting it into the
trait at Phase 5 is breaking change territory.*

**What to do.** Not asking the user to solve it now — but the chassis doc
should explicitly enumerate the design space and pick one direction so Phase
5 isn't a surprise. Three reasonable patterns:

1. **Orthogonal opt-in trait** (matches the `Diagnose` pattern):
   ```rust
   pub trait Stochastic: PassiveComponent {
       /// When false, apply() must be deterministic (zero noise, or
       /// frozen at the last sample). Used by FD/autograd contexts.
       fn set_stochastic(&self, active: bool);
   }
   ```
   `LangevinThermostat: PassiveComponent + Stochastic`. Generic FD code
   requires `T: PassiveComponent + Stochastic`. Components that have nothing
   stochastic just don't implement it.

2. **RNG snapshot/restore on `PassiveStack`**: the stack exposes
   `pub fn snapshot_rng(&self) -> RngState` / `restore_rng(&self, state)`.
   FD code wraps the perturbation loop in snapshot → perturb → restore.
   More invasive (the stack has to know about RNG state), but doesn't
   require components to opt in.

3. **Gate at the model level**: a `model.set_disable_passive_callback(bool)`
   flag that the FD loop toggles. Cheapest, but requires a sim-core change —
   forbidden by item 8.

Option 1 is the cleanest fit for the existing chassis. **Decision: add a
"Decision 7 — stochastic gating for derivative contexts" section** that
names the option space, picks one, and sets the precedent. ~30 lines, zero
code, saves a Phase 5 retrofit.

The reason this is rated Must-fix and not Should-fix: D3 (co-design) is the
*headline* experiment, it's gated behind Phase 5, and if Phase 5 derivatives
are unreliable because the chassis didn't anticipate this, you re-design the
chassis, you re-implement everything that depends on it, and the whole "the
bolt patterns must outlive every individual component" promise becomes
hollow. This is precisely the scenario the chassis exists to prevent.

#### M3. Q5 (cf-design end-to-end differentiability) is load-bearing and should be recon'd in parallel with Phase 1, not after it

Currently Q5 is listed as "Unresolved. Gates Phase 5." Phase 5 is far away.
So in the current plan, the user finds out whether the headline experiment
(D3) is even feasible *after* months on Phases 1-4.

This is asymmetric risk. If Q5 turns out to be "yes, cf-design is fully
differentiable end-to-end" — great, no harm done, but the wait wasn't
necessary. If it turns out to be "no, the SDF library uses non-differentiable
booleans / non-differentiable mesh extraction / FD-only gradients past a
certain layer" — *that changes the build order*. Either:
- Build the differentiable layer earlier (potentially before D1)
- Re-prioritize Research Directions away from D3 toward D1+D2+D4 (which
  need much less differentiability)
- Use a surrogate model (PyTorch in the loop) and explicitly accept the
  boundary

The cost of doing Q5 recon now is small — maybe a half-day reading cf-design's
autograd integration. The cost of doing it later is months of build commitment
toward an experiment that may need to be re-architected.

**Concrete ask:** open Q5 as a foreground recon task either before Phase 1
starts or in parallel with the Phase 1 spec drafting. It does not need to be
solved before Phase 1 ships — but it needs to be *known* before committing
to the build order past Phase 1.

#### M4. `WelfordOnline` is missing `reset()` and the burn-in story is broken

The chassis ships streaming Welford for the explicit reason that test authors
don't want to materialize 10⁵–10⁷ samples in a Vec. But the chassis also
says (Decision 5 sub-decisions) "burn_in_skip is not shipping — it's literal
slice indexing."

Slice indexing is only available if samples are materialized. For the
streaming case (which is the entire reason Welford ships), either:
- Run the burn-in steps separately, *not pushing anything to Welford*, then
  start a fresh Welford post-burn-in. Awkward — requires reaching into the
  inner loop.
- Push to Welford during burn-in, then... what? There's no way to discard
  the burn-in samples from the accumulator.

Add `Welford::reset()`. One line. Solves the burn-in case for the streaming
path:

```rust
for step in 0..burn_in {
    data.step(&model)?;
    welford.push(0.5 * data.qvel[0].powi(2));
}
welford.reset();  // ← discard burn-in
for step in 0..n_samples {
    data.step(&model)?;
    welford.push(0.5 * data.qvel[0].powi(2));
}
let mean = welford.mean();
```

Smaller note in the same vein: `WelfordOnline` should also expose a
`merge(&other)` method. Option (β) (100 independent trajectories) wants to
combine 100 per-trajectory accumulators into one global statistic. The
Chan/Pébay parallel-merge formula is six lines and is the natural composition
operator for streaming Welford. Phase 1 will want this for option (β); Phase
4+ will *definitely* want it for parallel envs. Ship both `reset()` and
`merge()` in the Phase 1 chassis.

#### M5. The `PassiveComponent` trait's contract is silent on what it's allowed to write to

Decision 1 settled the trait shape:
```rust
pub trait PassiveComponent: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &mut Data);
}
```

The doc-comment says "Reads from `model` and writes additive contributions
into `data.qfrc_passive`." But "additive contributions into qfrc_passive" is
*not enforced anywhere* — the trait gives `&mut Data`, which means an
implementation could legally write into `qfrc_actuator`, `qpos`, `qvel`,
`time`, anything. A buggy or hostile component could trample everything.

For Phase 1 with one component, this is a "user discipline" issue. For
Phase 4+ with multi-component stacks (D1 + D2 + D3 territory), it's an actual
hazard: component A writes into `qfrc_passive[i]`, component B writes into
`qfrc_actuator[i]` thinking it's being clever, the stack composes them in
order, and the resulting physics is wrong in a way that takes a week of
debugging to find.

Two options:
1. **Cheap**: rewrite the doc-comment to be a hard contract, document it
   loudly, and add a debug-mode assertion in `PassiveStack::install` that
   snapshots `data` before/after each component's `apply` and panics if
   anything outside `qfrc_passive` changed. This catches bugs in tests but
   has zero release-mode cost.
2. **Expensive but right**: change the trait signature so the only mutable
   access is to a `&mut DVector<f64>` that the stack provides:
   ```rust
   fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);
   ```
   The stack passes a scratch buffer, accumulates from each component, then
   writes the total into `data.qfrc_passive` once at the end. This is a more
   honest API but requires the stack to allocate (or reuse) the buffer.

Option 1 is fine for Phase 1 — but **the chassis doc should pick one
explicitly and write the contract into the trait doc**. Right now the
contract is implied by prose, not by anything load-bearing.

---

### Should-fix (6 items)

#### S1. Q3 (does `thrml-rs` exist?) should be resolved now, not deferred to Phase 6

This is a five-minute web search. There is no defensible reason to leave it
open — it costs nothing to resolve now and the answer could change Phase 6's
planning entirely. If `thrml-rs` doesn't exist, the plan needs to either name
a fallback project (e.g., implement a minimal block-Gibbs sampler in ~few
hundred LOC) or pick a different bridge target. Letting "Q3 — five-minute
search" sit on the open-questions list is a small but real failure of the
discipline being enforced.

#### S2. The Master Plan is hard to read forward — Current State drifts behind Recon Log

The recon log is ~1500 of the master plan's 2000 lines, and the **Current
State** section is ~30 lines that don't fully reflect what the recon
discovered. A future-Jon trying to answer "what is the actual current
design?" has to read 10 recon-log entries and synthesize them into a current
snapshot, *because the snapshot section doesn't get updated as recon
completes*.

The recon log is good — keep it append-only as the historical record. But
the **Current State** section should be the canonical "where are we right
now," and it should be rewritten (not appended to) at the end of every recon
round to reflect what changed. Right now §3 still says things like "What
does *not* yet exist" with a list that's already partially answered by the
chassis design round.

Quick fix: rewrite §3 to reflect the chassis-design-round outcome. Two
paragraphs:
- *"As of 2026-04-09: chassis design round complete. The thermostat will
  live in `sim/L0/thermostat/` (per Decision 6) and bolt onto `cb_passive`
  via the `PassiveStack` builder. Phase 1 implementation is the next round
  and starts with the spec draft."*
- *"What still does not exist: any code, any Cargo.toml, the Phase 1 spec,
  the validation parameter set fix (α/β/γ — see Phase 1)."*

Bigger fix (recommended): split the master plan into two files.
`MASTER_PLAN.md` contains Vision + Research Directions + Current State + The
Gap + Open Questions + Spec Index — the *forward-looking* doc, kept short
and current. `MASTER_PLAN_RECON_LOG.md` contains the recon entries —
append-only, never edited, the historical record. The two are linked.
Future-you reading one or the other gets the right thing for the question
being asked.

#### S3. `install_per_env` returning `Vec<Model>` discards the stacks unnecessarily

Decision 3 considered `Vec<(Model, Arc<PassiveStack>)>` and rejected it as
"imposing a cost on the common case for an uncommon need." Push back gently.
The cost is one tuple field. The need is real for any Phase 2+ test that
wants to introspect the per-env stacks (e.g., `diagnostic_summary` for a
debug HUD, or extracting per-env RNG state for checkpointing).

Can also have both, with zero cost to the common case:

```rust
pub struct EnvBatch {
    pub models: Vec<Model>,
    pub stacks: Vec<Arc<PassiveStack>>,
}

impl PassiveStack {
    pub fn install_per_env<F>(prototype: &Model, n: usize, build_one: F) -> EnvBatch { ... }
}

// Common case: `let envs = ...; envs.models` is a one-token unwrap.
// Introspection case: `envs.stacks[i].diagnostic_summary()` works.
```

Borderline call — would push for the change but not die on the hill.

#### S4. The chassis is silent on "passive forces only" vs. controlled forces

D1 (Brownian motor / flashing ratchet) is on the priority ladder at #2 —
earliest payoff. But D1 *is* a controlled passive force: an RL policy decides
when to flip the ratchet potential on and off. That's a `cb_control` consumer
(the policy → on/off state) plus a `cb_passive` consumer (the ratchet
potential, gated by the on/off state). They share state.

The chassis doesn't address this. That's fine — the chassis is *passive
forces only*, by design — but the doc should *say so explicitly* and sketch
the multi-callback pattern, because D1 hits this at the earliest payoff. One
paragraph in §0 of the chassis doc: *"This chassis is for passive forces
only (`cb_passive` and `qfrc_passive`). Controlled forces still flow through
`cb_control` and `qfrc_applied`/`ctrl` per existing sim-core patterns.
Components that need both — e.g., a flashing ratchet whose potential is
gated by an RL policy — install one `PassiveComponent` for the potential
and one `cb_control` callback for the schedule, sharing state via
`Arc<…>`."*

This unblocks D1 mentally in 5 sentences and prevents a future "wait, the
chassis can't do flashing ratchets" panic.

#### S5. D4 (sim-to-real on a 3D-printed device) has external dependencies that aren't called out

The Vision claims D4 is "the moat" because no software-only stack and no
fab-based stack can do it. True. But D4 depends on:
- A 3D printer with reproducible mechanical properties (FDM? SLA? what
  tolerances?)
- Print-material consistency (PLA stiffness varies batch-to-batch by ~10%)
- High-speed video rig + state classification infrastructure for measuring
  the printed device's actual sample distribution
- Some calibration loop to measure the printed device's *effective* `T`,
  `γ`, barrier height — these will not exactly match the simulated parameters

Currently D4 just says "Hardware loop is real but fully owned — no fab
dependency." That's true at the "no fab house" level but understates the
infrastructure cost. The plan should call this out as a real engineering
effort, not a simulation-side experiment. Adding a one-paragraph "external
dependencies" subsection to D4 is enough.

#### S6. There's no story for thermostat persistence / checkpointing

The codebase already has policy persistence for ml-bridge (recent merge
`4260a58`). For long-running thermo experiments — especially Phase 5+ where
EBMs are being trained — checkpointing and resuming is desirable. A
LangevinThermostat's full state is ~32 bytes (ChaCha8 internal state) + gamma
(DVector) + kT + seed. Trivially serializable.

Not Phase 1. But the chassis should at least mention it as a known additive
direction, the way it mentions `HasTemperature` for `Diagnose`. A
one-sentence note in Decision 6's "future extensions" or under §2 of the
chassis doc.

---

### Nits (4 items)

#### N1. The "headline research claim" paragraph wants to be a tagline

The current Vision §1 has a paragraph that almost-but-not-quite is the
elevator pitch. Take a swing at compressing it to one sentence, the way
Synthesis at the end of §2 already does (*"CortenForge becomes the only
physics + ML stack where the same energy function specifies the geometry of
a fabricable device, the dynamics of its simulation, the gradient of its
training, and the policy of its control — and where you can print the result
and measure whether reality agrees."*) — that sentence is the tagline. The
§1 paragraph is the longer version. Consider swapping their roles or just
deleting the §1 longer version, since the §2 Synthesis says it better.

#### N2. `assert_within_n_sigma`'s default n_sigma should probably be specified

The chassis doesn't say what `n_sigma` Phase 1 will use. 3σ is the natural
default (gives ~99.7% pass rate for true-positive tests). 2σ is too tight,
5σ is overkill. Pick one and document it as the chassis convention, even if
individual tests can override.

#### N3. Decision 1 mentions "Status: idea / mid-term reach / very high novelty" — that's a Research Directions metadata convention that doesn't appear elsewhere

The Research Directions section uses a "Status: idea / X reach / Y novelty"
line per direction, and the priority ladder table at the end uses "Reach"
and "Novelty" columns. Good metadata, but the categories
(idea/early/mid/long-term, low/med/high novelty) aren't defined anywhere. A
5-line legend would make it clearer what "extreme novelty" means vs. "very
high."

#### N4. The `clear_passive_callback()` defensive call inside `install_per_env` deserves a debug-assert

Decision 3 includes this defensive line:
```rust
model.clear_passive_callback();  // defensive — non-negotiable
```

Belt-and-suspenders: add a `debug_assert!(model.cb_passive.is_none())` after
the call (assuming the field is debuggable, which it should be). The defense
is "make sure it's clear before installing" — *asserting* that it's clear
after `clear_passive_callback()` validates the assumption. Costs nothing in
release mode, catches bugs in test runs.
