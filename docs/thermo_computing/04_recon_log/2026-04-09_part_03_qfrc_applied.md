# 2026-04-09 (part 3) — Phase-1-blocking item 2: `qfrc_applied` lifecycle

> Extracted from `MASTER_PLAN.md` §7 part 03 during the 2026-04-09 doc-tree refactor.

- **Trigger**: First item in the Phase-1-blocking recon round, taken under
  the sharpen-the-axe discipline (one item at a time, fully logged before
  moving on). Picked first because it is the most correctness-critical of
  the open items and it shapes the thermostat API.
- **Question**: Does the forward pipeline auto-clear `qfrc_applied` at the
  start of each step, or does the user own clearing? Answer determines
  whether the thermostat does `=` (overwrite) or `+=` (accumulate), and
  shapes the API for composing the thermostat with future writers.
- **Finding**: `qfrc_applied` is **user-owned and persistent across steps**.
  The forward pipeline never clears it. It is only zeroed in two places,
  both reset functions:
  - `Data::reset()` at `data.rs:1123` (full reset, in the "force vectors —
    zero" block)
  - `Data::reset_to_keyframe()` at `data.rs:1245` (keyframe reset, with
    explicit doc-comment: *"Clears derived quantities (...) and user-applied
    forces (qfrc_applied, xfrc_applied) — matching the convention of
    Data::reset()"*)

  Initialization at `model_init.rs:540` is `DVector::zeros(self.nv)`. No
  per-step clear anywhere in `forward/`, `integrate/`, or `step()`.
- **Convention is documented loudly** in the batch-sim API
  (`batch.rs:106-110`): *"Use this to set ctrl, qfrc_applied, xfrc_applied,
  or any other input field before calling step_all()."* And
  `batch.rs:171-175`: *"[reset_one] does **not** zero qfrc_applied /
  xfrc_applied (see Data::reset documentation). Callers must zero these
  explicitly if needed."* `inverse.rs:23-25` confirms the categorization:
  `qfrc_applied + qfrc_actuator + J^T*xfrc_applied = "the total
  user-supplied generalized forces"`. This matches MuJoCo's convention
  exactly: external forces persist to model constant external loads; user
  owns the lifecycle.
- **Implication**: Thermal noise is i.i.d. per step. Accumulating across
  steps without clearing is wrong physics — `qfrc_applied` would random-walk
  and the equilibrium temperature would drift upward over time (variance
  compounding). The thermostat must either overwrite, clear-then-accumulate,
  be the only writer, or use a separate field.
- **Two schemes compared** (per sharpen-the-axe rule "Two schemes, then
  choose"):
  - **Option A — Sole-writer overwrite**: Thermostat does
    `data.qfrc_applied[i] = -γ·qvel + noise`. Documented contract:
    *"LangevinThermostat is the sole writer of qfrc_applied while active.
    For additional forces, use xfrc_applied or ctrl."*
    - **Pros**: Foolproof — no silent compounding possible by construction.
      A-grade enforceable (debug-assert invariant in tests). Zero cost for
      Phase 1 validation (no other forces in the test). Failure mode (a
      user trying to compose) is **loud and immediate** — they see their
      force vanish.
    - **Cons**: Doesn't compose with controllers / RL policies that also
      want to write `qfrc_applied`. Future RL/controller users must use
      `xfrc_applied` or `ctrl` instead, or wait for an upstream redesign.
  - **Option B — Additive, caller clears**: Thermostat does `+=`. Caller
    must `data.qfrc_applied.fill(0.0)` each step before calling `apply()`.
    - **Pros**: Composable with other writers. Matches MuJoCo's "user owns
      lifecycle" convention exactly.
    - **Cons**: **Silent correctness failure** if caller forgets to clear.
      Early steps would still pass equipartition (per-step variance is
      right), but temperature would drift upward as the random walk in
      `qfrc_applied` accumulates. Failure mode is invisible, delayed, and
      exactly the hardest kind of bug to debug.
- **Decision: Option A for Phase 1.** Reasoning:
  1. Option B's failure mode (silent, delayed, wrong physics that passes
     early validation) is precisely what sharpen-the-axe is meant to
     prevent. Option A makes that failure mode impossible by construction.
  2. Phase 1 has zero composability requirement — the 1-DOF damped harmonic
     oscillator validation has no other forces. Designing the more flexible
     API for a feature we don't yet need is premature optimization, *and
     the flexibility is itself the source of the bug*.
  3. Composability is a Phase 4-7 concern; we'll know what the right
     interface looks like by then. Designing it now based on guesses is
     speculative scaffolding.
  4. Migration A → C (see below) is a focused localized change. Migration
     B → A would be a breaking change after users have written code against
     additive semantics — much more painful.
  5. Option A's downside is loud and immediate; Option B's is silent and
     delayed. Loud failures are cheap, silent failures are expensive.
- **Forward-looking — Option C, NOT for Phase 1**: The eventual right
  answer for Phase 4+ composability is probably a dedicated
  `qfrc_thermostat: DVector<f64>` field on `Data`, summed alongside
  `qfrc_applied` in the canonical aggregation in `constraint/mod.rs:78-87`.
  This preserves `qfrc_applied` for user/RL writes while giving the
  thermostat its own clean slate. Invasive (modifies the canonical
  force-sum, requires upstream sim-core support, touches the constraint
  module). Explicitly out of scope for Phase 1. Flag in the Phase 1 spec
  as the eventual end-state; revisit when Phase 4 (coupled bistable
  arrays) starts running into composition needs.
- **Item 2 RESOLVED**: Phase 1 thermostat uses sole-writer overwrite
  semantics. Documented contract, debug-assert invariant in tests, Option
  C flagged as future work.
- **What item 2 leaves open for item 3**: Whether a `cb_passive` (or
  similar passive-force) callback hook exists. If it does, the thermostat
  might more idiomatically write into `qfrc_passive` instead, sharing the
  existing damping/spring infrastructure — which would change the Option
  A/C tradeoff entirely. This question naturally belongs to item 3
  (existing usage patterns for writing applied forces). If found, revisit
  item 2's decision before drafting the Phase 1 spec.
- **Next action**: Phase-1-blocking recon item 3 — existing usage patterns
  for writing into `qfrc_applied`, including the `cb_passive` hook
  question.

