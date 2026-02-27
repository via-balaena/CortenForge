# {TITLE} — Spec

**Status:** Draft
**Phase:** Roadmap Phase {N} — {phase_name}
**Effort:** {S/M/L}
**MuJoCo ref:** `{function_name()}` in `{source_file}.c`
**Prerequisites:**
- {list prerequisites — include commit hashes for already-landed work}
- {e.g., "DT-103 spatial transport utilities (landed in `abc1234`)"}
- {or "None"}

---

## Problem Statement

{What is wrong, missing, or non-conformant. Be specific — name the function,
field, or behavior. State the gap between "what we do" and "what MuJoCo does."
2–4 paragraphs max.}

{Lead with the strongest motivation. Categorize the problem:}

{- **Correctness bug** — "We compute wrong values for X." (Strongest — ship-blocking.)}
{- **Conformance gap** — "MuJoCo does X; we don't implement it yet." (Strong — blocks feature completeness.)}
{- **Efficiency/ergonomic issue** — "X works but the API is awkward / performance is poor." (Valid but weaker.)}

---

## MuJoCo Reference

{Describe how MuJoCo handles this. Cite exact source files, function names,
field names, and calling conventions. Include code snippets from MuJoCo C where
they clarify behavior. Cover edge cases.}

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| {behavior 1} | {what MuJoCo does — cite function + source file} | {what we do / gap} |
| {behavior 2} | ... | ... |

### Convention Notes

{Document every MuJoCo → CortenForge translation. This is the #1 silent-bug
risk — be explicit. Use the table format below.}

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| {field/concept} | {what MuJoCo does} | {what we do} | {substitution rule: "use X wherever MuJoCo uses Y"} |
| {e.g., `cacc` ref point} | {at `subtree_com[rootid]`} | {at `xpos[body_id]`} | {Substitute `xpos[body_id]` for `subtree_com + 3*rootid`} |
| {e.g., spatial layout} | {`[angular; linear]`} | {`[angular; linear]` (same)} | {Direct port — no translation needed} |

> **Porting Rule column:** The actionable instruction an implementer follows
> when translating MuJoCo C code to our Rust. Written as an imperative
> substitution: "use X wherever MuJoCo uses Y." If conventions match, say
> "Direct port — no translation needed."

---

## Specification

{The core of the spec. Divide into numbered sections (S1, S2, ...) for
independent changes. Each section should be self-contained enough that an
implementer can work on it without reading the others.}

### S1. {First change}

**File:** `{exact path relative to sim/L0/}`
**MuJoCo equivalent:** `{function_name()}` in `{source_file}.c` {lines N–M}
**Design decision:** {Why this approach over alternatives. What trade-offs
were considered. Reference specific codebase patterns this follows.}

{Describe the change. Include before/after Rust code where the before state
aids understanding of the change.}

**Before** (current code, if applicable):
```rust
// {Current implementation — only include if it helps show the change}
```

**After** (new implementation):
```rust
// {Rust implementation — line-for-line implementable, not pseudocode}
```

### S2. {Second change}

**File:** `{exact path}`
**MuJoCo equivalent:** `{function()}` in `{file}.c`
**Design decision:** {Why this approach.}

{Same structure as S1.}

{Continue S3, S4, ... as needed.}

---

## Acceptance Criteria

{Each AC must be specific, testable, and falsifiable. Every runtime AC follows
the three-part structure: (1) concrete input model/state, (2) exact expected
value or tolerance, (3) field to check.}

{Label each AC as either **runtime test** or **code review**.}

### AC1: {name} *(runtime test)*
**Given:** {concrete input model/state — e.g., "one hinge joint, motor actuator,
`ctrl[0] = 1.0`, `gear = [50.0]`"}
**After:** {operation — e.g., "`mj_step()`"}
**Assert:** {exact expected value — e.g., "`qfrc_actuator[0]` = 50.0 ± 1e-12"}
**Field:** {what to check — e.g., "`Data.qfrc_actuator`"}

### AC2: {name} *(runtime test)*
**Given:** ...
**After:** ...
**Assert:** ...
**Field:** ...

### AC{N}: {name} *(code review — not a runtime test)*
{Structural properties verified by inspection, not execution. Be specific
about what to look for — e.g., "No `unsafe` blocks in new code," or "All
public functions have `#[doc]` with MuJoCo reference," or "Module re-exports
match the consumer list in S3."}

---

## AC → Test Traceability Matrix

{Every AC maps to at least one test. Every test maps to at least one AC or
appears in the Supplementary Tests table. No orphans in either direction.}

| AC | Test(s) | Coverage Type |
|----|---------|---------------|
| AC1 ({brief name}) | T1 | Direct |
| AC2 ({brief name}) | T2, T3 | Direct + edge case |
| AC{N} ({code review}) | — | Code review (manual) |

> **Coverage Type values:** Direct (test exercises AC exactly), Edge case
> (tests boundary conditions of AC), Regression (verifies AC isn't broken by
> other changes), Integration (tests AC in combination with other features),
> Code review (manual inspection, no runtime test).

---

## Test Plan

{Organized by test type. Every test references the AC(s) it covers.}

### T1: {Happy path} → AC1
{Model setup. Expected output. Tolerance.}

### T2: {Edge case — zero mass / world body / nv=0} → AC2
{Model setup. Expected output. Why this edge case matters.}

### T3: {Negative case — flag NOT triggered / feature disabled} → AC{x}
{Verify the absence of behavior is also correct.}

### T4: {Regression — existing tests still pass} → AC{x}
{Compare before/after values. Tolerance.}

### T5: {Interaction — sleep / disable flags / multi-body} → AC{x}
{Combined scenarios.}

### Edge Case Inventory

{Explicit list of all edge cases this spec must test. Cross-reference with
MuJoCo Reference section to ensure nothing is missed.}

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| World body (`body_id == 0`) | {often a special case in MuJoCo — zero parent, no joint} | T{x} | AC{x} |
| Zero mass body | {changes inertia calculations, division-by-zero risk} | T{x} | AC{x} |
| {task-specific edge case} | {why} | T{x} | AC{x} |

### Supplementary Tests

{Tests that don't map 1:1 to a specific AC but are justified. Every
supplementary test needs a rationale — no orphan tests.}

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T{x} ({name}) | {what scenario} | {why this test exists despite no dedicated AC — e.g., "sanity check subsumed by AC7 regression suite"} |

---

## Risk & Blast Radius

### Behavioral Changes

{First-class section — not buried in a table. For each behavior that changes,
document the old behavior, new behavior, who is affected, and what they need
to do.}

| What changes | Old behavior | New behavior | Who is affected | Migration path |
|-------------|-------------|-------------|-----------------|---------------|
| {behavior} | {what happens today} | {what will happen after} | {downstream code / tests} | {what to do — or "None — transparent change"} |

### Files Affected

| File | Change | Est. lines |
|------|--------|-----------|
| `{path}` | {description of change} | {+N / -M / ~K modified} |
| `{path}` | {description of change} | ... |
| `{test path}` | New test file | +{N} |

### Existing Test Impact

{Name specific test functions/files and state whether breakage is expected or
unexpected. "Some tests might fail" is never acceptable.}

| Test | File | Expected impact | Reason |
|------|------|----------------|--------|
| `test_accelerometer_at_rest` | `sensor/tests.rs:142` | Pass (unchanged) | Does not exercise new code path |
| `test_phase4_regression_suite` | `conformance/phase4.rs` | 2 values change within tolerance | New computation is more correct; update expected values |
| `test_{name}` | `{file}:{line}` | {Pass/Fail/Value change} | {why} |

> **If you can't name specific tests:** Go read the test suite. The blast
> radius section exists to prevent surprises during implementation. Vague
> entries defeat the purpose.

---

## Execution Order

{State the implementation order. If this spec depends on other specs, state the
dependency chain. Be explicit about what must land first.}

1. {S1 first because ...}
2. {S2 after S1 because ...}
3. {S3 independent — can parallelize with S2}

---

## Out of Scope

{Explicitly list related work that is NOT part of this spec. Reference tracking
IDs (DT-xxx, §xxx) where applicable. This prevents scope creep during
implementation.}

- {Related but excluded item 1} — tracked as {DT-xxx}
- {Related but excluded item 2} — deferred to Phase {N}
