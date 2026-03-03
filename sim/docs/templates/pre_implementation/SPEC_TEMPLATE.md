# {TITLE} — Spec

**Status:** Draft
**Phase:** Roadmap Phase {N} — {phase_name}
**Effort:** {S/M/L}
**MuJoCo ref:** `{function_name()}` in `{source_file}.c`, lines {N–M}
**MuJoCo version:** {version the C source was read from — e.g., "3.5.0"}
**Test baseline:** {current domain test count — e.g., "2,238+ sim domain tests"}
**Prerequisites:**
- {list prerequisites — include commit hashes for already-landed work}
- {e.g., "DT-103 spatial transport utilities (landed in `abc1234`)"}
- {or "None"}

{**Independence** (include if spec is part of a phase with parallel specs)**:**
This spec is independent of / depends on {Spec X, Spec Y} per the umbrella
dependency graph. Shared files: {name them and explain why they don't conflict,
or state "None"}.}

> **Conformance mandate:** This spec exists to make CortenForge produce
> numerically identical results to MuJoCo for the feature described below.
> Every algorithm, every acceptance criterion, and every test is in service
> of this goal. The MuJoCo C source code is the single source of truth —
> not the MuJoCo documentation, not intuition about "what should happen,"
> not a Rust-idiomatic reinterpretation. When in doubt, read the C source
> and match its behavior exactly.

{**Extension specs:** If this spec introduces behavior that intentionally extends
beyond MuJoCo, replace the conformance mandate above with a split version that
names: (1) which parts must match MuJoCo exactly, (2) which parts are
CortenForge extensions, and (3) the source of truth for each. See Phase 5
Spec C (Hill-type muscle) for a worked example. Example:

> **Conformance mandate (split):** This spec has a split conformance posture:
> 1. **{conformant part}** — MUST produce numerically identical results to
>    MuJoCo's `{function()}`. The MuJoCo C source code is the source of truth.
> 2. **{extension part}** — is a CortenForge EXTENSION. {source} is the source
>    of truth for this behavior.

}

---

## Scope Adjustment

{Delete this section if the spec is standalone (not part of a phase/umbrella).

If the spec is part of a phase or umbrella, document any scope corrections
discovered during empirical verification against MuJoCo. This section records
what changed from the umbrella's original task list and why.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| {feature X listed in umbrella} | {what MuJoCo 3.x.x actually shows} | {In scope / Drop / Defer to DT-{N} / Add (newly discovered)} |

**Final scope:** {numbered list of what this spec actually covers.}

This section was necessary in Phase 6 Spec C (dropped `geompoint`, deferred
`camprojection`), Phase 6 Spec D (added `interval` attribute discovered during
empirical testing), and Phase 5 Spec C (split mandate for extension).}

---

## Problem Statement

{What is wrong, missing, or non-conformant. Be specific — name the function,
field, or behavior. State the gap between "what we do" and "what MuJoCo does."
2–4 paragraphs max. The gap should be stated in concrete MuJoCo terms: "MuJoCo
computes X in `function()` at `file.c:N`; CortenForge does not implement this /
computes it incorrectly."}

{Lead with the strongest motivation. Categorize the problem:}

{- **Correctness bug** — "We compute wrong values for X. MuJoCo produces Y; we produce Z." (Strongest — ship-blocking.)}
{- **Conformance gap** — "MuJoCo implements X in `function()`; we don't implement it yet." (Strong — blocks feature completeness.)}
{- **Efficiency/ergonomic issue** — "X works and matches MuJoCo, but the API is awkward / performance is poor." (Valid but weaker — conformance is already achieved.)}

---

## MuJoCo Reference

> **This is the most important section of the spec.** Everything downstream —
> the algorithm, the acceptance criteria, the tests — is derived from what's
> documented here. If this section is wrong, everything is wrong. Read the
> actual MuJoCo C source code and describe what it does, not what you think
> it should do.

{Describe how MuJoCo handles this. For each function:}
{- **Source file and line range** — e.g., `engine_forward.c:1234–1289`}
{- **Function signature** — inputs, outputs, side effects}
{- **Algorithm** — step by step, with C code snippets for non-obvious logic}
{- **Edge cases** — what happens for zero, null, disabled, world body, etc.}
{- **Numerical behavior** — what value does MuJoCo produce for a simple test
   input? (This becomes an AC expected value.)}

{**Structuring a large MuJoCo Reference:** When the reference covers multiple
functions, struct layouts, or subsystems, organize by topic — not by the order
you discovered things. Common subsections:}
{- Per-function algorithm descriptions (with C snippets)}
{- Struct/field layouts (spec-level, compiled model, data)}
{- Compiler validation rules (what MuJoCo accepts vs rejects, with exact error messages)}
{- Edge cases (verified against MuJoCo, not assumed from docs)}
{- Numerical verification data (values from running MuJoCo — these become AC expected values)}

### Key Behaviors

| Behavior | MuJoCo | CortenForge (current) |
|----------|--------|-----------------------|
| {behavior 1} | {what MuJoCo does — cite function + source file + line range} | {what we do / conformance gap} |
| {behavior 2} | ... | ... |

### Convention Notes

{Document every MuJoCo → CortenForge translation. Convention mismatches are
the #1 source of conformance bugs — a wrong reference point or flipped index
silently produces different numerical results. Be exhaustively explicit. If
there are no convention differences, state "Direct port — all conventions
match" and explain why you're confident.}

| Field | MuJoCo Convention | CortenForge Convention | Porting Rule |
|-------|-------------------|------------------------|-------------|
| {field/concept} | {what MuJoCo does} | {what we do} | {substitution rule: "use X wherever MuJoCo uses Y"} |
| {e.g., `cacc` ref point} | {at `subtree_com[rootid]`} | {at `xpos[body_id]`} | {Substitute `xpos[body_id]` for `subtree_com + 3*rootid`} |
| {e.g., spatial layout} | {`[angular; linear]`} | {`[angular; linear]` (same)} | {Direct port — no translation needed} |

> **Porting Rule column:** The actionable instruction an implementer follows
> when translating MuJoCo C code to our Rust. Written as an imperative
> substitution: "use X wherever MuJoCo uses Y." If conventions match, say
> "Direct port — no translation needed."

{### Cross-Subsystem Parity

Include this subsection when the spec implements for one subsystem what already
exists for another (e.g., sensor history mirroring actuator history). A
field-by-field parity table prevents subtle inconsistencies.

| Existing field | New field | Type | Default | Parity |
|---------------|----------|------|---------|--------|
| `actuator_nsample` (`model.rs:592`) | `sensor_nsample` | `Vec<i32>` | `0` | Exact match |
| *(no equivalent)* | `sensor_interval` | `Vec<(f64, f64)>` | `(0.0, 0.0)` | **New — subsystem-specific** |

Delete this subsection if the spec does not mirror an existing subsystem.}

{### Conformant vs Extension Boundary

Include this subsection for extension specs (split conformance mandate). Table
showing which aspects are MuJoCo-conformant and which are extensions.

| Aspect | MuJoCo behavior | This spec | Conformance |
|--------|----------------|-----------|-------------|
| {aspect 1} | {what MuJoCo does} | {what we do} | **IDENTICAL** — conformant |
| {aspect 2} | {what MuJoCo does} | {what we do differently} | **EXTENSION** — documented |

Delete this subsection for pure conformance specs.}

---

## Architecture Decisions

{Delete this section if no cross-cutting design decisions are needed — i.e.,
every decision is local to a single S-section and fits in its `**Design
decision:**` field.

Include this section when a design choice affects multiple S-sections or
involves a non-obvious trade-off between alternatives. Each decision should
follow the pattern: state the problem, enumerate alternatives, justify the
choice.

### AD-1: {Decision name}

**Problem:** {What needs to be decided and why it's non-obvious.}

**Alternatives evaluated:**

| Option | Approach | Pros | Cons |
|--------|----------|------|------|
| 1 | {approach} | {pros} | {cons} |
| 2 | {approach} | {pros} | {cons} |

**Chosen:** Option {N} — {brief rationale referencing pros/cons above}.

This section was necessary in Phase 5 Spec C (Gain/Bias type architecture,
state management) and Phase 6 Spec A (architectural decisions for objtype
dispatch). Most specs don't need it — the per-S-section `**Design decision:**`
field suffices for local choices.}

---

## Specification

{The core of the spec. Divide into numbered sections (S1, S2, ...) for
independent changes. Each section should be self-contained enough that an
implementer can work on it without reading the others.}

{**Conformance rule:** Every algorithm in this section must produce numerically
identical results to the MuJoCo C code cited in the MuJoCo Reference section.
If the Rust code deviates from MuJoCo's approach (e.g., using a different but
mathematically equivalent formulation), the deviation must be explicitly called
out with a proof of numerical equivalence. Silent deviations are conformance
bugs.}

### S1. {First change}

**File:** `{exact path relative to sim/L0/}` {with line range if modifying existing code}
**MuJoCo equivalent:** `{function_name()}` in `{source_file}.c` {lines N–M}
**Design decision:** {Why this approach over alternatives. What trade-offs
were considered. Reference specific codebase patterns this follows. If the
implementation differs structurally from MuJoCo's C code, explain why and
prove numerical equivalence. For cross-cutting decisions, reference AD-N
above instead of repeating the rationale.}

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

**File:** `{exact path}` {with line range}
**MuJoCo equivalent:** `{function()}` in `{file}.c`
**Design decision:** {Why this approach.}

{Same structure as S1.}

{Continue S3, S4, ... as needed.}

---

## Acceptance Criteria

{Each AC must be specific, testable, and falsifiable. Every runtime AC follows
the three-part structure: (1) concrete input model/state, (2) exact expected
value or tolerance, (3) field to check.}

{**Conformance ACs:** The gold standard runtime AC states: "MuJoCo produces X
for this input; CortenForge must produce X ± tolerance." Where possible, derive
expected values by running MuJoCo on the test model — not by hand-calculating
what you think the answer should be. State the MuJoCo version used to produce
expected values.}

{Label each AC as either **runtime test** or **code review**. For runtime tests,
also label whether the expected value is **MuJoCo-verified** (from running
MuJoCo) or **analytically derived** (from manual calculation of what MuJoCo
should produce).}

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
about what to look for — e.g., "No `unsafe` blocks in new code," "No
`#[allow(unreachable_patterns)]` in modified files," "New enum variant has
an arm in all exhaustive match sites listed in Files Affected."}

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

{**Conformance test convention:** Tests that verify against MuJoCo's actual
output must include a comment in the test body stating: (1) MuJoCo version,
(2) how the expected value was obtained (e.g., "ran `test_model.xml` through
MuJoCo 3.5.0 Python bindings"), (3) the exact expected value. This creates an
audit trail for future MuJoCo version updates.}

### T1: {MuJoCo conformance — happy path} → AC1
{Model setup. MuJoCo-verified expected output. Tolerance. State MuJoCo version.}

### T2: {Edge case} → AC2
{Model setup. Expected output. Why this edge case matters. State whether
expected value is MuJoCo-verified or analytically derived.}

### T3: {Negative case — flag NOT triggered / feature disabled} → AC{x}
{Verify the absence of behavior is also correct.}

### T4: {Regression — existing tests still pass} → AC{x}
{Compare before/after values. Tolerance.}

### T5: {Interaction — multi-body / combined features} → AC{x}
{Combined scenarios. At least one test should use a non-trivial model to catch
bugs that only appear in multi-body or multi-element configurations.}

### Edge Case Inventory

{Explicit list of all edge cases this spec must test. Cross-reference with
MuJoCo Reference section to ensure nothing is missed. Replace the examples
below with edge cases specific to this task.}

| Edge Case | Why it matters | Test(s) | AC(s) |
|-----------|---------------|---------|-------|
| {task-specific edge case 1} | {why — cite MuJoCo behavior if relevant} | T{x} | AC{x} |
| {task-specific edge case 2} | {why} | T{x} | AC{x} |
| {task-specific edge case 3} | {why} | T{x} | AC{x} |

{Common edge case patterns (include only those relevant to this task):
body/sensor tasks — world body, zero-mass body, sleeping bodies, disabled flags;
actuator tasks — `nv=0`, `nu=0`, non-unit gear, position-actuator fingerprint;
parser tasks — invalid keywords, missing attributes, default class inheritance;
buffer tasks — zero-length, negative values, boundary indices.}

### Supplementary Tests

{Tests that don't map 1:1 to a specific AC but are justified. Every
supplementary test needs a rationale — no orphan tests.}

| Test | What it covers | Rationale |
|------|---------------|-----------|
| T{x} ({name}) | {what scenario} | {why this test exists despite no dedicated AC} |

---

## Risk & Blast Radius

### Behavioral Changes

{First-class section — not buried in a table. For each behavior that changes,
document the old behavior, new behavior, who is affected, and what they need
to do.}

| What changes | Old behavior | New behavior | Conformance direction | Who is affected | Migration path |
|-------------|-------------|-------------|----------------------|-----------------|---------------|
| {behavior} | {what happens today} | {what will happen after} | {Toward MuJoCo / Already conformant / N/A} | {downstream code / tests} | {what to do — or "None — transparent change"} |

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
| `test_{name}` | `{file}:{line}` | Pass (unchanged) | Does not exercise new code path |
| `test_{name}` | `{file}:{line}` | 2 values change within tolerance | New computation is more correct; update expected values |
| `test_{name}` | `{file}:{line}` | {Pass/Fail/Value change} | {why} |

> **If you can't name specific tests:** Go read the test suite. The blast
> radius section exists to prevent surprises during implementation. Vague
> entries defeat the purpose.

{### Non-Modification Sites

Include this subsection when the spec touches code near other important match
sites, consumers, or callers that should explicitly NOT be modified. Naming
them prevents accidental changes and documents that the analysis was thorough.

| File:line | What it does | Why NOT modified |
|-----------|-------------|-----------------|
| `{path}:{line}` | {description} | {reason — e.g., "handles a different enum variant", "already conformant"} |

Delete this subsection if the spec's blast radius is small and contained.}

---

## Execution Order

{State the implementation order. If this spec depends on other specs, state the
dependency chain. Be explicit about what must land first.}

{After each section lands, run the relevant conformance tests to verify MuJoCo
equivalence before proceeding. Do not accumulate untested conformance changes
across multiple sections.}

1. {S1 first because ...} → verify S1 conformance
2. {S2 after S1 because ...} → verify S2 conformance
3. {S3 independent — can parallelize with S2} → verify S3 conformance

---

{## Performance Characterization

Include this section when the spec introduces a hot loop, clones a model,
allocates large buffers, or runs simulation steps at compile time. Characterize
the cost and state whether it's acceptable for target model complexity.

**Cost:** {asymptotic complexity, e.g., "O(nu × steps) integration steps where
steps = inttotal/timestep (default 1,000)"}
**Worst case:** {e.g., "humanoid with 20 muscle actuators: ~40,000 integration
steps at compile time, ~1–5s"}
**Acceptable?** {yes/no for target use cases, with optimization notes for future}

Delete this section if the spec has no performance-sensitive code paths.
Phase 5 Spec A needed this for `mj_setLengthRange` (simulation loop at compile
time); most specs don't.

---}

## Out of Scope

{Explicitly list related work that is NOT part of this spec. Reference tracking
IDs (DT-xxx, §xxx) where applicable. This prevents scope creep during
implementation. For each excluded item, state the conformance impact: does
excluding it leave a conformance gap? If so, is it acceptable for v1.0?}

- {Related but excluded item 1} — tracked as {DT-xxx}. {Conformance impact: none / gap acceptable for v1.0 / gap requires future work}
- {Related but excluded item 2} — deferred to Phase {N}. {Conformance impact: ...}
