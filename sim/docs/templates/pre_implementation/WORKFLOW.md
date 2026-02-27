# Spec Workflow — From Roadmap Task to A+ Spec

This document describes the iterative process for turning a roadmap task into
a shippable A+ spec. Follow it exactly. Do not skip steps. Do not rush.

The two templates in this directory are your starting points:
- `RUBRIC_TEMPLATE.md` — the grading rubric skeleton
- `SPEC_TEMPLATE.md` — the spec skeleton

> **The core principle:** The rubric defines "A+" *before* the spec exists. The
> spec is written *to the rubric*. The grading is *honest*. The iteration
> *converges*. If you do these four things, the spec will be A+. If you skip
> any one, it won't.

---

## Phase 1: Understand the Task

Before touching either template, build a mental model that's deep enough to
write a rubric. "Read the docs" is not sufficient — you need to reach the point
where you could *explain* the task to an implementer.

### 1.1 Read the roadmap entry

Read the task description in `sim/docs/ROADMAP_V1.md`. Understand the *why*
behind the task — is it a correctness fix, a conformance gap, an efficiency
improvement, or a new capability?

### 1.2 Read the codebase context

- Check `sim/docs/todo/` for related future_work files.
- Read the relevant crate's `docs/` directory.
- Read the source files that will be affected — not just the function, but the
  module it lives in, who calls it, and how its outputs are consumed.

### 1.3 Read the MuJoCo reference

Identify the exact MuJoCo implementation. This is the reference behavior
against which our spec will be judged.

### 1.4 Understanding checklist

Do not proceed to Phase 2 until you can check every box:

- [ ] I can name the exact MuJoCo C function(s) involved, their source files,
      and their call sites in the MuJoCo pipeline.
- [ ] I can describe the inputs and outputs of each function (field names,
      types, array dimensions).
- [ ] I can identify at least 3 edge cases and explain what MuJoCo does for
      each (e.g., world body, zero mass, sleeping bodies, disabled flags).
- [ ] I can describe the current CortenForge state — what we have, what's
      missing, and exactly where the gap is.
- [ ] I know which CortenForge crate(s) and module(s) will be affected.
- [ ] I know whether MuJoCo conventions differ from ours (reference points,
      spatial vector layout, field naming) for this task.

**If any box is unchecked, keep reading source code.** The rubric you build in
Phase 2 can only be as good as your understanding of the problem.

---

## Phase 2: Build the Rubric

The rubric comes first. It defines what "A+" means *before* the spec exists.
This prevents the spec from grading itself.

### 2.1 Create the rubric file

Copy `RUBRIC_TEMPLATE.md` into the task's spec directory. Name it
`{TASK}_RUBRIC.md`.

### 2.2 Keep all standard criteria (P1–P8)

These are the universal quality bars distilled from every rubric we've written.
Only drop one if it is genuinely inapplicable (this is rare — P6 Dependency
Clarity might be dropped for a fully standalone task, but think twice).

### 2.3 Tailor the standard criteria

The grade bars in the template are generic. Your job is to make them *specific*
to this task. A tailored rubric produces a better spec because the author knows
exactly what A+ looks like.

**How to tailor — three steps:**

1. **Name the specific MuJoCo functions/fields.** Replace generic references
   with exact names: not "MuJoCo function" but "`mj_sensorAcc()` in
   `engine_sensor.c`".

2. **Name the specific edge cases.** Replace generic "zero mass, world body"
   with the ones that actually matter for *this* task. Remove edge cases that
   are irrelevant; add domain-specific ones.

3. **Add domain criteria (P9+).** Every task has 1–3 things that the generic
   criteria don't cover. If you can't think of any, you probably don't
   understand the task well enough — go back to Phase 1.

**Example — tailoring P1 (MuJoCo Reference Fidelity):**

| | Generic (template) | Tailored (real rubric) |
|-|--------------------|-----------------------|
| A+ bar | "Every MuJoCo function/field/flag cited with source file + behavior." | "Spec cites `mj_subtreeVel`, `mj_comAcc`, and `mj_rnePostConstraint` with source files, inputs/outputs, and Coriolis correction formula. World body guard documented." |

The tailored version tells the spec author *exactly* what to research and write.

**Common domain criteria that signal they're needed:**

| Signal | Criterion to add |
|--------|-----------------|
| New public API is introduced | API Design |
| Multiple call sites must be updated | Consumer Completeness |
| Multiple flags or conditions interact | Interaction/Compound Guards |
| Pipeline stage ordering matters | Pipeline Integration |
| New fields need init/reset/populate lifecycle | Data Integrity |
| Performance-sensitive hot loop | Performance Characterization |

### 2.4 Audit the rubric itself

Before grading the spec, grade the rubric. This step was discovered in DT-103,
where the rubric itself had 8 gaps in its first revision. A flawed rubric
produces a flawed spec no matter how many iterations you do.

**Rubric self-audit (must pass all 4):**

- [ ] **Specificity:** Every A+ bar is specific enough that two independent
      reviewers would agree on the grade. If a bar says "spec is thorough"
      without saying *what* thoroughness means, rewrite it.
- [ ] **Non-overlap:** No two criteria overlap significantly. Each gap should
      map to exactly one criterion. If a gap could belong to P1 or P3, the
      criteria boundary is unclear — sharpen it.
- [ ] **Completeness:** The criteria cover every dimension of the task. Test:
      could the spec be A+ on all criteria but still have a gap? If yes, a
      criterion is missing.
- [ ] **Gradeability:** Each criterion can be graded by reading specific spec
      sections, not by "general impression." If you can't point to *which*
      section you'd read to grade a criterion, add guidance to the rubric.

Log any rubric fixes as R-prefixed gaps (R1, R2, ...) in the gap log with
the revision "Rubric Rev N."

Present the rubric to the user for review. Iterate until approved.

---

## Phase 3: Write the Spec (First Draft)

Now write the spec, using `SPEC_TEMPLATE.md` as the skeleton.

### 3.1 Create the spec file

Copy `SPEC_TEMPLATE.md` into the same directory. Name it `{TASK}_SPEC.md`.

### 3.2 Write to the A+ bar

For every section you fill in, open the rubric and read the A+ bar of the
criterion that grades that section. Write to *that* bar, not to "good enough."

**Which rubric criteria grade which spec sections:**

| Rubric Criterion | Spec Section(s) Graded |
|-----------------|------------------------|
| P1. MuJoCo Reference Fidelity | MuJoCo Reference, Convention Notes |
| P2. Algorithm Completeness | Specification (S1, S2, ...) |
| P3. Convention Awareness | Convention Notes, Specification code |
| P4. AC Rigor | Acceptance Criteria |
| P5. Test Coverage | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6. Dependency Clarity | Prerequisites, Execution Order |
| P7. Blast Radius & Risk | Risk & Blast Radius (all subsections) |
| P8. Internal Consistency | *Cross-cutting — all sections* |

### 3.3 Section-by-section guidance

**Problem Statement:** Lead with the strongest motivation. Is this a
correctness bug, a conformance gap, or an efficiency/ergonomic issue?
Correctness bugs are the strongest ("we compute wrong values"), conformance
gaps next ("MuJoCo does X, we don't"), ergonomic issues last ("the API is
awkward").

**MuJoCo Reference:** Cite exact function names, source files, field names.
Include C code snippets where they clarify behavior. Don't paraphrase when you
can quote.

**Convention Notes:** Use the formal table structure in the template: Field |
MuJoCo Convention | CortenForge Convention | Porting Rule. The "Porting Rule"
column is the actionable instruction that prevents silent bugs during
implementation.

**Specification sections (S1, S2, ...):** Each section needs: file path, MuJoCo
equivalent, design decision (why this approach over alternatives), and
before/after Rust code. Write real Rust, not pseudocode.

**Acceptance Criteria:** Every AC follows the three-part structure:
(1) concrete input model/state, (2) exact expected value or tolerance,
(3) field to check. Label each AC as either "runtime test" or "code review."

**AC→Test Traceability Matrix:** After the AC list, add the matrix. Every AC
maps to at least one test. Every test maps to at least one AC (or is in the
Supplementary Tests table with a justification).

**Test Plan:** Include an explicit Edge Case Inventory subsection. Include
Supplementary Tests for tests that don't map 1:1 to an AC — each one needs
a rationale for why it exists.

**Risk & Blast Radius:** Name *specific* test names and file paths for expected
breakage. "Some tests might fail" is never acceptable. If you don't know which
tests, go read the test suite.

### 3.4 What A+ vs A looks like

**A acceptance criterion (testable but vague on values):**
> AC3: After stepping a model with a heavy actuator, `qfrc_actuator` should
> reflect the applied force.

**A+ acceptance criterion (copy-pasteable into a test):**
> AC3: Model with one hinge joint + one motor actuator at `ctrl[0] = 1.0`,
> `gear = [50.0]`, `gainprm[0] = 1.0`. After `mj_step()`:
> `qfrc_actuator[0]` = 50.0 ± 1e-12. Field: `Data.qfrc_actuator`.

The difference: the A+ version names the model, the input values, the exact
expected output, the tolerance, and the field to check.

---

## Phase 4: Grade the Spec

This is where quality happens. Grade honestly — assume the spec has gaps and
look for what's *missing*, not what's *present*.

### 4.1 The honest grading protocol

1. **Grade before you want to.** The temptation is to keep polishing the spec
   until you "feel" it's ready, then grade it as A+ to confirm. Resist. Grade
   as soon as the first draft is complete. The gaps you find are the point.

2. **Assume the spec has gaps.** Your job is to find them, not to confirm their
   absence. If you grade every criterion A+ on the first pass, you're probably
   not looking hard enough.

3. **Look for what's missing, not what's present.** "The MuJoCo reference
   section is detailed" is not a grading rationale. "The MuJoCo reference
   section names all 3 functions, cites their source files, describes inputs/
   outputs, and covers the world body edge case" is.

4. **Provide evidence, not impressions.** For each grade, cite the specific
   spec content that justifies it. For each gap, state exactly what's missing
   and what the fix would look like.

### 4.2 Grade procedure

Go criterion by criterion (P1, P2, ... P{N}):

1. Read the A+ bar in the rubric.
2. Read the corresponding spec section(s) — refer to the mapping table in §3.2.
3. Assign a grade: A+, A, B, or C.
4. Write evidence: what's present that justifies the grade?
5. If not A+, write a specific gap description: what's missing, and what would
   close the gap?

### 4.3 Fill in the rubric

1. **Scorecard table** — grades and evidence for each criterion.
2. **Gap log** — every gap, numbered G1, G2, ..., with the criterion, the gap
   description, and (initially blank) resolution and revision columns.

### 4.4 Present results

Present to the user:
- The scorecard table (with evidence, not just grades)
- The full gap log
- Your honest assessment: how far is the spec from A+? How many revisions
  do you estimate?

### Anti-pattern: grading your own spec generously

The most common failure mode. You wrote the spec, so you know what you *meant*.
But the rubric asks what the spec *says*. If an implementer can't derive the
expected behavior from the spec text alone — without reading your mind — the
criterion is not A+.

---

## Phase 5: Close Gaps (Iterate to A+)

### 5.1 Fix gaps

For each gap in the gap log:

1. **Fix the gap in the spec.** Make the specific change the gap description
   calls for. Do not make unrelated changes — they introduce new gaps.
2. **Log the resolution** in the gap log (fill in the Resolution and Revision
   columns). Start the resolution with an action verb: "Added...", "Fixed...",
   "Reworded...", "Removed...".
3. **Re-grade the affected criterion.** Verify the fix actually closes the gap.
   A fix that introduces a new gap in a different criterion doesn't count.

### 5.2 Convergence tracking

**Gaps must decrease monotonically.** After each revision:

- Count total open gaps.
- If the count went *up*, stop and figure out why. Either the fix was
  incomplete, or the fix introduced a regression. Fix the regression before
  continuing.
- Present the convergence trend to the user: "Gaps: 8 → 3 → 1 → 0."

### 5.3 Revision discipline

- **Number your revisions.** Rev 1, Rev 2, etc. The gap log tracks which
  revision closed each gap. The rubric scorecard notes which revision was
  graded.
- **Don't boil the ocean.** Fix the gaps identified in the current grading
  round. Don't rewrite sections that are already A+.
- **If a revision introduces new gaps,** log them as new entries (G{N+1},
  G{N+2}, ...) and close them in the same or next revision.
- **Present each revision to the user** with the updated scorecard.

### Anti-pattern: fixing one gap introduces two new ones

This happens when a fix changes shared concepts (field names, terminology,
section structure) without updating all references. Before committing a fix,
search the entire spec for every reference to the changed concept.

---

## Phase 6: Final Confirmation

When you believe every criterion is A+:

1. **Do one final full re-grade.** Read the entire spec start to finish against
   every criterion. Fresh eyes catch things incremental fixes miss. Don't
   re-grade incrementally — read the whole spec as a coherent document.

2. **Present the final scorecard** to the user with the complete gap log
   showing all gaps closed (from G1 through G{N}, all with resolutions).

3. **Get explicit approval** from the user that the spec is ready for
   implementation.

Only after user approval does the spec status change from "Draft" to
"Approved." Only approved specs get implemented.

---

## Phase 7: Implement

Implementation follows the approved spec. The spec is the source of truth.

1. Follow the execution order defined in the spec.
2. Each spec section (S1, S2, ...) is a logical unit of work — commit after
   each one.
3. Verify each acceptance criterion as you go.
4. Run the test plan.
5. Update the spec status to "Implemented" when done.

If you discover a spec gap during implementation (something the spec didn't
anticipate), **stop and update the spec first.** Do not silently deviate.
Log the gap, fix the spec, re-grade the affected criterion, and get approval
for the spec change before continuing.

---

## Anti-Patterns

Distilled from actual mistakes across our spec history. If you catch yourself
doing any of these, stop and correct course.

| Anti-pattern | Why it's bad | What to do instead |
|-------------|-------------|-------------------|
| **Skipping the rubric** | The spec grades itself — every section looks "good enough" with no external bar. | Build the rubric first. Always. |
| **Generic rubric** | "MuJoCo reference is thorough" doesn't tell the spec author *what* to write. | Tailor every criterion with specific function names, field names, and edge cases. |
| **Grading your own spec generously** | You know what you *meant*; the rubric asks what the spec *says*. | Grade as if you're a different person reading the spec for the first time. |
| **Polishing before grading** | You spend 3 hours perfecting a section that the rubric would have flagged in 5 minutes. | Grade early, grade often. First drafts should be complete, not perfect. |
| **Fixing gaps without re-grading** | The fix may be incomplete or introduce new gaps. | Always re-grade the affected criterion after every fix. |
| **Non-monotonic convergence** | Gap count goes 8 → 5 → 7. Fixes are creating more problems than they solve. | Stop. Identify why fixes are regressing. Usually: shared concepts changed without full search-and-replace. |
| **"Some tests might fail"** | Vague blast radius means surprise breakage during implementation. | Name specific test files and test functions. Read the test suite. |
| **Pseudocode in Specification** | Implementer has to guess the real Rust. Bugs hide in the translation. | Write real Rust with exact types, field names, and function signatures. |
| **ACs without values** | "Should work correctly" is not testable. | Every runtime AC needs: input, expected value, tolerance, field to check. |
| **Orphan tests** | Tests in the plan that don't map to any AC — unclear what they're verifying. | Every test maps to an AC, or is in the Supplementary Tests table with a justification. |

---

## Decision Points

### When to split into multiple specs

Split when:
- The task has independent sections that could ship separately.
- Different sections affect different crates with no shared code changes.
- The combined spec would exceed ~500 lines (a sign of too much scope).

How: Create one spec per independent deliverable. Each spec has its own rubric.
Cross-reference between specs using "Prerequisites" sections.

### When an umbrella spec is needed

Use an umbrella when:
- 3+ related specs form a coherent feature/phase.
- The specs have ordering dependencies.
- You need a single document to track overall progress.

The umbrella spec defines the scope, ordering, and shared conventions. The
individual specs handle the details.

### When to create a sub-spec

Create a sub-spec when:
- During spec writing, you discover a prerequisite that needs its own spec
  (e.g., a utility function multiple specs will need).
- A section grows beyond ~150 lines and has its own independent ACs.

The sub-spec gets its own rubric and grading cycle. The parent spec lists it as
a prerequisite.

---

## Summary

```
Roadmap Task
    │
    ▼
Phase 1: Understand (read roadmap + MuJoCo + codebase → pass checklist)
    │
    ▼
Phase 2: Build rubric (RUBRIC_TEMPLATE → tailor → self-audit → user approval)
    │
    ▼
Phase 3: Write spec first draft (SPEC_TEMPLATE → write to A+ bar → complete)
    │
    ▼
Phase 4: Grade spec against rubric (honest grading → scorecard + gap log)
    │
    ▼
Phase 5: Close gaps (fix → re-grade → converge monotonically → repeat)
    │
    ▼
Phase 6: Final re-grade + user approval
    │
    ▼
Phase 7: Implement from approved spec
```
