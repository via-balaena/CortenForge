# {TITLE} — Spec Quality Rubric

Grades the {spec_name} spec on {N} criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
P2–P8 but has P1 wrong is worse than useless: it would produce a clean,
well-tested implementation of the wrong behavior.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Tailoring Instructions

Before using this rubric, make it specific to the task. A generic rubric
produces a generic spec — and a generic spec produces conformance bugs.
Follow these three steps, **starting with MuJoCo specifics:**

### Step 1: Name the specific MuJoCo functions and fields *(most critical step)*

In every criterion that references MuJoCo behavior (P1, P2, P3 at minimum),
replace the generic placeholders with exact names from the C source:

- Not "the MuJoCo function" → but "`mj_sensorAcc()` in `engine_sensor.c`"
- Not "the relevant fields" → but "`data->cacc`, `data->cfrc_int`"
- Not "edge cases" → but "world body (`body_id == 0`), zero-mass bodies,
  `mjDSBL_SENSOR` flag"

### Step 2: Name the specific edge cases

For P1 and P5, replace the generic edge case list with the ones that actually
matter for this task. Remove irrelevant ones (e.g., `nu=0` is irrelevant for
sensor tasks). Add domain-specific ones (e.g., "sensor attached to world body"
for sensor tasks).

### Step 3: Add domain-specific criteria (P9+)

Every task has 1–3 dimensions that the 8 standard criteria don't cover. If
you can't think of any, you probably don't understand the task well enough —
go back to Phase 1 of the workflow.

**Common signals and the criteria they call for:**

| Signal | Criterion to add |
|--------|-----------------|
| New public API is introduced | API Design |
| Multiple call sites must be updated | Consumer Completeness |
| Multiple flags or conditions interact | Interaction/Compound Guards |
| Pipeline stage ordering matters | Pipeline Integration |
| New fields need init/reset/populate lifecycle | Data Integrity |
| Performance-sensitive hot loop | Performance Characterization |

---

## Criteria

{Keep all standard criteria (P1–P8) unless one is genuinely inapplicable
(rare). Add domain-specific criteria at the end (P9+). Most specs need 8–11
criteria total.}

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on P2–P8 and still be worthless if P1 is wrong — because
> an incorrect MuJoCo reference means every algorithm, every AC, and every
> test is verifying the wrong behavior. **Grade P1 first and grade it hardest.**
> If P1 is not A+, do not proceed to grading P2–P8 until it is fixed.

### P1. MuJoCo Reference Fidelity *(cardinal criterion)*

> Spec accurately describes what MuJoCo does — exact function names, field
> names, calling conventions, and edge cases. No hand-waving. This is the
> single most important criterion: everything else in the spec is downstream
> of getting the MuJoCo reference right. An error here propagates to every
> algorithm, every AC, and every test.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo function/field/flag cited with source file, line range, and exact behavior. Calling conventions (reference points, spatial layout, frame, index conventions) explicit. Edge cases ({name the specific edge cases for this task}) addressed with what MuJoCo does for each — verified against C source, not assumed from docs. Numerical expectations stated for at least one representative input (ideally from running MuJoCo). C code snippets included where they clarify non-obvious behavior. |
| **A** | MuJoCo behavior described correctly from C source. Minor gaps in edge-case coverage. |
| **B** | Correct at high level, but missing specifics (e.g., "MuJoCo shifts the velocity" without saying from where to where) — or description based on docs rather than C source. |
| **C** | Partially correct. Some MuJoCo behavior misunderstood, assumed, or invented. |

> **Tailoring note:** Replace the generic edge case placeholder above with the
> exact edge cases for this task. The A+ bar should name the specific MuJoCo
> functions the spec must cite — an author reading this bar should know exactly
> what C source to read. If you can't name the functions, you haven't done
> enough Phase 1 research.
>
> **Why this criterion is cardinal:** Every conformance bug in our history traces
> to one of these root causes: (1) MuJoCo behavior assumed from docs instead of
> verified against C source, (2) edge case missed because the C code wasn't read
> thoroughly, (3) convention difference (index, layout, reference point) not
> documented. P1 exists to prevent all three.

### P2. Algorithm Completeness

> Every algorithmic step is specified unambiguously. No "see MuJoCo source"
> or "TBD" gaps. Rust code is line-for-line implementable. The algorithm must
> reproduce MuJoCo's behavior exactly — not an approximation, not a "Rust
> improvement," but a faithful port that produces identical numerical results.

| Grade | Bar |
|-------|-----|
| **A+** | Every loop, every formula, every edge-case guard is written out in Rust. An implementer can type it in without reading MuJoCo source. No "verify against source" notes. Algorithm matches MuJoCo's computational steps — same formulas, same ordering, same guards. Any intentional deviation from MuJoCo's algorithm is explicitly justified with proof that it produces identical results. |
| **A** | Algorithm is complete and MuJoCo-conformant. One or two minor details left implicit. |
| **B** | Algorithm structure is clear but some steps are hand-waved or deferred, or algorithm deviates from MuJoCo without justification. |
| **C** | Skeleton only — "implement this somehow." |

### P3. Convention Awareness

> Spec explicitly addresses our codebase's conventions where they differ from
> MuJoCo (reference points, SpatialVector layout, field naming, etc.) and
> provides the correct translation. Convention mismatches are conformance bugs
> — a wrong reference point or flipped index produces numerically different
> results from MuJoCo, even if the algorithm is otherwise correct.

| Grade | Bar |
|-------|-----|
| **A+** | Every MuJoCo → CortenForge translation called out: reference points, SpatialVector layout `[angular; linear]`, field names, sleep arrays, index conventions, Model vs mjModel mapping. Convention difference table present with explicit porting rules — each rule verified to preserve numerical equivalence (e.g., "substitute `xpos[body_id]` for `subtree_com + 3*rootid` — produces identical values because our reference point matches for non-subtree-com bodies"). No empty or hand-waved porting rule cells. |
| **A** | Major conventions documented. Minor field-name mappings left to implementer. |
| **B** | Some conventions noted, others not — risk of silent mismatch during implementation. |
| **C** | MuJoCo code pasted without adaptation to our conventions. |

### P4. Acceptance Criteria Rigor

> Each AC is specific, testable, and falsifiable. Contains concrete values,
> tolerances, and model configurations. No "should work correctly." The gold
> standard: every runtime AC is a conformance assertion — "given this input,
> CortenForge produces the same value as MuJoCo."

| Grade | Bar |
|-------|-----|
| **A+** | Every runtime AC has the three-part structure: (1) concrete input model/state, (2) exact expected value or tolerance — ideally derived from MuJoCo's output on the same model, (3) what field/sensor to check. Could be copy-pasted into a test function. At least one AC per spec section has expected values verified against MuJoCo (stated as "MuJoCo produces X; we assert X ± tolerance"). Code-review ACs explicitly labeled as such with specific structural properties to verify. |
| **A** | ACs are testable. Some lack exact numerical expectations or MuJoCo-verified values. |
| **B** | ACs are directionally correct but vague ("output should change"), or values are manually calculated without MuJoCo verification. |
| **C** | ACs are aspirational statements, not tests. |

> **Tailoring note:** For this task, "concrete" means: {numerical tolerances?
> specific model configurations? flag combinations? sensor types?}. Specify
> what kind of concreteness the ACs need. Always prefer expected values from
> MuJoCo's actual output over hand-calculated values.

### P5. Test Plan Coverage

> Tests cover happy path, edge cases, regressions, and interactions. Each AC
> maps to at least one test. Negative cases (feature NOT triggered) are tested.
> At least one test per major feature is a direct MuJoCo conformance test:
> same model, same input, verify our output matches MuJoCo's output.

| Grade | Bar |
|-------|-----|
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test, every test maps to ≥1 AC or is in Supplementary Tests table with justification. Explicit edge case inventory: {name the specific edge cases}. Negative cases are first-class tests. At least one MuJoCo-vs-CortenForge conformance test per major code path (test comment states MuJoCo version and expected value source). Supplementary tests (if any) justified. |
| **A** | Good coverage. Minor edge-case gaps. Conformance tests present but not for all code paths. |
| **B** | Happy path covered. Edge cases and negative cases sparse. No explicit MuJoCo conformance tests. |
| **C** | Minimal test plan. |

> **Tailoring note:** Replace the generic edge case placeholder with the
> specific edge cases this task must test. Common ones: world body, zero mass,
> sleeping bodies, disabled flags, `nv=0`/`nu=0`, reset mid-step — but only
> include the ones relevant to this task.

### P6. Dependency Clarity

> Prerequisites, ordering constraints, and interactions with other specs are
> explicitly stated. No circular dependencies or implicit ordering.

| Grade | Bar |
|-------|-----|
| **A+** | Execution order is unambiguous. Each section states what it requires from prior sections/specs. Cross-spec interactions called out. Prerequisites include commit hashes for already-landed work. |
| **A** | Order is clear. Minor interactions left implicit. |
| **B** | Order suggested but not enforced — implementer must infer. |
| **C** | No ordering discussion. |

### P7. Blast Radius & Risk

> Spec identifies every file touched, every behavior that changes, and every
> existing test that might break. Surprises are spec bugs.

| Grade | Bar |
|-------|-----|
| **A+** | Complete file list with per-file change description. Behavioral changes in dedicated section with migration paths — each behavioral change states whether it moves *toward* MuJoCo conformance or is conformance-neutral. Existing test impact names specific test functions/files and states whether breakage is expected (behavior change) or unexpected (regression). If existing test values change, the new values are stated and verified as more MuJoCo-conformant. Data staleness guards addressed. Backward-compat callers confirmed unaffected or migration documented. |
| **A** | File list complete. Most regressions identified. |
| **B** | File list present but incomplete. Some regression risk unaddressed. |
| **C** | No blast-radius analysis. |

> **Tailoring note:** Name the crates/modules likely affected, the existing
> test suites to check (e.g., "Phase 4 39-test suite," "full sim domain
> 2,141+ tests"), and any data staleness guards (e.g., `EXPECTED_SIZE`
> constants).

### P8. Internal Consistency

> No contradictions within the spec. Shared concepts use identical terminology
> throughout. Section references are accurate.

| Grade | Bar |
|-------|-----|
| **A+** | Terminology is uniform. Every cross-reference (section, AC, test) is accurate. Concrete consistency checks pass: consumer/caller counts match between sections, file lists match between Specification and Files Affected, AC numbers match between AC section and Test Plan / Traceability Matrix, edge case lists are consistent across MuJoCo Reference and Test Plan. |
| **A** | Consistent. One or two minor terminology inconsistencies. |
| **B** | Some sections use different names for the same concept. |
| **C** | Contradictions between sections. |

> **Consistency checks to perform:**
> - Do file paths in Specification sections match the Files Affected table?
> - Do AC numbers in the AC section match the Traceability Matrix?
> - Do edge cases in the MuJoCo Reference appear in the Test Plan?
> - Do consumer/caller counts cited in one section match another?
> - Are field names spelled identically everywhere they appear?

{--- Add domain-specific criteria below as needed ---}

{### P9. {Domain-Specific Criterion}

> {What this criterion evaluates — specific to this spec's problem domain.}

| Grade | Bar |
|-------|-----|
| **A+** | {Exemplary bar} |
| **A** | {Solid bar} |
| **B** | {Gap bar} |
| **C** | {Insufficient bar} |}

---

## Rubric Self-Audit

Before using this rubric to grade the spec, grade the rubric itself. A flawed
rubric cannot produce an A+ spec no matter how many iterations you run. This
step was learned from DT-103, where the rubric had 8 gaps in Rev 1.

### Self-audit checklist

- [ ] **Specificity:** Every A+ bar is specific enough that two independent
      reviewers would agree on the grade. Test: could you assign a grade by
      pointing to specific lines in the spec? If the bar says "thorough" or
      "complete" without defining what that means, rewrite it.

- [ ] **Non-overlap:** No two criteria overlap significantly. Test: for any
      gap you might find, can you assign it to exactly one criterion? If a
      gap could belong to P1 or P3, sharpen the boundary.

- [ ] **Completeness:** The criteria cover every dimension of the task. Test:
      could the spec be A+ on all criteria but still have a meaningful gap? If
      yes, add a criterion.

- [ ] **Gradeability:** Each criterion maps to specific spec section(s). Test:
      for each criterion, can you name the section(s) you'd read to grade it?
      If not, either the criterion is too vague or the spec template is
      missing a section.

- [ ] **Conformance primacy:** P1 is tailored with specific MuJoCo C function
      names, source files, and edge cases. The A+ bar for P1 requires
      verification against C source (not just docs). At least P4 and P5
      reference MuJoCo-derived expected values. If the rubric could produce
      an A+ spec that diverges from MuJoCo's behavior, the rubric is broken.

### Criterion → Spec section mapping

Verify this mapping is complete — every criterion knows where to look, and
every spec section is graded by at least one criterion.

| Criterion | Spec Section(s) to Grade |
|-----------|-------------------------|
| P1 | MuJoCo Reference, Key Behaviors table, Convention Notes |
| P2 | Specification (S1, S2, ...) |
| P3 | Convention Notes, Specification code |
| P4 | Acceptance Criteria |
| P5 | Test Plan, AC→Test Traceability Matrix, Edge Case Inventory |
| P6 | Prerequisites, Execution Order |
| P7 | Risk & Blast Radius (Behavioral Changes, Files Affected, Existing Test Impact) |
| P8 | *Cross-cutting — all sections checked for mutual consistency* |
| P9+ | *{Map domain criteria to their spec sections}* |

Log any rubric fixes as R-prefixed gaps (R1, R2, ...) in the gap log.

---

## Scorecard

| Criterion | Grade | Evidence |
|-----------|-------|----------|
| P1. MuJoCo Reference Fidelity | | |
| P2. Algorithm Completeness | | |
| P3. Convention Awareness | | |
| P4. Acceptance Criteria Rigor | | |
| P5. Test Plan Coverage | | |
| P6. Dependency Clarity | | |
| P7. Blast Radius & Risk | | |
| P8. Internal Consistency | | |
{| P9. {Domain-Specific} | | |}

**Overall: {grade}**

> **Grading note:** The Evidence column is not optional. For each grade, cite
> the specific spec content that justifies it. For each gap, state exactly
> what's missing. "Looks good" is not evidence.

---

## Gap Log

{Track gaps found during grading and how they were resolved. This is the
audit trail that proves the spec earned its grade.}

**Rubric gaps** (from self-audit) use R-prefixed IDs. **Spec gaps** (from
grading) use G-prefixed IDs. This distinguishes rubric fixes from spec fixes.

| # | Criterion | Gap | Discovery Source | Resolution | Revision |
|---|-----------|-----|-----------------|------------|----------|
| R1 | {self-audit} | {rubric gap} | Rubric self-audit | {fix} | Rubric Rev {n} |
| G1 | P{x} | {what was missing} | {Initial grading / Re-grading / Implementation feedback / User review} | {what was added/fixed} | Rev {n} |
| G2 | P{x} | ... | ... | ... | Rev {n} |

> **Discovery Source values:** Initial grading, Re-grading (Rev N), Rubric
> self-audit, Implementation feedback, User review, Stress-test audit. This
> column tracks *when* gaps are found, which helps identify whether the process
> is catching gaps early (good) or late (needs improvement).
