# {TITLE} — Spec Quality Rubric

Grades the {spec_name} spec on {N} criteria. Target: A+ on every criterion
before implementation begins. A+ means "an implementer could build this
without asking a single clarifying question — and the result would produce
numerically identical output to MuJoCo."

**MuJoCo conformance is the cardinal goal.** Every criterion in this rubric
ultimately serves conformance. P1 (MuJoCo Reference Fidelity) is the most
important criterion — grade it first and hardest. A spec that scores A+ on
all other criteria but has P1 wrong is worse than useless: it would produce
a clean, well-tested implementation of the wrong behavior.

> **Extension specs:** If the spec introduces behavior that intentionally
> extends beyond MuJoCo (e.g., a higher-fidelity muscle model, a new sensor
> type MuJoCo doesn't have), P1 has a **split mandate**: the MuJoCo-conformant
> subset must match MuJoCo exactly, and the extension must be explicitly
> documented as a CortenForge extension with a clear boundary. State which
> parts of the spec are conformance-critical and which are extensions in the
> header above. See Phase 5 Spec C (Hill-type muscle) for a worked example.

Grade scale: A+ (exemplary) / A (solid) / B (gaps) / C (insufficient).
Anything below B does not ship.

---

## Tailoring Instructions

Before using this rubric, make it specific to the task. A generic rubric
produces a generic spec — and a generic spec produces conformance bugs.
Follow these steps, **starting with empirical verification:**

### Step 1: Establish Empirical Ground Truth *(most critical step)*

Run MuJoCo and grep the codebase **before writing any criterion bars.** A
rubric written from header-file assumptions instead of empirical verification
will produce criterion bars that miss real MuJoCo behavior — and the spec will
inherit those blind spots. This was learned the hard way: Phase 5 Spec D R6
("Rev 0 rubric had no Empirical Ground Truth section — rubric was writing bars
based on header-file assumptions"), Phase 5 Spec D R8 ("history buffer layout
is contiguous blocks, NOT interleaved pairs" — discovered only by running
MuJoCo), Phase 6 Spec C R1 ("`geompoint` does not exist" — discovered only by
running MuJoCo).

Fill in the Empirical Ground Truth section below. If you find yourself writing
criterion bars without having filled in EGT first, stop — you're building on
assumptions.

### Step 2: Name the specific MuJoCo functions and fields

In every criterion that references MuJoCo behavior (P1, P2, P3 at minimum),
replace the generic placeholders with exact names from the C source:

- Not "the MuJoCo function" → but "`mj_sensorAcc()` in `engine_sensor.c`"
- Not "the relevant fields" → but "`data->cacc`, `data->cfrc_int`"
- Not "edge cases" → but "world body (`body_id == 0`), zero-mass bodies,
  `mjDSBL_SENSOR` flag"

### Step 3: Name the specific edge cases

For P1 and P5, replace the generic edge case list with the ones that actually
matter for this task. Remove irrelevant ones (e.g., `nu=0` is irrelevant for
sensor tasks). Add domain-specific ones (e.g., "sensor attached to world body"
for sensor tasks).

### Step 4: Research the CortenForge codebase

Grep the codebase for the specific files, functions, match sites, and line
ranges that the task will affect. Document these in the Codebase Context
subsection of Empirical Ground Truth so the spec author knows exactly what
code to address. Every rubric that skipped this step discovered missing files
mid-grading (see Spec B R6, Spec D R26).

Examples of what to find:
- Exhaustive match sites that will break when new enum variants are added
- Builder pipeline files and the push→transfer→postprocess flow
- Staleness guard constants (e.g., `EXPECTED_SIZE`) that will need updating
- Parser attribute gates that could silently skip new attributes
- Manual `impl Clone`/`impl Default` blocks where forgetting a field is a
  silent bug
- Catch-all `_ => {}` arms that silently skip new variants (see Spec B R10)

### Step 5: Add domain-specific criteria (P9+)

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
| New fields need init/reset/populate lifecycle | Data Lifecycle |
| Performance-sensitive hot loop | Performance Characterization |
| Non-trivial geometric derivation | Geometric Correctness |
| Enum/type-system refactoring or new variants | Type Architecture |
| Compiler/builder validation rules to match MuJoCo | Compiler Validation Completeness |
| Replicating a pattern from another subsystem (e.g., sensor history mirrors actuator history) | Cross-Subsystem Parity |
| Feature reads from two objects (e.g., distance between two geoms) | Dual Object Resolution |
| Mathematical transform chains (rotation, projection, transport) | Transform Correctness |
| Complex conformance depth (not a simple field read) | Conformance Depth |
| New utility function introduced as prerequisite | Interface Contract |
| Active force environment matters for correctness | Force-Environment Conformance |

### Step 6: Verify scope against parent *(if spec is part of a phase/umbrella)*

If this spec is part of a larger phase or umbrella spec, fill in the Scope
Adjustment section. Empirical verification (Step 1) regularly discovers scope
corrections — features that don't exist, features the umbrella missed, features
that require infrastructure outside the phase's scope.

---

## Scope Adjustment

{Delete this section if the spec is standalone (not part of a phase/umbrella).

If the spec is part of a phase or umbrella, document any scope corrections
discovered during empirical verification. This section is the authoritative
record of what changed from the umbrella's original task list and why.

| Umbrella claim | MuJoCo reality | Action |
|----------------|---------------|--------|
| {feature X listed in umbrella} | {what MuJoCo 3.x.x actually shows} | {In scope / Drop / Defer to DT-{N} / Add (newly discovered)} |

**Final scope:** {numbered list of what this spec actually covers}

This section was added because Phase 6 Spec C discovered `geompoint` doesn't
exist (replaced with `geomfromto`), Phase 6 Spec D discovered `interval` was
missing from the umbrella, and Phase 5 Spec C needed to document its split
mandate.}

---

## Empirical Ground Truth

{This section is **required.** Fill it in before writing criterion bars.

Number each subsection EGT-N for cross-referencing from criterion bars, ACs,
and the gap log (e.g., "per EGT-3, MuJoCo produces X").

### MuJoCo behavioral verification

**What to verify empirically (run MuJoCo, don't assume from docs):**
- Exact field values MuJoCo produces for representative models (these become
  the expected values in P4 ACs)
- Edge case behavior (what MuJoCo actually does, not what docs suggest)
- Validation rules the compiler enforces (accepted vs rejected inputs)
- Initial data state (zero-initialized? pre-populated? by what formula?)
- Runtime interactions that affect the compile-time design (e.g., how
  downstream consumers use the fields being specified)

**Format:** State the MuJoCo version, the Python/C test used, and the exact
values observed. This section is the authoritative reference — any claim in
the spec that contradicts it is wrong.

### Codebase context

**What to find in the CortenForge codebase:**
- Every file, function, and match site the task will touch (with file:line)
- Exhaustive match sites for enums being extended (the exact lines that will
  break or silently skip new variants)
- Existing field mappings between MuJoCo C names and CortenForge Rust names
- Builder pipeline flow (parse → push → transfer → postprocess) for the
  relevant subsystem
- Existing tests that exercise the code paths being changed

Document findings as tables with file:line references. These become the
authoritative codebase reference for P2 (algorithm sites), P7 (blast radius),
and domain-specific criteria.

### EGT-1: {first behavioral fact}

{MuJoCo version, C source location, empirical values, derivation.}

### EGT-2: {second behavioral fact}

{...}

Delete this guidance block and replace with your empirical findings.}

---

## Criteria

{Keep all standard criteria (P1–P8) unless one is genuinely inapplicable
(rare). Add domain-specific criteria at the end (P9+). Most specs need 8–11
criteria total.}

> **Criterion priority:** P1 (MuJoCo Reference Fidelity) is the cardinal
> criterion. MuJoCo conformance is the entire reason CortenForge exists.
> A spec can be A+ on every other criterion and still be worthless if P1 is
> wrong — because an incorrect MuJoCo reference means every algorithm, every
> AC, and every test is verifying the wrong behavior. **Grade P1 first and
> grade it hardest.** If P1 is not A+, do not proceed to grading other
> criteria until it is fixed.

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
>
> **Extension specs:** If this spec has a split mandate (some behavior matches
> MuJoCo, some is a CortenForge extension), the A+ bar must clearly delineate
> which parts require MuJoCo conformance and which parts are extensions. The
> conformance subset must meet the full A+ bar. The extension subset must be
> documented as non-MuJoCo with a clear boundary statement. See Phase 5 Spec C
> P1 for a worked example of a split mandate.

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
| **A+** | Every runtime AC has the three-part structure: (1) concrete input model/state, (2) exact expected value or tolerance — ideally derived from MuJoCo's output on the same model, (3) what field/sensor to check. Could be copy-pasted into a test function. At least one AC per major feature has expected values verified against MuJoCo (stated as "MuJoCo produces X; we assert X ± tolerance"). Code-review ACs explicitly labeled as such with specific structural properties to verify. |
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
| **A+** | AC→Test traceability matrix present — every AC maps to ≥1 test, every test maps to ≥1 AC or is in Supplementary Tests table with justification. Explicit edge case inventory: {name the specific edge cases}. Negative cases are first-class tests. At least one MuJoCo-vs-CortenForge conformance test per major code path (test comment states MuJoCo version and expected value source). At least one test uses a non-trivial model (multi-body, multi-joint, or multi-element) to catch bugs that only appear in non-trivial configurations. Supplementary tests (if any) justified. |
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
> constants). Use the Codebase Context subsection of EGT to enumerate every
> exhaustive match site — these are the highest-risk blast-radius items.

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

Expect multiple revision rounds. Across Phases 5–6, rubrics averaged 4–8
revisions before reaching A+ (Spec B Phase 5 reached Rev 8 with 23 R-gaps).
Each round of self-audit, codebase research, and empirical verification
typically surfaces 3–8 gaps. This is normal — the rubric is doing its job.
Track every revision in the gap log.

### Self-audit checklist

- [ ] **Specificity:** Every A+ bar is specific enough that two independent
      reviewers would agree on the grade. Test: could you assign a grade by
      pointing to specific lines in the spec? If the bar says "thorough" or
      "complete" without defining what that means, rewrite it.

- [ ] **Non-overlap:** No two criteria overlap significantly. Test: for any
      gap you might find, can you assign it to exactly one criterion? If a
      gap could belong to P1 or P3, sharpen the boundary. For every pair of
      criteria that intentionally share content (common with P1 vs domain
      criteria), write a one-sentence boundary explanation: same content
      graded from which different angle? (e.g., "P1 grades whether the spec
      GOT the MuJoCo reference right; P10 grades whether the geometric
      derivation is mathematically rigorous.")

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

- [ ] **Empirical grounding:** The Empirical Ground Truth section is filled in
      with verified MuJoCo values and codebase context. Every A+ bar that
      references specific MuJoCo behavior has a corresponding EGT-N entry.
      No criterion bar was written from header-file assumptions.

> **Important:** Replace the generic descriptions above with rubric-specific
> evidence when checking each box. Name the exact functions, edge cases, and
> criteria boundaries that justify each checkmark. A checked box without
> specific evidence is a rubber stamp, not an audit.

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

**Overall: {grade} (Rev {n})**

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
