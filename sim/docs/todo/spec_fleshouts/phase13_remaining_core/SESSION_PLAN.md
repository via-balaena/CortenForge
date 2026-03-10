# Phase 13 — Remaining Core: Session Plan

18 sessions, each self-contained. The umbrella spec
(`PHASE13_UMBRELLA.md`) coordinates across context boundaries.

Phase 13 closes the gap between "conformance suite passes" and "v1.0
ships." It has two categories:

1. **Conformance-critical** (5 constraint solver items) — un-ignores
   the 26 golden flag tests left `#[ignore]`d by Phase 12. These are
   the single remaining dependency for v1.0 conformance.
2. **Infrastructure** (§46 composite bodies, §66 plugin system) —
   v1.0 features that don't affect conformance pass/fail but are needed
   for a complete MuJoCo-compatible API surface.

The conformance items are sequenced first (Session 0 diagnostic →
Specs A → B → golden flag gate). Infrastructure follows (Specs C → D).
This lets the conformance gate close early — once Specs A+B land and
golden flags pass, the physics engine is v1.0-conformant regardless
of §46/§66 progress.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Scope

### Phase 13 Items (from ROADMAP_V1.md)

| Task | Tier | Category | Spec | Description | Current State |
|------|------|----------|------|-------------|---------------|
| DT-39 | T2 | Conformance | A | `diagApprox` remaining code paths — body-weight diagonal approximation for equality constraints | Partially fixed in Phase 12 S15 Round 2; weld equality path may have gaps |
| DT-23 | T2 | Conformance | A | Per-DOF friction loss solver params (`dof_solref_fri`/`dof_solimp_fri`) | Fields exist on Model; assembly may not route them correctly |
| DT-19 | T3 | Conformance | B | QCQP-based cone projection for normal+friction force projection (MuJoCo PGS style) | **Already implemented** — `qcqp2/3/nd` in `qcqp.rs`, 13 unit tests. Needs conformance verification, not fresh implementation. |
| DT-128 | T2 | Conformance | B | PGS early termination — `improvement * scale < tolerance` convergence check | **Not implemented** — solver always runs `max_iters` |
| DT-129 | T3 | Conformance | B | PGS warmstart two-phase projection — ray+QCQP on warmstart forces | **Partially implemented** — cold/warm dual-cost selection exists; pre-iteration projection missing. Independent of DT-19 (uses `classify_constraint_states`, not QCQP). |
| §46 | — | Infrastructure | C | `<composite>` procedural body generation (grid, rope, cable, cloth, box, cylinder, ellipsoid) | Zero existing code; clean slate |
| §66 | — | Infrastructure | D | Plugin/extension system — `<plugin>`/`<extension>` MJCF parsing + Rust trait dispatch | Zero existing code; clean slate |
| FILTERPARENT | — | Conformance | S0→A or B | `golden_disable_filterparent` ~51.9 divergence when parent-child pairs enter collision. Session 0 diagnoses root cause and assigns to Spec A (assembly) or Spec B (solver), or creates a new DT item if it's an independent collision filtering bug. | Gate assessment attributes to "collision filtering"; may be compounding effect of baseline ~1.69 |

### Phase 8 Remaining (not moved to Phase 13, but still v1.0 core)

| Task | Tier | Description | Note |
|------|------|-------------|------|
| DT-28 | T2 | Ball/free joints in fixed tendons — validation + qvel DOF index mapping | Low priority; no golden flag tests exercise this path |
| DT-33 | T2 | Tendon `margin` attribute for limit activation distance | Follow-up to Phase 7 §64a; 4 hardcoded sites in `assembly.rs` |
| DT-130 | T3 | Dense AR matrix optimization — sparse row-level `ARblock` | Performance only, no conformance impact |

These are tracked here for completeness. DT-28 and DT-33 can be
addressed as T1 items if time permits after the main Phase 13 work.
DT-130 is post-v1.0 (performance optimization).

## Design Decisions

### Why Session 0 (diagnostic) before any specs

The Phase 12 gate assessment identifies ~1.69 divergence across 22
golden flag tests and attributes it to "equality constraint solver."
But this is a **symptom description, not a root cause diagnosis.** The
stress test revealed:

- DT-39 (diagApprox) is computed during constraint *assembly*, before
  the solver loop. It feeds into R-scaling via `compute_regularization()`.
  Phase 12 Session 15 Round 2 rewrote `compute_invweight0()` extensively.
  What "remaining paths" are still broken is **unspecified**.
- DT-19 (QCQP) is **already implemented** with 13 unit tests. If it's
  the root cause, the fix is correctness verification, not fresh code.
- DT-128 (early termination) affects convergence behavior but the
  solver still produces correct results at `max_iters` — it just
  over-iterates.

Without empirical data, we risk building specs for the wrong root cause.
Session 0 dumps CortenForge vs MuJoCo constraint data for the golden
flag model and traces the ~1.69 divergence to a specific code path.
This data then **informs** Specs A and B — we write specs for what's
actually broken, not what we hypothesize is broken.

### Why Spec A (assembly) before Spec B (solver loop)

diagApprox is computed **once during assembly** and baked into R/D
before the solver loop runs. The solver loop reads R from the
pre-computed `efc_R` array — it never calls `diagApprox` itself. So
assembly correctness is a prerequisite for solver correctness. If
R-scaling is wrong, fixing the solver loop won't help.

### Spec A → B cascade hypothesis (UNVALIDATED)

The session plan hypothesizes:
- **Spec A** (DT-39 + DT-23) fixes 22–23 tests (assembly correctness)
- **Spec B** (DT-19 verify + DT-128 + DT-129) fixes remaining 3–4 tests

This is the **best available hypothesis** based on the gate assessment's
divergence pattern (22 tests at identical ~1.69 magnitude → single
root cause in constraint setup). But it has not been empirically
validated. Session 0's diagnostic data will confirm or refute this
before we invest in full spec cycles.

**If Session 0 refutes the hypothesis**, the spec grouping will be
revised before Session 1 begins. The diagnostic session exists
precisely to prevent wasted spec work.

### Why §46 and §66 are separate specs

`<composite>` (§46) is XML macro-expansion — it generates explicit MJCF
from templates before the builder sees it. No physics changes; purely
parser infrastructure.

Plugin/extension (§66) is trait-based dispatch — it adds plugin hooks
at pipeline stages. This is an extensibility API, orthogonal to physics
correctness.

Neither depends on the other. Both depend on the constraint solver work
being done first (conformance gate must be green before adding features).

### Session-per-review vs combined reviews

Following the established Phase 5/6 pattern: review-create and
review-execute are separate sessions. The review document acts as a
checklist; creating it without executing ensures nothing is missed when
the actual audit runs in the next session.

---

## Sessions

### Session 0 — Diagnostic Investigation

**Goal:** Empirically identify the root cause of the ~1.69 golden flag
divergence. Produce data that validates or refutes the Spec A/B
grouping hypothesis.

---

## Session 0: Root cause diagnosis for golden flag divergence

- [ ] Complete

```
Phase 13 Remaining Core — diagnostic investigation.

Read these first:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SESSION_PLAN.md
2. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/GATE_ASSESSMENT.md

Goal: trace the ~1.69 qacc divergence in golden_baseline to a specific
code path. This determines whether Spec A (assembly) or Spec B (solver)
is the primary fix target.

Step 1 — Build a diagnostic test:
Write a test (or extend the existing golden_baseline test) that loads
`flag_golden_test.xml`, runs one step, and dumps per-constraint-row data:
  - efc_type (constraint type per row)
  - efc_J (Jacobian row)
  - efc_pos (constraint violation)
  - efc_aref (reference acceleration)
  - efc_diagApprox (diagonal approximation)
  - efc_R (regularization)
  - efc_D (constraint stiffness)
  - efc_force (solver output)
  - qacc (final result)

Step 2 — Compare against MuJoCo reference:
Use the existing Python reference generation infrastructure
(gen_conformance_reference.py or similar) to dump the same fields from
MuJoCo 3.4.0 for the flag_golden_test.xml model. If Python infra isn't
available, manually compute expected values from MuJoCo C source logic.

Step 3 — Identify divergence:
Compare row-by-row. Find WHICH constraint row(s) diverge first and
WHERE in the pipeline (diagApprox? R? efc_force? somewhere else?).

Step 4 — Write findings:
Create DIAGNOSTIC_REPORT.md in this directory with:
  - Which constraint rows diverge
  - Which field diverges first (diagApprox vs R vs efc_force)
  - Root cause code path (file:line)
  - Whether DT-39 (diagApprox) or DT-19/128/129 (solver) is the fix
  - Whether golden_disable_filterparent (~51.9) is a compounding effect
    or an independent collision filtering bug
  - Revised Spec A/B grouping if the hypothesis is refuted

Step 5 — Assign FILTERPARENT:
`golden_disable_filterparent` (~51.9 divergence) must be assigned to a
spec. Based on the diagnostic data, either:
  (a) Assign to Spec A if root cause is assembly-side (diagApprox, R-scaling)
  (b) Assign to Spec B if root cause is solver-side (cone projection, iteration)
  (c) Create a new DT item and assign to a new Spec if root cause is
      independent (collision filtering bug). Add a session for it.
Update SESSION_PLAN.md scope table with the assignment.

Step 6 — Update session plan:
If the diagnosis changes the spec grouping, update SESSION_PLAN.md
accordingly. Do NOT proceed to Session 1 until the diagnostic is
complete and the plan is validated.

Do NOT implement fixes in this session — diagnosis only.
Run `cargo test -p sim-core -p sim-conformance-tests` to verify no regressions.
```

---

### Spec A — Constraint Assembly Conformance (DT-39 + DT-23)

**Goal:** Fix constraint row initialization so equality constraints and
friction loss match MuJoCo.

**Expected impact (hypothesis — to be validated by Session 0):**
22–23 golden flag tests un-ignored. The hypothesis is that the ~1.69
divergence across 22 tests traces to a single assembly-side root cause
(diagApprox R-scaling for equality constraints). Session 0's diagnostic
report will confirm this or redirect the spec scope.

---

## Session 1: Spec A rubric + spec

- [ ] Complete

```
Phase 13 Remaining Core — write Spec A rubric and spec.

Read these in order:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md
   (Session 0 output — this is the ground truth for what needs fixing)
2. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
3. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
4. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
5. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
6. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/GATE_ASSESSMENT.md

Spec A covers:
- DT-39: `diagApprox` remaining code paths. The DIAGNOSTIC_REPORT.md
  identifies the specific code path(s) that diverge. Read MuJoCo
  `engine_core_constraint.c` — `mj_diagApprox()` function and all
  call sites. Focus on the path(s) flagged by the diagnostic.
- DT-23: Per-DOF friction loss solver params (`dof_solref_fri`/
  `dof_solimp_fri`). The fields exist on Model (model.rs). Verify
  whether the assembly code routes them correctly or falls back to
  global defaults. Read MuJoCo `engine_core_constraint.c` —
  `mj_makeConstraint()` frictionloss section.

IMPORTANT: Scope Spec A to what the DIAGNOSTIC_REPORT.md identifies.
If Session 0 found that the root cause is NOT in assembly, adjust this
spec accordingly or skip directly to Spec B.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for these functions
- Phase 2: Build SPEC_A_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_A.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 2: Spec A implementation

- [ ] Complete

```
Phase 13 Remaining Core — implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md
2. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
4. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: Spec A review — create review document

- [ ] Complete

```
Phase 13 Remaining Core — create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A.md

Copy the review template into this directory as SPEC_A_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
code.

Write to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A_REVIEW.md
```

---

## Session 4: Spec A review — execute + golden flag checkpoint

- [ ] Complete

```
Phase 13 Remaining Core — execute Spec A review and run golden flag checkpoint.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_A.md
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md

Part 1 — Execute the review:
Walk every row in the review document. For each item, read the actual
implementation code, fill in the "Implementation does", "Status", and
verdict fields. If any item fails, fix it before proceeding.

Part 2 — Golden flag checkpoint:
Run: cargo test -p sim-conformance-tests --test integration -- golden --ignored
Record which tests now pass vs. still fail. Update this session plan's
Golden Flag Test Tracker with actual results.

Part 3 — Assess:
Compare actual results against Session 0's predictions. If significantly
fewer tests pass than predicted, investigate and document in this
session's notes — it may indicate the diagnostic missed a root cause.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` for full domain check.
```

---

### Spec B — PGS Solver Conformance (DT-19 verify + DT-128 + DT-129)

**Goal:** Verify QCQP correctness against MuJoCo, implement PGS early
termination, and implement warmstart pre-iteration projection. These
are solver *loop* items — they affect how the PGS solver iterates,
not how constraints are initialized.

**Key insight from stress test:** DT-19 (QCQP cone projection) is
**already implemented** with `qcqp2/3/nd` solvers and 13 unit tests.
Spec B's DT-19 work is conformance *verification* (compare against
MuJoCo C source line-by-line), not fresh implementation. DT-128
(early termination) and DT-129 (warmstart projection) are the real
implementation work.

**DT-129 independence:** Warmstart uses `classify_constraint_states()`
to initialize forces, not QCQP. The "ray+QCQP" in the roadmap
description refers to MuJoCo's approach — CortenForge's warmstart
implementation may use a different projection strategy. The spec
should study MuJoCo's actual warmstart code in `mj_solPGS()`.

---

## Session 5: Spec B rubric + spec

- [ ] Complete

```
Phase 13 Remaining Core — write Spec B rubric and spec.

Read these in order:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md
   (Session 0 output — check which solver-loop items the diagnostic flagged)
2. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
3. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
4. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
5. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
6. sim/docs/todo/spec_fleshouts/phase12_conformance_test_suite/GATE_ASSESSMENT.md

Spec B covers three items at different maturity levels:

- DT-19 (VERIFY, not implement): QCQP cone projection is already
  implemented in `sim/L0/core/src/constraint/solver/qcqp.rs` with
  `qcqp2()`, `qcqp3()`, `qcqp_nd()` and 13 unit tests. The spec must:
  (a) Read MuJoCo `engine_core_smooth.c` — `mj_projectConstraint()`
  (b) Line-by-line compare CortenForge's QCQP against MuJoCo's
  (c) Identify any divergences (scaling, convergence criteria, edge cases)
  (d) If divergences found, spec the fixes. If none found, mark DT-19 done.

- DT-128 (IMPLEMENT): PGS early termination — accumulate `improvement`
  from `costChange()`, break when `improvement * scale < tolerance`.
  Currently the solver always runs `max_iters` (pgs.rs line 127).
  Read MuJoCo `engine_core_smooth.c` — PGS convergence check.

- DT-129 (IMPLEMENT): PGS warmstart two-phase projection. Currently
  warmstart does cold/warm dual-cost selection (pgs.rs lines 88–125)
  via `classify_constraint_states()` but does NOT project warmstart
  forces before iteration. Read MuJoCo `engine_core_smooth.c` —
  warmstart handling in `mj_solPGS()`. Note: DT-129 is INDEPENDENT
  of DT-19 — warmstart uses force classification, not QCQP.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for these functions
- Phase 2: Build SPEC_B_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_B.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 6: Spec B implementation

- [ ] Complete

```
Phase 13 Remaining Core — implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md
2. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
4. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B.md

Implementation order:
1. DT-19 fixes (if any — may be zero changes if verification passes)
2. DT-128: PGS early termination (new code in solver loop)
3. DT-129: Warmstart pre-iteration projection (new code before solver loop)

Commit after each item. Verify each AC as you go. The spec is the source
of truth — if you discover a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each item.
MuJoCo conformance is the cardinal goal.
```

---

## Session 7: Spec B review — create review document

- [ ] Complete

```
Phase 13 Remaining Core — create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B.md

Copy the review template into this directory as SPEC_B_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
code.

Write to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B_REVIEW.md
```

---

## Session 8: Spec B review — execute + golden flag final gate

- [ ] Complete

```
Phase 13 Remaining Core — execute Spec B review and run golden flag final gate.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_B.md
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/DIAGNOSTIC_REPORT.md

Part 1 — Execute the review:
Walk every row in the review document. For each item, read the actual
implementation code, fill in the "Implementation does", "Status", and
verdict fields. If any item fails, fix it before proceeding.

Part 2 — Golden flag final gate:
Run: cargo test -p sim-conformance-tests --test integration -- golden --ignored
Record which tests pass. Update the Golden Flag Test Tracker.

Part 3 — Remaining failures:
If any golden flag tests still fail after Specs A+B, investigate root
cause. Session 0 should have assigned all known gaps (including
FILTERPARENT) to specs — if something still fails, it's either a
missed root cause or an implementation bug. Fix before declaring gate.

Run full domain check:
  cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests -p sim-physics -p sim-constraint

Expected outcome: 79 conformance + 24–26 golden flag = 103–105 tests passing.
Update GATE_ASSESSMENT.md with final results.
This is the v1.0 conformance gate. Do not proceed to Spec C until green
(or until remaining failures are triaged to known non-blocking gaps).
```

---

### Spec C — Composite Body Generation (§46)

**Goal:** Implement `<composite>` MJCF element that procedurally generates
body/geom/joint/tendon trees from 7 template types. This is XML
macro-expansion — the builder never sees `<composite>`, only the
expanded elements. Zero existing code; clean slate.

---

## Session 9: Spec C rubric + spec

- [ ] Complete

```
Phase 13 Remaining Core — write Spec C rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
5. sim/docs/todo/future_work_12.md (§46 section)

Spec C covers §46: `<composite>` procedural body generation.
MuJoCo supports 7 composite types: grid, rope, cable, cloth, box,
cylinder, ellipsoid. Read MuJoCo source `user_composite.cc` —
`mjCComposite::MakeModel()` and per-type generation functions.

Key design question: where does expansion happen?
- Option A: XML DOM pre-processing (before builder)
- Option B: Builder-level expansion (during model construction)
MuJoCo uses Option B (C++ `mjCComposite` class in compiler). Consider
which approach is cleaner for CortenForge's Rust architecture.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source
- Phase 2: Build SPEC_C_RUBRIC.md
- Phase 3: Write SPEC_C.md
- Phase 4: Grade honestly
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 10: Spec C implementation — part 1 (rope, cable, grid)

- [ ] Complete

```
Phase 13 Remaining Core — implement Spec C part 1.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C.md

Implement the first three composite types in the spec's Execution Order:
- rope (1D chain, hinge joints)
- cable (1D chain, ball joints)
- grid (2D body array)

These are the simpler types that establish the expansion infrastructure.
Commit after each type. Verify ACs for each type as you go.

Run `cargo test -p sim-core -p sim-mjcf` after each type.
MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec C implementation — part 2 (cloth, box, cylinder, ellipsoid)

- [ ] Complete

```
Phase 13 Remaining Core — implement Spec C part 2.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C.md

Implement the remaining four composite types:
- cloth (2D deformable mesh with tendons)
- box (3D volumetric)
- cylinder (3D volumetric)
- ellipsoid (3D volumetric)

These build on the infrastructure from part 1. Commit after each type.
Verify ACs for each type as you go.

Run `cargo test -p sim-core -p sim-mjcf` after each type.
MuJoCo conformance is the cardinal goal.
```

---

## Session 12: Spec C review — create review document

- [ ] Complete

```
Phase 13 Remaining Core — create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C.md

Copy the review template into this directory as SPEC_C_REVIEW.md. Fill in the
structure by walking the spec. This session creates the review document — it
does NOT execute the review.

Write to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C_REVIEW.md
```

---

## Session 13: Spec C review — execute

- [ ] Complete

```
Phase 13 Remaining Core — execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_C.md

Walk every row in the review document. For each item, read the actual
implementation code, fill in the "Implementation does", "Status", and
verdict fields. If any item fails, fix it before proceeding.

Run `cargo test -p sim-core -p sim-mjcf` for domain check.
```

---

### Spec D — Plugin/Extension System (§66)

**Goal:** Implement the plugin infrastructure: MJCF `<plugin>`/`<extension>`
parsing, Rust trait-based plugin dispatch, and pipeline integration points
for actuator, sensor, passive, and SDF plugins. Zero existing code;
clean slate.

---

## Session 14: Spec D rubric + spec

- [ ] Complete

```
Phase 13 Remaining Core — write Spec D rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
5. sim/docs/todo/future_work_16.md (§66 section)

Spec D covers §66: Plugin/extension system.
MuJoCo supports 4 plugin types: actuator, sensor, passive, SDF.
Read MuJoCo source `mjplugin.h`, `engine_plugin.c`, and `user_model.cc`
plugin handling.

Key design decisions:
- Trait design: `ActuatorPlugin`, `SensorPlugin`, `PassivePlugin`, `SdfPlugin`
- Registration: build-time registry vs runtime registration
- MJCF integration: `<extension>` declarations + `<plugin>` instances
- Pipeline hooks: where in the forward pass does each plugin type fire?

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source
- Phase 2: Build SPEC_D_RUBRIC.md
- Phase 3: Write SPEC_D.md
- Phase 4: Grade honestly
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 15: Spec D implementation

- [ ] Complete

```
Phase 13 Remaining Core — implement Spec D.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/PHASE13_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 16: Spec D review — create review document

- [ ] Complete

```
Phase 13 Remaining Core — create Spec D review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D.md

Copy the review template into this directory as SPEC_D_REVIEW.md. Fill in the
structure by walking the spec. This session creates the review document — it
does NOT execute the review.

Write to: sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D_REVIEW.md
```

---

## Session 17: Spec D review — execute

- [ ] Complete

```
Phase 13 Remaining Core — execute Spec D review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase13_remaining_core/SPEC_D.md

Walk every row in the review document. For each item, read the actual
implementation code, fill in the "Implementation does", "Status", and
verdict fields. If any item fails, fix it before proceeding.

Run `cargo test -p sim-core -p sim-mjcf` for domain check.

Final: run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests`
to verify no regressions from Phase 13 infrastructure work.
```

---

## Session History

| Session | Date | Status | Commit | Notes |
|---------|------|--------|--------|-------|
| 0 | — | — | — | Root cause diagnosis for golden flag divergence |
| 1 | — | — | — | Spec A rubric + spec |
| 2 | — | — | — | Spec A implement |
| 3 | — | — | — | Spec A review create |
| 4 | — | — | — | Spec A review execute + golden flag checkpoint |
| 5 | — | — | — | Spec B rubric + spec |
| 6 | — | — | — | Spec B implement |
| 7 | — | — | — | Spec B review create |
| 8 | — | — | — | Spec B review execute + golden flag final gate |
| 9 | — | — | — | Spec C rubric + spec |
| 10 | — | — | — | Spec C implement part 1 (rope, cable, grid) |
| 11 | — | — | — | Spec C implement part 2 (cloth, box, cylinder, ellipsoid) |
| 12 | — | — | — | Spec C review create |
| 13 | — | — | — | Spec C review execute |
| 14 | — | — | — | Spec D rubric + spec |
| 15 | — | — | — | Spec D implement |
| 16 | — | — | — | Spec D review create |
| 17 | — | — | — | Spec D review execute |

---

## Golden Flag Test Tracker

26 golden flag tests are `#[ignore]`d as of Phase 12 completion. Track
un-ignoring progress here.

### After Session 0 (diagnostic)

Session 0 produces predictions for each test. Fill in after diagnostic.

| # | Test | Divergence | Predicted Root Cause | Predicted Fix Spec |
|---|------|-----------|---------------------|-------------------|
| 1 | `golden_baseline` | ~1.69 | — | — |
| 2 | `golden_disable_constraint` | ~25.9 | — | — |
| 3 | `golden_disable_equality` | ~0.002 | — | — |
| 4 | `golden_disable_frictionloss` | ~3.99 | — | — |
| 5 | `golden_disable_limit` | ~1.69 | — | — |
| 6 | `golden_disable_contact` | ~1.69 | — | — |
| 7 | `golden_disable_spring` | ~1.69 | — | — |
| 8 | `golden_disable_damper` | ~1.69 | — | — |
| 9 | `golden_disable_gravity` | ~0.40 | — | — |
| 10 | `golden_disable_clampctrl` | ~2.48 | — | — |
| 11 | `golden_disable_warmstart` | ~1.69 | — | — |
| 12 | `golden_disable_filterparent` | ~51.9 | — | — |
| 13 | `golden_disable_actuation` | ~1.69 | — | — |
| 14 | `golden_disable_refsafe` | ~1.69 | — | — |
| 15 | `golden_disable_sensor` | ~1.69 | — | — |
| 16 | `golden_disable_midphase` | ~1.69 | — | — |
| 17 | `golden_disable_eulerdamp` | ~1.69 | — | — |
| 18 | `golden_disable_autoreset` | ~1.69 | — | — |
| 19 | `golden_disable_nativeccd` | ~1.69 | — | — |
| 20 | `golden_disable_island` | ~1.69 | — | — |
| 21 | `golden_enable_override` | ~1.69 | — | — |
| 22 | `golden_enable_energy` | ~1.69 | — | — |
| 23 | `golden_enable_fwdinv` | ~1.69 | — | — |
| 24 | `golden_enable_invdiscrete` | ~1.69 | — | — |
| 25 | `golden_enable_multiccd` | ~1.69 | — | — |
| 26 | `golden_enable_sleep` | ~1.69 | — | — |

### After Spec A (Session 4 checkpoint)

| Result | Count | Tests |
|--------|-------|-------|
| Pass | — | — |
| Fail | — | — |
| Total | 26 | — |

### After Spec B (Session 8 final gate)

| Result | Count | Tests |
|--------|-------|-------|
| Pass | — | — |
| Fail | — | — |
| Total | 26 | — |

---

## Stress Test Findings (incorporated into plan)

These findings from the pre-Session-0 stress test are recorded here
for context. They informed the Session 0 diagnostic and the amended
spec descriptions.

1. **DT-19 (QCQP) is already implemented.** `qcqp2/3/nd` solvers in
   `qcqp.rs` with 13 unit tests. Spec B changed from "implement" to
   "verify correctness against MuJoCo + fix divergences."

2. **DT-128 (early termination) is confirmed NOT implemented.** PGS
   loop always runs `max_iters` with no convergence check.

3. **DT-129 (warmstart) is partially implemented.** Cold/warm dual-cost
   selection exists but pre-iteration projection is missing. DT-129
   is **independent of DT-19** — warmstart uses
   `classify_constraint_states()`, not QCQP.

4. **The "22 from Spec A, 4 from Spec B" cascade is unvalidated.**
   The gate assessment identifies divergence magnitudes but not root
   cause code paths. Session 0 exists to provide this data.

5. **`golden_disable_filterparent` (~51.9) root cause unknown.**
   May be collision filtering, constraint solver, or a compounding
   effect. Tracked in scope table; Session 0 assigns to a spec.

6. **diagApprox is assembly-side** (computed once, baked into R/D).
   Confirmed correct placement in Spec A.

---

## Deferred Items Discovered During Phase 13

Track any new DT items discovered during implementation here. These will
be added to the post-v1.0 sections of ROADMAP_V1.md at phase completion.

| DT-# | Tier | Description | Discovered In |
|-------|------|-------------|---------------|
| — | — | — | — |
