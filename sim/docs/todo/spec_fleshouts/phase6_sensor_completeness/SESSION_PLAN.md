# Phase 6 — Sensor Completeness: Session Plan

Each spec follows a four-phase cycle: **rubric** → **spec** →
**implement** → **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Session 1: Spec A rubric (objtype + touch + geom acc)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md

Spec A covers DT-62 (frame sensor objtype attribute parsing), DT-64 (multi-geom
touch sensor aggregation), and DT-102 (geom-attached FrameLinAcc/FrameAngAcc).
MuJoCo ref: `mj_sensorPos()`, `mj_sensorAcc()` in `engine_sensor.c`, touch
sensor evaluation in `mj_sensorAcc()`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for these functions
- Phase 2: Build SPEC_A_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 2: Spec A spec (objtype + touch + geom acc)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: Spec A implementation

- [x] Complete

```
Phase 6 Sensor Completeness — implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests`
after each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 4: Spec A review — create review document

- [x] Complete

```
Phase 6 Sensor Completeness — create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A.md

Copy the review template into this directory as SPEC_A_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A_REVIEW.md
```

---

## Session 5: Spec A review — execute review

- [x] Complete

```
Phase 6 Sensor Completeness — execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_A.md
3. The implementation files listed in SPEC_A.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) to the user when done.
```

---

## Session 6: Spec B rubric (reftype/refid relative-frame)

- [x] Complete

**Prerequisite:** Session 3 (Spec A implementation) must be complete. Spec B
depends on Spec A's objtype parsing and parser refactoring.

```
Phase 6 Sensor Completeness — write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md

Spec B covers DT-63 (frame sensor reftype/refid — relative-frame measurements).
MuJoCo ref: `mj_sensorPos()`, `mj_sensorVel()`, `mj_sensorAcc()` in
`engine_sensor.c` — specifically the reference-frame transform sections.

IMPORTANT: Spec A has already been implemented. Read the current state of
the parser and builder to see the objtype + reftype separation that Spec B
must build on (Contract 1 and 3 in the umbrella).

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for reference-frame transforms in sensors
- Phase 2: Build SPEC_B_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 7: Spec B spec (reftype/refid relative-frame)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B_RUBRIC.md

IMPORTANT: Spec A has already been implemented. Read the current state of
the sensor evaluation files to see the frame sensor arms that Spec B must
add reference-frame transforms to (Contract 4 in the umbrella).

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 8: Spec B implementation

- [x] Complete

```
Phase 6 Sensor Completeness — implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests`
after each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 9: Spec B review — create review document

- [x] Complete

```
Phase 6 Sensor Completeness — create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B.md

Copy the review template into this directory as SPEC_B_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B_REVIEW.md
```

---

## Session 10: Spec B review — execute review

- [x] Complete

```
Phase 6 Sensor Completeness — execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_B.md
3. The implementation files listed in SPEC_B.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) to the user when done.
```

---

## Session 11: Spec C rubric (missing sensor types)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec C rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md

Spec C covers §62 (missing sensor types: clock, jointactuatorfrc, camprojection,
geomdist, geompoint, geomnormal). MuJoCo ref: `mj_sensorPos()`, `mj_sensorAcc()`
in `engine_sensor.c` — sensor type dispatch for these specific types.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for each missing sensor type
- Phase 2: Build SPEC_C_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 12: Spec C spec (missing sensor types)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec C spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C_RUBRIC.md

Write SPEC_C.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 13: Spec C implementation

- [x] Complete

```
Phase 6 Sensor Completeness — implement Spec C.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-sensor -p sim-conformance-tests`
after each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 14: Spec C review — create review document

- [x] Complete

```
Phase 6 Sensor Completeness — create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C.md

Copy the review template into this directory as SPEC_C_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C_REVIEW.md
```

---

## Session 15: Spec C review — execute review

- [x] Complete

```
Phase 6 Sensor Completeness — execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_C.md
3. The implementation files listed in SPEC_C.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) to the user when done.
```

---

## Session 16: Spec D rubric (sensor history attributes)

- [x] Done — `8239eda`

```
Phase 6 Sensor Completeness — write Spec D rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md

Spec D covers DT-109 (sensor history attributes: nsample, interp, delay).
This mirrors Phase 5 Spec D — MJCF parsing and model storage only, no runtime
behavior change. MuJoCo ref: `mjsSensor_` in `mjspec.h`, sensor model arrays
in `mjmodel.h`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for sensor history attributes
- Phase 2: Build SPEC_D_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 17: Spec D spec (sensor history attributes)

- [x] Complete

```
Phase 6 Sensor Completeness — write Spec D spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D_RUBRIC.md

Write SPEC_D.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 18: Spec D implementation

- [x] Done — `8b3c248`

```
Phase 6 Sensor Completeness — implement Spec D.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 19: Spec D review — create review document

- [x] Complete

```
Phase 6 Sensor Completeness — create Spec D review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D.md

Copy the review template into this directory as SPEC_D_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document — it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D_REVIEW.md
```

---

## Session 20: Spec D review — execute review

- [x] Complete

```
Phase 6 Sensor Completeness — execute Spec D review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/SPEC_D.md
3. The implementation files listed in SPEC_D.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final review of Phase 6. After the verdict, verify the Phase 6
aggregate ACs from the umbrella (PH6-AC1 through PH6-AC5) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Spec A rubric | Done | a6b6ab1 |
| 2 | Spec A spec | Done | b31e1b9 |
| 3 | Spec A implementation | Done | 28bc9f4 |
| 4 | Spec A review — create document | Done | f095ff0 |
| 5 | Spec A review — execute | Done | fce896d |
| 6 | Spec B rubric | Done | `948c898` |
| 7 | Spec B spec | Done | `91ac547` |
| 8 | Spec B implementation | Done | `fb8ec66` |
| 9 | Spec B review — create document | Done | `9916dd4` |
| 10 | Spec B review — execute | Done | `0683822` |
| 11 | Spec C rubric | Done | `9a63a6d` |
| 12 | Spec C spec | Done | `0a90285` |
| 13 | Spec C implementation | Done | `1b2c422` |
| 14 | Spec C review — create document | Done | `1703105` |
| 15 | Spec C review — execute | Done | `7e6ded4` |
| 16 | Spec D rubric | Done | `8239eda` |
| 17 | Spec D spec | Done | `c914cfe` |
| 18 | Spec D implementation | Done | `8b3c248` |
| 19 | Spec D review — create document | Done | `a103a0d` |
| 20 | Spec D review — execute | Done | — |
