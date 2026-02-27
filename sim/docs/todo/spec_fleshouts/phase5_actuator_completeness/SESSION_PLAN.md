# Phase 5 — Actuator Completeness: Session Plan

9 sessions, each self-contained. The umbrella spec
(`PHASE5_UMBRELLA.md`) coordinates across context boundaries.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Session 1: T1 items (DT-6 + §63)

- [x] Complete

```
Phase 5 Actuator Completeness — implement T1 items.

Read these first:
- sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
  (Scope, File Ownership Matrix, API Contracts, Convention Registry sections)
- sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only — these
  are small enough to implement directly)

Two deliverables, commit each separately:

T1-b (§63) first: Resize `actuator_dynprm` from `[f64; 3]` to `[f64; 10]`.
The umbrella's file ownership matrix lists every file and literal to update.
The Convention Registry §5 defines the post-resize layout. Update
`muscle_activation_dynamics()` signature per Contract 1.

T1-a (DT-6) second: Verify `actearly` runtime wiring in actuation.rs. Add
2–4 conformance tests. No code change expected — verification + tests only.

Run `cargo test -p sim-core -p sim-mjcf -p sim-muscle -p sim-sensor` after
each. MuJoCo conformance is the cardinal goal.
```

---

## Session 2: Spec A rubric + spec (acc0, dampratio, lengthrange)

- [ ] Complete

```
Phase 5 Actuator Completeness — write Spec A rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md

Spec A covers DT-57 (acc0 non-muscle), DT-56 (dampratio), DT-59 (lengthrange
bisection for unlimited slides), DT-77 (site-transmission lengthrange).
MuJoCo ref: `mj_setLengthRange()` and `mj_setConst()` in `engine_setconst.c`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for these functions
- Phase 2: Build SPEC_A_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_A.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 3: Spec A implementation

- [ ] Complete

```
Phase 5 Actuator Completeness — implement Spec A.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-muscle` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 4: Spec B rubric + spec (transmission types + slider-crank)

- [ ] Complete

```
Phase 5 Actuator Completeness — write Spec B rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md

Spec B covers DT-8 (transmission types: cranksite, slidersite, jointinparent)
and §61 (slider-crank runtime moment arm computation).
MuJoCo ref: `mj_transmission()` in `engine_forward.c`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for `mj_transmission()`
- Phase 2: Build SPEC_B_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_B.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 5: Spec B implementation

- [ ] Complete

```
Phase 5 Actuator Completeness — implement Spec B.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-muscle` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec D rubric + spec (interpolation attributes)

- [ ] Complete

```
Phase 5 Actuator Completeness — write Spec D rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md

Spec D covers DT-9 (nsample, interp, delay interpolation attributes). This
is the smallest spec — MJCF parsing and model storage only, no runtime
behavior change. MuJoCo ref: actuator element parsing in `user_model.cc`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for actuator attribute parsing
- Phase 2: Build SPEC_D_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_D.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 7: Spec D implementation

- [ ] Complete

```
Phase 5 Actuator Completeness — implement Spec D.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_D.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf` after each section.
MuJoCo conformance is the cardinal goal.
```

---

## Session 8: Spec C rubric + spec (Hill-type muscle dynamics)

- [ ] Complete

**Prerequisite:** Session 3 (Spec A implementation) must be complete. Spec C
depends on Spec A's extended `compute_muscle_params()` signature (Contract 2
in the umbrella).

```
Phase 5 Actuator Completeness — write Spec C rubric and spec.

Read these in order:
1. sim/docs/templates/pre_implementation/WORKFLOW.md — follow phases 1–6
2. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
3. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
4. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md

Spec C covers DT-58 (Hill-type muscle dynamics variant). This requires an
architectural decision about integration with the sim-muscle crate.
MuJoCo ref: `mj_muscleActuation()` in `engine_forward.c`, muscle dynamics
in `engine_forward.c`.

IMPORTANT: Spec A has already been implemented. Read the current state of
`sim/L0/core/src/forward/muscle.rs` to see the extended
`compute_muscle_params()` / `compute_actuator_params()` that Spec C must
integrate with (Contract 2 in the umbrella).

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for muscle actuation
- Phase 2: Build SPEC_C_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Write SPEC_C.md (MuJoCo Reference section first)
- Phase 4: Grade honestly (P1 first — do not proceed to P2–P8 until P1 is A+)
- Phase 5: Close gaps
- Phase 6: Present for approval — do NOT implement

Write files to: sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 9: Spec C implementation

- [ ] Complete

```
Phase 5 Actuator Completeness — implement Spec C.

Read these:
1. sim/docs/templates/pre_implementation/WORKFLOW.md (Phase 7 only)
2. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-muscle` after each section.
MuJoCo conformance is the cardinal goal.

After Spec C lands, verify Phase 5 aggregate ACs from the umbrella:
- PH5-AC1: All 10 tasks ship-complete
- PH5-AC2: All 2,148+ baseline tests pass
- PH5-AC3: `cargo xtask check` passes
- PH5-AC4: Domain test count increased by at least 22
- PH5-AC5: Conformance coverage table matches post-Phase 5 column
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | T1-a (DT-6) + T1-b (§63) | Done | d4db634, dc12b8b |
| 2 | Spec A rubric + spec | Pending | |
| 3 | Spec A implementation | Pending | |
| 4 | Spec B rubric + spec | Pending | |
| 5 | Spec B implementation | Pending | |
| 6 | Spec D rubric + spec | Pending | |
| 7 | Spec D implementation | Pending | |
| 8 | Spec C rubric + spec | Pending | |
| 9 | Spec C implementation | Pending | |
