# Phase 7 — MJCF Parsing & Defaults Gaps: Session Plan

17 sessions, each self-contained. The umbrella spec
(`PHASE7_UMBRELLA.md`) coordinates across context boundaries.

Unlike Phases 5 and 6 (single-domain depth passes), Phase 7 is a
heterogeneous breadth pass across defaults, joint physics, user data,
and flex attributes. Tasks are grouped by shared design decisions, not
by subsystem.

Each spec follows a four-phase cycle: **rubric** → **spec** →
**implement** → **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt — the
documents and commits are the state, not the context window.

---

## Task Assignment

Every Phase 7 task is assigned to exactly one deliverable:

| Task | Deliverable | Rationale |
|------|-------------|-----------|
| DT-2 | Spec A | Equality defaults — shares defaults cascade design with DT-11/13/14 |
| DT-3 | T1 session | File-based hfield from PNG — mechanical, parent spec (§6a) defines the "what" |
| DT-11 | Spec A | Joint `range` defaultable — defaults cascade |
| DT-13 | Spec A | `qpos_spring` — defaults cascade + joint attribute parsing |
| DT-14 | Spec A | Actuator type-specific defaults — defaults cascade |
| DT-85 | T1 session | Flex contact runtime attributes — mechanical (4 fields) |
| DT-88 | Spec C | Flexcomp attributes — parsing + model storage |
| §55 | Spec C | Per-element user data arrays — parsing + model storage |
| §60 | Spec B | `springinertia` — joint physics (CRBA diagonal) |
| §64 | Spec B | Ball/free spring energy — joint physics (energy.rs) |
| §64a | Spec B | `jnt_margin` — joint physics (constraint activation) |

---

## Dependency Graph

```
Session 1 (Umbrella)
    │
    ├── Session 2 (T1: DT-3, DT-85)     ← independent
    │
    ├── Sessions 3–7 (Spec A: Defaults)  ← independent
    │
    ├── Sessions 8–12 (Spec B: Joint)    ← independent
    │
    └── Sessions 13–17 (Spec C: Parsing) ← independent
```

No cross-spec dependencies. All three specs and the T1 session are
independent of each other — ordering is for clarity, not necessity.

---

## Session 1: Phase 7 Umbrella

- [x] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 7 section)
2. sim/docs/todo/future_work_10b.md (DT-2, DT-3, DT-11, DT-13, DT-14)
3. sim/docs/todo/future_work_10i.md (DT-85, DT-88)
4. sim/docs/todo/future_work_13.md (§55)
5. sim/docs/todo/future_work_15.md (§60, §64, §64a)
6. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SESSION_PLAN.md
7. sim/docs/todo/spec_fleshouts/phase6_sensor_completeness/PHASE6_UMBRELLA.md
   (structural template — 4 specs, single-domain depth pass)
8. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
   (structural template — T1 session + 4 specs, includes T1 handling pattern)

Adapt these templates — do not copy blindly. Phase 7 is heterogeneous
(defaults, joint physics, parsing breadth) unlike Phases 5/6 (single domain).

Write PHASE7_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Defaults Completeness (DT-2, DT-11, DT-13, DT-14)
  - Spec B: Joint Physics (§60, §64, §64a)
  - Spec C: Parsing Breadth (§55, DT-88)
- Dependency Graph (flat — no cross-spec dependencies)
- File Ownership Matrix (which spec touches each shared file)
- API Contracts (cross-spec boundaries, if any)
- Shared Convention Registry (naming, defaults, enum ordering)
- Cross-Spec Blast Radius
- Phase-Level Acceptance Criteria (PH7-AC1 through PH7-AC5)
- Out of Scope (explicit exclusions)

Key differences from Phase 6 umbrella:
- Phase 7 is heterogeneous (defaults, joint physics, parsing) not single-domain
- No cross-spec dependencies — flat dependency graph
- T1 items (DT-3, DT-85) handled in a single session, not specced

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 items (DT-3 + DT-85)

- [x] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — implement T1 items.

Read these first:
- sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
  (Scope, File Ownership Matrix, Convention Registry sections)
- sim/docs/todo/future_work_10b.md (DT-3 entry)
- sim/docs/todo/future_work_10i.md (DT-85 entry)

Two deliverables, commit each separately:

T1-a (DT-3): File-based hfield loading from PNG. Make the `elevation` attribute
optional on `<hfield>`, add `file` attribute parsing, and load elevation data from
PNG grayscale. The `image` crate (v0.25) is already in workspace dependencies.
Use `resolve_asset_path()` from `builder/asset.rs` for path resolution. MuJoCo ref:
hfield file loading in `user_model.cc`. Test assets with `file=` already exist in
`sim/L0/tests/assets/mujoco_menagerie/` and `sim/L0/tests/assets/dm_control/`.

T1-b (DT-85): Flex `<contact>` runtime attributes. These four attributes are NOT
currently parsed, stored, or wired — full scope is parse + store + wire:
- Parse `internal` (bool), `activelayers` (int), `vertcollide` (bool), `passive`
  (bool) in `parse_flex_contact_attrs()` (parser.rs:2704)
- Add fields to `MjcfFlex` (types.rs:3624)
- Add `flex_internal`, `flex_activelayers`, `flex_vertcollide`, `flex_passive`
  arrays to Model (model.rs:306)
- Wire through `process_flex_bodies()` in builder/flex.rs

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each.
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: Spec A rubric (Defaults Completeness)

- [x] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md

Spec A covers the "Defaults Completeness" group:
- DT-2: Equality constraint defaults — add `solref`/`solimp` to `MjcfEqualityDefaults`,
  implement `apply_to_equality()` cascade
- DT-11: Joint `range` — add to `MjcfJointDefaults` as defaultable attribute
- DT-13: `qpos_spring` — distinct from `qpos0`, parse + store + wire default cascade
- DT-14: Actuator type-specific defaults — cylinder area/timeconst, muscle params
  defaultable per actuator type

MuJoCo ref: default class application in `mjXReader::Default()` and
`mjCModel::ResolvePlugin()` / `mjCModel::CopyBack()` in `user_model.cc`.
Equality defaults in `mjCEquality`. Joint defaults in `mjCJoint`.
Actuator defaults in `mjCActuator`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for default class resolution
- Phase 2: Build SPEC_A_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 4: Spec A spec (Defaults Completeness)

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A implementation

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each
section. MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec A review — create review document

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A.md

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

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A_REVIEW.md
```

---

## Session 7: Spec A review — execute review

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_A.md
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

## Session 8: Spec B rubric (Joint Physics)

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md

Spec B covers three joint physics features:
- §60: `springinertia` — parse from `<joint>` and `<default><joint>`, store on
  Model, add `springinertia * stiffness[dof]` to CRBA diagonal (`M[dof,dof]`).
  MuJoCo ref: `mj_crb()` in `engine_core_smooth.c`.
- §64: Ball/free joint spring potential energy — fix the stub in `energy.rs`.
  Ball: `E = 0.5 * k * θ²` where `θ = 2 * arccos(|q · q_ref|)`. Free: translational +
  rotational. MuJoCo ref: `mj_energyPos()` in `engine_core_smooth.c`.
- §64a: `jnt_margin` — parse `margin` from `<joint>`, store as `jnt_margin`,
  replace 6 hardcoded `< 0.0` activation checks in constraint assembly + pass
  margin to `finalize_row!`. MuJoCo ref: `mj_instantiateLimit()` in
  `engine_core_constraint.c`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for each function
- Phase 2: Build SPEC_B_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 9: Spec B spec (Joint Physics)

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B_RUBRIC.md

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal.
```

---

## Session 10: Spec B implementation

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-conformance-tests`
after each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec B review — create review document

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B.md

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

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B_REVIEW.md
```

---

## Session 12: Spec B review — execute review

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_B.md
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

## Session 13: Spec C rubric (Parsing Breadth)

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec C rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md

Spec C covers two parsing + model storage tasks:
- §55: Per-element `*_user` custom data arrays — parse `user` attribute from
  body/geom/joint/tendon/actuator/sensor/site elements. Parse `nuser_*` from
  `<size>`. Store as `body_user: Vec<Vec<f64>>` etc. on Model. No runtime effect.
  MuJoCo ref: element parsing in `user_model.cc`, `nuser_*` in `mjModel`.
- DT-88: `<flexcomp>` attributes — `inertiabox`, `scale`, `quat`, `file`. Parse
  and store on flexcomp model data. MuJoCo ref: `mjCFlexcomp` in `user_objects.cc`.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for user data handling and flexcomp attributes
- Phase 2: Build SPEC_C_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly — do not proceed to P2–P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec — that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 14: Spec C spec (Parsing Breadth)

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — write Spec C spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C_RUBRIC.md

Write SPEC_C.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval — do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/
MuJoCo conformance is the cardinal goal.
```

---

## Session 15: Spec C implementation

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — implement Spec C.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth — if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after each
section. MuJoCo conformance is the cardinal goal.
```

---

## Session 16: Spec C review — create review document

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C.md

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

Write to: sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C_REVIEW.md
```

---

## Session 17: Spec C review — execute review

- [ ] Complete

```
Phase 7 MJCF Parsing & Defaults Gaps — execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/SPEC_C.md
3. The implementation files listed in SPEC_C.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final review of Phase 7. After the verdict, verify the Phase 7
aggregate ACs from the umbrella (PH7-AC1 through PH7-AC5) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 7 Umbrella | Done | b025795 |
| 2 | T1: DT-3 (hfield PNG) + DT-85 (flex contact attrs) | Done | cea5f4c, cf76731 |
| 3 | Spec A rubric | Done | pending |
| 4 | Spec A spec | Pending | |
| 5 | Spec A implementation | Pending | |
| 6 | Spec A review — create document | Pending | |
| 7 | Spec A review — execute | Pending | |
| 8 | Spec B rubric | Pending | |
| 9 | Spec B spec | Pending | |
| 10 | Spec B implementation | Pending | |
| 11 | Spec B review — create document | Pending | |
| 12 | Spec B review — execute | Pending | |
| 13 | Spec C rubric | Pending | |
| 14 | Spec C spec | Pending | |
| 15 | Spec C implementation | Pending | |
| 16 | Spec C review — create document | Pending | |
| 17 | Spec C review — execute | Pending | |
