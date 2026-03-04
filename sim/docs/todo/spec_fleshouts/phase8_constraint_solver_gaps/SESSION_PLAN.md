# Phase 8 — Constraint & Solver Gaps: Session Plan

13 sessions, each self-contained. The umbrella spec
(`PHASE8_UMBRELLA.md`) coordinates across context boundaries.

Phase 8 covers constraint formulation and solver completeness: solver
parameter fields, friction cone projection, tendon/DOF validation, and
the body-weight diagonal approximation. The tasks span three subsystems
(constraint assembly, PGS solver projection, tendon DOF mapping) but
converge on the unified constraint pipeline in `sim-core`.

Each spec follows a four-phase cycle: **rubric** -> **spec** ->
**implement** -> **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt -- the
documents and commits are the state, not the context window.

---

## Rescope Notes (post-DT-32)

DT-32 (per-tendon `solref_limit`/`solimp_limit`) was completed as a
pre-umbrella naming conformance task. The fields already existed and
were fully wired; the only gap was naming alignment with MuJoCo
convention. Commit: `9f9cf9f`.

DT-25 (deformable-rigid friction projection) downgraded from a 5-session
T3 spec to a 1-session verification pass. The friction infrastructure
already exists (condim support, Jacobian construction, unified projection
in assembly). What remains is: confirm correctness with targeted tests,
verify R-scaling with asymmetric Jacobians, and document any real gaps
as deferred items.

---

## Task Assignment

Every Phase 8 task is assigned to exactly one deliverable:

| Task | Deliverable | Status | Rationale |
|------|-------------|--------|-----------|
| ~~DT-32~~ | Pre-spec | **Done** | Naming conformance only — commit `9f9cf9f` |
| DT-23 | Spec A | Pending | Per-DOF friction loss solver params — finish item (fields exist, verify defaults cascade + test coverage) |
| DT-33 | Spec A | Pending | Tendon `margin` — extends Phase 7 `jnt_margin` pattern to tendon limit sites |
| DT-28 | T1 session | **Done** | Ball/Free joints in fixed tendons — validation + DOF mapping |
| DT-39 | T1 session | **Done** | Body-weight diagonal approximation — `body_invweight0`/`dof_invweight0` + `mj_diagApprox()` |
| DT-19 | Spec B | Pending | QCQP cone projection — algorithmic T3, needs individual spec |
| DT-25 | Verification | Pending | Deformable-rigid friction — downgraded to 1-session verification pass |

---

## Dependency Graph

```
Session 1 (Umbrella — rescoped)
    |
    +-- Session 2 (T1: DT-28, DT-39)                <- independent
    |
    +-- Sessions 3-7 (Spec A: DT-23 + DT-33)        <- independent
    |
    +-- Sessions 8-12 (Spec B: DT-19 QCQP)          <- independent
    |
    +-- Session 13 (DT-25 verification)              <- independent
```

No cross-spec dependencies. All deliverables are independent of each
other — ordering is for clarity, not necessity.

---

## Session 1: Phase 8 Umbrella

- [x] Complete (rescoped: DT-32 dropped, DT-25 downgraded; PHASE8_UMBRELLA.md written)

```
Phase 8 Constraint & Solver Gaps -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 8 section)
2. sim/docs/todo/future_work_10c.md (DT-19, DT-23, DT-25)
3. sim/docs/todo/future_work_10d.md (DT-28, DT-33 -- first 25 lines + DT entries)
4. sim/docs/todo/future_work_10e.md (DT-39)
5. sim/docs/todo/future_work_5.md (§15 constraint pipeline -- diagApprox sections)
6. sim/docs/todo/future_work_8.md (§28-29 friction loss, solver params, cone projection)
7. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SESSION_PLAN.md
8. sim/docs/todo/spec_fleshouts/phase7_mjcf_parsing_defaults/PHASE7_UMBRELLA.md
   (structural template -- heterogeneous breadth pass)
9. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
   (structural template -- T1 session + 4 specs, includes T1 handling pattern)

Phase 8 is a constraint/solver depth pass: solver parameter fields,
friction cone projection algorithms, and tendon/DOF validation. Unlike
Phase 7 (parsing breadth), Phase 8 tasks all converge on the unified
constraint pipeline (assembly.rs, solver/).

Rescope from original plan:
- DT-32 already complete (naming conformance, commit 9f9cf9f)
- DT-25 downgraded from 5-session Spec C to 1-session verification pass
- Spec A now covers DT-23 + DT-33 only (both touch assembly.rs constraint
  parameter wiring)

Write PHASE8_UMBRELLA.md covering:
- Scope statement with conformance mandate (note DT-32 pre-completed)
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Solver Param & Margin Completeness (DT-23, DT-33)
  - Spec B: QCQP Cone Projection (DT-19)
- DT-25 Verification Pass scope (1 session, not a full spec)
- Dependency Graph (flat -- no cross-spec dependencies)
- File Ownership Matrix (which spec touches each shared file)
- API Contracts (cross-spec boundaries, if any)
- Shared Convention Registry (naming, solver param layout, constraint type ordering)
- Cross-Spec Blast Radius
- Phase-Level Acceptance Criteria (PH8-AC1 through PH8-AC5)
- Out of Scope (explicit exclusions)

Key differences from Phase 7 umbrella:
- Phase 8 is constraint/solver depth (single subsystem) not parsing breadth
- T1 items (DT-28, DT-39) are more algorithmic than Phase 7's T1s
- One T3 spec (Spec B) with genuine algorithmic complexity
- DT-25 is a verification pass, not a full spec
- Shared file: assembly.rs touched by Spec A, Spec B, and DT-25 verification

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 items (DT-28 + DT-39)

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- implement T1 items.

Read these first:
- sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
  (Scope, File Ownership Matrix, Convention Registry sections)
- sim/docs/todo/future_work_10d.md (DT-28 entry)
- sim/docs/todo/future_work_10e.md (DT-39 entry)
- sim/docs/todo/future_work_1.md (§4 step 5: fixed tendon length -- lines 1538-1575,
  documents the ball/free DOF mapping issue)
- sim/docs/todo/future_work_5.md (§15.1 diagApprox -- lines 246-271,
  documents body_invweight0 / dof_invweight0 computation)

Two deliverables, commit each separately:

T1-a (DT-28): Ball/Free joints in fixed tendons. Currently
`compute_fixed_tendon_length()` in sim-tendon uses `dof_adr` for qvel
indexing which is correct for hinge/slide (1 DOF, dof_adr == qpos_adr)
but needs validation for ball (nv=3) and free (nv=6) joints where
dof_adr != qpos_adr. MuJoCo ref: `mj_tendon()` in
`engine_core_smooth.c` -- fixed tendon length uses `coef * qpos[adr]`
for length and `coef * qvel[dof_adr]` for velocity. CortenForge must:
1. Validate qvel DOF index mapping for multi-DOF joints
2. Add tests with ball/free joints in fixed tendons
3. Either fix or document any conformance gaps

T1-b (DT-39): Body-weight diagonal approximation (`diagApprox`). Add
`body_invweight0: Vec<[f64; 2]>` and `dof_invweight0: Vec<f64>` to
Model, computed at build time. MuJoCo ref: `setInertia()` inside
`mj_setConst()` in `engine_setconst.c`. The algorithm:
- `body_invweight0[b][0]` = 1 / (total mass of subtree rooted at b)
- `body_invweight0[b][1]` = 1 / (trace of subtree rotational inertia at b)
- `dof_invweight0[dof]` = computed from body_invweight0 using joint type
Then implement `mj_diagApprox()` following MuJoCo's per-constraint-type
dispatch (engine_core_constraint.c) and wire it into `finalize_row!` as
an alternative to the current exact diagonal computation. Add a model
option or solver param to select exact vs approximate.

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-tendon
-p sim-conformance-tests` after each. MuJoCo conformance is the
cardinal goal.
```

---

## Session 3: Spec A rubric (Solver Param & Margin Completeness)

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md

Spec A covers the "Solver Param & Margin Completeness" group:
- DT-23: Per-DOF friction loss solver params -- the fields may partially
  exist. Verify: `dof_solref_fri` / `dof_solimp_fri` in Model, parse
  `solreffriction` / `solimpfriction` from `<joint>`, defaults cascade
  verification (can you set these in `<default><joint>`?), wire to
  friction loss constraint rows in assembly.rs. This is a finish item,
  not greenfield.
  MuJoCo ref: `dof_solref[nv*mjNREF]`, `dof_solimp[nv*mjNIMP]` in
  `mjModel`; `mj_instantiateFriction()` in `engine_core_constraint.c`.
- DT-33: Tendon `margin` attribute -- add `tendon_margin: Vec<f64>` to
  Model, parse from `<tendon>`, replace 4 hardcoded `< 0.0` activation
  checks in assembly.rs (lines 148, 152, 540, 564) with margin-aware
  checks. Phase 7 Spec B (jnt_margin) is the direct reference pattern.
  MuJoCo ref: `tendon_margin[ntendon]` in `mjModel`.

Note: DT-32 (tendon solref_limit/solimp_limit naming) is already done.
Spec A no longer includes it.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for these functions
- Phase 2: Build SPEC_A_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 4: Spec A spec (Solver Param & Margin Completeness)

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

DT-23 is a finish item: audit what already exists, identify remaining
gaps (defaults cascade, naming consistency, test coverage), and spec
only the delta. DT-33 follows the Phase 7 Spec B jnt_margin pattern.

Reference the Phase 7 Spec B implementation of jnt_margin (§64a) as a
pattern for DT-33. The solver param lifecycle (parse -> defaults cascade
-> model field -> constraint assembly) should be consistent across both
items.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A implementation

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint -p sim-tendon
-p sim-conformance-tests` after each section. MuJoCo conformance is the
cardinal goal.
```

---

## Session 6: Spec A review -- create review document

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A.md

Copy the review template into this directory as SPEC_A_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A_REVIEW.md
```

---

## Session 7: Spec A review -- execute review

- [x] Complete

```
Phase 8 Constraint & Solver Gaps -- execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_A.md
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

## Session 8: Spec B rubric (QCQP Cone Projection)

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md

Spec B covers DT-19: QCQP-based cone projection for normal+friction
force projection following MuJoCo's PGS style. This is the core friction
cone enforcement algorithm used during PGS/CG/Newton solver iterations.

MuJoCo ref:
- `mj_projectConstraint()` in `engine_solver.c` -- the main projection
  function called per-contact during solver iteration
- Cone projection logic: given a contact with condim > 1, jointly project
  the normal force and friction forces onto the friction cone surface
  using a QCQP formulation (minimize distance to unconstrained solution
  subject to cone constraint)
- The projection depends on condim (1=frictionless, 3=pyramidal, 4/6=elliptic)
- Key: MuJoCo does NOT use simple clamping -- it solves a small QCQP per
  contact to find the nearest feasible point on the friction cone

Also read:
- sim/docs/todo/future_work_8.md (§29 PGS solver -- cone projection sections)
- sim/L0/core/src/constraint/solver/pgs.rs (current projection implementation)

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for `mj_projectConstraint()`
- Phase 2: Build SPEC_B_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 9: Spec B spec (QCQP Cone Projection)

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B_RUBRIC.md

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- The QCQP formulation must be precisely specified with all cases:
  condim=1 (frictionless), condim=3 (pyramidal), condim=4 (elliptic with
  torsional friction), condim=6 (full elliptic)
- Include the mathematical formulation: objective function, constraints,
  KKT conditions, and closed-form / iterative solution
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 10: Spec B implementation

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

The cone projection must be integrated into all three solvers (PGS, CG, Newton)
and the noslip post-processor. Verify that existing contact tests still pass
and add new tests for cone projection edge cases.

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint
-p sim-conformance-tests` after each section. MuJoCo conformance is the
cardinal goal.
```

---

## Session 11: Spec B review -- create review document

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B.md

Copy the review template into this directory as SPEC_B_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B_REVIEW.md
```

---

## Session 12: Spec B review -- execute review

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_B.md
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

## Session 13: DT-25 verification pass (Deformable Friction Projection)

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- DT-25 verification pass.

This is a 1-session verification pass, NOT a full spec. The deformable-rigid
friction infrastructure already exists (condim support, Jacobian construction,
unified projection in assembly). This session confirms correctness and
documents any real gaps as deferred items.

Read these first:
- sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
- sim/docs/todo/future_work_10c.md (DT-25 entry)
- sim/L0/core/src/constraint/assembly.rs (deformable constraint rows)
- sim/L0/core/src/forward/deformable.rs (deformable contact generation)

Deliverables:
1. Audit: Read existing deformable contact + friction code path end-to-end.
   Document what condim values are actually supported for deformable contacts.
2. Test: Write targeted tests confirming deformable-rigid friction works for
   condim=3 (the common case). Include a flex body sliding on a plane with
   friction and verify nonzero tangential constraint forces.
3. R-scaling: Verify R-scaling behavior with asymmetric Jacobians (zero
   angular component on flex side). Confirm `diagApprox` or exact diagonal
   handles this correctly.
4. Gap doc: If any real gaps exist (condim=4/6 for deformable, specific
   Jacobian issues), document them as deferred items in the roadmap with
   clear descriptions. Do NOT implement fixes -- just document.
5. Mark DT-25 in ROADMAP_V1.md as done (or partially done with deferred
   items noted).

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint
-p sim-conformance-tests` after tests are added. MuJoCo conformance is
the cardinal goal.

This is the final session of Phase 8. After completing, verify the Phase 8
aggregate ACs from the umbrella (PH8-AC1 through PH8-AC5) are satisfied.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| Pre-spec | DT-32 naming conformance | **Done** | `9f9cf9f` |
| 1 | Phase 8 Umbrella | **Done** | `4574a03` |
| 2 | T1: DT-28 (tendon ball/free) + DT-39 (diagApprox) | **Done** | `22c7c3d` |
| 3 | Spec A rubric | **Done** | `8db9545` |
| 4 | Spec A spec | **Done** | `32b966f` |
| 5 | Spec A implementation | **Done** | `ac54666` |
| 6 | Spec A review -- create document | **Done** | `5ae575a` |
| 7 | Spec A review -- execute | **Done** | |
| 8 | Spec B rubric | | |
| 9 | Spec B spec | | |
| 10 | Spec B implementation | | |
| 11 | Spec B review -- create document | | |
| 12 | Spec B review -- execute | | |
| 13 | DT-25 verification pass | | |
