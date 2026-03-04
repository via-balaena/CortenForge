# Phase 8 — Constraint & Solver Gaps: Session Plan

17 sessions, each self-contained. The umbrella spec
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

## Task Assignment

Every Phase 8 task is assigned to exactly one deliverable:

| Task | Deliverable | Rationale |
|------|-------------|-----------|
| DT-23 | Spec A | Per-DOF friction loss solver params -- shares solver param pattern with DT-32/33 |
| DT-32 | Spec A | Per-tendon `solref_limit`/`solimp_limit` -- same parse + model + assembly pattern |
| DT-33 | Spec A | Tendon `margin` -- extends Phase 7 `jnt_margin` pattern to tendon limit sites |
| DT-28 | T1 session | Ball/Free joints in fixed tendons -- validation + DOF mapping, parent spec (future_work_1.md) defines scope |
| DT-39 | T1 session | Body-weight diagonal approximation -- `body_invweight0`/`dof_invweight0` model fields + `mj_diagApprox()` |
| DT-19 | Spec B | QCQP cone projection -- algorithmic T3, needs individual spec |
| DT-25 | Spec C | Deformable-rigid friction cone projection -- algorithmic T3, needs individual spec |

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: DT-28, DT-39)           <- independent
    |
    +-- Sessions 3-7 (Spec A: Solver Params)    <- independent
    |
    +-- Sessions 8-12 (Spec B: QCQP)           <- independent
    |
    +-- Sessions 13-17 (Spec C: Deformable)     <- independent
```

No cross-spec dependencies. All three specs and the T1 session are
independent of each other -- ordering is for clarity, not necessity.

---

## Session 1: Phase 8 Umbrella

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 8 section)
2. sim/docs/todo/future_work_10c.md (DT-19, DT-23, DT-25)
3. sim/docs/todo/future_work_10d.md (DT-28, DT-32, DT-33 -- first 25 lines + DT entries)
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

Write PHASE8_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Solver Param & Margin Completeness (DT-23, DT-32, DT-33)
  - Spec B: QCQP Cone Projection (DT-19)
  - Spec C: Deformable Friction Projection (DT-25)
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
- Two T3 specs (Spec B, Spec C) with distinct algorithmic complexity
- Shared file: assembly.rs touched by Spec A, Spec B, and Spec C

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 items (DT-28 + DT-39)

- [ ] Complete

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

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md

Spec A covers the "Solver Param & Margin Completeness" group:
- DT-23: Per-DOF friction loss solver params -- add `dof_solref_fri` /
  `dof_solimp_fri` fields to Model, parse `solreffriction` / `solimpfriction`
  from `<joint>`, wire to friction loss constraint rows in assembly.rs.
  MuJoCo ref: `dof_solref[nv*mjNREF]`, `dof_solimp[nv*mjNIMP]` in
  `mjModel`; `mj_instantiateFriction()` in `engine_core_constraint.c`.
- DT-32: Per-tendon `solref_limit`/`solimp_limit` -- add
  `tendon_solref_lim` / `tendon_solimp_lim` fields to Model, parse from
  `<tendon><fixed>` and `<tendon><spatial>`, wire to tendon limit
  constraint rows. MuJoCo ref: `tendon_solref_lim[ntendon*mjNREF]`,
  `tendon_solimp_lim[ntendon*mjNIMP]` in `mjModel`;
  `mj_instantiateLimit()` in `engine_core_constraint.c`.
- DT-33: Tendon `margin` attribute -- add `tendon_margin: Vec<f64>` to
  Model, parse from `<tendon>`, replace 4 hardcoded `< 0.0` activation
  checks in assembly.rs (lines 148, 152, 540, 564) with margin-aware
  checks. Phase 7 Spec B (jnt_margin) is the direct reference pattern.
  MuJoCo ref: `tendon_margin[ntendon]` in `mjModel`.

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

- [ ] Complete

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

Reference the Phase 7 Spec B implementation of jnt_margin (§64a) as a
pattern for DT-33. The solver param lifecycle (parse -> defaults cascade
-> model field -> constraint assembly) should be consistent across all
three items.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A implementation

- [ ] Complete

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

- [ ] Complete

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

- [ ] Complete

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

## Session 13: Spec C rubric (Deformable Friction Projection)

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec C rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md

Spec C covers DT-25: Deformable-rigid friction cone projection. Currently
the deformable contact pipeline handles normal forces only (condim=1
scaffold). This spec extends it to support friction (condim=3/4/6) for
deformable-rigid contact pairs.

MuJoCo ref:
- Flex contact processing in `engine_collision_convex.c` -- deformable
  contacts generate the same constraint structure as rigid contacts
- `mj_instantiateContact()` in `engine_core_constraint.c` -- contact
  constraint row creation, same path for rigid and deformable
- The deformable contact Jacobian structure differs (vertex-based vs
  body-based), but the friction cone projection in the solver is unified

Also read:
- sim/docs/todo/future_work_10c.md (DT-25 entry)
- sim/L0/core/src/constraint/assembly.rs (current deformable constraint rows)
- sim/L0/core/src/forward/deformable.rs (deformable contact generation)

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for deformable contact constraint creation
- Phase 2: Build SPEC_C_RUBRIC.md (tailor P1 first with specific C functions)
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 14: Spec C spec (Deformable Friction Projection)

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- write Spec C spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C_RUBRIC.md

Write SPEC_C.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Specify how deformable contact Jacobians map to friction constraint rows
- Define the condim upgrade path (1 -> 3/4/6) for deformable contacts
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/
MuJoCo conformance is the cardinal goal.
```

---

## Session 15: Spec C implementation

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- implement Spec C.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Run `cargo test -p sim-core -p sim-mjcf -p sim-constraint
-p sim-conformance-tests` after each section. MuJoCo conformance is the
cardinal goal.
```

---

## Session 16: Spec C review -- create review document

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C.md

Copy the review template into this directory as SPEC_C_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C_REVIEW.md
```

---

## Session 17: Spec C review -- execute review

- [ ] Complete

```
Phase 8 Constraint & Solver Gaps -- execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/SPEC_C.md
3. The implementation files listed in SPEC_C.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final review of Phase 8. After the verdict, verify the Phase 8
aggregate ACs from the umbrella (PH8-AC1 through PH8-AC5) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 8 Umbrella | | |
| 2 | T1: DT-28 (tendon ball/free) + DT-39 (diagApprox) | | |
| 3 | Spec A rubric | | |
| 4 | Spec A spec | | |
| 5 | Spec A implementation | | |
| 6 | Spec A review -- create document | | |
| 7 | Spec A review -- execute | | |
| 8 | Spec B rubric | | |
| 9 | Spec B spec | | |
| 10 | Spec B implementation | | |
| 11 | Spec B review -- create document | | |
| 12 | Spec B review -- execute | | |
| 13 | Spec C rubric | | |
| 14 | Spec C spec | | |
| 15 | Spec C implementation | | |
| 16 | Spec C review -- create document | | |
| 17 | Spec C review -- execute | | |
