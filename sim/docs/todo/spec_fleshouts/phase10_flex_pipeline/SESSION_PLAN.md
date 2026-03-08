# Phase 10 — Flex Pipeline: Session Plan

23 sessions, each self-contained. The umbrella spec
(`PHASE10_UMBRELLA.md`) coordinates across context boundaries.

Phase 10 closes the flex (deformable body) pipeline gaps in the v1.0
roadmap: sparse edge Jacobian infrastructure, pre-computed data fields,
MuJoCo-conformant cotangent Laplacian bending, flex self-collision
dispatch, and flex-flex cross-object collision. The tasks span two
pipeline stages — passive force computation (§42A-i, §42A-ii, §42A-iii,
§42B) and collision detection (§42A-iv, §42A-v) — but converge on the
flex subsystem in `sim-core`.

Each spec follows a four-phase cycle: **rubric** -> **spec** ->
**implement** -> **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt -- the
documents and commits are the state, not the context window.

---

## Rescope Notes

### Pre-launch stress test (2026-03-06)

Two sizing adjustments based on stress-test analysis comparing against
Phase 5/8/9 session plans:

**Spec B (§42B Cotangent Bending): split implementation into 2 sessions
(6 total).** §42B is the most algorithmically complex item in Phase 10:
build-time precomputation (flexedge_flap topology, Wardetzky cotangent
weights, Garg curved reference, 4×4 stiffness matrix) + runtime force
application rewrite + FlexBendingModel trait (first trait in the sim
pipeline) + Bridson preservation. The natural seam is build-time vs
runtime: Session 10 handles precomputation and storage, Session 11
handles runtime force application, trait architecture, and Bridson
refactor.

~~**Spec D (§42A-v Flex-Flex): compressed from 5 to 3 sessions.**~~ Reverted
to full 5-session cycle (rubric, spec, implement, review-create,
review-execute) for consistency with Specs A/B/C.

Net effect: +1 (Spec B split) = **23 sessions** (up from 22).

---

## Task Assignment

Every Phase 10 task is assigned to exactly one deliverable:

| Task | Deliverable | Status | Rationale |
|------|-------------|--------|-----------|
| §42A-ii | T1 session | Pending | `flex_rigid`/`flexedge_rigid` boolean arrays — S effort, mechanical pre-computed Model fields |
| §42A-iii | T1 session | Pending | `flexedge_length`/`flexedge_velocity` pre-computed Data fields — S effort, mechanical |
| §42A-i | Spec A | Pending | Sparse flex edge Jacobian (`flexedge_J`) — L effort, algorithmic (CSR format, body Jacobian projection) |
| §42B | Spec B | Pending | Cotangent Laplacian bending — L effort, largest algorithmic task (Wardetzky/Garg precomputation + trait abstraction). 6 sessions (split implementation). |
| §42A-iv / DT-142 | Spec C | Pending | Flex self-collision dispatch — L effort, BVH/SAP midphase + narrowphase, `FlexSelfCollide` enum |
| §42A-v / DT-143 | Spec D | Pending | Flex-flex cross-object collision filtering — M effort, reuses Spec C primitives. 5 sessions (full cycle). |

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: §42A-ii + §42A-iii)              <- independent
    |       |
    |       v (hard dep: Spec C needs flex_rigid from §42A-ii)
    |   Sessions 14-18 (Spec C: §42A-iv Self-Collision)
    |       |
    |       v (hard dep: Spec D reuses Spec C narrowphase primitives)
    |   Sessions 19-23 (Spec D: §42A-v Flex-Flex)
    |
    +-- Sessions 3-7 (Spec A: §42A-i Edge Jacobian)     <- independent
    |
    +-- Sessions 8-13 (Spec B: §42B Bending)             <- independent (6 sessions)
```

### Dependency edges

| From -> To | Type | Specific dependency |
|-----------|------|---------------------|
| T1 (§42A-ii) -> Spec C (§42A-iv) | **Hard** | §42A-iv's self-collision gate requires `flex_rigid[f]` from §42A-ii. Cannot dispatch self-collision without the rigid-flex skip flag. |
| Spec C (§42A-iv) -> Spec D (§42A-v) | **Hard** | §42A-v reuses element-element narrowphase primitives from §42A-iv (`mj_collide_flex_internal`, `mj_collide_flex_self`). |
| Spec A (§42A-i) -> Spec B (§42B) | **Soft** | §42B's cotangent bending force application *could* use §42A-i's edge Jacobian for body-attached vertex projection. For free vertices (current test cases), inline application is correct. |

All other deliverables are independent — T1, Spec A, and Spec B can
proceed in parallel. Spec C must wait for T1. Spec D must wait for
Spec C.

### Why Spec A and Spec B are independent

§42A-i (edge Jacobian) provides force projection infrastructure for
body-attached flex vertices. §42B (cotangent bending) replaces the
bending force model with MuJoCo's cotangent Laplacian. Both apply
forces to `qfrc_passive` but through different mechanisms:

- §42A-i changes *how* forces are projected onto DOFs (sparse `J^T * F`)
- §42B changes *what* forces are computed (cotangent matrix vs dihedral angle)

For free vertices (all current models), both inline `±direction` and
sparse Jacobian produce identical results. §42B can apply cotangent
forces directly to vertex DOFs without the edge Jacobian. When
body-attached vertices are added (future work), both mechanisms
combine: cotangent computes forces, edge Jacobian projects them.

---

## Session 1: Phase 10 Umbrella

- [x] Complete

```
Phase 10 Flex Pipeline -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 10 section)
2. sim/docs/todo/future_work_10.md (§42A-i through §42A-v, §42B sections
   — lines 7119-7768)
3. sim/docs/todo/future_work_10i.md (DT-142, DT-143 context)
4. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SESSION_PLAN.md
5. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
   (structural template — multi-spec collision depth pass)
6. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
   (structural template — T1 session + multi-spec, includes T1 handling pattern)

Phase 10 is a flex pipeline depth pass: pre-computed data fields
(§42A-ii, §42A-iii), sparse Jacobian infrastructure (§42A-i),
MuJoCo-conformant bending (§42B), and flex collision completeness
(§42A-iv, §42A-v). The tasks span passive force computation and
collision detection but converge on the flex subsystem in sim-core.

Write PHASE10_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Sparse Flex Edge Jacobian (§42A-i)
  - Spec B: Cotangent Laplacian Bending (§42B) — 6 sessions (split impl)
  - Spec C: Flex Self-Collision Dispatch (§42A-iv / DT-142)
  - Spec D: Flex-Flex Cross-Object Collision (§42A-v / DT-143) — 3 sessions (compressed)
- T1 scope: §42A-ii (flex_rigid arrays) + §42A-iii (flexedge_length/velocity)
- Dependency Graph (Spec C depends on T1; Spec D depends on Spec C)
- File Ownership Matrix (which spec touches each shared file)
- API Contracts (cross-spec boundaries: e.g., T1 exposes flex_rigid
  that Spec C consumes)
- Shared Convention Registry (flex data field naming, force application
  patterns, collision dispatch conventions)
- Cross-Spec Blast Radius
- Phase-Level Acceptance Criteria (PH10-AC1 through PH10-AC6)
- Out of Scope (explicit exclusions: GPU flex, volume constraints,
  NeoHookean, flex equality constraints, shared-body vertices)

Key differences from Phase 9 umbrella:
- Phase 10 has two dependency chains (T1 -> Spec C -> Spec D)
- Mix of passive force (§42A-i, §42B) and collision (§42A-iv, §42A-v)
- §42B introduces a trait boundary (FlexBendingModel) — first trait
  in the sim pipeline, validates pattern for post-v1.0 trait architecture
- Two S-effort T1 items combined into a single session
- Spec B has split implementation (build-time / runtime seam)
- Spec D uses compressed 3-session cycle (M effort + Spec C reuse)

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 items (§42A-ii flex_rigid + §42A-iii flexedge_length/velocity)

- [x] Complete

```
Phase 10 Flex Pipeline -- implement T1 items.

Read these first:
- sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
  (Scope, File Ownership Matrix, Convention Registry sections)
- sim/docs/todo/future_work_10.md (§42A-ii at line 7203, §42A-iii at line 7262)

Two deliverables in one session:

**§42A-ii — `flex_rigid` / `flexedge_rigid` boolean arrays:**
Pre-compute two Model fields during build:
- `flex_rigid[f]` = true if ALL vertices of flex f have invmass == 0
- `flexedge_rigid[e]` = true if BOTH endpoints of edge e have invmass == 0
Replace per-vertex invmass checks in flex loops with boolean lookups.
MuJoCo ref: `engine_passive.c` loop structure, `flex_rigid` gate condition.

Steps:
1. Add `flex_rigid: Vec<bool>` and `flexedge_rigid: Vec<bool>` to Model
2. Compute during model build after `flexvert_invmass` is populated
3. Replace per-vertex invmass checks in dynamics/flex.rs outer/inner loops
4. Tests: rigid flex skipped, non-rigid flex unchanged, mixed cases

**§42A-iii — `flexedge_length` / `flexedge_velocity` pre-computed Data fields:**
Pre-compute two Data fields after forward kinematics:
- `flexedge_length[e]` = Euclidean distance between edge endpoints
- `flexedge_velocity[e]` = rate of length change (dot product of
  relative velocity with edge unit direction)
Replace inline computation in all consumers (edge spring-damper, Newton
penalty).
MuJoCo ref: `engine_passive.c` field access patterns,
`d->flexedge_length[e]`, `d->flexedge_velocity[e]`.

Steps:
1. Add `flexedge_length: Vec<f64>` and `flexedge_velocity: Vec<f64>` to Data
2. Compute after forward kinematics (when flexvert_xpos and qvel are current)
3. Replace inline length/velocity computation in dynamics/flex.rs consumers
4. Tests: pre-computed values match inline computation, multiple consumers
   read same values, identical simulation results

Both items are optimization + MuJoCo Data layout parity — no behavior change.
Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after.
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: Spec A rubric (Sparse Flex Edge Jacobian)

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md

Spec A covers §42A-i: Sparse flex edge Jacobian (`flexedge_J`) — a
pre-computed CSR-format Jacobian that projects edge forces through body
Jacobians for correct force application on body-attached flex vertices.

MuJoCo ref:
- `engine_passive.c` — flex edge/bending/penalty force application
  using `flexedge_J` sparse Jacobian for `J^T * force` projection.
- `engine_forward.c` — `mj_flex()` computes `flexedge_J` after
  forward kinematics, before passive force computation.
- CSR format: `flex_edge_J_rownnz`, `flex_edge_J_rowadr`,
  `flex_edge_J_colind`, `flexedge_J` (data array).
- For free vertices: Jacobian row is `±direction` on 3 translational DOFs.
- For body-attached vertices: Jacobian row is `mj_jac()` at vertex
  position for the attached body, multiplied by ±1.

Key areas to validate:
- CSR sparse matrix construction and storage
- Free vertex degenerate case (±direction on 3 DOFs, identical to current)
- Body-attached vertex projection through mj_jac()
- Three consumer sites: edge spring-damper, bending, Newton penalty
- Unified J^T * force code path replacing three inline patterns
- Performance: precompute once, read 3x vs inline compute 3x

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for flexedge_J computation
- Phase 2: Build SPEC_A_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 4: Spec A spec (Sparse Flex Edge Jacobian)

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- CSR sparse matrix format: rownnz, rowadr, colind, data layout
- Jacobian computation: per-edge, per-endpoint, free vs body-attached
- Free vertex path: ±direction on 3 translational DOFs
- Body-attached path: mj_jac() at vertex position for attached body
- Force application: unified J^T * force replacing 3 inline patterns
  (edge spring-damper, bending, Newton penalty)
- Model fields (CSR metadata) vs Data fields (Jacobian values)
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A implementation

- [x] Complete

```
Phase 10 Flex Pipeline -- implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Start with CSR structure and Model fields, then Data computation, then
consumer migration (replace inline ±direction with J^T * force). Verify
that for free-vertex models, results are bit-identical to pre-Spec-A.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec A review -- create review document

- [x] Complete

```
Phase 10 Flex Pipeline -- create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A.md

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

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A_REVIEW.md
```

---

## Session 7: Spec A review -- execute review

- [x] Complete

```
Phase 10 Flex Pipeline -- execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_A.md
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

## Session 8: Spec B rubric (Cotangent Laplacian Bending)

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md

Spec B covers §42B: Cotangent Laplacian bending — MuJoCo's actual
bending formulation (Wardetzky et al. "Discrete Quadratic Curvature
Energies" with Garg et al. "Cubic Shells" curved reference correction).
This replaces the current Bridson dihedral angle spring model as the
default, with Bridson preserved via a FlexBendingModel trait.

MuJoCo ref:
- `user_mesh.cc` → `ComputeBending()` (lines ~3740-3784) — precomputation
  of 17-coefficient bending matrix per edge: 4x4 cotangent stiffness
  matrix + 1 curved reference coefficient.
- `engine_passive.c` (lines ~206-268) — runtime bending force application
  via matrix-vector multiply: `spring[3*i+x] += b[4*i+j] * xpos[3*v[j]+x]`
- `flex_bending` model array: 17 f64 per edge, CSR-like flat layout.
- `flexedge_flap` topology: 2 vertices per edge (opposite vertices in
  adjacent triangles forming the diamond stencil).

Key areas to validate:
- Cotangent weight computation from rest geometry
- Material stiffness: mu = young / (2*(1+poisson)), scaled by thickness^3
- 4x4 stiffness matrix outer product construction
- Curved reference coefficient for non-flat rest meshes (Garg correction)
- Boundary edge handling (one adjacent triangle: zero coefficients)
- FlexBendingModel trait: Cotangent (default) and Bridson implementations
- flexedge_flap topology computation from element connectivity
- No stability clamp needed for cotangent (constant matrix)

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for bending computation
- Phase 2: Build SPEC_B_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 9: Spec B spec (Cotangent Laplacian Bending)

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B_RUBRIC.md

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- S1: flexedge_flap topology — diamond stencil vertex discovery from
  element connectivity, boundary edge detection
- S2: Cotangent weight precomputation — Wardetzky operator, per-edge
  4-element weight vector from rest configuration cotangent angles
- S3: Material stiffness — shear modulus, plate bending stiffness
  formula, thickness^3 scaling
- S4: 4x4 stiffness matrix — outer product c[i]*c[j]*stiffness
- S5: Curved reference coefficient — Garg "Cubic Shells" correction
  for non-zero rest curvature (b[16] term)
- S6: Runtime force application — matrix-vector spring + damper,
  no stability clamp
- S7: FlexBendingModel trait — Cotangent and Bridson implementations,
  dispatch from mj_fwd_passive()
- S8: Bridson preservation — move current dihedral code into
  BridsonBending impl, no algorithm changes

The spec must clearly mark the implementation split point:
- Sessions 10 scope: S1-S5 (build-time: topology, precomputation, storage)
- Session 11 scope: S6-S8 (runtime: force application, trait, Bridson refactor)

Grade each spec section against the rubric as you write.
Present for approval -- do NOT implement.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal.
```

---

## Session 10: Spec B implementation — build-time (S1-S5)

- [x] Complete

```
Phase 10 Flex Pipeline -- implement Spec B build-time sections.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md

Implement S1 through S5 ONLY (build-time precomputation):
- S1: flexedge_flap topology computation from element connectivity
- S2: Cotangent weight precomputation (Wardetzky operator)
- S3: Material stiffness computation (shear modulus, plate bending)
- S4: 4x4 stiffness matrix construction (outer product)
- S5: Curved reference coefficient (Garg correction, b[16] term)

This session delivers `flex_bending: Vec<f64>` (17 coefficients per edge)
and `flexedge_flap: Vec<[usize; 2]>` fully populated at model build time.
The runtime force application (S6-S8) is Session 11.

Commit after each section. Verify precomputation ACs as you go. The
cotangent weights and stiffness matrix must match MuJoCo exactly —
verify against user_mesh.cc:ComputeBending. Test with known geometries
(flat quad, single triangle pair) where cotangent weights can be
hand-verified.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec B implementation — runtime + trait (S6-S8)

- [x] Complete

```
Phase 10 Flex Pipeline -- implement Spec B runtime and trait sections.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md
3. Session 10 commits (verify S1-S5 build-time precomputation is complete)

Implement S6 through S8 (runtime force application + trait architecture):
- S6: Cotangent runtime force application — matrix-vector spring + damper
  loop using precomputed flex_bending coefficients. No stability clamp.
- S7: FlexBendingModel trait definition, CotangentBending implementation,
  dispatch from mj_fwd_passive()
- S8: BridsonBending implementation — move existing dihedral angle code
  (passive.rs:581-699) into trait impl, no algorithm changes. Verify
  Bridson path still produces identical results.

Also implement:
- FlexBendingType enum + MJCF `bending_model` attribute parsing
- Default: Cotangent (MuJoCo conformance)

Commit after each section. Test cotangent bending against MuJoCo: 4x4
grid shell (young=1e4, poisson=0.3, thickness=0.01, density=1000),
compare qfrc_passive after 1 step with gravity. Test Bridson regression:
switching to bending_model="bridson" must produce identical results to
pre-Spec-B behavior.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 12: Spec B review -- create review document

- [x] Complete

```
Phase 10 Flex Pipeline -- create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md

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

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B_REVIEW.md
```

---

## Session 13: Spec B review -- execute review

- [x] Complete

```
Phase 10 Flex Pipeline -- execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_B.md
3. The implementation files listed in SPEC_B.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section (S1-S8,
spanning both implementation sessions), verify each AC has a passing test,
check every planned test was written, compare blast radius predictions
against reality, audit convention notes, scan for weak implementations,
and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

Present the Review Verdict (section 10) to the user when done.
```

---

## Session 14: Spec C rubric (Flex Self-Collision Dispatch)

**Prerequisite:** Session 2 (T1: §42A-ii flex_rigid) must be complete.
Spec C's self-collision gate requires `flex_rigid[f]` from §42A-ii.

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec C rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md

Spec C covers §42A-iv / DT-142: Flex self-collision dispatch — BVH/SAP
midphase + element-element narrowphase for non-adjacent elements within
a single flex body.

MuJoCo ref:
- `engine_collision_driver.c` — flex self-collision dispatch from
  `mj_collision()`. Three conjunctive gate conditions: `!flex_rigid[f]`,
  `(flex_contype[f] & flex_conaffinity[f]) != 0`, per-path enable flags.
- `mjtFlexSelf` enum: NONE=0, NARROW=1, BVH=2, SAP=3, AUTO=4.
- Two independent paths: `mj_collideFlexInternal()` (adjacent elements)
  and `mj_collideFlexSelf()` (non-adjacent elements with midphase).
- `flex_internal` flag gates adjacent-element collision.
- `flex_selfcollide` enum gates non-adjacent collision with algorithm
  selection (brute-force, BVH, SAP, or auto).

Key areas to validate:
- Three-condition gate: flex_rigid, contype/conaffinity self-bitmask,
  per-path enable flags
- FlexSelfCollide enum upgrade from Vec<bool> to Vec<FlexSelfCollide>
- Internal collision: adjacent element-element contact generation
- Self-collision: non-adjacent element pairs with midphase acceleration
- BVH: per-element AABB tree, updated per-step from flexvert_xpos
- SAP: sweep-and-prune on element AABBs
- AUTO mode: BVH for dim=3, SAP otherwise
- Contact generation consistent with existing flex-rigid contacts

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for flex self-collision dispatch
- Phase 2: Build SPEC_C_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 15: Spec C spec (Flex Self-Collision Dispatch)

- [x] Complete — commit `f384a2f`

```
Phase 10 Flex Pipeline -- write Spec C spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C_RUBRIC.md

Write SPEC_C.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Three-condition gate logic (flex_rigid, self-bitmask, enable flags)
- FlexSelfCollide enum: None, Narrow, Bvh, Sap, Auto
- Internal collision path: adjacent element-element narrowphase for
  elements sharing vertices/edges
- Self-collision path: non-adjacent element pairs
- BVH midphase: per-element AABB tree construction and query
- SAP midphase: sweep-and-prune on element AABBs
- AUTO dispatch: BVH for dim=3 solids, SAP otherwise
- Element-element narrowphase: triangle-triangle (dim=2),
  tetrahedron-tetrahedron (dim=3) contact generation
- Contact normal and penetration depth conventions
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal.
```

---

## Session 16: Spec C implementation

- [x] Complete

```
Phase 10 Flex Pipeline -- implement Spec C.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Start with the FlexSelfCollide enum upgrade and gate logic, then implement
internal collision (adjacent elements), then self-collision narrowphase
(brute-force NARROW mode), then BVH/SAP midphase acceleration. Test with
a deformable body that folds onto itself (e.g., a cloth draped over a
sharp edge generating self-contacts).

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 17: Spec C review -- create review document

- [x] Complete

```
Phase 10 Flex Pipeline -- create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C.md

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

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C_REVIEW.md
```

---

## Session 18: Spec C review -- execute review

- [x] Complete

```
Phase 10 Flex Pipeline -- execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C.md
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

## Session 19: Spec D rubric (Flex-Flex Cross-Object Collision)

**Prerequisite:** Session 18 (Spec C review) must be complete. Spec D
reuses element-element narrowphase primitives from Spec C.

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec D rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md

Spec D covers §42A-v / DT-143: Flex-flex cross-object collision filtering
— broadphase filtering via contype/conaffinity bitmasks + element-element
narrowphase between two different flex objects.

MuJoCo ref:
- `engine_collision_driver.c` → `canCollide2()` — unified bodyflex index
  space where bodies occupy [0, nbody) and flexes occupy [nbody, nbody+nflex).
- Flex-flex narrowphase: element-element collision tests
  (triangle-triangle for dim=2, tetrahedron-tetrahedron for dim=3)
  with BVH acceleration per flex object.
- Contact parameter combination: priority + solmix protocol matching
  flex-rigid contact params.

Key areas to validate:
- Broadphase: bodyflex index space, canCollide2() bitmask filter
- Narrowphase: element-element collision between two flex objects,
  reusing Spec C's primitives (triangle-triangle, tet-tet)
- BVH acceleration: per-flex AABB trees for cross-body pair pruning
- Contact parameter combination: contact_param_flex_flex() with
  priority + solmix protocol
- Pipeline integration: dispatch loop after flex-rigid and flex-self

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for flex-flex collision dispatch
- Phase 2: Build SPEC_D_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 20: Spec D spec (Flex-Flex Cross-Object Collision)

- [x] Complete

```
Phase 10 Flex Pipeline -- write Spec D spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D_RUBRIC.md

Write SPEC_D.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Broadphase: bodyflex index space, canCollide2() bitmask filter
- Narrowphase: element-element collision between two flex objects,
  reusing Spec C's primitives (triangle-triangle, tet-tet)
- BVH acceleration: per-flex AABB trees for cross-body pair pruning
- Contact parameter combination: contact_param_flex_flex() with
  priority + solmix protocol
- Pipeline integration: dispatch loop after flex-rigid and flex-self
- Contact encoding: flex_vertex + flex_vertex2 from different flex objects,
  geom1/geom2 sentinel convention
- Constraint Jacobian: reuse self-collision Jacobian pattern with
  DOFs from two different flex objects
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 21: Spec D implementation

- [x] Complete

```
Phase 10 Flex Pipeline -- implement Spec D.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/PHASE10_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Verify Spec C (§42A-iv self-collision) is complete before implementing —
Spec D reuses Spec C's element-element narrowphase primitives. Test with
two flex bodies falling toward each other (e.g., two cloth sheets colliding).
Verify bitmask filtering: incompatible contype/conaffinity produces zero
flex-flex contacts.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 22: Spec D review -- create review document

- [ ] Complete

```
Phase 10 Flex Pipeline -- create Spec D review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D.md

Copy the review template into this directory as SPEC_D_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D_REVIEW.md
```

---

## Session 23: Spec D review -- execute review

- [ ] Complete

```
Phase 10 Flex Pipeline -- execute Spec D review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_D.md
3. The implementation files listed in SPEC_D.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final session of Phase 10. After completing, verify the Phase 10
aggregate ACs from the umbrella (PH10-AC1 through PH10-AC6) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 10 Umbrella | Done | 6bc8acf |
| 2 | T1: §42A-ii + §42A-iii | Done | 1f0230c |
| 3 | Spec A rubric (§42A-i Edge Jacobian) | Done | 6d03aed |
| 4 | Spec A spec | Done | 7b1efb1 |
| 5 | Spec A implementation | Done | 0a46fef |
| 6 | Spec A review -- create document | Done | 957fe2d |
| 7 | Spec A review -- execute | Done | 5df93f9 |
| 8 | Spec B rubric (§42B Bending) | Done | bc5b4ac |
| 9 | Spec B spec | Done | 27ef6e5 |
| 10 | Spec B implementation -- build-time (S1-S5) | Done | 4612f00 |
| 11 | Spec B implementation -- runtime + trait (S6-S8) | Done | 082dd4b |
| 12 | Spec B review -- create document | Done | 7a808fb |
| 13 | Spec B review -- execute | Done | 7e9e3c3 |
| 14 | Spec C rubric (§42A-iv Self-Collision) | Done | 3db4561 |
| 15 | Spec C spec | Done | `f384a2f` |
| 16 | Spec C implementation | Done | 469c61d |
| 17 | Spec C review -- create document | Done | b1263d0 |
| 18 | Spec C review -- execute | Done | 772b8a3 |
| 19 | Spec D rubric (§42A-v Flex-Flex) | Done | 460705b |
| 20 | Spec D spec | Done | `fb851d4` |
| 21 | Spec D implementation | Done | `8945051` |
| 22 | Spec D review -- create document | Pending | |
| 23 | Spec D review -- execute | Pending | |
