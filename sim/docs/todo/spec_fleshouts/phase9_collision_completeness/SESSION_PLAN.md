# Phase 9 — Collision Completeness: Session Plan

27 sessions, each self-contained. The umbrella spec
(`PHASE9_UMBRELLA.md`) coordinates across context boundaries.

Phase 9 covers collision pipeline completeness: mesh inertia modes,
continuous collision detection (CCD), missing heightfield collision
pairs, SDF option wiring, mesh convex hull auto-computation, and
deformable-vs-complex-geom narrowphase. The tasks span three subsystems
(build-time mesh processing, broadphase/narrowphase collision, and
deformable contact) but converge on the collision pipeline in `sim-core`.

Each spec follows a four-phase cycle: **rubric** -> **spec** ->
**implement** -> **review** (create review document, then execute it).
The review phase catches weak implementations, spec deviations, and
untracked deferred work before moving to the next spec.

**Check off each session as it completes.** If a session runs out of
context mid-task, start a new session with the same prompt -- the
documents and commits are the state, not the context window.

---

## Rescope Notes

No rescopes yet.

---

## Task Assignment

Every Phase 9 task is assigned to exactly one deliverable:

| Task | Deliverable | Status | Rationale |
|------|-------------|--------|-----------|
| §57 | T1 session | Pending | SDF option wiring — parse 2 int attributes from `<option>`, wire to `sdf.rs` collision functions |
| §65 | Spec A | Pending | Mesh convex hull auto-computation (Quickhull) — foundational for mesh collision + §43 Convex mode |
| §43 | Spec B | Pending | Mesh inertia modes (exact/shell/convex/legacy) — depends on Spec A for Convex mode |
| §54 | Spec C | Pending | Missing heightfield collision pairs (hfield-mesh, hfield-plane, hfield-hfield) |
| §50 | Spec D | Pending | Continuous Collision Detection — conservative-advancement CCD, largest algorithmic task |
| DT-70 | Spec E | Pending | Deformable-vs-mesh/hfield/SDF narrowphase — three new collision pair types |

---

## Dependency Graph

```
Session 1 (Umbrella)
    |
    +-- Session 2 (T1: §57 SDF options)                     <- independent
    |
    +-- Sessions 3-7 (Spec A: §65 Convex Hull)              <- independent
    |       |
    |       v (hard dep)
    |   Sessions 8-12 (Spec B: §43 Mesh Inertia)            <- depends on Spec A
    |       :
    |       v (soft dep — flex-vs-mesh may use convex hull)
    |   Sessions 23-27 (Spec E: DT-70 Deformable NP)        <- can proceed without Spec A
    |
    +-- Sessions 13-17 (Spec C: §54 Hfield Pairs)           <- independent
    |
    +-- Sessions 18-22 (Spec D: §50 CCD)                    <- independent
```

**Hard dependency:** Spec B depends on Spec A — §43's `Convex` inertia
mode delegates to §65's convex hull computation.

**Soft dependency:** Spec E (DT-70) flex-vs-mesh *may* use Spec A's
convex hull for vertex-sphere-vs-convex-mesh queries, but can fall back
to brute-force per-triangle distance if Spec A is not yet complete.

All other deliverables are independent — ordering is for clarity, not
necessity.

---

## Session 1: Phase 9 Umbrella

- [x] Complete

```
Phase 9 Collision Completeness -- write the umbrella spec.

Read these in order:
1. sim/docs/ROADMAP_V1.md (Phase 9 section)
2. sim/docs/todo/future_work_11.md (§43 mesh inertia modes)
3. sim/docs/todo/future_work_12.md (§50 CCD)
4. sim/docs/todo/future_work_13.md (§54 hfield collision pairs)
5. sim/docs/todo/future_work_14.md (§57 SDF option wiring)
6. sim/docs/todo/future_work_16.md (§65 mesh convex hull)
7. sim/docs/todo/future_work_10i.md (DT-70 deformable narrowphase)
8. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SESSION_PLAN.md
9. sim/docs/todo/spec_fleshouts/phase8_constraint_solver_gaps/PHASE8_UMBRELLA.md
   (structural template -- single-subsystem depth pass)
10. sim/docs/todo/spec_fleshouts/phase5_actuator_completeness/PHASE5_UMBRELLA.md
    (structural template -- T1 session + multi-spec, includes T1 handling pattern)

Phase 9 is a collision pipeline depth pass: build-time mesh processing,
narrowphase collision pair coverage, and CCD. Unlike Phase 8 (constraint
subsystem), Phase 9 tasks span build-time processing (§65, §43) and
runtime collision (§50, §54, DT-70), with §57 being pure plumbing.

Write PHASE9_UMBRELLA.md covering:
- Scope statement with conformance mandate
- Task Assignment table (mirror SESSION_PLAN.md assignments)
- Sub-spec scope statements with MuJoCo C source citations:
  - Spec A: Mesh Convex Hull Auto-Computation (§65)
  - Spec B: Mesh Inertia Modes (§43) -- depends on Spec A
  - Spec C: Heightfield Collision Pairs (§54)
  - Spec D: Continuous Collision Detection (§50)
  - Spec E: Deformable Complex-Geom Narrowphase (DT-70)
- T1 scope: §57 SDF options (1 session, direct implementation)
- Dependency Graph (Spec B depends on Spec A; all others independent)
- File Ownership Matrix (which spec touches each shared file)
- API Contracts (cross-spec boundaries: e.g., Spec A exposes ConvexHull
  that Spec B consumes)
- Shared Convention Registry (collision pair dispatch patterns, contact
  generation signatures, geom type enums)
- Cross-Spec Blast Radius
- Phase-Level Acceptance Criteria (PH9-AC1 through PH9-AC6)
- Out of Scope (explicit exclusions: GPU collision, XPBD, flex self-collision)

Key differences from Phase 8 umbrella:
- Phase 9 has one cross-spec dependency (Spec B -> Spec A)
- One T1 item (§57) vs Phase 8's two
- Five specs vs Phase 8's two — more breadth
- Mix of build-time (§65, §43) and runtime (§50, §54, DT-70) work
- Spec D (CCD) is the largest single algorithmic task in the phase

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 2: T1 item (§57 SDF options)

- [x] Complete

```
Phase 9 Collision Completeness -- implement T1 item.

Read these first:
- sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
  (Scope, File Ownership Matrix, Convention Registry sections)
- sim/docs/todo/future_work_14.md (§57 entry)

One deliverable: §57 — Wire `sdf_iterations` and `sdf_initpoints` from
`<option>` to the SDF collision solver. MuJoCo ref: `mjOption` struct
fields `sdf_iterations` (default 10) and `sdf_initpoints` (default 40).

Current state (audit before implementing):
- `sdf_iterations`: hardcoded as `let max_iterations = 10;` in
  `sim/L0/core/src/sdf.rs` (line ~531, `closest_surface_point()`).
  NOT in `sdf_collide.rs` — the dispatch wrapper has no iteration
  constants; the actual Newton iterations live in `sdf.rs`.
- `sdf_initpoints`: NOT a single constant to replace. Multiple SDF
  contact functions use different hardcoded sample counts:
  `sdf_cylinder_contact` → 26, `sdf_ellipsoid_contact` → 26,
  `sdf_box_contact` → 8 corners, etc. The MuJoCo `sdf_initpoints`
  controls the number of initial sample points for SDF collision
  search — understand which functions should use it and which have
  fixed geometric patterns (e.g., box corners are always 8).

Steps:
1. Parse `sdf_iterations` (int, default 10) and `sdf_initpoints` (int,
   default 40) from `<option>` element in MJCF parser
2. Add fields to `MjcfOption` / `MjOption` / model option struct
3. Thread `sdf_iterations` to `closest_surface_point()` in `sdf.rs`,
   replacing the hardcoded `max_iterations = 10`
4. Thread `sdf_initpoints` to the appropriate SDF contact functions —
   audit each `sdf_*_contact` function to determine which should use
   the configurable init point count vs. fixed geometric sampling
5. Add tests: verify parsing, verify default values, verify non-default
   values reach the SDF solver

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after.
MuJoCo conformance is the cardinal goal.
```

---

## Session 3: Spec A rubric (Mesh Convex Hull Auto-Computation)

- [X] Complete

```
Phase 9 Collision Completeness -- write Spec A rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md

Spec A covers §65: Mesh convex hull auto-computation at build time
using Quickhull (or equivalent) for GJK/EPA collision detection.

MuJoCo ref:
- MuJoCo always computes convex hulls for mesh geoms used in collision.
  The hull is computed from the mesh vertices during model compilation
  (`user_mesh.cc` / `mjCMesh::MakeConvex()`).
- The convex hull is stored as a separate vertex/face set and used by
  the GJK/EPA narrowphase for mesh-vs-* collision pairs.
- `convexhull` attribute on `<mesh>` controls whether hull is computed.

Key areas to validate:
- Quickhull algorithm correctness (degenerate cases, coplanar points,
  numerical stability)
- Integration with existing mesh data structures
- Storage format compatible with GJK support function
- Build-time computation performance for large meshes
- Convex hull used as collision geometry while original mesh used for
  rendering/inertia

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for mesh convex hull computation
- Phase 2: Build SPEC_A_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 4: Spec A spec (Mesh Convex Hull Auto-Computation)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec A spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A_RUBRIC.md

Write SPEC_A.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Quickhull algorithm specification: incremental convex hull construction,
  horizon edge detection, conflict graph management
- Degenerate handling: coplanar points, collinear points, duplicate vertices
- Storage format: vertex list + face list (half-edge or indexed) compatible
  with GJK support function queries
- Integration: `ConvexHull` struct stored per mesh at build time, used by
  collision dispatch for mesh geom types
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 5: Spec A implementation

- [x] Complete

```
Phase 9 Collision Completeness -- implement Spec A.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

The convex hull must be computed at build time and stored on the mesh. The
GJK support function must be able to query the hull efficiently. Test with
simple known meshes (cube, tetrahedron) and verify hull vertex/face counts.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 6: Spec A review -- create review document

- [x] Complete

```
Phase 9 Collision Completeness -- create Spec A review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A.md

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

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A_REVIEW.md
```

---

## Session 7: Spec A review -- execute review

- [x] Complete

```
Phase 9 Collision Completeness -- execute Spec A review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_A.md
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

## Session 8: Spec B rubric (Mesh Inertia Modes)

**Prerequisite:** Session 7 (Spec A review) must be complete. Spec B's
`Convex` inertia mode delegates to Spec A's `ConvexHull` computation.

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec B rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md

Spec B covers §43: Mesh inertia modes — `exact`, `shell`, `convex`, `legacy`.
This adds a 4-mode `MeshInertia` enum to the `<mesh>` element's `inertia`
attribute, plus backward-compatible `shellinertia` on `<geom>`.

MuJoCo ref:
- `user_mesh.cc` → `ComputeInertia()` — shell inertia algorithm
  (area-weighted triangle decomposition, divisor 12 instead of 20)
- `mjtMeshInertia` enum in `mjspec.h` (exact, shell, convex, legacy)
- Shell inertia: `mass = density * total_surface_area`, second moments
  computed per-triangle with surface-distribution formulas
- Convex mode: `compute_mesh_inertia(&convex_hull(mesh))` — delegates
  to Spec A's convex hull, then computes exact volumetric inertia on hull
- Primitive shell formulas: sphere, box, capsule, cylinder, ellipsoid
  (analytical hollow-shell inertia tensors)

Depends on Spec A (§65) for `Convex` mode — verify Spec A is complete
before implementing that mode. `Legacy` mode is backward compat and low
priority.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for mesh inertia computation
- Phase 2: Build SPEC_B_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 9: Spec B spec (Mesh Inertia Modes)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec B spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B_RUBRIC.md

Write SPEC_B.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Four inertia modes: exact (volumetric), shell (surface area-weighted),
  convex (volumetric on convex hull), legacy (backward compat)
- Shell inertia algorithm: area-weighted surface distribution per-triangle,
  mass = density * total_area, second moments via surface formulas
- Primitive shell formulas for 5 geom types (closed-form substitutions)
- `shellinertia` backward-compat attribute on `<geom>`
- Convex mode integration with Spec A's ConvexHull (API contract)
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 10: Spec B implementation

**Prerequisite:** Spec A (§65 convex hull) must be complete before
implementing the `Convex` inertia mode. `Exact` and `Shell` modes
can be implemented independently.

- [x] Complete

```
Phase 9 Collision Completeness -- implement Spec B.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Verify Spec A (§65 convex hull) is complete before implementing Convex mode.
Test shell inertia against known analytical results (e.g., hollow sphere
shell inertia = 2/3 * m * r^2).

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 11: Spec B review -- create review document

- [x] Complete

```
Phase 9 Collision Completeness -- create Spec B review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B.md

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

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B_REVIEW.md
```

---

## Session 12: Spec B review -- execute review

- [x] Complete

```
Phase 9 Collision Completeness -- execute Spec B review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_B.md
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

## Session 13: Spec C rubric (Heightfield Collision Pairs)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec C rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md

Spec C covers §54: Missing heightfield collision pairs — hfield-mesh
(highest priority), hfield-plane, and hfield-hfield.

Currently heightfield collision supports sphere, capsule, and box
directly. Cylinder is approximated as capsule, and ellipsoid is
approximated as sphere (using `max_r`). Three pairs are explicitly
missing: hfield-mesh, hfield-plane, hfield-hfield (see comment in
`hfield.rs` line ~71: `// Hfield-Hfield, Hfield-Plane, Hfield-Mesh:
not supported`). MuJoCo ref: `engine_collision_primitive.c`
heightfield dispatch + `mjCOLLISION_TABLE` for supported pairs.

Key areas to validate:
- hfield-mesh: triangle-cell intersection against heightfield grid
  (this is the main deliverable and hardest algorithm)
- hfield-plane: plane clipping against heightfield boundary cells
- hfield-hfield: dual-grid intersection (lowest priority, rarely used)
- Grid traversal infrastructure reuse from existing sphere/capsule paths
- Contact normal and penetration depth computation for each pair
- Existing hfield-cylinder and hfield-ellipsoid approximations: audit
  whether MuJoCo has dedicated implementations for these pairs or also
  approximates. If MuJoCo has exact implementations, scope them into
  this spec or defer explicitly.

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for heightfield collision
- Phase 2: Build SPEC_C_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 14: Spec C spec (Heightfield Collision Pairs)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec C spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C_RUBRIC.md

Write SPEC_C.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- hfield-mesh algorithm: for each mesh triangle, identify overlapping
  heightfield cells, compute triangle-vs-cell contact points
- hfield-plane algorithm: identify boundary cells below plane, generate
  contacts at cell vertices/edges
- hfield-hfield algorithm: dual-grid approach (if MuJoCo supports this
  at all — verify in source; may be out of scope)
- Reuse existing heightfield grid traversal infrastructure
- Contact generation signatures consistent with existing pairs
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 15: Spec C implementation

- [x] Complete

```
Phase 9 Collision Completeness -- implement Spec C.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Focus on hfield-mesh first (highest priority). Test with a mesh object
resting on / intersecting a heightfield terrain. Verify contact points
and normals match expected geometry.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 16: Spec C review -- create review document

- [x] Complete

```
Phase 9 Collision Completeness -- create Spec C review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C.md

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

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C_REVIEW.md
```

---

## Session 17: Spec C review -- execute review

- [x] Complete

```
Phase 9 Collision Completeness -- execute Spec C review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_C.md
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

## Session 18: Spec D rubric (Continuous Collision Detection)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec D rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md

Spec D covers §50: Continuous Collision Detection (CCD) using
conservative advancement for convex geom pairs. This is the largest
algorithmic task in Phase 9.

**Hidden prerequisite:** The existing GJK in `gjk_epa.rs` is
intersection-only — it returns bool + EPA penetration depth but has
NO separating distance query for non-overlapping shapes. Conservative
advancement requires knowing the minimum separation distance at each
iteration step. Spec D must add a `gjk_distance()` function (or
equivalent closest-point query) to `gjk_epa.rs` as its first step
before implementing the CCD algorithm itself.

MuJoCo ref:
- `engine_collision_driver.c` — CCD dispatch after broadphase
- Conservative advancement: iterative time-of-impact estimation using
  upper-bound velocity and GJK minimum separation distance
- `ccd_iterations` option controls max iterations (already parsed/stored
  as `MjcfOption.ccd_iterations`, default 50 in CortenForge types.rs)
- `ccd_tolerance` — distance threshold for contact. **NOT currently
  parsed** (only in dm_control schema). Must be parsed as part of this
  spec or explicitly deferred.
- `nativeccd` / `multiccd` enable flags (already wired as no-op guards
  with `tracing::warn!` in `builder/mod.rs`)
- Convex-convex only (sphere, capsule, box, ellipsoid, convex mesh)
- Multi-point CCD for flat surfaces (MULTICCD flag)

Key areas to validate:
- GJK distance query: extend `gjk_epa.rs` with a closest-point /
  separating distance function for non-overlapping convex shapes
- Conservative advancement algorithm: velocity upper bound, GJK distance
  query in inner loop, convergence guarantee
- Time-of-impact contact generation (interpolate to TOI, run narrowphase)
- Integration with existing broadphase (which pairs get CCD?)
- `ccd_tolerance` parsing from `<option>` (or explicit deferral)
- Flag wiring: DISABLE_NATIVECCD guard, ENABLE_MULTICCD guard
- Performance: CCD is expensive, only needed for high-speed thin objects

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for CCD implementation
- Phase 2: Build SPEC_D_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 19: Spec D spec (Continuous Collision Detection)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec D spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D_RUBRIC.md

Write SPEC_D.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- S1 (prerequisite): GJK distance query — extend `gjk_epa.rs` with a
  `gjk_distance()` / closest-point function for non-overlapping convex
  shapes. The existing GJK is intersection-only and cannot provide the
  separating distance needed by conservative advancement.
- Conservative advancement algorithm: mathematical formulation, convergence
  proof sketch, iteration control via `ccd_iterations`
- GJK distance query integration: how the inner loop calls the new
  `gjk_distance()` for minimum separation distance at interpolated
  configurations
- Velocity upper bound computation: max linear + angular velocity across
  the geom pair
- Time-of-impact contact generation: interpolate bodies to TOI, run standard
  narrowphase, report contacts at TOI configuration
- CCD pair selection: which broadphase pairs qualify for CCD (velocity
  threshold, geom type filter)
- `ccd_tolerance` parsing: parse from `<option>`, add to model options,
  use as distance threshold for CCD contact (or defer with rationale)
- Multi-point CCD (MULTICCD): additional contact points for flat surfaces
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 20: Spec D implementation

- [x] Complete

```
Phase 9 Collision Completeness -- implement Spec D.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Start with S1 (GJK distance query) — this is a prerequisite for the CCD
algorithm. The existing GJK in `gjk_epa.rs` only handles intersection;
you must add separating distance computation for non-overlapping shapes.
Test the distance query independently before building CCD on top of it.

CCD is algorithmic — pay special attention to numerical robustness (epsilon
guards in GJK distance, velocity bound computation, TOI interpolation).
Test with a fast-moving sphere passing through a thin box (the canonical
tunneling scenario).

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 21: Spec D review -- create review document

- [x] Complete

```
Phase 9 Collision Completeness -- create Spec D review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D.md

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

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D_REVIEW.md
```

---

## Session 22: Spec D review -- execute review

- [x] Complete

```
Phase 9 Collision Completeness -- execute Spec D review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_D.md
3. The implementation files listed in SPEC_D.md's Files Affected section

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

## Session 23: Spec E rubric (Deformable Complex-Geom Narrowphase)

- [x] Complete

```
Phase 9 Collision Completeness -- write Spec E rubric.

Read these in order:
1. sim/docs/templates/pre_implementation/RUBRIC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md

Spec E covers DT-70: Deformable-vs-mesh/hfield/SDF narrowphase. Currently
`flex_collide.rs` only supports primitive geom types (plane, sphere, box,
capsule, cylinder, ellipsoid). This adds three new collision pair types:
flex-vs-mesh, flex-vs-hfield, and flex-vs-SDF.

MuJoCo ref:
- `engine_collision_primitive.c` / flex collision dispatch
- Flex vertex sphere vs. complex geom: each flex vertex is treated as a
  sphere with radius `flex_radius`; the narrowphase computes distance from
  vertex sphere to the complex geom

**Soft dependency on Spec A (§65):** Flex-vs-mesh *may* use Spec A's
convex hull for vertex-sphere-vs-convex-mesh queries. If Spec A is not
yet complete, the spec should define a brute-force per-triangle fallback.
This is a design decision for the spec, not a hard blocker.

Key areas to validate:
- Flex-vs-mesh: vertex sphere against triangle mesh (or convex hull from
  Spec A). Needs per-triangle or BVH distance query. If Spec A is done,
  can use convex hull; otherwise brute-force per-triangle with early exit.
- Flex-vs-hfield: vertex sphere against heightfield grid. May reuse
  existing `heightfield_sphere_contact` function directly.
- Flex-vs-SDF: vertex sphere against SDF. May reuse existing
  `sdf_sphere_contact` function directly.
- Integration with `narrowphase_sphere_geom` dispatch in `flex_collide.rs`
- Contact generation signatures consistent with existing flex contacts

Follow the workflow exactly:
- Phase 1: Read the MuJoCo C source for flex collision dispatch
- Phase 2: Build SPEC_E_RUBRIC.md
- Phase 3: Grade P1 honestly -- do not proceed to P2-P8 until P1 is A+
- Phase 4: Close gaps, re-grade until all criteria are A+

Do NOT write the spec -- that is the next session.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal. C source is the single source of truth.
```

---

## Session 24: Spec E spec (Deformable Complex-Geom Narrowphase)

- [ ] Complete

```
Phase 9 Collision Completeness -- write Spec E spec.

Read these in order:
1. sim/docs/templates/pre_implementation/SPEC_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
3. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E_RUBRIC.md

Write SPEC_E.md using the rubric as the quality bar:
- MuJoCo Reference section first (C source is the single source of truth)
- Flex-vs-mesh: strategy for vertex sphere distance to triangle mesh
  (per-triangle with early exit? BVH? convex hull from Spec A?)
- Flex-vs-hfield: reuse strategy for heightfield sphere contact
- Flex-vs-SDF: reuse strategy for SDF sphere contact
- Dispatch integration in `flex_collide.rs`
- Contact normal and depth computation for each pair type
- Grade each spec section against the rubric as you write
- Present for approval -- do NOT implement

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/
MuJoCo conformance is the cardinal goal.
```

---

## Session 25: Spec E implementation

- [ ] Complete

```
Phase 9 Collision Completeness -- implement Spec E.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/PHASE9_UMBRELLA.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E.md

Implement per the spec's Execution Order. Commit after each section (S1, S2, ...).
Verify each AC as you go. The spec is the source of truth -- if you discover
a gap, stop and update the spec first.

Flex-vs-hfield and flex-vs-SDF may largely reuse existing sphere contact
functions. Flex-vs-mesh is the hardest pair — ensure robust distance
computation. Test with a flex body falling onto a mesh surface.

Run `cargo test -p sim-core -p sim-mjcf -p sim-conformance-tests` after
each section. MuJoCo conformance is the cardinal goal.
```

---

## Session 26: Spec E review -- create review document

- [ ] Complete

```
Phase 9 Collision Completeness -- create Spec E review document.

Read these:
1. sim/docs/templates/post_implementation/REVIEW_TEMPLATE.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E.md

Copy the review template into this directory as SPEC_E_REVIEW.md. Fill in the
structure by walking the spec: populate the Key Behaviors table from the spec's
Key Behaviors section, list every S1..SN section, every AC, every planned test
T1..TN, every edge case from the Edge Case Inventory, every row from the
Convention Notes table, every item from the Blast Radius section, and every
Out of Scope item.

This session creates the review document -- it does NOT execute the review.
Leave the "Implementation does", "Status", "CortenForge After", and all
verdict fields blank. The next session fills those in by reading the actual
implementation.

Write to: sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E_REVIEW.md
```

---

## Session 27: Spec E review -- execute review

- [ ] Complete

```
Phase 9 Collision Completeness -- execute Spec E review.

Read these:
1. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E_REVIEW.md
2. sim/docs/todo/spec_fleshouts/phase9_collision_completeness/SPEC_E.md
3. The implementation files listed in SPEC_E.md's Files Affected section

Execute the review: for every row in the review document, read the actual
implementation and fill in the verdict. Grade each spec section, verify each
AC has a passing test, check every planned test was written, compare blast
radius predictions against reality, audit convention notes, scan for weak
implementations, and verify all deferred work is tracked.

Fix any "fix before shipping" items in this session. For each fix, update the
review document to reflect the resolution. Run domain tests after fixes.

This is the final session of Phase 9. After completing, verify the Phase 9
aggregate ACs from the umbrella (PH9-AC1 through PH9-AC6) are satisfied.

Present the Review Verdict (section 10) to the user when done.
```

---

## Progress Tracker

| Session | Deliverable | Status | Commit |
|---------|-------------|--------|--------|
| 1 | Phase 9 Umbrella | Done | `ade7750` |
| 2 | T1: §57 (SDF options) | Done | `63dd7f0` |
| 3 | Spec A rubric (§65 Convex Hull) | Done | `9b2a7e9` |
| 4 | Spec A spec | Done | `349e3d5` |
| 5 | Spec A implementation | Done | `8a6b67b` |
| 6 | Spec A review -- create document | Done | `9011f04` |
| 7 | Spec A review -- execute | Done | `ba6c261` |
| 8 | Spec B rubric (§43 Mesh Inertia) | Done | `3e7f593` |
| 9 | Spec B spec | Done | `765d1fa` |
| 10 | Spec B implementation | Done | `5a30583` |
| 11 | Spec B review -- create document | Done | `8188d0a` |
| 12 | Spec B review -- execute | Done | `df4a0df`, `5a2b949` |
| 13 | Spec C rubric (§54 Hfield Pairs) | Done | `ef9ca1e`, `f0981e4` |
| 14 | Spec C spec | Done | `0c59409` |
| 15 | Spec C implementation | Done | `4867fc5` |
| 16 | Spec C review -- create document | Done | `a37eadb` |
| 17 | Spec C review -- execute | Done | `47a4fba` |
| 18 | Spec D rubric (§50 CCD) | Done | `6639c66` |
| 19 | Spec D spec | Done | `119aab9` |
| 20 | Spec D implementation | Done | `990cb85` |
| 21 | Spec D review -- create document | Done | `fa8c6fa` |
| 22 | Spec D review -- execute | Done | `2ba7444` |
| 23 | Spec E rubric (DT-70 Deformable NP) | Done | `db43047` |
| 24 | Spec E spec | Pending | — |
| 25 | Spec E implementation | Pending | — |
| 26 | Spec E review -- create document | Pending | — |
| 27 | Spec E review -- execute | Pending | — |
