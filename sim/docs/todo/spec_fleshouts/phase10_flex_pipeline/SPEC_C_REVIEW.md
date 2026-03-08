# Spec C — Flex Self-Collision Dispatch: Post-Implementation Review

**Spec:** `sim/docs/todo/spec_fleshouts/phase10_flex_pipeline/SPEC_C.md`
**Implementation session(s):** 16
**Reviewer:** AI agent
**Date:** 2026-03-08

---

## Purpose

This review verifies that the implementation matches the approved spec,
surfaces weakly implemented items that should be fixed now, and ensures
deferred work is properly tracked so nothing falls through the cracks.

**Five questions this review answers:**

1. **Closed?** Are the conformance gaps from the spec's Key Behaviors
   table actually closed?
2. **Faithful?** Does the implementation match the spec — every section,
   every AC, every convention note, every planned test?
3. **Predicted?** Did the blast radius match the spec's predictions, or
   were there surprises?
4. **Solid?** Are there any items that technically "work" but are weakly
   implemented (hacks, TODOs, incomplete edge cases, loose tolerances)?
5. **Tracked?** Is every piece of deferred or out-of-scope work tracked
   in `sim/docs/todo/` or `sim/docs/ROADMAP_V1.md`?

---

## 1. Key Behaviors Gap Closure

The spec's Key Behaviors table has a "CortenForge (current)" column showing
the conformance gap *before* implementation. Verify each gap is now closed.

| Behavior | MuJoCo (from spec) | CortenForge Before | CortenForge After | Gap Closed? |
|----------|-------------------|--------------------|-------------------|-------------|
| Self-collision dispatch | Three-condition gate + `mjtFlexSelf` enum dispatch in `mj_collision()` | No self-collision dispatch. `flex_selfcollide` stored as `Vec<bool>` (lossy). | | |
| Internal collision | `mj_collideFlexInternal()` for adjacent-element vertex-face contacts | `flex_internal` parsed and stored but never consumed. | | |
| Midphase acceleration | BVH, SAP, AUTO algorithm selection per `mjtFlexSelf` enum | No midphase for flex. | | |
| Element adjacency | Precomputed from element topology, used to partition internal vs self paths | No element adjacency data structure. | | |
| Contact parameter combination | Trivial identity (both sides same flex) with additive margin/gap | No `contact_param_flex_self()` function. | | |
| Contact encoding | Element-based encoding in `mjContact.geom[0/1]` | `Contact.flex_vertex` supports one flex vertex only. | | |
| Constraint Jacobian | Both sides use flex vertex DOFs | `compute_flex_contact_jacobian()` assumes one flex vertex + one rigid body. | | |

**Unclosed gaps:**
{To be filled during review execution.}

---

## 2. Spec Section Compliance

Walk through every spec section (S1, S2, ...) and grade the implementation
against it. This is the core of the review.

### S1. `FlexSelfCollide` enum + parser/builder pipeline

**Grade:**

**Spec says:**
Add `FlexSelfCollide` enum (None=0, Narrow=1, Bvh=2, Sap=3, Auto=4) in
`enums.rs` with `#[repr(u8)]` discriminants matching MuJoCo's `mjtFlexSelf`.
Derives: Debug, Clone, Copy, PartialEq, Eq, serde (feature-gated). Default
`Auto`. Migrate `Model.flex_selfcollide` from `Vec<bool>` to
`Vec<FlexSelfCollide>` (`model.rs:374`). Update builder (`builder/flex.rs:414`)
to convert MJCF string → enum via match (5 keywords + absent → Auto + unknown
fallback → Auto). Pipeline migration: 7 sites across model.rs, model_init.rs,
builder/flex.rs, builder/mod.rs, builder/init.rs, builder/build.rs. Two sites
intentionally unchanged: `mjcf/types.rs:3802` (keeps `Option<String>`),
`mjcf/parser.rs:3034` (keeps string parsing — conversion in builder).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S2. Contact encoding + contact parameter combination

**Grade:**

**Spec says:**
Add `flex_vertex2: Option<usize>` to `Contact` struct (`contact_types.rs`,
after `flex_vertex` line 86). Initialize to `None` in all existing
constructors (`new`, `with_solver_params`, `with_condim`,
`make_contact_flex_rigid`). Self-collision contacts identified by
`flex_vertex.is_some() && flex_vertex2.is_some()` with `geom1 = geom2 =
usize::MAX` (sentinel). Add `contact_param_flex_self(model, flex_id)` in
`collision/mod.rs` returning `(condim, 2*gap, solref, solimp, friction)` —
gap doubled (additive: both sides same flex). Add `make_contact_flex_self(
model, vertex1, vertex2, pos, normal, depth)` in `collision/flex_collide.rs`
— factory computes `margin = 2 * flex_margin[f]`, `includemargin = margin -
gap`. Margin doubled in the factory (not in `contact_param_flex_self`).

**Implementation does:**

**Gaps (if any):**

**Action:**

### S3. Constraint pipeline updates (Jacobian + assembly)

**Grade:**

**Spec says:**
Add `compute_flex_self_contact_jacobian(model, _data, contact, vi1, vi2) ->
DMatrix<f64>` in `jacobian.rs`. Matrix is `dim × nv` (uses full `model.nv`).
Pinned vertex guard: `if dof_base == usize::MAX { return; }`. Normal row 0:
`+n` on dof1, `-n` on dof2. Tangent rows 1-2: same pattern with t1, t2, but
only if `dim >= 3` (guard condition). Rows 3+ (torsional/rolling) all-zero
(flex vertices have no angular DOFs — zero rows, zero force from solver).
Dispatch update (`jacobian.rs:154`): nested check `if let Some(vi2)` inside
existing `if let Some(vi)` — self-collision before flex-rigid path.
Assembly bodyweight update (`assembly.rs:601-612`): conditional branch — if
`(flex_vertex, flex_vertex2)` both `Some`, use `flexvert_bodyid[vi1]` and
`flexvert_bodyid[vi2]`; else fall back to existing `geom_body` logic.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S4. Element adjacency precomputation

**Grade:**

**Spec says:**
Three new Model fields in `model.rs` (after `flexelem_flexid`), following
CSR-style indexing: `flex_elem_adj: Vec<usize>` (flat sorted adjacency data,
length = sum of all adjacency counts), `flex_elem_adj_adr: Vec<usize>` (start
indices, length `nflexelem`), `flex_elem_adj_num: Vec<usize>` (neighbor
counts, length `nflexelem`). Computation in `builder/flex.rs` via
`compute_element_adjacency(flexelem_data, dataadr, datanum, nflexelem,
nflexvert)` returning `(Vec<usize>, Vec<usize>, Vec<usize>)`. Algorithm:
(1) build vertex-to-element map (`Vec<Vec<usize>>` indexed by vertex),
(2) per element, collect all elements sharing at least one vertex minus self,
(3) sort_unstable + dedup, (4) flatten into flat arrays. Helper
`elements_adjacent(model, e1, e2) -> bool` with `#[inline]` — binary search
on `flex_elem_adj[adr..adr+num]`.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S5. Gate logic + dispatch structure

**Grade:**

**Spec says:**
New `mj_collision_flex_self()` called from `mj_collision()` after
`mj_collision_flex()`. Three conjunctive gate conditions per flex:
(1) `!flex_rigid[f]`, (2) `(contype & conaffinity) != 0`, (3) per-path
enable flags (`flex_internal[f]` for internal, `flex_selfcollide[f] != None`
for self). Dispatch: internal path calls `mj_collide_flex_internal()`,
self-collision path uses match on `FlexSelfCollide` enum to select
Narrow/Bvh/Sap/Auto algorithms. AUTO: BVH for dim=3, SAP otherwise.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S6. Internal collision (adjacent elements)

**Grade:**

**Spec says:**
`mj_collide_flex_internal(model, data, f)` in `flex_collide.rs`. Iterates
adjacent element pairs from S4 adjacency data (e1 < e2 for unique pairs,
both in same flex). **Key algorithm:** for each pair, identifies *non-shared*
vertices of each element (vertices in e1 that are NOT in e2, and vice versa)
and tests only those against opposing element faces via
`test_vertex_against_element_faces()`. Depth threshold: contact generated
only when `depth > -includemargin` (where `includemargin = 2*margin -
2*gap`). dim=2: sphere-triangle test on one face. dim=3: sphere-face against
4 tet faces `[(0,1,2), (0,1,3), (0,2,3), (1,2,3)]`. dim=1: no faces, early
return (no internal contacts for cables). `sphere_triangle_contact()`
implements sphere-plane distance + barycentric inside-check + boundary
closest point fallback. `nearest_vertex()` finds closest face vertex for
`flex_vertex2`. `closest_point_on_triangle()` handles edge/vertex boundary
cases. Helper `elem_vertices()` returns vertex indices for an element.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S7. Self-collision narrowphase (NARROW brute-force)

**Grade:**

**Spec says:**
`mj_collide_flex_self_narrow()` iterates all O(n²) element pairs, skips
adjacent pairs via `elements_adjacent()`, calls `collide_element_pair()`.
`collide_element_pair()` dispatches by dim: dim=2 →
`collide_triangles()` (SAT-based `triangle_triangle_intersection()` from
mesh.rs), dim=3 → `collide_tetrahedra()` (vertex-face both directions,
edge-edge deferred to DT-151), dim=1 → `collide_edges()` (edge-edge
proximity via `closest_points_segments()`). All generate contacts via
`make_contact_flex_self()`. This function is the narrowphase primitive
reused by Spec D.

**Implementation does:**

**Gaps (if any):**

**Action:**

### S8. Midphase acceleration (BVH + SAP + AUTO)

**Grade:**

**Spec says:**
BVH: `mj_collide_flex_self_bvh()` builds per-element AABB tree each step
using existing `Bvh` from `mid_phase.rs`. Per-element query loop: each
element queries BVH for overlapping neighbors, filters self/duplicate/
adjacent, runs narrowphase. O(n log n) typical.
SAP: `mj_collide_flex_self_sap()` computes element AABBs, finds axis of
maximum variance, sorts by AABB min along chosen axis, sweeps for
overlapping projections, checks full 3D overlap, filters adjacent, runs
narrowphase. Helper: `axis_of_max_variance()`, `aabb_overlap()`.
AUTO: BVH for dim=3, SAP otherwise.
Shared: `element_aabb()` computes AABB from vertex positions.

**Implementation does:**

**Gaps (if any):**

**Action:**

---

## 3. Acceptance Criteria Verification

Walk through every AC from the spec. For runtime ACs, confirm the test
exists and passes. For code-review ACs, perform the review now.

| AC | Description | Test(s) | Status | Notes |
|----|-------------|---------|--------|-------|
| AC1 | FlexSelfCollide enum parsing — all 5 keywords parse to correct variants | T1 | | |
| AC2 | FlexSelfCollide default — absent attribute defaults to `Auto` | T1 | | |
| AC3 | Gate — rigid flex produces zero self-contacts | T2 | | |
| AC4 | Gate — incompatible self-bitmask (`contype=2, conaffinity=4`) zero contacts | T3 | | |
| AC5 | Gate — `selfcollide=none` produces zero non-adjacent contacts (internal still possible) | T4 | | |
| AC6 | Gate — `internal=false` produces zero adjacent contacts (self still possible) | T5 | | |
| AC7 | Internal collision — adjacent triangles generate vertex-face contacts (depth ≈ 0.03, normal ≈ (0,0,1)) | T6 | | |
| AC8 | Self-collision — folded cloth generates non-adjacent contacts | T7 | | |
| AC9 | BVH/SAP equivalence — narrow, bvh, sap produce identical contact sets | T8 | | |
| AC10 | AUTO dispatch — BVH for dim=3, SAP for dim≤2 | T9 | | |
| AC11 | Contact encoding — `flex_vertex` and `flex_vertex2` both Some, `geom1/geom2 = usize::MAX` | T7 | | |
| AC12 | Jacobian — `+n` on dof1, `-n` on dof2, exactly 6 nonzero columns, rows 3+ all-zero | T10 | | |
| AC13 | Assembly bodyweight — uses `flexvert_bodyid`, not `geom_body[usize::MAX]` | T10 | | |
| AC14 | Margin formula — `includemargin = 2*margin - 2*gap` (0.016 for margin=0.01, gap=0.002) | T11 | | |
| AC15 | Element adjacency precomputation — adjacency lists match expected topology | T12 | | |
| AC16 | No existing test regression (code review) | — (code review) | | |
| AC17 | Narrowphase primitives reusable by Spec D — `collide_element_pair()`, `sphere_triangle_contact()`, `make_contact_flex_self()` are pub | — (code review) | | |
| AC18 | MuJoCo conformance — self-collision contact count matches MuJoCo exactly | T18 | | |

**Missing or failing ACs:**
{To be filled during review execution.}

---

## 4. Test Plan Completeness

Verify every planned test from the spec was actually written. The AC table
above checks that ACs have *some* test — this section checks that the
spec's *full* test plan was executed.

> **Test numbering note:** The spec uses T1–T18. If the implementation's
> test function names differ (e.g., `t01_*`, `t02_*`), the mapping will be
> noted here during review execution.

### Planned Tests

| Test | Spec Description | Implemented? | Test Function | Notes |
|------|-----------------|-------------|---------------|-------|
| T1 | FlexSelfCollide enum parsing — all 5 keywords + default + `FlexSelfCollide::default() == Auto` | | | |
| T2 | Gate — rigid flex zero contacts (all vertices pinned, compatible bitmask, selfcollide=auto) | | | |
| T3 | Gate — incompatible self-bitmask (`contype=2, conaffinity=4`, selfcollide=auto) | | | |
| T4 | Gate — `selfcollide=none` with `internal=true`, non-adjacent overlap → only adjacent contacts | | | |
| T5 | Gate — `internal=false` with `selfcollide="narrow"`, adjacent penetration → only non-adjacent contacts | | | |
| T6 | Internal collision — 2-triangle mesh, vertex-face contact (depth ≈ 0.03, normal ≈ (0,0,1)) | | | |
| T7 | Self-collision — 4×4 folded cloth, `selfcollide="narrow"`, `internal=false` → ≥1 self-contact | | | |
| T8 | Midphase equivalence — narrow/bvh/sap produce identical contacts on folded cloth | | | |
| T9 | AUTO dispatch — dim=2 → SAP, dim=3 → BVH, contacts match explicit mode | | | |
| T10 | Jacobian + assembly — verify Jacobian DOF structure and bodyweight uses `flexvert_bodyid` | | | |
| T11 | Margin/gap formula — `includemargin ≈ 0.016` for margin=0.01, gap=0.002 | | | |
| T12 | Element adjacency — 4-triangle mesh (2×2 grid), verify adjacency lists and `elements_adjacent()` | | | |

### Supplementary Tests

Tests that were added beyond the spec's planned test list — additional
coverage discovered during implementation or review. These don't map 1:1
to ACs but strengthen the test suite.

| Test | Description | Test Function | Notes |
|------|-------------|---------------|-------|
| T13 | Multi-flex model: two flexes, one selfcollide=auto, one selfcollide=none — only enabled flex generates contacts | | |
| T14 | Sphere-triangle primitive unit test: sphere at (0,0,0.05) r=0.1 vs triangle at z=0 → depth=0.05, normal=(0,0,1) | | |
| T15 | Single-element flex — zero contacts (no adjacent/non-adjacent pairs) | | |
| T16 | Zero-element flex — zero contacts (early return on elem_count < 2) | | |
| T17 | dim=1 cable self-collision — AUTO→SAP, edge-edge contacts, internal→zero (no faces) | | |
| T18 | MuJoCo conformance — 4×4 shell selfcollide=narrow, fold dz=-0.05, match ncon/depth/normal within 1e-10 | | |

### Edge Case Inventory

Cross-reference the spec's Edge Case Inventory. Were all edge cases tested?

| Edge Case | Spec Says | Tested? | Test Function | Notes |
|-----------|----------|---------|---------------|-------|
| Rigid flex (`flex_rigid=true`) | Gate condition 1 must skip | | | |
| Self-bitmask zero (`contype & conaffinity == 0`) | Gate condition 2 must skip | | | |
| `selfcollide=none` | Gate condition 3b must skip non-adjacent | | | |
| `internal=false` | Gate condition 3a must skip adjacent | | | |
| Single-element flex | No non-adjacent/adjacent pairs → zero contacts | | | |
| Zero-element flex (`nelem=0`) | No element pairs → no contacts | | | |
| dim=1 cable | Edge-edge proximity, AUTO→SAP, no internal contacts | | | |
| All elements coplanar (flat mesh) | No self-penetration → zero contacts | | | |
| `activelayers` filtering | Deferred to DT-150 | — | — | Deferred |

**Missing tests:**
{To be filled during review execution.}

---

## 5. Blast Radius Verification

The spec predicted behavioral changes, files affected, and existing test
impact. Compare predictions against reality. Surprises here reveal spec
gaps worth learning from.

### Behavioral Changes: Predicted vs Actual

| Predicted Change | Actually Happened? | Notes |
|-----------------|-------------------|-------|
| `flex_selfcollide` type: `Vec<bool>` → `Vec<FlexSelfCollide>` | | |
| Self-collision contacts: not generated → generated for non-rigid flexes with compatible bitmask | | |
| Internal contacts: not generated → generated for adjacent elements when `internal=true` | | |
| Contact struct size: `flex_vertex2: Option<usize>` added | | |

### Files Affected: Predicted vs Actual

| Predicted File | Actually Changed? | Unexpected Files Changed |
|---------------|-------------------|------------------------|
| `sim/L0/core/src/types/enums.rs` — add `FlexSelfCollide` enum (+25) | | |
| `sim/L0/core/src/types/model.rs` — change `flex_selfcollide` type, add adjacency fields (~15 mod, +10 new) | | |
| `sim/L0/core/src/types/model_init.rs` — init adjacency fields (+3) | | |
| `sim/L0/core/src/types/contact_types.rs` — add `flex_vertex2` field (+8) | | |
| `sim/L0/core/src/collision/mod.rs` — add dispatch + `contact_param_flex_self()` (+60) | | |
| `sim/L0/core/src/collision/flex_collide.rs` — internal collision, self-collision, midphase, primitives (+400) | | |
| `sim/L0/core/src/constraint/jacobian.rs` — add `compute_flex_self_contact_jacobian()` + dispatch update (+45) | | |
| `sim/L0/core/src/constraint/assembly.rs` — bodyweight update for self-collision contacts (+15) | | |
| `sim/L0/mjcf/src/builder/flex.rs` — string→enum conversion, adjacency computation (+60) | | |
| `sim/L0/mjcf/src/builder/mod.rs` — change `flex_selfcollide` type (~1 mod) | | |
| `sim/L0/mjcf/src/builder/init.rs` — init as `Vec<FlexSelfCollide>` (~1 mod) | | |
| `sim/L0/core/src/collision/flex_collide.rs` (tests) — new test module (+250) | | |

{List any files changed that were NOT in the spec's prediction:}

| Unexpected File | Why It Was Changed |
|----------------|-------------------|
| | |

### Existing Test Impact: Predicted vs Actual

| Test | Predicted Impact | Actual Impact | Surprise? |
|------|-----------------|---------------|-----------|
| `test_flex_internal_false` (`builder/mod.rs:1302`) | Pass (unchanged) | | |
| `test_flex_internal_default` (`builder/mod.rs:1336`) | Pass (unchanged) | | |
| Flex-rigid collision tests (Spec E tests) | Pass (unchanged) | | |
| Flex passive force tests (`forward/passive.rs`) | Pass (unchanged) | | |
| Phase 9 collision tests (`collision/mod.rs`) | Pass (unchanged) | | |

**Unexpected regressions:**
{To be filled during review execution.}

### Non-Modification Sites

The spec explicitly lists files/functions that should NOT be modified.
Verify none of these were changed.

| File:line | What it does | Why NOT modified | Actually unchanged? |
|-----------|-------------|-----------------|---------------------|
| `collision/mod.rs:549-600` | `mj_collision_flex()` — flex-rigid dispatch | Different collision path (flex-vs-rigid, not self) | |
| `forward/passive.rs` | Flex passive forces (spring, bending) | Different pipeline stage — not collision | |
| `collision/mod.rs:174-226` | `contact_param_flex_rigid()` | Different pair type — flex-rigid, not flex-self | |
| `constraint/jacobian.rs:20-139` | `compute_flex_contact_jacobian()` | Handles flex-rigid only — new function for self | |

---

## 6. Convention Notes Audit

Check every row in the spec's Convention Notes table. Convention mismatches
are the #1 source of silent conformance bugs.

| Convention | Spec's Porting Rule | Implementation Follows? | Notes |
|------------|--------------------|-----------------------|-------|
| Element connectivity | Use `flexelem_dataadr`/`flexelem_datanum` for element-to-vertex lookup. Element global index = `flex_elemadr[f] + e`. | | |
| `mjtFlexSelf` enum | `FlexSelfCollide` Rust enum with `#[repr(u8)]` discriminants matching MuJoCo values. Direct port — variant names match. | | |
| Vertex positions | Use `data.flexvert_xpos[v]` directly — no manual `3*v+x` indexing. | | |
| Contact encoding (self) | `flex_vertex` + `flex_vertex2` (both `Option<usize>`). When both `is_some()` → self-collision. `geom1 = geom2 = usize::MAX` (sentinel). | | |
| Contact encoding (flex-rigid) | `Contact.geom1 = geom2 = geom_idx`, `flex_vertex = Some(vi)`. No change — existing convention preserved. | | |
| Flex element indexing | `model.flex_elemadr[f]..model.flex_elemadr[f]+model.flex_elemnum[f]`. Direct port — no translation needed. | | |
| BVH API | Build with `BvhPrimitive { aabb, index: elem_idx, data: 0 }`. `query()` returns local primitive indices (0..n), not global. Map back: `elem_start + returned_index`. | | |
| Triangle-triangle test | Use existing SAT-based `triangle_triangle_intersection()` for dim=2 non-adjacent. `TriTriContact` has `point`, `normal`, `depth` → map to `make_contact_flex_self()`. | | |
| Contact frame | `compute_tangent_frame(&normal)` produces orthonormal tangent basis. Same convention for self-collision and flex-rigid. | | |

---

## 7. Weak Implementation Inventory

Items that technically work but aren't solid. These should be fixed now —
"weak" items left unfixed tend to become permanent technical debt.

| # | Location | Description | Severity | Action |
|---|----------|-------------|----------|--------|
| | | | | |

{To be filled during review execution.}

---

## 8. Deferred Work Tracker

Every item that was in the spec's scope but not fully implemented, plus
anything discovered during implementation or review that's out of scope.
**The goal: nothing deferred is untracked.**

### From Spec's "Out of Scope" Section

| Item | Spec Reference | Tracked In | Tracking ID | Verified? |
|------|---------------|------------|-------------|-----------|
| `activelayers` runtime filtering | Out of Scope, bullet 1 | | DT-150 | |
| Edge-edge tests for dim=3 tet self-collision | Out of Scope, bullet 2 | | DT-151 | |
| Barycentric force distribution on face side | Out of Scope, bullet 3 | | DT-152 | |
| Flex-flex cross-object collision (§42A-v) | Out of Scope, bullet 4 | Spec D, Sessions 19–21 | DT-143 | |
| Body-attached flex vertex Jacobian (§27D) | Out of Scope, bullet 5 | | §27D | |
| Contact parameter combination for flex-flex | Out of Scope, bullet 6 | Spec D | — | |

### Discovered During Implementation

Items that came up during implementation that weren't anticipated by the
spec. These are the most likely to be untracked.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Discovered During Review

Items found during this review that were not surfaced during
implementation. Reviews are a discovery mechanism.

| Item | Discovery Context | Tracked In | Tracking ID | Verified? |
|------|-------------------|------------|-------------|-----------|
| | | | | |

{To be filled during review execution.}

### Spec Gaps Found During Implementation

Items where the spec was wrong or incomplete and was (or should have been)
updated. Verify the spec was actually updated.

| Gap | What Happened | Spec Updated? | Notes |
|-----|--------------|---------------|-------|
| | | | |

{To be filled during review execution.}

---

## 9. Test Coverage Summary

Quick sanity check on test health after the implementation.

**Domain test results:**
```
{To be filled during review execution.}
```

**New tests added:** {count}
**Tests modified:** {count}
**Pre-existing test regressions:** {count — should be 0}

**Clippy:** {clean / N warnings}
**Fmt:** {clean / issues}

---

## 10. Review Verdict

| Category | Section | Status |
|----------|---------|--------|
| Key behaviors gap closure | 1 | |
| Spec section compliance | 2 | |
| Acceptance criteria | 3 | |
| Test plan completeness | 4 | |
| Blast radius accuracy | 5 | |
| Convention fidelity | 6 | |
| Weak items | 7 | |
| Deferred work tracking | 8 | |
| Test health | 9 | |

**Overall:**

**Items fixed during review:**

**Items to fix before shipping:**

**Items tracked for future work:**
